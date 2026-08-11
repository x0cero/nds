//! NDS 3D geometry command processor (phase 1: no rasterizer).
//!
//! Commands arrive either packed through the GXFIFO port at 0x04000400 (up to
//! four command bytes in one word, then each command's parameters in order) or
//! through the per-command register ports at 0x04000440-0x05FF, where the
//! command index is (addr >> 2) & 0x7F.
//!
//! Timing model: every command executes the instant its last parameter
//! arrives, so the FIFO count reads as zero. That keeps the property Platinum's
//! title sequence depends on: with an empty FIFO, both GXSTAT IRQ conditions
//! ("less than half full" and "empty") are permanently true, so any enabled
//! IRQ mode holds the geometry IRQ asserted (main.rs re-raises it per line).
//!
//! Matrices are 4x4 in 20.12 fixed point, row-vector convention (v' = v * M),
//! stored row-major, multiply-accumulate in i64.

static GXLOG: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_GXLOG").is_ok());

/// NDS_LIGHTLOG=1: for the first few vertices of each geometry frame, report
/// where the vertex colour came from (an explicit COLOR command or the
/// lighting unit) and, when it came from lighting, every input and every
/// per-light contribution. Capped per frame so a long run stays readable.
static LIGHTLOG: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_LIGHTLOG").is_ok());
const LIGHTLOG_PER_FRAME: u32 = 6;
fn lightlog_budget() -> u32 {
    LIGHTLOG_PER_FRAME
}

/// A never-uploaded shininess table reads back as zeroes on hardware.
fn default_shininess() -> Vec<u8> {
    vec![0u8; 128]
}

pub type Mtx = [i32; 16];

const IDENTITY: Mtx = [
    0x1000, 0, 0, 0, //
    0, 0x1000, 0, 0, //
    0, 0, 0x1000, 0, //
    0, 0, 0, 0x1000,
];

/// r = a * b (row-vector order: applying `a` first, then `b`).
fn mtx_mul(a: &Mtx, b: &Mtx) -> Mtx {
    let mut r = [0i32; 16];
    for i in 0..4 {
        for j in 0..4 {
            let mut acc: i64 = 0;
            for k in 0..4 {
                acc += a[i * 4 + k] as i64 * b[k * 4 + j] as i64;
            }
            r[i * 4 + j] = (acc >> 12) as i32;
        }
    }
    r
}

/// Parameter count for each geometry command (GBATEK "DS 3D Video").
fn param_count(cmd: u8) -> Option<usize> {
    Some(match cmd {
        0x10 => 1,          // MTX_MODE
        0x11 => 0,          // MTX_PUSH
        0x12 => 1,          // MTX_POP
        0x13 => 1,          // MTX_STORE
        0x14 => 1,          // MTX_RESTORE
        0x15 => 0,          // MTX_IDENTITY
        0x16 => 16,         // MTX_LOAD_4x4
        0x17 => 12,         // MTX_LOAD_4x3
        0x18 => 16,         // MTX_MULT_4x4
        0x19 => 12,         // MTX_MULT_4x3
        0x1A => 9,          // MTX_MULT_3x3
        0x1B => 3,          // MTX_SCALE
        0x1C => 3,          // MTX_TRANS
        0x20 => 1,          // COLOR
        0x21 => 1,          // NORMAL
        0x22 => 1,          // TEXCOORD
        0x23 => 2,          // VTX_16
        0x24 => 1,          // VTX_10
        0x25 => 1,          // VTX_XY
        0x26 => 1,          // VTX_XZ
        0x27 => 1,          // VTX_YZ
        0x28 => 1,          // VTX_DIFF
        0x29 => 1,          // POLYGON_ATTR
        0x2A => 1,          // TEXIMAGE_PARAM
        0x2B => 1,          // PLTT_BASE
        0x30 => 1,          // DIF_AMB
        0x31 => 1,          // SPE_EMI
        0x32 => 1,          // LIGHT_VECTOR
        0x33 => 1,          // LIGHT_COLOR
        0x34 => 32,         // SHININESS
        0x40 => 1,          // BEGIN_VTXS
        0x41 => 0,          // END_VTXS
        0x50 => 1,          // SWAP_BUFFERS
        0x60 => 1,          // VIEWPORT
        0x70 => 3,          // BOX_TEST
        0x71 => 2,          // POS_TEST
        0x72 => 1,          // VEC_TEST
        _ => return None,
    })
}

#[derive(Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct Vertex {
    /// Clip-space coordinates (x, y, z, w), 20.12.
    pub clip: [i32; 4],
    pub color: u16, // RGB15
    pub tex: [i16; 2],
}

#[derive(Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct Polygon {
    pub attr: u32,
    pub texparam: u32,
    pub pltt: u32,
    pub verts: [u16; 4],
    pub nverts: u8,
}

pub const MAX_POLYS: usize = 2048;
pub const MAX_VERTS: usize = 6144;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Gpu3d {
    // Command decode state.
    pending: std::collections::VecDeque<u8>, // packed commands awaiting params
    cur_cmd: Option<u8>,
    params: Vec<u32>,

    // Matrix engine.
    pub mtx_mode: u8,
    proj: Mtx,
    pos: Mtx,
    vec: Mtx,
    tex: Mtx,
    proj_stack: Mtx,
    pos_stack: Box<[Mtx; 32]>,
    vec_stack: Box<[Mtx; 32]>,
    tex_stack: Mtx,
    pub proj_sp: u8,
    pub pos_sp: u8, // shared by the coupled position/vector stacks
    pub stack_error: bool,
    clip_dirty: bool,
    clip: Mtx,

    // Vertex state.
    cur_color: u16,
    cur_tex: [i16; 2],
    last_vtx: [i32; 3], // 4.12 model-space, for VTX_XY/XZ/YZ/DIFF
    poly_attr: u32,     // written value; latched into cur_attr at BEGIN_VTXS
    cur_attr: u32,
    teximage: u32,
    pltt_base: u32,
    prim_mode: u8,
    strip_v: Vec<Vertex>, // vertices submitted in the current primitive
    strip_ci: Vec<Option<u16>>, // vertex-RAM index once committed
    strip_parity: bool,

    // Lighting.
    light_vec: [[i32; 3]; 4],  // .12, already through the vector matrix
    light_half: [[i32; 3]; 4], // .12
    light_color: [[i32; 3]; 4],
    dif: [i32; 3],
    amb: [i32; 3],
    spe: [i32; 3],
    emi: [i32; 3],
    /// Specular shininess table: 128 entries, 0.8 fixed point (SHININESS, cmd
    /// 34h). Only consulted when SPE_EMI bit 15 asks for it.
    ///
    /// Kept OUT of savestates (`skip`) even though everything else in this
    /// struct is snapshotted: the state file is positional bincode, so adding a
    /// serialized field here would invalidate every existing .state fixture,
    /// including one saved from a real playthrough that cannot be regenerated.
    /// The cost is small - SPE_EMI (which carries the enable bit) is re-sent
    /// with every model, so a resumed state re-latches it on its first frame.
    #[serde(skip, default = "default_shininess")]
    shine: Vec<u8>,
    #[serde(skip)]
    shine_table_en: bool,

    // Geometry RAM, double-buffered.
    pub verts: Vec<Vertex>,
    pub polys: Vec<Polygon>,
    pub disp_verts: Vec<Vertex>,
    pub disp_polys: Vec<Polygon>,
    pub ram_overflow: bool,

    pub swap_param: u32,
    pub swap_count: u64,
    pub viewport: [u8; 4],
    pub clear_color: u32,
    pub clear_depth: u16,
    pub pos_result: [i32; 4],
    /// Polygons stored during the latest completed geometry frame.
    pub last_poly_count: usize,
    pub last_vert_count: usize,
    /// Polygons dropped by storage-side trivial reject this geometry frame.
    pub rejected: u32,
    /// NDS_LIGHTLOG budget for the current geometry frame (diagnostic only).
    /// Not part of the machine state; savestates restore it to a full budget.
    #[serde(skip, default = "lightlog_budget")]
    lightlog_left: u32,
    /// NDS_LIGHTLOG: distinct (material, light) setups used by NORMAL this
    /// geometry frame, with a vertex count each. Diagnostic only.
    #[serde(skip)]
    lightlog_setups: Vec<(LightSetup, u32)>,
}

/// Every input the lighting unit uses, so NDS_LIGHTLOG can report how many
/// genuinely different material/light configurations a frame contains rather
/// than only the first few vertices.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
struct LightSetup {
    lights: u32,
    dif: [i32; 3],
    amb: [i32; 3],
    spe: [i32; 3],
    emi: [i32; 3],
    color: [[i32; 3]; 4],
    shine_table_en: bool,
}

impl Gpu3d {
    pub fn new() -> Self {
        Self {
            pending: Default::default(),
            cur_cmd: None,
            params: Vec::new(),
            mtx_mode: 0,
            proj: IDENTITY,
            pos: IDENTITY,
            vec: IDENTITY,
            tex: IDENTITY,
            proj_stack: IDENTITY,
            pos_stack: Box::new([IDENTITY; 32]),
            vec_stack: Box::new([IDENTITY; 32]),
            tex_stack: IDENTITY,
            proj_sp: 0,
            pos_sp: 0,
            stack_error: false,
            clip_dirty: true,
            clip: IDENTITY,
            cur_color: 0x7FFF,
            cur_tex: [0; 2],
            last_vtx: [0; 3],
            poly_attr: 0,
            cur_attr: 0,
            teximage: 0,
            pltt_base: 0,
            prim_mode: 0,
            strip_v: Vec::new(),
            strip_ci: Vec::new(),
            strip_parity: false,
            light_vec: [[0, 0, -0x1000]; 4],
            light_half: [[0, 0, -0x1000]; 4],
            light_color: [[31, 31, 31]; 4],
            dif: [31; 3],
            amb: [0; 3],
            spe: [0; 3],
            emi: [0; 3],
            shine: default_shininess(),
            shine_table_en: false,
            verts: Vec::new(),
            polys: Vec::new(),
            disp_verts: Vec::new(),
            disp_polys: Vec::new(),
            ram_overflow: false,
            swap_param: 0,
            swap_count: 0,
            viewport: [0, 0, 255, 191],
            clear_color: 0,
            clear_depth: 0x7FFF,
            pos_result: [0; 4],
            last_poly_count: 0,
            last_vert_count: 0,
            rejected: 0,
            lightlog_left: LIGHTLOG_PER_FRAME,
            lightlog_setups: Vec::new(),
        }
    }

    // ---- Command input --------------------------------------------------

    /// Packed write to GXFIFO (0x04000400).
    pub fn write_fifo(&mut self, v: u32) {
        if self.cur_cmd.is_none() && self.pending.is_empty() {
            // Command word: up to four command bytes, low byte first.
            for i in 0..4 {
                let c = (v >> (i * 8)) as u8;
                if c != 0 && param_count(c).is_some() {
                    self.pending.push_back(c);
                }
            }
            self.advance();
        } else {
            self.feed_param(v);
        }
    }

    /// Direct per-command register write (0x04000440-0x05FF).
    pub fn write_port(&mut self, cmd: u8, v: u32) {
        if param_count(cmd).is_none() {
            return;
        }
        // A register write while a packed sequence is mid-flight shouldn't
        // happen in practice; treat it as its own immediate command stream.
        match self.cur_cmd {
            Some(c) if c == cmd => self.feed_param(v),
            _ => {
                self.cur_cmd = Some(cmd);
                self.params.clear();
                self.feed_param(v);
            }
        }
    }

    /// Pop zero-parameter commands off the queue and set up the next one
    /// that still needs parameters.
    fn advance(&mut self) {
        while self.cur_cmd.is_none() {
            let Some(c) = self.pending.pop_front() else { return };
            if param_count(c) == Some(0) {
                self.execute(c, &[]);
            } else {
                self.cur_cmd = Some(c);
                self.params.clear();
            }
        }
    }

    fn feed_param(&mut self, v: u32) {
        let Some(cmd) = self.cur_cmd else {
            // Stray parameter with nothing pending: treat as a command word.
            self.write_fifo(v);
            return;
        };
        self.params.push(v);
        if self.params.len() >= param_count(cmd).unwrap_or(1) {
            let params = std::mem::take(&mut self.params);
            self.cur_cmd = None;
            self.execute(cmd, &params);
            self.advance();
        }
    }

    // ---- Matrix helpers -------------------------------------------------

    fn mult(&mut self, m: &Mtx) {
        match self.mtx_mode {
            0 => self.proj = mtx_mul(m, &self.proj),
            1 => self.pos = mtx_mul(m, &self.pos),
            2 => {
                self.pos = mtx_mul(m, &self.pos);
                self.vec = mtx_mul(m, &self.vec);
            }
            _ => self.tex = mtx_mul(m, &self.tex),
        }
        self.clip_dirty = true;
    }

    fn load(&mut self, m: Mtx) {
        match self.mtx_mode {
            0 => self.proj = m,
            1 => self.pos = m,
            2 => {
                self.pos = m;
                self.vec = m;
            }
            _ => self.tex = m,
        }
        self.clip_dirty = true;
    }

    fn clip_matrix(&mut self) -> Mtx {
        if self.clip_dirty {
            self.clip = mtx_mul(&self.pos, &self.proj);
            self.clip_dirty = false;
        }
        self.clip
    }

    fn mtx_from_4x4(p: &[u32]) -> Mtx {
        let mut m = [0i32; 16];
        for i in 0..16 {
            m[i] = p[i] as i32;
        }
        m
    }

    fn mtx_from_4x3(p: &[u32]) -> Mtx {
        let mut m = IDENTITY;
        for r in 0..4 {
            for c in 0..3 {
                m[r * 4 + c] = p[r * 3 + c] as i32;
            }
            m[r * 4 + 3] = if r == 3 { 0x1000 } else { 0 };
        }
        m
    }

    fn mtx_from_3x3(p: &[u32]) -> Mtx {
        let mut m = IDENTITY;
        for r in 0..3 {
            for c in 0..3 {
                m[r * 4 + c] = p[r * 3 + c] as i32;
            }
            m[r * 4 + 3] = 0;
        }
        m[12] = 0;
        m[13] = 0;
        m[14] = 0;
        m
    }

    // ---- Lighting -------------------------------------------------------

    /// Rotate a .12 vector by the 3x3 part of the vector matrix.
    fn vec_rotate(&self, v: [i32; 3]) -> [i32; 3] {
        let mut out = [0i32; 3];
        for j in 0..3 {
            let acc = v[0] as i64 * self.vec[j] as i64
                + v[1] as i64 * self.vec[4 + j] as i64
                + v[2] as i64 * self.vec[8 + j] as i64;
            out[j] = (acc >> 12) as i32;
        }
        out
    }

    fn apply_lighting(&mut self, raw_n: [i32; 3]) {
        let n = self.vec_rotate(raw_n);
        let dot = |a: [i32; 3], b: [i32; 3]| -> i64 {
            ((a[0] as i64 * b[0] as i64 + a[1] as i64 * b[1] as i64 + a[2] as i64 * b[2] as i64)
                >> 12)
                .clamp(-0x1000, 0x1000)
        };
        if *LIGHTLOG {
            let setup = LightSetup {
                lights: self.cur_attr & 15,
                dif: self.dif,
                amb: self.amb,
                spe: self.spe,
                emi: self.emi,
                color: self.light_color,
                shine_table_en: self.shine_table_en,
            };
            match self.lightlog_setups.iter_mut().find(|(s, _)| *s == setup) {
                Some((_, n)) => *n += 1,
                None => self.lightlog_setups.push((setup, 1)),
            }
        }
        let log = *LIGHTLOG && self.lightlog_left > 0;
        if log {
            self.lightlog_left -= 1;
            eprintln!(
                "[light] NORMAL raw=({},{},{}) rotated=({},{},{}) attr={:#010X} lights={:04b}",
                raw_n[0], raw_n[1], raw_n[2], n[0], n[1], n[2],
                self.cur_attr, self.cur_attr & 15,
            );
            eprintln!(
                "[light]   dif={:?} amb={:?} spe={:?} emi={:?}",
                self.dif, self.amb, self.spe, self.emi
            );
        }
        let mut c = [self.emi[0] as i64, self.emi[1] as i64, self.emi[2] as i64];
        for l in 0..4 {
            if self.cur_attr >> l & 1 == 0 {
                continue;
            }
            let difl = (-dot(self.light_vec[l], n)).max(0); // 0..0x1000
            let s = (-dot(self.light_half[l], n)).max(0);
            let mut specl = (s * s) >> 12; // 0..0x1000
            if self.shine_table_en {
                // SPE_EMI bit 15: remap the squared level through the game's
                // 128-entry table (entries are 0.8 fixed point).
                let idx = ((specl >> 5) as usize).min(127);
                specl = (self.shine[idx] as i64) << 4;
            }
            let mut contrib = [0i64; 3];
            for ch in 0..3 {
                let lc = self.light_color[l][ch] as i64;
                contrib[ch] = (self.spe[ch] as i64 * lc * specl) / (31 << 12)
                    + (self.dif[ch] as i64 * lc * difl) / (31 << 12)
                    + (self.amb[ch] as i64 * lc) / 31;
                c[ch] += contrib[ch];
            }
            if log {
                let dl = |v: i64| v as f64 / 4096.0;
                eprintln!(
                    "[light]   light{l} col={:?} dir=({},{},{}) diffuse_lvl={:.3} shininess_lvl={:.3} adds={:?}",
                    self.light_color[l],
                    self.light_vec[l][0], self.light_vec[l][1], self.light_vec[l][2],
                    dl(difl), dl(specl), contrib,
                );
            }
        }
        let r = c[0].clamp(0, 31) as u16;
        let g = c[1].clamp(0, 31) as u16;
        let b = c[2].clamp(0, 31) as u16;
        if log {
            eprintln!("[light]   => vertex colour ({r},{g},{b}) (unclamped {c:?})");
        }
        self.cur_color = r | g << 5 | b << 10;
    }

    // ---- Vertex pipeline ------------------------------------------------

    fn submit_vertex(&mut self, x: i32, y: i32, z: i32) {
        self.last_vtx = [x, y, z];
        let m = self.clip_matrix();
        let mut clip = [0i32; 4];
        for j in 0..4 {
            let acc = x as i64 * m[j] as i64
                + y as i64 * m[4 + j] as i64
                + z as i64 * m[8 + j] as i64
                + ((m[12 + j] as i64) << 12);
            clip[j] = (acc >> 12) as i32;
        }
        // Vertices are only committed to vertex RAM when a polygon using
        // them is accepted: outdoor scenes trivially reject over a thousand
        // offscreen polygons per frame, and storing their vertices anyway
        // fills the 6144-vertex RAM before late-drawn billboards (the player
        // character) arrive.
        self.strip_v.push(Vertex { clip, color: self.cur_color, tex: self.cur_tex });
        self.strip_ci.push(None);
        let n = self.strip_v.len();
        match self.prim_mode {
            0 => {
                if n == 3 {
                    self.emit_poly([0, 1, 2, 0], 3);
                    self.strip_v.clear();
                    self.strip_ci.clear();
                }
            }
            1 => {
                if n == 4 {
                    self.emit_poly([0, 1, 2, 3], 4);
                    self.strip_v.clear();
                    self.strip_ci.clear();
                }
            }
            2 => {
                if n >= 3 {
                    let (a, b, c) = (n - 3, n - 2, n - 1);
                    // Alternate winding so every triangle faces the same way.
                    let v = if self.strip_parity { [b, a, c, 0] } else { [a, b, c, 0] };
                    self.strip_parity = !self.strip_parity;
                    self.emit_poly(v, 3);
                }
            }
            _ => {
                if n >= 4 && n % 2 == 0 {
                    // Submission order a,b,c,d forms the quad a,b,d,c.
                    self.emit_poly([n - 4, n - 3, n - 1, n - 2], 4);
                }
            }
        }
    }

    /// Accept or reject a completed polygon whose corners are indices into
    /// the current primitive's vertex list.
    fn emit_poly(&mut self, v: [usize; 4], nverts: u8) {
        if self.polys.len() >= MAX_POLYS {
            self.ram_overflow = true;
            return;
        }
        // Trivial reject: all vertices outside the same frustum plane. Only
        // valid when every w is positive; polygons crossing w<=0 are kept and
        // clipped properly at render time.
        let any_wneg =
            v[..nverts as usize].iter().any(|&i| self.strip_v[i].clip[3] <= 0);
        if !any_wneg {
            for axis in 0..3 {
                let mut all_lo = true;
                let mut all_hi = true;
                for &i in &v[..nverts as usize] {
                    let c = self.strip_v[i].clip;
                    let (x, w) = (c[axis] as i64, c[3] as i64);
                    if x >= -w {
                        all_lo = false;
                    }
                    if x <= w {
                        all_hi = false;
                    }
                }
                if all_lo || all_hi {
                    self.rejected += 1;
                    return;
                }
            }
        }
        // Commit vertices to vertex RAM (sharing strip vertices already
        // committed by an earlier accepted polygon).
        let mut idx = [0u16; 4];
        for (k, &i) in v[..nverts as usize].iter().enumerate() {
            idx[k] = match self.strip_ci[i] {
                Some(ci) => ci,
                None => {
                    if self.verts.len() >= MAX_VERTS {
                        self.ram_overflow = true;
                        return;
                    }
                    let ci = self.verts.len() as u16;
                    self.verts.push(self.strip_v[i]);
                    self.strip_ci[i] = Some(ci);
                    ci
                }
            };
        }
        self.polys.push(Polygon {
            attr: self.cur_attr,
            texparam: self.teximage,
            pltt: self.pltt_base,
            verts: idx,
            nverts,
        });
    }

    // ---- Execution ------------------------------------------------------

    fn execute(&mut self, cmd: u8, p: &[u32]) {
        let s10 = |v: u32| ((v as i32) << 22) >> 22; // sign-extend 10 bits
        match cmd {
            0x10 => self.mtx_mode = (p[0] & 3) as u8,
            0x11 => {
                // MTX_PUSH
                match self.mtx_mode {
                    0 => {
                        if self.proj_sp >= 1 {
                            self.stack_error = true;
                        }
                        self.proj_stack = self.proj;
                        self.proj_sp = 1;
                    }
                    3 => {
                        self.tex_stack = self.tex;
                    }
                    _ => {
                        if self.pos_sp >= 31 {
                            self.stack_error = true;
                        }
                        let sp = (self.pos_sp & 31) as usize;
                        self.pos_stack[sp] = self.pos;
                        self.vec_stack[sp] = self.vec;
                        self.pos_sp = (self.pos_sp + 1) & 63;
                    }
                }
            }
            0x12 => {
                // MTX_POP: signed 6-bit count for pos/vec, ignored for proj.
                match self.mtx_mode {
                    0 => {
                        self.proj_sp = 0;
                        self.proj = self.proj_stack;
                    }
                    3 => self.tex = self.tex_stack,
                    _ => {
                        let n = ((p[0] as i32) << 26) >> 26;
                        let sp = (self.pos_sp as i32 - n) & 63;
                        self.pos_sp = sp as u8;
                        if self.pos_sp >= 31 {
                            self.stack_error = true;
                        }
                        let i = (self.pos_sp & 31) as usize;
                        self.pos = self.pos_stack[i];
                        self.vec = self.vec_stack[i];
                    }
                }
                self.clip_dirty = true;
            }
            0x13 => {
                // MTX_STORE
                match self.mtx_mode {
                    0 => self.proj_stack = self.proj,
                    3 => self.tex_stack = self.tex,
                    _ => {
                        let i = (p[0] & 31) as usize;
                        if p[0] & 31 == 31 {
                            self.stack_error = true;
                        }
                        self.pos_stack[i] = self.pos;
                        self.vec_stack[i] = self.vec;
                    }
                }
            }
            0x14 => {
                // MTX_RESTORE
                match self.mtx_mode {
                    0 => self.proj = self.proj_stack,
                    3 => self.tex = self.tex_stack,
                    _ => {
                        let i = (p[0] & 31) as usize;
                        if p[0] & 31 == 31 {
                            self.stack_error = true;
                        }
                        self.pos = self.pos_stack[i];
                        self.vec = self.vec_stack[i];
                    }
                }
                self.clip_dirty = true;
            }
            0x15 => self.load(IDENTITY),
            0x16 => self.load(Self::mtx_from_4x4(p)),
            0x17 => self.load(Self::mtx_from_4x3(p)),
            0x18 => self.mult(&Self::mtx_from_4x4(p)),
            0x19 => self.mult(&Self::mtx_from_4x3(p)),
            0x1A => self.mult(&Self::mtx_from_3x3(p)),
            0x1B => {
                // MTX_SCALE: never touches the vector matrix, even in mode 2.
                let mut m = IDENTITY;
                m[0] = p[0] as i32;
                m[5] = p[1] as i32;
                m[10] = p[2] as i32;
                match self.mtx_mode {
                    0 => self.proj = mtx_mul(&m, &self.proj),
                    1 | 2 => self.pos = mtx_mul(&m, &self.pos),
                    _ => self.tex = mtx_mul(&m, &self.tex),
                }
                self.clip_dirty = true;
            }
            0x1C => {
                let mut m = IDENTITY;
                m[12] = p[0] as i32;
                m[13] = p[1] as i32;
                m[14] = p[2] as i32;
                self.mult(&m);
            }
            0x20 => {
                self.cur_color = (p[0] & 0x7FFF) as u16;
                if *LIGHTLOG && self.lightlog_left > 0 {
                    self.lightlog_left -= 1;
                    eprintln!(
                        "[light] COLOR explicit ({},{},{}) attr={:#010X} lights={:04b}",
                        self.cur_color & 31,
                        self.cur_color >> 5 & 31,
                        self.cur_color >> 10 & 31,
                        self.poly_attr,
                        self.poly_attr & 15,
                    );
                }
            }
            0x21 => {
                // NORMAL: 3x10-bit, 0.9 fixed -> .12.
                let n = [s10(p[0]) << 3, s10(p[0] >> 10) << 3, s10(p[0] >> 20) << 3];
                self.apply_lighting(n);
            }
            0x22 => {
                // TEXCOORD. With TEXIMAGE_PARAM's transform mode 1 ("texcoord
                // source") the texture matrix transforms s,t at command time.
                // The translation terms are added before the >>12, unscaled.
                let raw = [p[0] as i16, (p[0] >> 16) as i16];
                self.cur_tex = if self.teximage >> 30 & 3 == 1 {
                    let m = &self.tex;
                    let (rs, rt) = (raw[0] as i64, raw[1] as i64);
                    let s = (rs * m[0] as i64 + rt * m[4] as i64 + m[8] as i64 + m[12] as i64) >> 12;
                    let t = (rs * m[1] as i64 + rt * m[5] as i64 + m[9] as i64 + m[13] as i64) >> 12;
                    [s as i16, t as i16]
                } else {
                    raw
                };
            }
            0x23 => {
                let x = p[0] as i16 as i32;
                let y = (p[0] >> 16) as i16 as i32;
                let z = p[1] as i16 as i32;
                self.submit_vertex(x, y, z);
            }
            0x24 => {
                // VTX_10: 10-bit 4.6 -> 4.12.
                let x = s10(p[0]) << 6;
                let y = s10(p[0] >> 10) << 6;
                let z = s10(p[0] >> 20) << 6;
                self.submit_vertex(x, y, z);
            }
            0x25 => {
                let (x, y) = (p[0] as i16 as i32, (p[0] >> 16) as i16 as i32);
                self.submit_vertex(x, y, self.last_vtx[2]);
            }
            0x26 => {
                let (x, z) = (p[0] as i16 as i32, (p[0] >> 16) as i16 as i32);
                self.submit_vertex(x, self.last_vtx[1], z);
            }
            0x27 => {
                let (y, z) = (p[0] as i16 as i32, (p[0] >> 16) as i16 as i32);
                self.submit_vertex(self.last_vtx[0], y, z);
            }
            0x28 => {
                // VTX_DIFF: sign-extended 10-bit deltas added raw to 4.12.
                let x = self.last_vtx[0] + s10(p[0]);
                let y = self.last_vtx[1] + s10(p[0] >> 10);
                let z = self.last_vtx[2] + s10(p[0] >> 20);
                self.submit_vertex(x, y, z);
            }
            0x29 => self.poly_attr = p[0],
            0x2A => self.teximage = p[0],
            0x2B => self.pltt_base = p[0] & 0x1FFF,
            0x30 => {
                // DIF_AMB
                for ch in 0..3 {
                    self.dif[ch] = (p[0] >> (ch * 5) & 31) as i32;
                    self.amb[ch] = (p[0] >> (16 + ch * 5) & 31) as i32;
                }
                if p[0] & 0x8000 != 0 {
                    self.cur_color = (p[0] & 0x7FFF) as u16;
                }
            }
            0x31 => {
                for ch in 0..3 {
                    self.spe[ch] = (p[0] >> (ch * 5) & 31) as i32;
                    self.emi[ch] = (p[0] >> (16 + ch * 5) & 31) as i32;
                }
                self.shine_table_en = p[0] & 0x8000 != 0;
            }
            0x32 => {
                // LIGHT_VECTOR: 10-bit 0.9 direction, through the vector matrix.
                let l = (p[0] >> 30) as usize;
                let v = [s10(p[0]) << 3, s10(p[0] >> 10) << 3, s10(p[0] >> 20) << 3];
                let v = self.vec_rotate(v);
                self.light_vec[l] = v;
                self.light_half[l] =
                    [v[0] / 2, v[1] / 2, (v[2] - 0x1000) / 2];
            }
            0x33 => {
                let l = (p[0] >> 30) as usize;
                for ch in 0..3 {
                    self.light_color[l][ch] = (p[0] >> (ch * 5) & 31) as i32;
                }
            }
            0x34 => {
                // SHININESS: 32 words, four 0.8 fixed-point entries each.
                for (i, w) in p.iter().enumerate().take(32) {
                    for b in 0..4 {
                        self.shine[i * 4 + b] = (w >> (b * 8)) as u8;
                    }
                }
            }
            0x40 => {
                self.prim_mode = (p[0] & 3) as u8;
                self.cur_attr = self.poly_attr;
                self.strip_v.clear();
                self.strip_ci.clear();
                self.strip_parity = false;
            }
            0x41 => {} // END_VTXS
            0x50 => self.swap_buffers(p[0]),
            0x60 => {
                self.viewport =
                    [p[0] as u8, (p[0] >> 8) as u8, (p[0] >> 16) as u8, (p[0] >> 24) as u8];
            }
            0x70 => {} // BOX_TEST: result reads back as "inside" via GXSTAT
            0x71 => {
                // POS_TEST: run x,y,z (last two params) through the clip matrix.
                let x = p[0] as i16 as i32;
                let y = (p[0] >> 16) as i16 as i32;
                let z = p[1] as i16 as i32;
                let m = self.clip_matrix();
                for j in 0..4 {
                    let acc = x as i64 * m[j] as i64
                        + y as i64 * m[4 + j] as i64
                        + z as i64 * m[8 + j] as i64
                        + ((m[12 + j] as i64) << 12);
                    self.pos_result[j] = (acc >> 12) as i32;
                }
                self.last_vtx = [x, y, z];
            }
            0x72 => {} // VEC_TEST
            _ => {}
        }
    }

    fn swap_buffers(&mut self, param: u32) {
        self.swap_param = param;
        self.last_poly_count = self.polys.len();
        self.last_vert_count = self.verts.len();
        self.disp_polys = std::mem::take(&mut self.polys);
        self.disp_verts = std::mem::take(&mut self.verts);
        self.ram_overflow = false;
        self.swap_count += 1;
        self.lightlog_left = LIGHTLOG_PER_FRAME;
        if *LIGHTLOG {
            let setups = std::mem::take(&mut self.lightlog_setups);
            eprintln!("[light] frame {}: {} distinct material/light setups", self.swap_count, setups.len());
            for (s, n) in setups.iter().take(8) {
                let mut cols = String::new();
                for l in 0..4 {
                    if s.lights >> l & 1 != 0 {
                        cols += &format!(" light{l}={:?}", s.color[l]);
                    }
                }
                eprintln!(
                    "[light]   {n:5} verts lights={:04b} dif={:?} amb={:?} spe={:?} emi={:?} shinetbl={}{cols}",
                    s.lights, s.dif, s.amb, s.spe, s.emi, s.shine_table_en as u8
                );
            }
        }
        let rejected = std::mem::take(&mut self.rejected);
        if *GXLOG {
            eprintln!(
                "[gx] swap #{}: {} polys ({rejected} rejected), {} verts, viewport ({},{})-({},{}), swap_param={:#X}",
                self.swap_count,
                self.last_poly_count,
                self.last_vert_count,
                self.viewport[0],
                self.viewport[1],
                self.viewport[2],
                self.viewport[3],
                param,
            );
            if self.swap_count <= 3 {
                for poly in self.disp_polys.iter().take(4) {
                    let mut s = String::new();
                    for &i in &poly.verts[..poly.nverts as usize] {
                        let c = self.disp_verts[i as usize].clip;
                        s += &format!(
                            " ({:.2},{:.2},{:.2};w={:.2})",
                            c[0] as f64 / 4096.0,
                            c[1] as f64 / 4096.0,
                            c[2] as f64 / 4096.0,
                            c[3] as f64 / 4096.0
                        );
                    }
                    eprintln!(
                        "[gx]   poly attr={:#010X} tex={:#010X}{}",
                        poly.attr, poly.texparam, s
                    );
                }
            }
        }
    }

    /// CLIPMTX_RESULT (0x04000640-0x67F): read back the current clip matrix,
    /// one 20.12 word per register. Pokemon's billboard code reads this to
    /// build camera-facing matrices on the CPU; returning zeros collapses
    /// every character sprite to a point.
    pub fn clip_word(&mut self, i: usize) -> u32 {
        self.clip_matrix()[i & 15] as u32
    }

    /// VECMTX_RESULT (0x04000680-0x6A3): the 3x3 part of the vector matrix.
    pub fn vec_word(&self, i: usize) -> u32 {
        let i = i % 9;
        self.vec[(i / 3) * 4 + i % 3] as u32
    }

    /// GXSTAT (0x04000600) low half. High half (FIFO + IRQ bits) is composed
    /// in bus.rs so the existing IRQ-mode plumbing stays untouched.
    pub fn gxstat_lo(&self) -> u16 {
        let mut v = 0u16;
        v |= 2; // box test result: report "inside"
        v |= ((self.pos_sp & 0x1F) as u16) << 8;
        v |= ((self.proj_sp & 1) as u16) << 13;
        if self.stack_error {
            v |= 1 << 15;
        }
        v
    }
}
