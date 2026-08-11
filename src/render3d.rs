//! Phase-2 software rasterizer for the 3D geometry latched by SWAP_BUFFERS.
//!
//! Runs once per displayed frame (from ppu.rs, when engine A's BG0 is in 3D
//! mode) over Gpu3d::disp_polys/disp_verts. Floating point throughout:
//! Sutherland-Hodgman clipping in clip space, perspective divide, viewport
//! transform, half-space triangle fill with perspective-correct (1/w)
//! interpolation of color and texture coordinates, depth test per
//! POLYGON_ATTR, W- or Z-buffering per the latched SWAP_BUFFERS parameter.

use crate::bus::Machine;

static POLYSTATS: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_POLYSTATS").is_ok());
use crate::ppu::{HEIGHT, WIDTH};

/// NDS_PIXDBG="x,y": log every rasterizer event affecting that pixel.
static PIXDBG: std::sync::LazyLock<Option<(usize, usize)>> = std::sync::LazyLock::new(|| {
    let v = std::env::var("NDS_PIXDBG").ok()?;
    let (x, y) = v.split_once(',')?;
    Some((x.trim().parse().ok()?, y.trim().parse().ok()?))
});

/// Sentinel for "nothing drawn here": the 2D compositor treats it as
/// transparent so lower layers show through.
pub const TRANSPARENT: u32 = u32::MAX;

/// Sub-texel bias applied before picking a texel (NDS_TEXBIAS overrides it).
/// Too small and coordinates that land exactly on a texel edge flip between
/// neighbours as the camera moves; too large and a sample at the edge of a
/// sub-image inside a texture atlas spills into the transparent gutter beside
/// it, punching a one-pixel hole in the ground.
static TEXBIAS: std::sync::LazyLock<f64> = std::sync::LazyLock::new(|| {
    std::env::var("NDS_TEXBIAS").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0 / 1024.0)
});

#[derive(Clone, Copy, Default)]
struct V {
    // Clip space (raw 20.12 units as floats; the scale cancels out).
    x: f64,
    y: f64,
    z: f64,
    w: f64,
    r: f64,
    g: f64,
    b: f64,
    s: f64, // texcoords in texels (input is 12.4 fixed)
    t: f64,
    // Screen space, filled after projection.
    sx: f64,
    sy: f64,
    d: f64,  // depth value
    rw: f64, // 1/w
}

fn lerp(a: &V, b: &V, f: f64) -> V {
    let l = |p: f64, q: f64| p + (q - p) * f;
    V {
        x: l(a.x, b.x),
        y: l(a.y, b.y),
        z: l(a.z, b.z),
        w: l(a.w, b.w),
        r: l(a.r, b.r),
        g: l(a.g, b.g),
        b: l(a.b, b.b),
        s: l(a.s, b.s),
        t: l(a.t, b.t),
        ..Default::default()
    }
}

/// Clip a polygon against one plane given by dist(v) >= 0.
fn clip_plane(poly: &[V], dist: impl Fn(&V) -> f64) -> Vec<V> {
    let mut out = Vec::with_capacity(poly.len() + 1);
    for i in 0..poly.len() {
        let a = &poly[i];
        let b = &poly[(i + 1) % poly.len()];
        let da = dist(a);
        let db = dist(b);
        if da >= 0.0 {
            out.push(*a);
        }
        if (da >= 0.0) != (db >= 0.0) {
            out.push(lerp(a, b, da / (da - db)));
        }
    }
    out
}

/// Sample a texture; None = transparent texel, Some((rgb15, alpha 0-31)).
fn sample_tex(m: &Machine, tp: u32, pltt: u32, s: f64, t: f64) -> Option<(u16, u32)> {
    let format = tp >> 26 & 7;
    let tw = 8i32 << (tp >> 20 & 7);
    let th = 8i32 << (tp >> 23 & 7);
    let wrap = |mut c: i32, size: i32, repeat: bool, flip: bool| -> i32 {
        if repeat {
            if flip {
                let two = size * 2;
                c = c.rem_euclid(two);
                if c >= size {
                    c = two - 1 - c;
                }
                c
            } else {
                c.rem_euclid(size)
            }
        } else {
            c.clamp(0, size - 1)
        }
    };
    // Bias off exact texel boundaries: with this game's pixel-aligned ortho
    // camera the interpolated coordinates land exactly on texel edges, and
    // sub-ulp float error would otherwise flip texels frame to frame
    // (visible as rippling in thin transparent gaps while the camera moves).
    let bias = *TEXBIAS;
    let x = wrap((s + bias).floor() as i32, tw, tp & 1 << 16 != 0, tp & 1 << 18 != 0);
    let y = wrap((t + bias).floor() as i32, th, tp & 1 << 17 != 0, tp & 1 << 19 != 0);
    let base = (tp & 0xFFFF) << 3;
    let i = (y * tw + x) as u32;
    let color0_clear = tp & 1 << 29 != 0;
    match format {
        2 => {
            // 4-color paletted, 4 texels per byte; palette base in 8-byte steps.
            let b = m.tex_read8(base + i / 4);
            let idx = (b >> ((i % 4) * 2) & 3) as u32;
            if idx == 0 && color0_clear {
                return None;
            }
            Some((m.texpal_read16((pltt << 3) + idx * 2), 31))
        }
        3 => {
            let b = m.tex_read8(base + i / 2);
            let idx = (if i & 1 == 0 { b & 0xF } else { b >> 4 }) as u32;
            if idx == 0 && color0_clear {
                return None;
            }
            Some((m.texpal_read16((pltt << 4) + idx * 2), 31))
        }
        4 => {
            let idx = m.tex_read8(base + i) as u32;
            if idx == 0 && color0_clear {
                return None;
            }
            Some((m.texpal_read16((pltt << 4) + idx * 2), 31))
        }
        1 => {
            // A3I5: 3-bit alpha expanded to 5 bits.
            let b = m.tex_read8(base + i);
            let a3 = (b >> 5) as u32;
            if a3 == 0 {
                return None;
            }
            Some((m.texpal_read16((pltt << 4) + (b as u32 & 0x1F) * 2), a3 << 2 | a3 >> 1))
        }
        6 => {
            let b = m.tex_read8(base + i);
            if b >> 3 == 0 {
                return None;
            }
            Some((m.texpal_read16((pltt << 4) + (b as u32 & 7) * 2), (b >> 3) as u32))
        }
        7 => {
            let lo = m.tex_read8(base + i * 2) as u16;
            let hi = m.tex_read8(base + i * 2 + 1) as u16;
            let c = lo | hi << 8;
            if c & 0x8000 == 0 {
                return None;
            }
            Some((c & 0x7FFF, 31))
        }
        5 => {
            // 4x4-texel compressed. Two parallel streams: 2 bits per texel in
            // 4-byte blocks here, and one 16-bit entry per block in the NEXT
            // texture slot (slot 1 for a slot-0 texture, slot 3 for slot 2)
            // giving the block's 4-colour palette offset plus a blend mode for
            // texel values 2 and 3. This is the DS terrain workhorse: without
            // it every ground/path polygon draws flat.
            let bw = tw / 4;
            let blk = ((y / 4) * bw + x / 4) as u32;
            let row = m.tex_read8(base + blk * 4 + (y % 4) as u32);
            let v = (row >> ((x % 4) * 2)) as u32 & 3;
            let idx_addr =
                (base & 0x4_0000) + 0x2_0000 + ((base & 0x1_FFFF) >> 1) + blk * 2;
            let info =
                m.tex_read8(idx_addr) as u32 | (m.tex_read8(idx_addr + 1) as u32) << 8;
            let pal = (pltt << 4) + (info & 0x3FFF) * 4;
            let c = |n: u32| m.texpal_read16(pal + n * 2);
            // Weighted per-channel blend of two RGB555 colours.
            let mix = |a: u16, b: u16, wa: u32, wb: u32, d: u32| -> u16 {
                let ch = |sh: u32| {
                    ((a as u32 >> sh & 0x1F) * wa + (b as u32 >> sh & 0x1F) * wb) / d & 0x1F
                };
                (ch(0) | ch(5) << 5 | ch(10) << 10) as u16
            };
            match (info >> 14, v) {
                (_, 0) => Some((c(0), 31)),
                (_, 1) => Some((c(1), 31)),
                (0 | 2, 2) => Some((c(2), 31)),
                (1, 2) => Some((mix(c(0), c(1), 1, 1, 2), 31)),
                (3, 2) => Some((mix(c(0), c(1), 5, 3, 8), 31)),
                (2, 3) => Some((c(3), 31)),
                (3, 3) => Some((mix(c(0), c(1), 3, 5, 8), 31)),
                _ => None, // modes 0/1 value 3 = transparent
            }
        }
        _ => None,
    }
}

fn rgb555_to_rgb(c: u16) -> u32 {
    let r = (c & 0x1F) as u32;
    let g = (c >> 5 & 0x1F) as u32;
    let b = (c >> 10 & 0x1F) as u32;
    (r << 19 | r >> 2 << 16) | (g << 11 | g >> 2 << 8) | (b << 3 | b >> 2)
}

/// Render the latched 3D frame into a WIDTH*HEIGHT buffer of 0xRRGGBB
/// pixels, TRANSPARENT where nothing was drawn (clear alpha 0).
pub fn render(m: &Machine) -> Vec<u32> {
    let gx = &m.gx;
    let wbuffer = gx.swap_param & 2 != 0;
    let clear_rgb = (gx.clear_color & 0x7FFF) as u16;
    let clear_alpha = gx.clear_color >> 16 & 0x1F;
    let clear_px =
        if clear_alpha == 0 { TRANSPARENT } else { rgb555_to_rgb(clear_rgb) };
    let mut fb = vec![clear_px; WIDTH * HEIGHT];
    let clear_d = if wbuffer {
        f64::MAX
    } else {
        gx.clear_depth as f64 / 0x7FFF as f64
    };
    let mut depth = vec![clear_d; WIDTH * HEIGHT];
    // Shadow polygons (POLYGON_ATTR mode 3) need two more per-pixel buffers:
    // the id of the polygon that owns each pixel, and the stencil marking
    // where the current shadow volume falls. See raster_poly.
    let mut poly_id = vec![0u8; WIDTH * HEIGHT];
    let mut stencil = vec![false; WIDTH * HEIGHT];

    // Viewport (y flipped: clip-space +Y is up, screen +Y is down).
    let vx0 = gx.viewport[0] as f64;
    let vy0 = gx.viewport[1] as f64;
    let vw = (gx.viewport[2] as f64 - vx0 + 1.0).max(1.0);
    let vh = (gx.viewport[3] as f64 - vy0 + 1.0).max(1.0);

    // Prepare (clip, project, cull) every polygon once; rasterize in two
    // passes: fully opaque pixels first, then translucent pixels back to
    // front so alpha blending composites correctly.
    let mut prepared: Vec<(usize, Vec<V>, f64)> = Vec::new();
    for (pi, poly) in gx.disp_polys.iter().enumerate() {
        let mut pv: Vec<V> = poly.verts[..poly.nverts as usize]
            .iter()
            .map(|&i| {
                let v = &gx.disp_verts[i as usize];
                V {
                    x: v.clip[0] as f64,
                    y: v.clip[1] as f64,
                    z: v.clip[2] as f64,
                    w: v.clip[3] as f64,
                    r: (v.color & 0x1F) as f64,
                    g: (v.color >> 5 & 0x1F) as f64,
                    b: (v.color >> 10 & 0x1F) as f64,
                    s: v.tex[0] as f64 / 16.0,
                    t: v.tex[1] as f64 / 16.0,
                    ..Default::default()
                }
            })
            .collect();

        // Frustum clipping (w first so the divides below are safe).
        pv = clip_plane(&pv, |v| v.w - 1e-4);
        for f in [
            |v: &V| v.w - v.x,
            |v: &V| v.w + v.x,
            |v: &V| v.w - v.y,
            |v: &V| v.w + v.y,
            |v: &V| v.w - v.z,
            |v: &V| v.w + v.z,
        ] {
            if pv.len() < 3 {
                break;
            }
            pv = clip_plane(&pv, f);
        }
        if pv.len() < 3 {
            continue;
        }

        // Project.
        for v in &mut pv {
            v.rw = 1.0 / v.w;
            // Hardware computes screen coordinates by integer division, so
            // every vertex lands on a whole pixel and a polygon's texture is
            // re-fitted to that integer outline. Keeping sub-pixel positions
            // here lets the interior texture phase drift against the snapped
            // silhouette, which shows up as buildings wobbling while walking.
            v.sx = (vx0 + (v.x * v.rw * 0.5 + 0.5) * vw).floor();
            v.sy = ((HEIGHT as f64) - (vy0 + (v.y * v.rw * 0.5 + 0.5) * vh)).floor();
            v.d = if wbuffer { v.w } else { v.z * v.rw * 0.5 + 0.5 };
        }

        // Face culling: signed area in screen space (y down).
        let mut area2 = 0.0f64;
        for i in 0..pv.len() {
            let a = &pv[i];
            let b = &pv[(i + 1) % pv.len()];
            area2 += a.sx * b.sy - b.sx * a.sy;
        }
        let front = area2 < 0.0;
        let show_back = poly.attr & 1 << 6 != 0;
        let show_front = poly.attr & 1 << 7 != 0;
        if (front && !show_front) || (!front && !show_back) {
            continue;
        }
        let avg_d = pv.iter().map(|v| v.d).sum::<f64>() / pv.len() as f64;
        prepared.push((pi, pv, avg_d));
    }

    // NDS_POLYSTATS=1: one line per frame summarising the polygon list by
    // mode, texture format and alpha. Answers "what is actually drawing
    // this?" without hunting for a pixel to trace.
    if *POLYSTATS {
        let mut modes = [0usize; 4];
        let mut fmts = [0usize; 8];
        let mut translucent = 0usize;
        for &(pi, _, _) in prepared.iter() {
            let p = &gx.disp_polys[pi];
            modes[(p.attr >> 4 & 3) as usize] += 1;
            fmts[(p.texparam >> 26 & 7) as usize] += 1;
            let a = p.attr >> 16 & 0x1F;
            if a != 0 && a < 31 {
                translucent += 1;
            }
        }
        eprintln!(
            "[poly] drawn={} mode(mod/decal/toon/shadow)={:?} fmt0-7={:?} translucent={}",
            prepared.len(),
            modes,
            fmts,
            translucent
        );
    }

    // NDS_TEXDBG=<hex TEXIMAGE_PARAM>: dump that texture's palette and its
    // index grid once, to tell "we decoded it wrong" from "the game really
    // drew that".
    if let Ok(want) = std::env::var("NDS_TEXDBG") {
        let want = u32::from_str_radix(want.trim_start_matches("0x"), 16).unwrap_or(0);
        for &(pi, _, _) in prepared.iter() {
            let p = &gx.disp_polys[pi];
            if p.texparam != want {
                continue;
            }
            let (fmt, tw, th) = (
                p.texparam >> 26 & 7,
                8usize << (p.texparam >> 20 & 7),
                8usize << (p.texparam >> 23 & 7),
            );
            let shift = if fmt == 2 { 3 } else { 4 };
            eprintln!(
                "[tex] param={:#010X} pltt={:#X} fmt={fmt} {tw}x{th} base={:#X} pal@{:#X} color0clear={}",
                p.texparam,
                p.pltt,
                (p.texparam & 0xFFFF) << 3,
                p.pltt << shift,
                p.texparam & 1 << 29 != 0
            );
            let vc: Vec<String> = prepared
                .iter()
                .find(|(q, _, _)| *q == pi)
                .map(|(_, pv, _)| {
                    pv.iter()
                        .map(|v| format!("({:.0},{:.0},{:.0})", v.r, v.g, v.b))
                        .collect()
                })
                .unwrap_or_default();
            eprintln!("[tex] vertex colours (0-31): {}", vc.join(" "));
            let pal: Vec<String> = (0..16)
                .map(|c| format!("{:04X}", m.texpal_read16((p.pltt << shift) + c * 2)))
                .collect();
            eprintln!("[tex] palette: {}", pal.join(" "));
            let base = (p.texparam & 0xFFFF) << 3;
            for y in 0..th.min(48) {
                let row: String = (0..tw.min(48))
                    .map(|x| {
                        let i = (y * tw + x) as u32;
                        let idx = match fmt {
                            3 => {
                                let b = m.tex_read8(base + i / 2);
                                if i & 1 == 0 { b & 0xF } else { b >> 4 }
                            }
                            4 => m.tex_read8(base + i),
                            _ => 0,
                        };
                        std::char::from_digit((idx & 0xF) as u32, 16).unwrap()
                    })
                    .collect();
                eprintln!("[tex] {row}");
            }
            break;
        }
    }

    // Pass 0: opaque pixels in submission order. Pass 1: translucent pixels,
    // farthest polygons first.
    let mut order: Vec<usize> = (0..prepared.len()).collect();
    for pass in 0..2 {
        if pass == 1 {
            order.sort_by(|&a, &b| {
                prepared[b].2.partial_cmp(&prepared[a].2).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        for &pidx in &order {
            let (pi, pv, _) = &prepared[pidx];
            let poly = &gx.disp_polys[*pi];
            let poly_alpha = {
                let a = poly.attr >> 16 & 0x1F;
                if a == 0 { 31 } else { a } // wireframe: drawn opaque
            };
            let format = poly.texparam >> 26 & 7;
            let shadow = poly.attr >> 4 & 3 == 3;
            let can_translucent = poly_alpha < 31 || format == 1 || format == 6;
            // Shadow volumes run entirely in the second pass so a mask and
            // the shadow it enables stay adjacent: the sort below is stable,
            // and the pair shares its geometry, so submission order holds.
            if pass == 0 && shadow {
                continue;
            }
            if pass == 1 && !can_translucent && !shadow {
                continue;
            }
            // A new mask (id 0) begins a new volume; the previous volume's
            // stencil must not leak into it.
            if shadow && poly.attr >> 24 & 0x3F == 0 {
                stencil.fill(false);
            }
            raster_poly(
                m, poly, pv, poly_alpha, pass, wbuffer, &mut fb, &mut depth,
                &mut poly_id, &mut stencil,
            );
        }
    }
    // Close single-pixel pinholes. Adjacent map tiles are separate polygons
    // whose shared edge lands on a sub-pixel boundary; hardware fills spans in
    // fixed point and rounds outward, so the two always meet, while an exact
    // float rasterizer can leave a gap thinner than a pixel that still happens
    // to contain a pixel centre. The result is the 2D backdrop showing through
    // the ground as a black speck that moves as the camera does. A crack is
    // one pixel wide across at least one axis: both opposite neighbours on
    // that axis were drawn. Filling per axis (instead of demanding all four
    // neighbours) also closes 1px-wide seams that run for several pixels,
    // which is what a tile edge between two ground/tree polygons actually
    // looks like; anything wider than one pixel on both axes is untouched.
    for y in 1..HEIGHT - 1 {
        for x in 1..WIDTH - 1 {
            let i = y * WIDTH + x;
            if fb[i] != TRANSPARENT {
                continue;
            }
            let pair = if fb[i - 1] != TRANSPARENT && fb[i + 1] != TRANSPARENT {
                [i - 1, i + 1]
            } else if fb[i - WIDTH] != TRANSPARENT && fb[i + WIDTH] != TRANSPARENT {
                [i - WIDTH, i + WIDTH]
            } else {
                continue;
            };
            let avg = |sh: u32| pair.iter().map(|&j| fb[j] >> sh & 0xFF).sum::<u32>() / 2;
            fb[i] = avg(16) << 16 | avg(8) << 8 | avg(0);
        }
    }

    // NDS_DUMP3D=path: write the 3D layer on its own (magenta where it is
    // transparent), which separates "the 3D engine drew it wrong" from "2D
    // compositing covered it".
    if let Ok(p) = std::env::var("NDS_DUMP3D") {
        let mut out = format!("P6\n{WIDTH} {HEIGHT}\n255\n").into_bytes();
        for px in fb.iter() {
            let (r, g, b) = if *px == TRANSPARENT {
                (255, 0, 255)
            } else {
                ((px >> 16) as u8, (px >> 8) as u8, *px as u8)
            };
            out.extend_from_slice(&[r, g, b]);
        }
        let _ = std::fs::write(p, out);
    }
    fb
}

#[allow(clippy::too_many_arguments)]
fn raster_poly(
    m: &Machine,
    poly: &crate::gpu3d::Polygon,
    pv: &[V],
    poly_alpha: u32,
    pass: usize,
    wbuffer: bool,
    fb: &mut [u32],
    depth: &mut [f64],
    poly_id: &mut [u8],
    stencil: &mut [bool],
) {
    {
        let depth_equal = poly.attr & 1 << 14 != 0;
        let depth_write_translucent = poly.attr & 1 << 11 != 0;
        // Shadow polygons come in pairs. The mask (id 0) draws nothing; it
        // marks the pixels where it is HIDDEN by real geometry, which is
        // exactly where the caster's shadow lands. The shadow itself (id
        // non-zero) then draws only on marked pixels whose owner has a
        // different id, so a caster never shadows itself. Ignoring the mode
        // paints the invisible mask as ordinary geometry, which is how tree
        // and building shadows turned into opaque slabs over the ground.
        let pid = (poly.attr >> 24 & 0x3F) as u8;
        let shadow_mask = poly.attr >> 4 & 3 == 3 && pid == 0;
        let shadow_draw = poly.attr >> 4 & 3 == 3 && pid != 0;
        let eps = if wbuffer { 0x200 as f64 } else { 1.0 / 4096.0 };
        let textured = poly.texparam >> 26 & 7 != 0;

        // Triangle fan over the clipped polygon. Rasterized with integer
        // 28.4 fixed-point edge functions and the top-left fill rule:
        // adjacent triangles sharing an edge then cover every edge pixel
        // exactly once, so a moving camera cannot open flickering seams
        // (the "ripple while walking" artifact of the old float rasterizer).
        for k in 1..pv.len() - 1 {
            let mut tri = [&pv[0], &pv[k], &pv[k + 1]];
            // Snap screen coordinates to the 1/16-pixel grid; shared
            // vertices snap identically in every polygon that uses them.
            // Snap to the 1/16-pixel grid. The epsilon matters: this game's
            // camera puts shared tile edges exactly on a half-step, where two
            // tiles whose vertices differ only in the last floating-point bit
            // would round in OPPOSITE directions and leave a sliver of bare
            // backdrop between them. Nudging both sides the same way makes a
            // shared vertex land on the same grid point from either polygon.
            let snap = |v: &V| ((v.sx * 16.0).round() as i64, (v.sy * 16.0).round() as i64);
            let orient = |a: (i64, i64), b: (i64, i64), c: (i64, i64)| {
                (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
            };
            let mut p0 = snap(tri[0]);
            let mut p1 = snap(tri[1]);
            let mut p2 = snap(tri[2]);
            let mut area = orient(p0, p1, p2);
            if area == 0 {
                continue;
            }
            if area < 0 {
                tri.swap(1, 2);
                std::mem::swap(&mut p1, &mut p2);
                area = -area;
            }
            // Top-left edges (y-down): a strictly "up" edge, or an exactly
            // horizontal edge running left.
            let top_left = |a: (i64, i64), b: (i64, i64)| b.1 < a.1 || (b.1 == a.1 && b.0 < a.0);
            let bias = |tl: bool| if tl { 0i64 } else { -1 };
            let b0 = bias(top_left(p1, p2));
            let b1 = bias(top_left(p2, p0));
            let b2 = bias(top_left(p0, p1));
            let xmin = p0.0.min(p1.0).min(p2.0);
            let xmax = p0.0.max(p1.0).max(p2.0);
            let ymin = p0.1.min(p1.1).min(p2.1);
            let ymax = p0.1.max(p1.1).max(p2.1);
            // Pixel centers sit at 16*p + 8 on the snapped grid.
            let px0 = (((xmin - 8) + 15).div_euclid(16)).max(0) as usize;
            let px1 = (((xmax - 8).div_euclid(16)) + 1).clamp(0, WIDTH as i64) as usize;
            let py0 = (((ymin - 8) + 15).div_euclid(16)).max(0) as usize;
            let py1 = (((ymax - 8).div_euclid(16)) + 1).clamp(0, HEIGHT as i64) as usize;
            let inv = 1.0 / area as f64;
            for py in py0..py1 {
                let cy = py as i64 * 16 + 8;
                for px in px0..px1 {
                    let c = (px as i64 * 16 + 8, cy);
                    let w0 = orient(p1, p2, c);
                    let w1 = orient(p2, p0, c);
                    let w2 = orient(p0, p1, c);
                    if w0 + b0 < 0 || w1 + b1 < 0 || w2 + b2 < 0 {
                        continue;
                    }
                    let l0 = w0 as f64 * inv;
                    let l1 = w1 as f64 * inv;
                    let l2 = w2 as f64 * inv;
                    let i = py * WIDTH + px;
                    let dbg = *PIXDBG == Some((px, py));
                    let d = l0 * tri[0].d + l1 * tri[1].d + l2 * tri[2].d;
                    let dpass = if depth_equal {
                        d <= depth[i] + eps
                    } else {
                        d < depth[i]
                    };
                    if dbg {
                        eprintln!(
                            "[pix] pass={pass} attr={:#010X} tex={:#010X} pltt={:#X} d={d:.5} zbuf={:.5} dpass={dpass}",
                            poly.attr, poly.texparam, poly.pltt, depth[i]
                        );
                    }
                    if shadow_mask {
                        // Failing the depth test is the interesting case: it
                        // means real geometry stands between this pixel and
                        // the volume, so the pixel is in shadow.
                        if !dpass {
                            stencil[i] = true;
                        }
                        continue; // never writes colour or depth
                    }
                    if !dpass {
                        continue;
                    }
                    if shadow_draw && (!stencil[i] || poly_id[i] == pid) {
                        continue;
                    }
                    // Perspective-correct interpolation via 1/w.
                    let rw = l0 * tri[0].rw + l1 * tri[1].rw + l2 * tri[2].rw;
                    let pc = |f: fn(&V) -> f64| {
                        (l0 * f(tri[0]) * tri[0].rw
                            + l1 * f(tri[1]) * tri[1].rw
                            + l2 * f(tri[2]) * tri[2].rw)
                            / rw
                    };
                    let (vr, vg, vb) = (pc(|v| v.r), pc(|v| v.g), pc(|v| v.b));
                    let (rgb, tex_alpha) = if textured {
                        let s = pc(|v| v.s);
                        let t = pc(|v| v.t);
                        match sample_tex(m, poly.texparam, poly.pltt, s, t) {
                            None => {
                                if dbg {
                                    eprintln!("[pix]   texel transparent at s={s:.2} t={t:.2}");
                                }
                                continue; // transparent texel: no depth write
                            }
                            Some((tc, ta)) => {
                                if dbg {
                                    eprintln!("[pix]   texel s={s:.3} t={t:.3} -> rgb15={tc:#06X} a={ta}");
                                }
                                // Modulation blend, GBATEK "DS 3D Texture
                                // Blending": the hardware works in 6-bit
                                // (a 5-bit value X becomes X*2+1, and 0 stays
                                // 0) and computes ((T+1)*(V+1)-1)/64. Halving
                                // that back to 5 bits is up to one level
                                // brighter than a plain T*(V+1)/32.
                                let to6 = |x: f64| if x > 0.0 { x * 2.0 + 1.0 } else { 0.0 };
                                let mix = |t: u32, v: f64| {
                                    let c6 = ((to6(t as f64) + 1.0) * (to6(v.clamp(0.0, 31.0)) + 1.0)
                                        - 1.0)
                                        / 64.0;
                                    ((c6 as u32) >> 1).min(31)
                                };
                                let tr = (tc & 0x1F) as u32;
                                let tg = (tc >> 5 & 0x1F) as u32;
                                let tb = (tc >> 10 & 0x1F) as u32;
                                (
                                    mix(tr, vr) as u16
                                        | (mix(tg, vg) as u16) << 5
                                        | (mix(tb, vb) as u16) << 10,
                                    ta,
                                )
                            }
                        }
                    } else {
                        (
                            (vr.clamp(0.0, 31.0) as u16)
                                | ((vg.clamp(0.0, 31.0) as u16) << 5)
                                | ((vb.clamp(0.0, 31.0) as u16) << 10),
                            31,
                        )
                    };
                    let alpha = poly_alpha * tex_alpha / 31;
                    if dbg {
                        eprintln!("[pix]   rgb={rgb:#06X} alpha={alpha}");
                    }
                    if alpha == 0 {
                        continue;
                    }
                    if pass == 0 {
                        if alpha < 31 {
                            continue; // translucent pixel: second pass
                        }
                        depth[i] = d;
                        fb[i] = rgb555_to_rgb(rgb);
                        poly_id[i] = pid;
                    } else {
                        if alpha >= 31 && !shadow_draw {
                            continue; // already drawn in pass 0
                        }
                        let src = rgb555_to_rgb(rgb);
                        let dst = fb[i];
                        fb[i] = if dst == TRANSPARENT {
                            src // nothing behind: draw unblended
                        } else {
                            let bl = |sh: u32| {
                                let s = src >> sh & 0xFF;
                                let dv = dst >> sh & 0xFF;
                                (s * alpha + dv * (31 - alpha)) / 31
                            };
                            bl(16) << 16 | bl(8) << 8 | bl(0)
                        };
                        if depth_write_translucent {
                            depth[i] = d;
                        }
                        if shadow_draw {
                            stencil[i] = false; // one shadow per marked pixel
                        } else {
                            poly_id[i] = pid;
                        }
                    }
                }
            }
        }
    }
}
