use crate::bus::{Machine, BANK_SIZE};

pub const WIDTH: usize = 256;
pub const HEIGHT: usize = 192;

fn rgb555(c: u16) -> u32 {
    let r = (c & 0x1F) as u32;
    let g = (c >> 5 & 0x1F) as u32;
    let b = (c >> 10 & 0x1F) as u32;
    (r << 19 | r >> 2 << 16) | (g << 11 | g >> 2 << 8) | (b << 3 | b >> 2)
}

/// Frame renderer for both 2D engines: text backgrounds (3D, affine,
/// extended, and sprites are not implemented yet) and LCDC VRAM display.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Ppu {
    pub fb_a: Vec<u32>,
    pub fb_b: Vec<u32>,
}

impl Ppu {
    pub fn new() -> Self {
        Self { fb_a: vec![0; WIDTH * HEIGHT], fb_b: vec![0; WIDTH * HEIGHT] }
    }

    pub fn render_frame(&mut self, m: &mut Machine) {
        for eng in 0..2 {
            let io = &m.io2d[eng];
            let dispcnt = u32::from_le_bytes([io[0], io[1], io[2], io[3]]);
            let fb = if eng == 0 { &mut self.fb_a } else { &mut self.fb_b };
            match dispcnt >> 16 & 3 {
                0 => fb.fill(0xFFFFFF), // display off: white
                2 => {
                    // LCDC: show the selected 128KB VRAM bank directly.
                    let bank = (dispcnt >> 18 & 3) as usize;
                    for i in 0..WIDTH * HEIGHT {
                        let off = i * 2;
                        let c = if off + 1 < BANK_SIZE[bank] {
                            u16::from_le_bytes([m.vram[bank][off], m.vram[bank][off + 1]])
                        } else {
                            0
                        };
                        fb[i] = rgb555(c);
                    }
                }
                _ => Self::render_graphics(m, eng, dispcnt, fb),
            }
        }
        self.display_capture(m);
        // Master brightness (0x6C, per engine) applies after capture: the
        // capture unit taps the pre-brightness picture.
        for eng in 0..2 {
            let brt = u16::from_le_bytes([m.io2d[eng][0x6C], m.io2d[eng][0x6D]]);
            let factor = (brt & 0x1F).min(16) as u32;
            let mode = brt >> 14 & 3;
            if factor == 0 || mode == 0 || mode == 3 {
                continue;
            }
            let fb = if eng == 0 { &mut self.fb_a } else { &mut self.fb_b };
            for px in fb.iter_mut() {
                let (r, g, b) = (*px >> 16 & 0xFF, *px >> 8 & 0xFF, *px & 0xFF);
                let f = |c: u32| -> u32 {
                    if mode == 1 {
                        c + (255 - c) * factor / 16 // brighten toward white
                    } else {
                        c - c * factor / 16 // darken toward black
                    }
                };
                *px = f(r) << 16 | f(g) << 8 | f(b);
            }
        }
    }

    /// DISPCAPCNT (0x04000064): capture engine A's output (and/or a VRAM
    /// source) into a VRAM bank once per frame while the enable bit is set.
    /// Games re-display the captured bank (LCDC mode or as a BG) for TV
    /// static, motion blur, and screen-transition effects; without this the
    /// displayed bank holds stale garbage.
    fn display_capture(&mut self, m: &mut Machine) {
        let cap = u32::from_le_bytes([
            m.io2d[0][0x64],
            m.io2d[0][0x65],
            m.io2d[0][0x66],
            m.io2d[0][0x67],
        ]);
        if cap >> 31 == 0 {
            return;
        }
        let (w, h) = match cap >> 20 & 3 {
            0 => (128usize, 128usize),
            1 => (256, 64),
            2 => (256, 128),
            _ => (256, 192),
        };
        let wbank = (cap >> 16 & 3) as usize;
        let wofs = (cap >> 18 & 3) as usize * 0x8000;
        let rbank = {
            let dispcnt = u32::from_le_bytes([m.io2d[0][0], m.io2d[0][1], m.io2d[0][2], m.io2d[0][3]]);
            (dispcnt >> 18 & 3) as usize
        };
        let rofs = (cap >> 26 & 3) as usize * 0x8000;
        let eva = (cap & 0x1F).min(16);
        let evb = (cap >> 8 & 0x1F).min(16);
        let mode = cap >> 29 & 3; // 0 = source A, 1 = source B, 2/3 = blend
        // Source A: engine A's composited output (u32 -> RGB15; the u32 was
        // expanded from 5-bit channels, so >>3 recovers them). Bit 24 selects
        // "3D only", which we approximate with the same composited frame.
        let src_a = |fb: &[u32], i: usize| -> u16 {
            let px = fb[i];
            let (r, g, b) = (px >> 19 & 0x1F, px >> 11 & 0x1F, px >> 3 & 0x1F);
            (r | g << 5 | b << 10) as u16
        };
        let src_b = |m: &Machine, i: usize| -> u16 {
            let off = rofs + i * 2;
            if off + 1 < BANK_SIZE[rbank] {
                u16::from_le_bytes([m.vram[rbank][off], m.vram[rbank][off + 1]])
            } else {
                0
            }
        };
        let mut out = vec![0u16; w * h];
        for y in 0..h {
            for x in 0..w {
                let i = y * w + x;
                let si = y * WIDTH + x; // source A is always a 256-wide frame
                let a = src_a(&self.fb_a, si);
                let b = src_b(m, i);
                let c = match mode {
                    0 => a,
                    1 => b,
                    _ => {
                        let ch = |sa: u16, sb: u16| -> u16 {
                            ((sa as u32 * eva + sb as u32 * evb) / 16).min(31) as u16
                        };
                        ch(a & 0x1F, b & 0x1F)
                            | ch(a >> 5 & 0x1F, b >> 5 & 0x1F) << 5
                            | ch(a >> 10 & 0x1F, b >> 10 & 0x1F) << 10
                    }
                };
                out[i] = c | 0x8000; // captured pixels have the alpha bit set
            }
        }
        for (i, c) in out.iter().enumerate() {
            let off = (wofs + i * 2) & (BANK_SIZE[wbank] - 1);
            m.vram[wbank][off] = *c as u8;
            m.vram[wbank][off + 1] = (*c >> 8) as u8;
        }
        // Capture-enable reads back 0 once the capture completes.
        m.io2d[0][0x67] &= 0x7F;
    }

    fn render_graphics(m: &Machine, eng: usize, dispcnt: u32, fb: &mut [u32]) {
        let io = &m.io2d[eng];
        let r16 = |off: usize| u16::from_le_bytes([io[off], io[off + 1]]) as u32;
        let pal_base = eng * 0x400;
        let backdrop = u16::from_le_bytes([m.pal[pal_base], m.pal[pal_base + 1]]);
        // BG VRAM window for this engine.
        let bg_vram_base: u32 = if eng == 0 { 0x0600_0000 } else { 0x0620_0000 };
        let vram8 = |off: u32| m.vram_read8(bg_vram_base + off);

        // Sprite layer: rendered up front into per-pixel buffers, composited
        // after the BGs so priority can be compared per pixel.
        // 3D layer: engine A's BG0 shows the rasterized 3D frame when
        // DISPCNT bit 3 is set. Rendered lazily, once per frame.
        let bg0_3d = if eng == 0 && dispcnt & 8 != 0 && dispcnt & 0x100 != 0 {
            Some(crate::render3d::render(m))
        } else {
            None
        };

        let mut obj_col = vec![0u16; WIDTH * HEIGHT];
        let mut obj_prio = vec![0xFFu8; WIDTH * HEIGHT];
        let mut obj_win = vec![false; WIDTH * HEIGHT];
        let mut obj_semi = vec![false; WIDTH * HEIGHT];
        if dispcnt & 1 << 12 != 0 {
            Self::render_objs(
                m, eng, dispcnt, &mut obj_col, &mut obj_prio, &mut obj_win, &mut obj_semi,
            );
        }

        // Color special effects (BLDCNT/BLDALPHA/BLDY). Layer ids: BG0-3 =
        // 0-3, OBJ = 4, backdrop = 5.
        let bldcnt = r16(0x50);
        let bld_mode = bldcnt >> 6 & 3;
        let eva = (r16(0x52) & 0x1F).min(16);
        let evb = (r16(0x52) >> 8 & 0x1F).min(16);
        let evy = (r16(0x54) & 0x1F).min(16);

        // Windows clip layers to a rectangle: WININ says which layers draw
        // inside WIN0 (low 6 bits) and WIN1 (next 8), WINOUT says which draw
        // outside both (low byte) or inside the OBJ window (high byte).
        // WIN0 takes precedence over WIN1, which beats the OBJ window.
        let bg_extpal_on = dispcnt & 1 << 30 != 0;
        let win0_on = dispcnt & 1 << 13 != 0;
        let win1_on = dispcnt & 1 << 14 != 0;
        let objwin_on = dispcnt & 1 << 15 != 0;
        let windows_on = dispcnt & 0xE000 != 0;
        let winin = r16(0x48);
        let winout = r16(0x4A);
        // Each register packs the low edge in the high byte. An end before the
        // start means the span runs to the edge of the screen.
        let span = |reg: u32, max: u32| {
            let (a, b) = (reg >> 8 & 0xFF, reg & 0xFF);
            (a, if b < a || b > max { max } else { b })
        };
        let (w0x1, w0x2) = span(r16(0x40), WIDTH as u32);
        let (w0y1, w0y2) = span(r16(0x44), HEIGHT as u32);
        let (w1x1, w1x2) = span(r16(0x42), WIDTH as u32);
        let (w1y1, w1y2) = span(r16(0x46), HEIGHT as u32);

        for y in 0..HEIGHT as u32 {
            let w0_row = win0_on && y >= w0y1 && y < w0y2;
            let w1_row = win1_on && y >= w1y1 && y < w1y2;
            // Which layer-enable bits apply at this pixel, given the windows.
            let layers_at = |x: u32| -> u32 {
                if w0_row && x >= w0x1 && x < w0x2 {
                    winin
                } else if w1_row && x >= w1x1 && x < w1x2 {
                    winin >> 8
                } else if objwin_on && obj_win[y as usize * WIDTH + x as usize] {
                    winout >> 8
                } else {
                    winout
                }
            };
            // Priority of the topmost BG pixel drawn at each x (4 = backdrop).
            let mut top_prio = [4u8; WIDTH];
            // Top and second-topmost pixel per x, with their layer ids, for
            // the color special effects.
            let mut top_col = [rgb555(backdrop); WIDTH];
            let mut top_layer = [5u8; WIDTH];
            let mut sec_col = [rgb555(backdrop); WIDTH];
            let mut sec_layer = [5u8; WIDTH];
            // Draw enabled text BGs in priority order (3..0 painted back to front).
            let mut order: Vec<u32> = (0..4).collect();
            // Painter's order: higher priority value first, and for ties the
            // higher BG number first, so the lower one ends up on top.
            order.sort_by_key(|&bg| std::cmp::Reverse((r16(0x8 + bg as usize * 2) & 3, bg)));
            for bg in order {
                if dispcnt & (1 << (8 + bg)) == 0 {
                    continue;
                }
                // BG0 with 3D enabled: composite the rasterized 3D frame at
                // BG0's priority; undrawn 3D pixels stay transparent.
                if eng == 0 && bg == 0 && dispcnt & 8 != 0 {
                    if let Some(buf) = &bg0_3d {
                        let prio = (r16(0x8) & 3) as u8;
                        for x in 0..WIDTH as u32 {
                            if windows_on && layers_at(x) & 1 == 0 {
                                continue;
                            }
                            let i = y as usize * WIDTH + x as usize;
                            if buf[i] != crate::render3d::TRANSPARENT {
                                let x = x as usize;
                                sec_col[x] = top_col[x];
                                sec_layer[x] = top_layer[x];
                                top_col[x] = buf[i];
                                top_layer[x] = 0;
                                top_prio[x] = prio;
                            }
                        }
                    }
                    continue;
                }
                let mode = dispcnt & 7;
                let is_text = match (mode, bg) {
                    (0, _) => true,
                    (1, 0..=2) => true,
                    (3, 0..=2) => true,
                    (4, 0..=1) => true,
                    (5, 0..=1) => true,
                    _ => false,
                };
                if !is_text {
                    continue;
                }
                let bgcnt = r16(0x8 + bg as usize * 2);
                let char_base =
                    ((dispcnt >> 24 & 7) * 0x10000 + (bgcnt >> 2 & 0xF) * 0x4000) as u32;
                let screen_base =
                    ((dispcnt >> 27 & 7) * 0x10000 + (bgcnt >> 8 & 0x1F) * 0x800) as u32;
                let eight_bpp = bgcnt & 0x80 != 0;
                let size = bgcnt >> 14 & 3;
                let hofs = r16(0x10 + bg as usize * 4) & 0x1FF;
                let vofs = r16(0x12 + bg as usize * 4) & 0x1FF;
                let (w_tiles, h_tiles) = match size {
                    0 => (32u32, 32u32),
                    1 => (64, 32),
                    2 => (32, 64),
                    _ => (64, 64),
                };
                let py = (y + vofs) % (h_tiles * 8);
                for x in 0..WIDTH as u32 {
                    if windows_on && layers_at(x) & 1 << bg == 0 {
                        continue;
                    }
                    let px = (x + hofs) % (w_tiles * 8);
                    let sbb = match size {
                        0 => 0,
                        1 => px / 256,
                        2 => py / 256,
                        _ => (px / 256) + (py / 256) * 2,
                    };
                    let tx = (px / 8) % 32;
                    let ty = (py / 8) % 32;
                    let entry_off = screen_base + sbb * 0x800 + (ty * 32 + tx) * 2;
                    let entry =
                        u16::from_le_bytes([vram8(entry_off), vram8(entry_off + 1)]) as u32;
                    let tile = entry & 0x3FF;
                    let mut fx = px % 8;
                    let mut fy = py % 8;
                    if entry & 0x400 != 0 {
                        fx = 7 - fx;
                    }
                    if entry & 0x800 != 0 {
                        fy = 7 - fy;
                    }
                    let color = if eight_bpp {
                        vram8(char_base + tile * 64 + fy * 8 + fx) as u32
                    } else {
                        let b = vram8(char_base + tile * 32 + fy * 4 + fx / 2) as u32;
                        if fx & 1 == 0 { b & 0xF } else { b >> 4 }
                    };
                    if color != 0 {
                        // With extended palettes a 256-color BG picks one of
                        // 16 palettes per tile from its own 8KB VRAM slot;
                        // BG0/BG1 can be pointed at slots 2/3 by BGCNT bit 13.
                        let c = if eight_bpp && bg_extpal_on {
                            let slot = if bg < 2 && bgcnt & 0x2000 != 0 {
                                bg as usize + 2
                            } else {
                                bg as usize
                            };
                            m.bg_extpal(eng, slot, entry >> 12, color)
                        } else {
                            let poff = if eight_bpp {
                                pal_base + color as usize * 2
                            } else {
                                pal_base + ((entry >> 12) * 32 + color * 2) as usize
                            };
                            u16::from_le_bytes([m.pal[poff], m.pal[poff + 1]])
                        };
                        let x = x as usize;
                        sec_col[x] = top_col[x];
                        sec_layer[x] = top_layer[x];
                        top_col[x] = rgb555(c);
                        top_layer[x] = bg as u8;
                        top_prio[x] = (bgcnt & 3) as u8;
                    }
                }
            }
            // Composite sprites: a sprite pixel shows over a BG pixel of the
            // same or lower priority (higher or equal value).
            let mut semi_top = [false; WIDTH];
            for x in 0..WIDTH as u32 {
                let i = y as usize * WIDTH + x as usize;
                if obj_prio[i] == 0xFF {
                    continue;
                }
                if windows_on && layers_at(x) & 0x10 == 0 {
                    continue;
                }
                if obj_prio[i] <= top_prio[x as usize] {
                    let x = x as usize;
                    sec_col[x] = top_col[x];
                    sec_layer[x] = top_layer[x];
                    top_col[x] = rgb555(obj_col[i]);
                    top_layer[x] = 4;
                    semi_top[x] = obj_semi[i];
                }
            }
            // Color special effects, then commit the row.
            for x in 0..WIDTH {
                let (top, sec) = (top_col[x], sec_col[x]);
                let first_ok = bldcnt >> top_layer[x] & 1 != 0;
                let second_ok = bldcnt >> (8 + sec_layer[x]) & 1 != 0;
                let alpha = |eva: u32, evb: u32| {
                    let bl = |sh: u32| {
                        (((top >> sh & 0xFF) * eva + (sec >> sh & 0xFF) * evb) / 16).min(255)
                    };
                    bl(16) << 16 | bl(8) << 8 | bl(0)
                };
                let out = if semi_top[x] && second_ok {
                    // Semi-transparent sprite pixels always alpha-blend.
                    alpha(eva as u32, evb as u32)
                } else {
                    match bld_mode {
                        1 if first_ok && second_ok => alpha(eva as u32, evb as u32),
                        2 if first_ok => {
                            let bl = |sh: u32| {
                                let c = top >> sh & 0xFF;
                                c + (255 - c) * evy as u32 / 16
                            };
                            bl(16) << 16 | bl(8) << 8 | bl(0)
                        }
                        3 if first_ok => {
                            let bl = |sh: u32| {
                                let c = top >> sh & 0xFF;
                                c - c * evy as u32 / 16
                            };
                            bl(16) << 16 | bl(8) << 8 | bl(0)
                        }
                        _ => top,
                    }
                };
                fb[y as usize * WIDTH + x] = out;
            }
        }
    }

    /// Render all OAM sprites for one engine into per-pixel color/priority
    /// buffers (lower OAM index wins) and the OBJ-window mask.
    fn render_objs(
        m: &Machine,
        eng: usize,
        dispcnt: u32,
        obj_col: &mut [u16],
        obj_prio: &mut [u8],
        obj_win: &mut [bool],
        obj_semi: &mut [bool],
    ) {
        let oam_base = eng * 0x400;
        let obj_vram_base: u32 = if eng == 0 { 0x0640_0000 } else { 0x0660_0000 };
        let pal_base = eng * 0x400 + 0x200;
        let one_d = dispcnt & 0x10 != 0;
        let obj_extpal_on = dispcnt & 1 << 31 != 0;
        // In 1D mapping the tile index steps in units of 32<<boundary bytes.
        let boundary = 32u32 << (dispcnt >> 20 & 3);
        // Painted highest index first so lower OAM indices overwrite (win).
        for idx in (0..128).rev() {
            let at = |o: usize| {
                u16::from_le_bytes([m.oam[oam_base + idx * 8 + o], m.oam[oam_base + idx * 8 + o + 1]])
            };
            let (a0, a1, a2) = (at(0), at(2), at(4));
            let rot = a0 & 0x100 != 0;
            if !rot && a0 & 0x200 != 0 {
                continue; // disabled
            }
            let mode = a0 >> 10 & 3;
            if mode == 3 {
                continue; // TODO: bitmap sprites not implemented
            }
            let eight_bpp = a0 & 0x2000 != 0;
            let (w, h): (i32, i32) = match (a0 >> 14 & 3, a1 >> 14 & 3) {
                (0, 0) => (8, 8),
                (0, 1) => (16, 16),
                (0, 2) => (32, 32),
                (0, 3) => (64, 64),
                (1, 0) => (16, 8),
                (1, 1) => (32, 8),
                (1, 2) => (32, 16),
                (1, 3) => (64, 32),
                (2, 0) => (8, 16),
                (2, 1) => (8, 32),
                (2, 2) => (16, 32),
                (2, 3) => (32, 64),
                _ => continue, // prohibited shape
            };
            let double = rot && a0 & 0x200 != 0;
            let (bw, bh) = if double { (w * 2, h * 2) } else { (w, h) };
            let mut top = (a0 & 0xFF) as i32;
            if top + bh > 256 {
                top -= 256; // Y wraps within 0..255
            }
            let mut left = (a1 & 0x1FF) as i32;
            if left >= 256 {
                left -= 512; // X is 9-bit signed
            }
            let (pa, pb, pc, pd) = if rot {
                let grp = oam_base + (a1 >> 9 & 0x1F) as usize * 32;
                let p = |o: usize| {
                    i16::from_le_bytes([m.oam[grp + o], m.oam[grp + o + 1]]) as i32
                };
                (p(6), p(14), p(22), p(30))
            } else {
                (0x100, 0, 0, 0x100)
            };
            let tile = (a2 & 0x3FF) as u32;
            let prio = (a2 >> 10 & 3) as u8;
            let palnum = (a2 >> 12) as u32;
            let tsz: u32 = if eight_bpp { 64 } else { 32 };
            let w_tiles = (w / 8) as u32;
            for sy in 0..bh {
                let y = top + sy;
                if y < 0 || y >= HEIGHT as i32 {
                    continue;
                }
                for sx in 0..bw {
                    let x = left + sx;
                    if x < 0 || x >= WIDTH as i32 {
                        continue;
                    }
                    // Texture coordinates within the w*h sprite.
                    let (tx, ty) = if rot {
                        let dx = sx - bw / 2;
                        let dy = sy - bh / 2;
                        ((w / 2 << 8) + pa * dx + pb * dy >> 8,
                         (h / 2 << 8) + pc * dx + pd * dy >> 8)
                    } else {
                        let mut tx = sx;
                        let mut ty = sy;
                        if a1 & 0x1000 != 0 {
                            tx = w - 1 - tx;
                        }
                        if a1 & 0x2000 != 0 {
                            ty = h - 1 - ty;
                        }
                        (tx, ty)
                    };
                    if tx < 0 || tx >= w || ty < 0 || ty >= h {
                        continue;
                    }
                    let (ctx, cty) = ((tx / 8) as u32, (ty / 8) as u32);
                    let (fx, fy) = ((tx % 8) as u32, (ty % 8) as u32);
                    let tile_addr = if one_d {
                        obj_vram_base + tile * boundary + (cty * w_tiles + ctx) * tsz
                    } else if eight_bpp {
                        // 2D map: 32 tile-index units (32 bytes each) per row.
                        obj_vram_base + (tile & !1) * 32 + cty * 1024 + ctx * 64
                    } else {
                        obj_vram_base + (tile + cty * 32 + ctx) * 32
                    };
                    let color = if eight_bpp {
                        m.vram_read8(tile_addr + fy * 8 + fx) as u32
                    } else {
                        let b = m.vram_read8(tile_addr + fy * 4 + fx / 2) as u32;
                        if fx & 1 == 0 { b & 0xF } else { b >> 4 }
                    };
                    if color == 0 {
                        continue;
                    }
                    let i = y as usize * WIDTH + x as usize;
                    if mode == 2 {
                        obj_win[i] = true;
                        continue;
                    }
                    obj_semi[i] = mode == 1;
                    obj_col[i] = if eight_bpp && obj_extpal_on {
                        m.obj_extpal(eng, palnum, color)
                    } else {
                        let poff = if eight_bpp {
                            pal_base + color as usize * 2
                        } else {
                            pal_base + (palnum * 32 + color * 2) as usize
                        };
                        u16::from_le_bytes([m.pal[poff], m.pal[poff + 1]])
                    };
                    obj_prio[i] = prio;
                }
            }
        }
    }
}
