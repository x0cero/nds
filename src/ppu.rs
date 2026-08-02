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
pub struct Ppu {
    pub fb_a: Vec<u32>,
    pub fb_b: Vec<u32>,
}

impl Ppu {
    pub fn new() -> Self {
        Self { fb_a: vec![0; WIDTH * HEIGHT], fb_b: vec![0; WIDTH * HEIGHT] }
    }

    pub fn render_frame(&mut self, m: &Machine) {
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
    }

    fn render_graphics(m: &Machine, eng: usize, dispcnt: u32, fb: &mut [u32]) {
        let io = &m.io2d[eng];
        let r16 = |off: usize| u16::from_le_bytes([io[off], io[off + 1]]) as u32;
        let pal_base = eng * 0x400;
        let backdrop = u16::from_le_bytes([m.pal[pal_base], m.pal[pal_base + 1]]);
        // BG VRAM window for this engine.
        let bg_vram_base: u32 = if eng == 0 { 0x0600_0000 } else { 0x0620_0000 };
        let vram8 = |off: u32| m.vram_read8(bg_vram_base + off);

        for y in 0..HEIGHT as u32 {
            for x in 0..WIDTH {
                fb[y as usize * WIDTH + x] = rgb555(backdrop);
            }
            // Draw enabled text BGs in priority order (3..0 painted back to front).
            let mut order: Vec<u32> = (0..4).collect();
            // Painter's order: higher priority value first, and for ties the
            // higher BG number first, so the lower one ends up on top.
            order.sort_by_key(|&bg| std::cmp::Reverse((r16(0x8 + bg as usize * 2) & 3, bg)));
            for bg in order {
                if dispcnt & (1 << (8 + bg)) == 0 {
                    continue;
                }
                // BG0 with 3D enabled: nothing to draw yet.
                if eng == 0 && bg == 0 && dispcnt & 8 != 0 {
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
                        let poff = if eight_bpp {
                            pal_base + color as usize * 2
                        } else {
                            pal_base + ((entry >> 12) * 32 + color * 2) as usize
                        };
                        let c = u16::from_le_bytes([m.pal[poff], m.pal[poff + 1]]);
                        fb[y as usize * WIDTH + x as usize] = rgb555(c);
                    }
                }
            }
        }
    }
}
