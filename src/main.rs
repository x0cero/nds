mod bus;
mod cpu;
mod gpu3d;
mod key1;
mod ppu;
mod render3d;
mod state;

use bus::{Bus, Machine, View, IRQ_VBLANK};
use cpu::Cpu;
use ppu::Ppu;
use std::cell::RefCell;
use std::process::ExitCode;
use std::rc::Rc;

/// Which engine drives which physical screen. POWCNT1 bit 15 is the display
/// swap: set sends engine A to the upper screen, clear sends it to the lower
/// one. Games flip it freely (Platinum's name entry does), so the compositor
/// has to honour it or the two screens come out transposed.
fn screens(ppu: &Ppu, powcnt1: u32) -> [&Vec<u32>; 2] {
    if powcnt1 & 0x8000 != 0 {
        [&ppu.fb_a, &ppu.fb_b]
    } else {
        [&ppu.fb_b, &ppu.fb_a]
    }
}

/// Stack both screens, upper first, into one PPM.
fn dump_frame(ppu: &Ppu, powcnt1: u32, path: &str) {
    let mut out = format!("P6\n{} {}\n255\n", ppu::WIDTH, ppu::HEIGHT * 2).into_bytes();
    for fb in screens(ppu, powcnt1) {
        for px in fb.iter() {
            out.push((px >> 16) as u8);
            out.push((px >> 8) as u8);
            out.push(*px as u8);
        }
    }
    std::fs::write(path, out).unwrap();
}

/// Scripted input: NDS_INPUT="120-130:a,200-205:start,..."
struct InputScript {
    spans: Vec<(u32, u32, u16)>,
}

impl InputScript {
    fn parse(s: &str) -> Self {
        let mut spans = Vec::new();
        for part in s.split(',') {
            let Some((range, key)) = part.split_once(':') else { continue };
            let Some((a, b)) = range.split_once('-') else { continue };
            let bit = match key {
                "a" => 0,
                "b" => 1,
                "select" => 2,
                "start" => 3,
                "right" => 4,
                "left" => 5,
                "up" => 6,
                "down" => 7,
                "r" => 8,
                "l" => 9,
                _ => continue,
            };
            if let (Ok(a), Ok(b)) = (a.parse(), b.parse()) {
                spans.push((a, b, 1 << bit));
            }
        }
        Self { spans }
    }

    fn keys_at(&self, frame: u32) -> u16 {
        let mut pressed = 0u16;
        for &(a, b, mask) in &self.spans {
            if frame >= a && frame <= b {
                pressed |= mask;
            }
        }
        !pressed & 0x3FF
    }
}

/// Scripted stylus taps: NDS_TOUCH="360-400:128,100;500-540:64,32".
/// Spans are semicolon-separated; coordinates are lower-screen pixels. The
/// ARM7 only samples every other frame, so a span shorter than about four
/// frames can be missed entirely.
fn parse_touch_script() -> Vec<(u32, u32, u32, u32)> {
    let Ok(spec) = std::env::var("NDS_TOUCH") else { return Vec::new() };
    let mut spans = Vec::new();
    for part in spec.split(';') {
        let Some((range, xy)) = part.split_once(':') else { continue };
        let Some((a, b)) = range.split_once('-') else { continue };
        let Some((x, y)) = xy.split_once(',') else { continue };
        if let (Ok(a), Ok(b), Ok(x), Ok(y)) = (a.parse(), b.parse(), x.parse(), y.parse()) {
            spans.push((a, b, x, y));
        }
    }
    spans
}

fn main() -> ExitCode {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: nds <rom.nds>");
        return ExitCode::FAILURE;
    };
    let mut rom = std::fs::read(&path).expect("read rom");
    // Clean dumps keep the secure area (2KB at 0x4000) KEY1-encrypted;
    // hardware decrypts it during boot, so direct boot must too.
    if rom.len() > 0x4800 {
        let gamecode = u32::from_le_bytes(rom[0x0C..0x10].try_into().unwrap());
        if key1::decrypt_secure_area(gamecode, &mut rom[0x4000..0x4800]) {
            eprintln!("secure area: KEY1-decrypted (encryObj ok)");
        }
    }
    let rom = rom;
    let r32 = |off: usize| u32::from_le_bytes(rom[off..off + 4].try_into().unwrap());
    let arm9_off = r32(0x20) as usize;
    let arm9_entry = r32(0x24);
    let arm9_addr = r32(0x28);
    let arm9_size = r32(0x2C) as usize;
    let arm7_off = r32(0x30) as usize;
    let arm7_entry = r32(0x34);
    let arm7_addr = r32(0x38);
    let arm7_size = r32(0x3C) as usize;
    eprintln!(
        "arm9: {:#X}+{:#X} -> {:#010X} entry {:#010X} | arm7: {:#X}+{:#X} -> {:#010X} entry {:#010X}",
        arm9_off, arm9_size, arm9_addr, arm9_entry, arm7_off, arm7_size, arm7_addr, arm7_entry
    );

    let m = Rc::new(RefCell::new(Machine::new()));
    m.borrow_mut().rom = rom.clone();
    // Save file lives next to the ROM. NDS_SAV redirects it, which keeps
    // scripted test runs from overwriting a real playthrough's save.
    let sav_path = match std::env::var("NDS_SAV") {
        Ok(p) => std::path::PathBuf::from(p),
        Err(_) => std::path::Path::new(&path).with_extension("sav"),
    };
    if let Ok(sav) = std::fs::read(&sav_path) {
        let n = sav.len().min(0x8_0000);
        m.borrow_mut().save[..n].copy_from_slice(&sav[..n]);
        eprintln!("loaded save: {}", sav_path.display());
    }
    let mut cpu9 = Cpu::new(
        View { m: m.clone(), cpu: 0, in_dma: false },
        true,
        arm9_entry,
        0x0300_2F7C,
        0x0300_3F80,
        0x0300_3FC0,
    );
    let mut cpu7 = Cpu::new(
        View { m: m.clone(), cpu: 1, in_dma: false },
        false,
        arm7_entry,
        0x0380_FD80,
        0x0380_FF80,
        0x0380_FFC0,
    );

    // Direct boot: copy the two binaries to their load addresses and stage
    // the header + boot flags the firmware would leave in main RAM.
    for i in 0..arm9_size {
        cpu9.bus.write8(arm9_addr + i as u32, rom[arm9_off + i]);
    }
    for i in 0..arm7_size {
        cpu7.bus.write8(arm7_addr + i as u32, rom[arm7_off + i]);
    }
    for i in 0..0x170.min(rom.len()) {
        cpu9.bus.write8(0x027F_FE00 + i as u32, rom[i]);
    }
    cpu9.bus.write32(0x027F_F800, 0x0000_1FC2); // chip ID
    cpu9.bus.write32(0x027F_F804, 0x0000_1FC2);
    cpu9.bus.write32(0x027F_FC00, 0x0000_1FC2); // boot-check copies
    cpu9.bus.write32(0x027F_FC04, 0x0000_1FC2);
    cpu9.bus.write16(0x027F_F850, 0x5835);
    cpu9.bus.write16(0x027F_FC10, 0x5835);
    cpu9.bus.write32(0x027F_FC40, 1); // boot indicator: cart

    // Firmware user settings copy at 0x027FFC80 (the firmware places this
    // before booting a game; SDK ARM7 code CRC-checks it and loops forever
    // on failure). Minimal valid block: version, nickname, touch
    // calibration, language, CRC16 over the first 0x70 bytes.
    let us = bus::user_settings_block();
    for (i, b) in us.iter().enumerate() {
        cpu9.bus.write8(0x027F_FC80 + i as u32, *b);
    }

    // Windowed mode unless NDS_FRAMES (headless test harness) is set.
    let headless = std::env::var("NDS_FRAMES").is_ok();
    let mut window = if headless {
        None
    } else {
        let mut w = minifb::Window::new(
            "NDS",
            ppu::WIDTH,
            ppu::HEIGHT * 2,
            minifb::WindowOptions {
                scale: minifb::Scale::X2,
                ..Default::default()
            },
        )
        .expect("window");
        w.set_target_fps(60);
        Some(w)
    };

    let frames: u32 = std::env::var("NDS_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(u32::MAX);
    let script = std::env::var("NDS_INPUT").ok().map(|s| InputScript::parse(&s));
    // Scripted taps are for the headless harness only; with a window open the
    // mouse is the stylus and must not be fought over.
    let touch_script = if headless { parse_touch_script() } else { Vec::new() };
    let mut ppu = Ppu::new();
    let trace = std::env::var("NDS_TRACE").is_ok();
    // NDS_MOUSELOG=1: report where each click lands, for diagnosing the
    // window-to-buffer coordinate mapping.
    let mouse_log = std::env::var("NDS_MOUSELOG").is_ok();
    let vid_log = std::env::var("NDS_VIDLOG").is_ok();
    let mut last_vid = String::new();
    // Consecutive frames the host mouse has read as released. See the stylus
    // handling in the frame loop for why the pen lags behind it.
    let mut pen_up_frames = u32::MAX;
    let mut pc_hist: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();

    // Savestates.
    //   NDS_STATE_LOAD=path   restore before the first frame runs
    //   NDS_STATE_SAVE=path   dump the machine when the run ends
    //   NDS_STATE_AT=N:path   dump after frame N has been rendered
    // In the window, F5 saves and F9 loads the current slot; the number keys
    // 0-9 pick a slot, stored beside the ROM as <rom>.ssN.
    let state_at: Option<(u32, String)> = std::env::var("NDS_STATE_AT").ok().and_then(|v| {
        let (n, p) = v.split_once(':')?;
        Some((n.trim().parse().ok()?, p.to_string()))
    });
    let mut slot = 0u32;
    let slot_path = |n: u32| format!("{path}.ss{n}");
    // A state a script asked for and did not get is a silent loss of a long
    // replay, so any such failure has to reach the exit status.
    let mut state_failed = false;
    let mut state_at_fired = false;
    // A loaded state carries its own backup-chip contents, which no longer
    // match the .sav on disk; don't write that file back out unless the game
    // itself saves again after the load.
    let mut state_loaded = false;
    // The snapshot stores its frame index, so a resumed run continues on the
    // original timeline: NDS_FRAMES stays an absolute end frame and every
    // NDS_INPUT / NDS_TOUCH span keeps its meaning.
    let mut start_frame = 0u32;
    if let Ok(p) = std::env::var("NDS_STATE_LOAD") {
        match state::read_file(&p) {
            Ok(st) => {
                start_frame = state::apply(st, &m, &mut cpu9, &mut cpu7, &mut ppu, &rom);
                state_loaded = true;
            }
            Err(e) => {
                eprintln!("state load failed: {e}");
                return ExitCode::FAILURE;
            }
        }
    }

    // Rough per-scanline instruction budgets (ARM9 66MHz, ARM7 33MHz).
    const LINES: u32 = 263;
    const INSTR9: u32 = 2000;
    const INSTR7: u32 = 500;

    let mut screen: Vec<u32> = vec![0; ppu::WIDTH * ppu::HEIGHT * 2];
    // Frame the run stopped on, so an exit-time snapshot records where it is.
    let mut last_frame = start_frame;
    for frame in start_frame..frames {
        if let Some(w) = &window {
            if !w.is_open() || w.is_key_down(minifb::Key::Escape) {
                break;
            }
            // Savestate hotkeys. is_key_pressed with KeyRepeat::No gives one
            // event per physical press, so holding the key cannot dump a state
            // every frame.
            {
                use minifb::{Key, KeyRepeat};
                const DIGITS: [Key; 10] = [
                    Key::Key0, Key::Key1, Key::Key2, Key::Key3, Key::Key4,
                    Key::Key5, Key::Key6, Key::Key7, Key::Key8, Key::Key9,
                ];
                for (n, k) in DIGITS.iter().enumerate() {
                    if w.is_key_pressed(*k, KeyRepeat::No) {
                        slot = n as u32;
                        eprintln!("savestate slot {slot}");
                    }
                }
                if w.is_key_pressed(Key::F5, KeyRepeat::No) {
                    if let Err(e) = state::save(&slot_path(slot), &m, &cpu9, &cpu7, &ppu, frame) {
                        eprintln!("state save failed: {e}");
                    }
                }
                if w.is_key_pressed(Key::F9, KeyRepeat::No) {
                    match state::read_file(&slot_path(slot)) {
                        // The frame counter keeps running forward here: in the
                        // window there is no scripted timeline to stay aligned
                        // with, and the loop index cannot be rewound.
                        Ok(st) => {
                            state::apply(st, &m, &mut cpu9, &mut cpu7, &mut ppu, &rom);
                            state_loaded = true;
                        }
                        Err(e) => eprintln!("state load failed: {e}"),
                    }
                }
            }
            // Keyboard -> KEYINPUT (active low): arrows, Z=B X=A, A=Y S=X,
            // Q=L W=R, Enter=Start, RShift=Select.
            use minifb::Key;
            let k = |key| !w.is_key_down(key) as u16;
            let mut mm = m.borrow_mut();
            mm.keyinput = k(Key::X)
                | k(Key::Z) << 1
                | k(Key::RightShift) << 2
                | k(Key::Enter) << 3
                | k(Key::Right) << 4
                | k(Key::Left) << 5
                | k(Key::Up) << 6
                | k(Key::Down) << 7
                | k(Key::W) << 8
                | k(Key::Q) << 9;
            // EXTKEYIN, also active low: bit0 X, bit1 Y, bit6 pen down, bit7
            // hinge. k() is already active low, so it drops straight in; an
            // extra `!` here would be a bitwise NOT of a u16, which forces
            // every unrelated bit (including pen-down) to zero. Bit 7 stays
            // CLEAR: GBATEK defines the hinge bit as 1 = lid closed, and a
            // "closed" lid sends Platinum into sleep mode.
            mm.extkeyin = 0x7C | k(Key::S) | k(Key::A) << 1;
            // Mouse on the lower screen = stylus.
            //
            // The host mouse is polled once per frame, but the ARM7 samples the
            // panel on its own schedule and reads the pen-down line separately
            // from the coordinates. A single dropped poll therefore produces a
            // sample real hardware cannot produce: flagged still-touched, but
            // carrying the released-corner ADC values, which makes the stylus
            // appear to jump to a corner and back. Keep the pen down for a few
            // frames after the host button reads as released, so one bad poll
            // cannot corrupt a press that is still in progress.
            const PEN_RELEASE_FRAMES: u32 = 3;
            let (ww, wh) = w.get_size();
            // minifb's coordinate space differs per platform/scale: on this
            // setup it's already buffer pixels; elsewhere it can be window
            // points. If the position fits the buffer, take it verbatim;
            // otherwise scale by window size.
            let mapped = w.get_mouse_pos(minifb::MouseMode::Discard).map(|(mx, my)| {
                if mx <= ppu::WIDTH as f32 && my <= (ppu::HEIGHT * 2) as f32 {
                    (mx, my)
                } else {
                    (
                        mx * ppu::WIDTH as f32 / ww.max(1) as f32,
                        my * (ppu::HEIGHT * 2) as f32 / wh.max(1) as f32,
                    )
                }
            });
            let down = w.get_mouse_down(minifb::MouseButton::Left);
            let on_lower = mapped.is_some_and(|(_, by)| by >= ppu::HEIGHT as f32);
            if down && on_lower {
                let (bx, by) = mapped.unwrap();
                mm.touch_x = (bx as u32).min(255);
                mm.touch_y = (by as u32 - ppu::HEIGHT as u32).min(191);
                pen_up_frames = 0;
            } else {
                pen_up_frames = pen_up_frames.saturating_add(1);
            }
            mm.touch_down = pen_up_frames < PEN_RELEASE_FRAMES;
            if mm.touch_down {
                mm.extkeyin &= !0x40; // pen down (active low)
            }
            {
                if mouse_log && down && frame % 15 == 0 {
                    let (mx, my) = mapped.unwrap_or((-1.0, -1.0));
                    let (bx, by) = (mx, my);
                    let where_ = if !on_lower {
                        "UPPER screen or outside the window, ignored".to_string()
                    } else {
                        format!("stylus at ({},{})", mm.touch_x, mm.touch_y)
                    };
                    // Follow the sample the rest of the way: the ARM7 packs it
                    // into shared RAM, then Platinum's input routine stores the
                    // calibrated point in its own globals.
                    let rd16 = |a: u32| {
                        let o = (a & 0x3F_FFFF) as usize;
                        u16::from_le_bytes([mm.main_ram[o], mm.main_ram[o + 1]]) as u32
                    };
                    let packed = rd16(0x027F_FFAA) | rd16(0x027F_FFAC) << 16;
                    eprintln!(
                        "mouse: pos=({mx:.0},{my:.0}) window={ww}x{wh} buffer=({bx:.0},{by:.0}) -> {where_}\n\
                         \x20      shared TPData={packed:#010X} raw=({},{}) touch={} validity={}\n\
                         \x20      game input globals: x={} y={} pressed={} held={}",
                        packed & 0xFFF,
                        packed >> 12 & 0xFFF,
                        packed >> 24 & 1,
                        packed >> 25 & 3,
                        rd16(0x021B_F6D8),
                        rd16(0x021B_F6DA),
                        rd16(0x021B_F6DC),
                        rd16(0x021B_F6DE),
                    );
                }
            }
        } else if let Some(s) = &script {
            m.borrow_mut().keyinput = s.keys_at(frame);
        }
        if !touch_script.is_empty() {
            let mut mm = m.borrow_mut();
            mm.touch_down = false;
            mm.extkeyin |= 0x40;
            for &(a, b, x, y) in &touch_script {
                if frame >= a && frame <= b {
                    mm.touch_x = x;
                    mm.touch_y = y;
                    mm.touch_down = true;
                    mm.extkeyin &= !0x40; // pen down (active low)
                }
            }
        }
        for line in 0..LINES {
            {
                let mut mm = m.borrow_mut();
                mm.now += 1;
                mm.vcount = line as u16;
                for cpu in 0..2 {
                    let in_vblank = (192..262).contains(&line);
                    let stat = &mut mm.dispstat[cpu];
                    *stat = (*stat & !1) | in_vblank as u16;
                    // VCOUNT match flag/irq.
                    let target = (*stat >> 8) | ((*stat & 0x80) << 1);
                    let matched = line as u16 == target;
                    *stat = (*stat & !4) | (matched as u16) << 2;
                    if matched && *stat & 0x20 != 0 {
                        mm.if_[cpu] |= 1 << 2;
                    }
                }
                if line == 192 {
                    for cpu in 0..2 {
                        if mm.dispstat[cpu] & 8 != 0 {
                            mm.if_[cpu] |= IRQ_VBLANK;
                        }
                    }
                }
                // Geometry FIFO IRQ: empty FIFO satisfies either mode.
                if mm.gxstat_irq != 0 {
                    mm.if_[0] |= 1 << 21;
                }
                let vblank_now = line == 192;
                drop(mm);
                if vblank_now {
                    cpu9.bus.dma_service(bus::DMA_VBLANK);
                    cpu7.bus.dma_service(bus::DMA_VBLANK);
                }
            }
            m.borrow_mut().tick_timers(2124); // ~33.51MHz / 263 lines / 60Hz
            // Fine interleave: cross-CPU handshakes assume near-concurrency.
            for _ in 0..INSTR7 / 8 {
                for _ in 0..INSTR9 / (INSTR7 / 8) {
                    cpu9.step();
                }
                for _ in 0..8 {
                    cpu7.step();
                }
            }
            if trace {
                *pc_hist.entry(cpu9.st.regs[15]).or_insert(0u32) += 1;
                *pc_hist.entry(0xF000_0000 | cpu7.st.regs[15]).or_insert(0u32) += 1;
            }
        }
        ppu.render_frame(&mut m.borrow_mut());
        // Snapshot point: a frame boundary, after rendering, is the only place
        // the whole machine lives in the structs the savestate covers (the
        // scanline loop above keeps live state in local variables).
        last_frame = frame + 1;
        // The snapshot's frame field is the NEXT frame to run, so resuming
        // does not replay the frame that was already rendered into it.
        if let Some((n, p)) = &state_at {
            if frame == *n {
                state_at_fired = true;
                if let Err(e) = state::save(p, &m, &cpu9, &cpu7, &ppu, frame + 1) {
                    eprintln!("state save failed: {e}");
                    state_failed = true;
                }
            }
        }
        // Flush dirty save data to disk once per second.
        if frame % 60 == 59 {
            let mut mm = m.borrow_mut();
            if mm.save_dirty {
                mm.save_dirty = false;
                state_loaded = false;
                let _ = std::fs::write(&sav_path, &mm.save);
            }
        }
        if let Some(w) = &mut window {
            let [upper, lower] = screens(&ppu, m.borrow().powcnt1);
            screen[..ppu::WIDTH * ppu::HEIGHT].copy_from_slice(upper);
            screen[ppu::WIDTH * ppu::HEIGHT..].copy_from_slice(lower);
            w.update_with_buffer(&screen, ppu::WIDTH, ppu::HEIGHT * 2).unwrap();
        }
        // NDS_VIDLOG=1: dump both 2D engines' layer setup, but only when it
        // actually changes, so a long play session leaves a short readable log
        // whose tail describes whatever is on screen now.
        if vid_log {
            let mm = m.borrow();
            let mut snap = String::new();
            for eng in 0..2 {
                let io = &mm.io2d[eng];
                let r16 = |o: usize| u16::from_le_bytes([io[o], io[o + 1]]);
                let dispcnt = u32::from_le_bytes([io[0], io[1], io[2], io[3]]);
                snap += &format!(
                    "eng{} dispcnt={dispcnt:#010X} mode={} bg_en={:04b} win_en={:03b} \
                     win0h={:#06X} win0v={:#06X} win1h={:#06X} win1v={:#06X} winin={:#06X} winout={:#06X}",
                    if eng == 0 { 'A' } else { 'B' },
                    dispcnt & 7,
                    dispcnt >> 8 & 0xF,
                    dispcnt >> 13 & 7,
                    r16(0x40), r16(0x44), r16(0x42), r16(0x46), r16(0x48), r16(0x4A),
                );
                snap += &format!(
                    " bldcnt={:#06X} bldalpha={:#06X} bldy={:#06X} brt={:#06X}\n",
                    r16(0x50), r16(0x52), r16(0x54), r16(0x6C)
                );
                if eng == 0 {
                    let cap = u32::from_le_bytes([io[0x64], io[0x65], io[0x66], io[0x67]]);
                    snap += &format!("   dispcapcnt={cap:#010X}\n");
                }
                for bg in 0..4 {
                    let cnt = r16(0x08 + bg * 2);
                    snap += &format!(
                        "   bg{bg} cnt={cnt:#06X} prio={} size={} charbase={} screenbase={} hofs={} vofs={}\n",
                        cnt & 3,
                        cnt >> 14 & 3,
                        cnt >> 2 & 0xF,
                        cnt >> 8 & 0x1F,
                        r16(0x10 + bg * 4),
                        r16(0x12 + bg * 4),
                    );
                }
            }
            snap += &format!("geometry commands submitted so far: {}\n", mm.gx_cmds);
            if snap != last_vid {
                // Poly counts change every 3D frame, so keep them out of the
                // change detection and just append the latest numbers.
                eprintln!(
                    "--- vid change at frame {frame} ---\n{snap}polys submitted last 3D frame: {} ({} verts, {} swaps)",
                    mm.gx.last_poly_count, mm.gx.last_vert_count, mm.gx.swap_count
                );
                last_vid = snap;
            }
        }
        if std::env::var("NDS_PALLOG").is_ok() {
            let mm = m.borrow();
            let p0 = u16::from_le_bytes([mm.pal[0], mm.pal[1]]);
            eprintln!("pal f{frame}: {:#06X} dispA={:#010X}", p0, {
                u32::from_le_bytes(mm.io2d[0][0..4].try_into().unwrap())
            });
        }
        if headless && frame % 30 == 0 {
            let mm = m.borrow();
            let d = |e: usize| u32::from_le_bytes(mm.io2d[e][0..4].try_into().unwrap());
            eprintln!(
                "frame {frame}: pc9={:#010X} pc7={:#010X} dispcntA={:#010X} dispcntB={:#010X} vramcnt={:02X?} ie9={:#010X} if9={:#010X} ime9={} h9={} ie7={:#010X} if7={:#010X} h7={}",
                cpu9.st.regs[15], cpu7.st.regs[15], d(0), d(1), mm.vramcnt,
                mm.ie[0], mm.if_[0], mm.ime[0], cpu9.st.halted, mm.ie[1], mm.if_[1], cpu7.st.halted
            );
            eprintln!(
                "  fifo to7={} to9={} ime7={} cnt7={:#06X}",
                mm.fifo_to7.len(), mm.fifo_to9.len(), mm.ime[1], mm.ipcfifocnt[1]
            );
        }
    }
    if let Ok(p) = std::env::var("NDS_STATE_SAVE") {
        if let Err(e) = state::save(&p, &m, &cpu9, &cpu7, &ppu, last_frame) {
            eprintln!("state save failed: {e}");
            state_failed = true;
        }
    }
    if let Some((n, p)) = &state_at {
        if !state_at_fired {
            eprintln!("state save failed: frame {n} never ran, {p} not written");
            state_failed = true;
        }
    }
    {
        let mm = m.borrow();
        if mm.save_dirty || (sav_path.exists() && !state_loaded) {
            let _ = std::fs::write(&sav_path, &mm.save);
        }
    }
    if trace {
        let mut v: Vec<_> = pc_hist.into_iter().collect();
        v.sort_by_key(|&(_, n)| std::cmp::Reverse(n));
        for (pc, n) in v.into_iter().take(16) {
            let (cpu, pc) = if pc & 0xF000_0000 == 0xF000_0000 { ("pc7", pc & 0x0FFF_FFFF) } else { ("pc9", pc) };
            eprintln!("{cpu} {:#010X}: {} line-samples (halted9={} halted7={})", pc, n, cpu9.st.halted, cpu7.st.halted);
        }
    }
    // NDS_GXDUMP=1: dump every polygon in the displayed 3D frame (attr,
    // texture params, projected screen position) for debugging.
    if std::env::var("NDS_GXDUMP").is_ok() {
        let mm = m.borrow();
        let gx = &mm.gx;
        for (n, p) in gx.disp_polys.iter().enumerate() {
            let mut s = String::new();
            for &i in &p.verts[..p.nverts as usize] {
                let c = gx.disp_verts[i as usize].clip;
                let w = c[3] as f32;
                if w.abs() > 1.0 {
                    s += &format!(
                        " ({:.0},{:.0})",
                        (c[0] as f32 / w * 0.5 + 0.5) * 256.0,
                        192.0 - (c[1] as f32 / w * 0.5 + 0.5) * 192.0
                    );
                } else {
                    s += " (w~0)";
                }
            }
            eprintln!(
                "gxdump {n}: attr={:#010X} tex={:#010X} pltt={:#06X} n={}{}",
                p.attr, p.texparam, p.pltt, p.nverts, s
            );
        }
    }
    if let Ok(path) = std::env::var("NDS_RAMDUMP") {
        // Write the full 4MB main RAM as raw binary at exit, for offline
        // pointer hunts / disassembly (address 0x02000000 = file offset 0).
        std::fs::write(&path, &m.borrow().main_ram).ok();
        eprintln!("ramdump -> {path}");
    }
    if let Ok(spec) = std::env::var("NDS_DUMPMEM") {
        // "7:addr:len" or "9:addr:len", hex addr/len — dump live memory words.
        let parts: Vec<&str> = spec.split(':').collect();
        if parts.len() == 3 {
            let addr = u32::from_str_radix(parts[1].trim_start_matches("0x"), 16).unwrap();
            let len = u32::from_str_radix(parts[2].trim_start_matches("0x"), 16).unwrap();
            for i in (0..len).step_by(4) {
                let v = if parts[0] == "7" {
                    cpu7.bus.read32(addr + i)
                } else {
                    cpu9.bus.read32(addr + i)
                };
                eprintln!("{:#010X}: {:#010X}", addr + i, v);
            }
        }
    }
    if let Some(log) = m.borrow().io_log.as_ref() {
        let mut v: Vec<_> = log.iter().collect();
        v.sort_by_key(|&(_, n)| std::cmp::Reverse(*n));
        // NDS_IOLOG=<n> prints n rows instead of the default 24: a low-traffic
        // register that matters (sound, AUXSPI) is invisible in the top rows.
        let rows = std::env::var("NDS_IOLOG")
            .ok()
            .and_then(|v| v.trim().parse::<usize>().ok())
            .filter(|n| *n > 0)
            .unwrap_or(24);
        for (&(cpu, off, w), n) in v.into_iter().take(rows) {
            eprintln!(
                "io {} {:#06X} {}: {}",
                if cpu == 0 { "arm9" } else { "arm7" },
                off,
                if w { "W" } else { "R" },
                n
            );
        }
    }
    // NDS_OUT redirects the frame dump so concurrent runs do not race on it.
    let out = std::env::var("NDS_OUT").unwrap_or_else(|_| "frame.ppm".into());
    dump_frame(&ppu, m.borrow().powcnt1, &out);
    eprintln!("dumped {out}");
    if state_failed { ExitCode::FAILURE } else { ExitCode::SUCCESS }
}
