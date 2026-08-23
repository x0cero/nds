mod audio;
mod bus;
mod cpu;
mod gpu3d;
mod key1;
mod ppu;
mod render3d;
mod soundtbl;
mod spu;
mod state;
mod wifi;

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
    parse_touch_script_var("NDS_TOUCH")
}

fn parse_touch_script_var(var: &str) -> Vec<(u32, u32, u32, u32)> {
    let Ok(spec) = std::env::var(var) else { return Vec::new() };
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

/// One emulated DS: its memory, both CPUs, and its video hardware. A single
/// process can run two of these side by side and let them talk over the
/// emulated air, which is how a wireless link between two consoles works here.
/// Running both in one process is what makes the link timing tractable: the
/// client has microseconds to answer the host, and two separate processes
/// would drift apart long before that.
struct Console {
    m: Rc<RefCell<Machine>>,
    cpu9: Cpu<View>,
    cpu7: Cpu<View>,
    ppu: Ppu,
    sav_path: std::path::PathBuf,
    /// A restored savestate carries its own backup-chip contents, which no
    /// longer match the .sav on disk; don't write that file back out unless
    /// the game itself saves again after the load.
    state_loaded: bool,
    /// Consecutive frames the host mouse has read as released. See the stylus
    /// handling in the frame loop for why the pen lags behind it.
    pen_up_frames: u32,
}

impl Console {
    fn boot(
        rom: &[u8],
        sav_path: std::path::PathBuf,
        air: Option<Rc<RefCell<wifi::Air>>>,
        id: usize,
    ) -> Console {
        let r32 = |off: usize| u32::from_le_bytes(rom[off..off + 4].try_into().unwrap());
        let (arm9_off, arm9_entry, arm9_addr, arm9_size) =
            (r32(0x20) as usize, r32(0x24), r32(0x28), r32(0x2C) as usize);
        let (arm7_off, arm7_entry, arm7_addr, arm7_size) =
            (r32(0x30) as usize, r32(0x34), r32(0x38), r32(0x3C) as usize);
        if id == 0 {
            eprintln!(
                "arm9: {:#X}+{:#X} -> {:#010X} entry {:#010X} | arm7: {:#X}+{:#X} -> {:#010X} entry {:#010X}",
                arm9_off, arm9_size, arm9_addr, arm9_entry, arm7_off, arm7_size, arm7_addr, arm7_entry
            );
        }
        let m = Rc::new(RefCell::new(Machine::new()));
        {
            let mut mm = m.borrow_mut();
            mm.rom = rom.to_vec();
            mm.wf.air = air;
            mm.wf.id = id;
        }
        if let Ok(sav) = std::fs::read(&sav_path) {
            let n = sav.len().min(0x8_0000);
            m.borrow_mut().save[..n].copy_from_slice(&sav[..n]);
            eprintln!("console {id}: loaded save {}", sav_path.display());
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
        // Each console needs its own MAC address: two DS units sharing one
        // could not tell each other's frames from their own. Console 0 keeps
        // the stock address so that single-console runs are unchanged.
        if id > 0 {
            let mac = [0x00, 0x09, 0xBF, 0x12, 0x34, 0x56 + id as u8];
            bus::set_firmware_mac(&mut m.borrow_mut().firmware, mac);
        }

        Console {
            m,
            cpu9,
            cpu7,
            ppu: Ppu::new(),
            sav_path,
            state_loaded: false,
            pen_up_frames: u32::MAX,
        }
    }

    /// Per-scanline register bookkeeping: VCOUNT, the display-status flags and
    /// the interrupts they raise.
    fn line_begin(&mut self, line: u32) {
        let mut mm = self.m.borrow_mut();
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
        drop(mm);
        if line == 192 {
            self.cpu9.bus.dma_service(bus::DMA_VBLANK);
            self.cpu7.bus.dma_service(bus::DMA_VBLANK);
        }
        self.m.borrow_mut().tick_timers(2124); // ~33.51MHz / 263 lines / 60Hz
    }
}

fn main() -> ExitCode {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: nds <rom.nds>");
        return ExitCode::FAILURE;
    };
    // Clean dumps keep the secure area (2KB at 0x4000) KEY1-encrypted;
    // hardware decrypts it during boot, so direct boot must too.
    let load_rom = |p: &str| -> Vec<u8> {
        let mut rom = std::fs::read(p).expect("read rom");
        if rom.len() > 0x4800 {
            let gamecode = u32::from_le_bytes(rom[0x0C..0x10].try_into().unwrap());
            if key1::decrypt_secure_area(gamecode, &mut rom[0x4000..0x4800]) {
                eprintln!("secure area: KEY1-decrypted (encryObj ok)");
            }
        }
        rom
    };
    let rom = load_rom(&path);

    // NDS_LINK boots a second console alongside the first and puts the two on
    // a shared air, so the wireless hardware of one can hear the other. Set it
    // to a ROM path to run a different game on console 2, or to 1 for the same
    // ROM. The second console gets its own save file: two Pokemon saves with
    // the same trainer ID refuse to trade with each other.
    let link = std::env::var("NDS_LINK").ok();
    let rom2 = match link.as_deref() {
        None => None,
        Some("1") | Some("") => Some(rom.clone()),
        Some(p) => Some(load_rom(p)),
    };
    let n_consoles = 1 + rom2.is_some() as usize;
    let air = (n_consoles > 1).then(|| Rc::new(RefCell::new(wifi::Air::new(n_consoles))));
    // Save file lives next to the ROM. NDS_SAV redirects it, which keeps
    // scripted test runs from overwriting a real playthrough's save.
    let sav_path = |n: usize| -> std::path::PathBuf {
        let var = if n == 0 { "NDS_SAV" } else { "NDS_SAV2" };
        match std::env::var(var) {
            Ok(p) => std::path::PathBuf::from(p),
            Err(_) if n == 0 => std::path::Path::new(&path).with_extension("sav"),
            Err(_) => std::path::Path::new(&path).with_extension(format!("sav{}", n + 1)),
        }
    };
    let mut consoles = vec![Console::boot(&rom, sav_path(0), air.clone(), 0)];
    if let Some(r2) = &rom2 {
        consoles.push(Console::boot(r2, sav_path(1), air.clone(), 1));
    }

    // Windowed mode unless NDS_FRAMES (headless test harness) is set.
    let headless = std::env::var("NDS_FRAMES").is_ok();
    let mut window = if headless {
        None
    } else {
        // Linked, the two consoles sit side by side in one window, each with
        // its screens stacked as usual.
        let mut w = minifb::Window::new(
            "NDS",
            ppu::WIDTH * n_consoles,
            ppu::HEIGHT * 2,
            minifb::WindowOptions {
                scale: if n_consoles > 1 { minifb::Scale::X1 } else { minifb::Scale::X2 },
                ..Default::default()
            },
        )
        .expect("window");
        // Frame pacing is handled below, not by minifb. Its own limiter is
        // sleep-based and overshoots badly: asking it for 60 delivered 55 on
        // this machine, which is an 8% speed error and, once there is sound,
        // an audible one.
        w.set_target_fps(0);
        Some(w)
    };

    let frames: u32 = std::env::var("NDS_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(u32::MAX);
    let script = std::env::var("NDS_INPUT").ok().map(|s| InputScript::parse(&s));
    // Scripted taps are for the headless harness only; with a window open the
    // mouse is the stylus and must not be fought over.
    let touch_script = if headless { parse_touch_script() } else { Vec::new() };
    // The linked console runs its own scripts, so a headless test can drive
    // both sides of a wireless session.
    let script2 = std::env::var("NDS_INPUT2").ok().map(|s| InputScript::parse(&s));
    let touch_script2 = if headless { parse_touch_script_var("NDS_TOUCH2") } else { Vec::new() };
    // Which console the keyboard drives, toggled with Tab.
    let mut focus = 0usize;
    // Audio goes to the speakers only when there is a window: a headless test
    // run has no business seizing the sound device. NDS_WAV=path records the
    // mixer output either way, which is how a scripted replay gets checked.
    let mut audio = audio::Audio::new(!headless, std::env::var("NDS_WAV").ok());
    // Reused across frames so draining the mixer does not allocate.
    let mut audio_buf: Vec<i16> = Vec::new();
    let trace = std::env::var("NDS_TRACE").is_ok();
    // NDS_PROF=1: split wall time between CPU execution and frame rendering,
    // the only two phases big enough to matter for the 60fps budget.
    let prof = std::env::var("NDS_PROF").is_ok();
    let (mut t_cpu, mut t_render) = (std::time::Duration::ZERO, std::time::Duration::ZERO);
    let (mut t_cpu_last, mut t_render_last) = (std::time::Duration::ZERO, std::time::Duration::ZERO);
    let mut t_phase = std::time::Instant::now();
    let (mut fps_frames, mut fps_t) = (0u32, std::time::Instant::now());
    // NDS_MOUSELOG=1: report where each click lands, for diagnosing the
    // window-to-buffer coordinate mapping.
    let mouse_log = std::env::var("NDS_MOUSELOG").is_ok();
    let vid_log = std::env::var("NDS_VIDLOG").is_ok();
    let mut last_vid = String::new();
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
    // NDS_FILM="stride:prefix" writes a frame dump every `stride` frames, so
    // one scripted run produces a contact sheet of where it went instead of
    // just its last frame. Invaluable when working out an input script.
    let film: Option<(u32, String)> = std::env::var("NDS_FILM").ok().and_then(|v| {
        let (n, p) = v.split_once(':')?;
        Some((n.trim().parse().ok()?, p.to_string()))
    });
    let mut slot = 0u32;
    // Savestate slot files sit beside the ROM; the linked console gets its
    // own set so the two cannot overwrite each other.
    let slot_path = |n: u32, console: usize| {
        if console == 0 { format!("{path}.ss{n}") } else { format!("{path}.c{}.ss{n}", console + 1) }
    };
    // A state a script asked for and did not get is a silent loss of a long
    // replay, so any such failure has to reach the exit status.
    let mut state_failed = false;
    let mut state_at_fired = false;
    // The snapshot stores its frame index, so a resumed run continues on the
    // original timeline: NDS_FRAMES stays an absolute end frame and every
    // NDS_INPUT / NDS_TOUCH span keeps its meaning.
    let mut start_frame = 0u32;
    if let Ok(p) = std::env::var("NDS_STATE_LOAD") {
        match state::read_file(&p) {
            Ok(st) => {
                let c = &mut consoles[0];
                start_frame = state::apply(st, &c.m, &mut c.cpu9, &mut c.cpu7, &mut c.ppu, &rom);
                c.state_loaded = true;
            }
            Err(e) => {
                eprintln!("state load failed: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    // The linked console can resume from its own snapshot, which is how a
    // wireless test starts both sides already deep inside the game.
    if let Ok(p) = std::env::var("NDS_STATE_LOAD2") {
        let Some(c) = consoles.get_mut(1) else {
            eprintln!("NDS_STATE_LOAD2 needs NDS_LINK");
            return ExitCode::FAILURE;
        };
        match state::read_file(&p) {
            Ok(st) => {
                let r = rom2.as_ref().unwrap_or(&rom);
                state::apply(st, &c.m, &mut c.cpu9, &mut c.cpu7, &mut c.ppu, r);
                c.state_loaded = true;
            }
            Err(e) => {
                eprintln!("state load failed: {e}");
                return ExitCode::FAILURE;
            }
        }
    }

    // One DS frame: 355 dots x 263 lines at the 5.585664 MHz dot clock (a sixth
    // of the system clock), which works out to 59.8261 Hz.
    const FRAME_TIME: std::time::Duration = std::time::Duration::from_nanos(16_716_038);
    let mut next_frame = std::time::Instant::now() + FRAME_TIME;

    // Rough per-scanline instruction budgets (ARM9 66MHz, ARM7 33MHz).
    const LINES: u32 = 263;
    const INSTR9: u32 = 2000;
    const INSTR7: u32 = 500;

    let mut screen: Vec<u32> = vec![0; ppu::WIDTH * n_consoles * ppu::HEIGHT * 2];
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
                // Tab moves the keyboard between the linked consoles.
                if n_consoles > 1 && w.is_key_pressed(Key::Tab, KeyRepeat::No) {
                    focus = (focus + 1) % n_consoles;
                    eprintln!("keyboard now drives console {}", focus + 1);
                }
                let c = &mut consoles[focus];
                // Comma and period double for F5/F9: on a Mac the function
                // row needs Fn held down, which is awkward mid-game.
                if w.is_key_pressed(Key::F5, KeyRepeat::No) || w.is_key_pressed(Key::Comma, KeyRepeat::No) {
                    let p = slot_path(slot, focus);
                    if let Err(e) = state::save(&p, &c.m, &c.cpu9, &c.cpu7, &c.ppu, frame) {
                        eprintln!("state save failed: {e}");
                    } else {
                        eprintln!("saved {p}");
                    }
                }
                if w.is_key_pressed(Key::F9, KeyRepeat::No) || w.is_key_pressed(Key::Period, KeyRepeat::No) {
                    match state::read_file(&slot_path(slot, focus)) {
                        // The frame counter keeps running forward here: in the
                        // window there is no scripted timeline to stay aligned
                        // with, and the loop index cannot be rewound.
                        Ok(st) => {
                            let r = if focus == 0 { &rom } else { rom2.as_ref().unwrap_or(&rom) };
                            state::apply(st, &c.m, &mut c.cpu9, &mut c.cpu7, &mut c.ppu, r);
                            c.state_loaded = true;
                        }
                        Err(e) => eprintln!("state load failed: {e}"),
                    }
                }
            }
            // Keyboard -> KEYINPUT (active low): arrows, Z=B X=A, A=Y S=X,
            // Q=L W=R, Enter=Start, RShift=Select.
            use minifb::Key;
            let k = |key| !w.is_key_down(key) as u16;
            // Only the focused console hears the keyboard; the other one sees
            // no buttons held, which is exactly what a DS sitting on the table
            // next to you sees.
            for (i, c) in consoles.iter_mut().enumerate() {
                if i != focus {
                    let mut mm = c.m.borrow_mut();
                    mm.keyinput = 0x3FF;
                    mm.extkeyin |= 0x43;
                }
            }
            let mut mm = consoles[focus].m.borrow_mut();
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
            let full_w = (ppu::WIDTH * n_consoles) as f32;
            let mapped = w.get_mouse_pos(minifb::MouseMode::Discard).map(|(mx, my)| {
                if mx <= full_w && my <= (ppu::HEIGHT * 2) as f32 {
                    (mx, my)
                } else {
                    (
                        mx * full_w / ww.max(1) as f32,
                        my * (ppu::HEIGHT * 2) as f32 / wh.max(1) as f32,
                    )
                }
            });
            let down = w.get_mouse_down(minifb::MouseButton::Left);
            let on_lower = mapped.is_some_and(|(_, by)| by >= ppu::HEIGHT as f32);
            // With two consoles on screen, the stylus belongs to whichever
            // one you clicked on.
            let touched = mapped
                .map(|(bx, _)| (bx as usize / ppu::WIDTH).min(n_consoles - 1))
                .unwrap_or(0);
            drop(mm);
            for (i, c) in consoles.iter_mut().enumerate() {
                let mut mm = c.m.borrow_mut();
                if down && on_lower && i == touched {
                    let (bx, by) = mapped.unwrap();
                    mm.touch_x = (bx as u32 - (i * ppu::WIDTH) as u32).min(255);
                    mm.touch_y = (by as u32 - ppu::HEIGHT as u32).min(191);
                    c.pen_up_frames = 0;
                } else {
                    c.pen_up_frames = c.pen_up_frames.saturating_add(1);
                }
                mm.touch_down = c.pen_up_frames < PEN_RELEASE_FRAMES;
                if mm.touch_down {
                    mm.extkeyin &= !0x40; // pen down (active low)
                }
            }
            let mm = consoles[focus].m.borrow();
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
        } else {
            for (i, s) in [&script, &script2].iter().enumerate() {
                if let (Some(s), Some(c)) = (s, consoles.get(i)) {
                    c.m.borrow_mut().keyinput = s.keys_at(frame);
                }
            }
        }
        for (i, taps) in [&touch_script, &touch_script2].iter().enumerate() {
            if taps.is_empty() {
                continue;
            }
            let Some(c) = consoles.get(i) else { continue };
            let mut mm = c.m.borrow_mut();
            mm.touch_down = false;
            mm.extkeyin |= 0x40;
            for &(a, b, x, y) in taps.iter() {
                if frame >= a && frame <= b {
                    mm.touch_x = x;
                    mm.touch_y = y;
                    mm.touch_down = true;
                    mm.extkeyin &= !0x40; // pen down (active low)
                }
            }
        }
        if prof {
            t_phase = std::time::Instant::now();
        }
        for line in 0..LINES {
            for c in consoles.iter_mut() {
                c.line_begin(line);
            }
            // Fine interleave: cross-CPU handshakes assume near-concurrency,
            // and with two consoles linked the same applies between machines.
            // A chunk is about a microsecond, which is the timescale the
            // wireless link answers on.
            const CHUNKS: u32 = INSTR7 / 8;
            for i in 0..CHUNKS {
                // Split the line's 2130 cycles across the chunks exactly.
                let cycles = 2130 * (i + 1) / CHUNKS - 2130 * i / CHUNKS;
                for c in consoles.iter_mut() {
                    c.cpu9.run_slice(INSTR9 / CHUNKS);
                    c.cpu7.run_slice(8);
                    // The wireless MAC keeps its own microsecond clock. It is
                    // stepped here, inside the interleave, so that a console
                    // hears the other's transmission within a microsecond or
                    // so rather than at the end of the scanline.
                    wifi::step(&mut c.m.borrow_mut(), cycles);
                }
            }
            for c in consoles.iter_mut() {
                // Mix after the CPUs have run, so a note started on this
                // scanline is audible in this scanline's two samples rather
                // than the next line's. At 1024 cycles a sample that is about
                // 2 samples a line.
                //
                // 2130, not the 2124 the timers use: a DS line is 33.513982
                // MHz / 59.8261 Hz / 263 lines = 2130 cycles, and the timer
                // figure above was derived from a round 60 Hz. The 0.3% gap
                // does not matter to a timer but it does here, because the
                // sample rate is what paces the whole emulator once the
                // speakers are open. Correcting the timer constant instead
                // would change every existing replay.
                c.m.borrow_mut().spu_run(2130);
            }
            if trace {
                let c = &consoles[0];
                *pc_hist.entry(c.cpu9.st.regs[15]).or_insert(0u32) += 1;
                *pc_hist.entry(0xF000_0000 | c.cpu7.st.regs[15]).or_insert(0u32) += 1;
            }
        }
        if prof {
            t_cpu += t_phase.elapsed();
            t_phase = std::time::Instant::now();
        }
        for c in consoles.iter_mut() {
            let mut mm = c.m.borrow_mut();
            c.ppu.render_frame(&mut mm);
        }
        if prof {
            t_render += t_phase.elapsed();
        }
        // Only the first console is audible: two games mixed together would be
        // noise. The other's samples are dropped, or its buffer would grow
        // without bound.
        std::mem::swap(&mut consoles[0].m.borrow_mut().spu.out, &mut audio_buf);
        audio.push(&audio_buf);
        audio_buf.clear();
        for c in consoles.iter_mut().skip(1) {
            c.m.borrow_mut().spu.out.clear();
        }
        // Pace the emulator. With speakers open the sound card's clock is the
        // master; without one, sleep until the next DS frame is due. A DS runs
        // at 59.8261 Hz, not 60, and the difference is a frame every four
        // seconds. Headless runs pace themselves by not pacing at all.
        if window.is_some() && !audio.pace() {
            let now = std::time::Instant::now();
            if next_frame > now {
                std::thread::sleep(next_frame - now);
            }
            next_frame = next_frame.max(now) + FRAME_TIME;
        }
        // Snapshot point: a frame boundary, after rendering, is the only place
        // the whole machine lives in the structs the savestate covers (the
        // scanline loop above keeps live state in local variables).
        last_frame = frame + 1;
        // The snapshot's frame field is the NEXT frame to run, so resuming
        // does not replay the frame that was already rendered into it.
        if let Some((n, p)) = &state_at {
            if frame == *n {
                state_at_fired = true;
                let c = &consoles[0];
                if let Err(e) = state::save(p, &c.m, &c.cpu9, &c.cpu7, &c.ppu, frame + 1) {
                    eprintln!("state save failed: {e}");
                    state_failed = true;
                }
            }
        }
        if let Some((stride, prefix)) = &film {
            if *stride > 0 && frame % *stride == 0 {
                for (i, c) in consoles.iter().enumerate() {
                    let tag = if i == 0 { String::new() } else { format!("-c{}", i + 1) };
                    let p = format!("{prefix}{frame:06}{tag}.ppm");
                    dump_frame(&c.ppu, c.m.borrow().powcnt1, &p);
                }
            }
        }
        // Flush dirty save data to disk once per second.
        if frame % 60 == 59 {
            for c in consoles.iter_mut() {
                let mut mm = c.m.borrow_mut();
                if mm.save_dirty {
                    mm.save_dirty = false;
                    c.state_loaded = false;
                    let _ = std::fs::write(&c.sav_path, &mm.save);
                }
            }
        }
        if let Some(w) = &mut window {
            // Consoles are laid out left to right, each with its two screens
            // stacked, so one buffer row spans every console's row.
            let stride = ppu::WIDTH * n_consoles;
            for (i, c) in consoles.iter().enumerate() {
                let [upper, lower] = screens(&c.ppu, c.m.borrow().powcnt1);
                for (half, fb) in [upper, lower].iter().enumerate() {
                    for y in 0..ppu::HEIGHT {
                        let dst = (half * ppu::HEIGHT + y) * stride + i * ppu::WIDTH;
                        screen[dst..dst + ppu::WIDTH]
                            .copy_from_slice(&fb[y * ppu::WIDTH..(y + 1) * ppu::WIDTH]);
                    }
                }
            }
            w.update_with_buffer(&screen, stride, ppu::HEIGHT * 2).unwrap();
            // Live speed readout: emulated fps (what the game actually gets)
            // alongside the 60 target, so a slowdown is visible immediately.
            fps_frames += 1;
            if fps_t.elapsed() >= std::time::Duration::from_millis(500) {
                let fps = fps_frames as f64 / fps_t.elapsed().as_secs_f64();
                if n_consoles > 1 {
                    w.set_title(&format!(
                        "NDS  [{fps:.0}/60]  keyboard: console {} (Tab to switch)",
                        focus + 1
                    ));
                } else {
                    w.set_title(&format!("NDS  [{fps:.0}/60]"));
                }
                if prof {
                    let (underruns, queued) = audio.health();
                    // Split this interval's wall time into the three phases, so
                    // a slow frame says which one is slow. Whatever is left
                    // over after CPU and render is the window itself.
                    let f = fps_frames.max(1) as f64;
                    let (c, r) = (t_cpu - t_cpu_last, t_render - t_render_last);
                    let total = fps_t.elapsed();
                    eprintln!(
                        "windowed {fps:.1} fps | cpu {:.1} render {:.1} window {:.1} ms/f | \
                         audio queued {queued}, {underruns} underruns",
                        c.as_secs_f64() * 1000.0 / f,
                        r.as_secs_f64() * 1000.0 / f,
                        (total.saturating_sub(c + r)).as_secs_f64() * 1000.0 / f,
                    );
                    (t_cpu_last, t_render_last) = (t_cpu, t_render);
                }
                fps_frames = 0;
                fps_t = std::time::Instant::now();
            }
        }
        // NDS_VIDLOG=1: dump both 2D engines' layer setup, but only when it
        // actually changes, so a long play session leaves a short readable log
        // whose tail describes whatever is on screen now.
        if vid_log {
            let mm = consoles[0].m.borrow();
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
            let mm = consoles[0].m.borrow();
            let p0 = u16::from_le_bytes([mm.pal[0], mm.pal[1]]);
            eprintln!("pal f{frame}: {:#06X} dispA={:#010X}", p0, {
                u32::from_le_bytes(mm.io2d[0][0..4].try_into().unwrap())
            });
        }
        if headless && frame % 30 == 0 {
            for (i, c) in consoles.iter().enumerate() {
                let mm = c.m.borrow();
                let d = |e: usize| u32::from_le_bytes(mm.io2d[e][0..4].try_into().unwrap());
                let tag = if i == 0 { String::new() } else { format!(" console{}", i + 1) };
                eprintln!(
                    "frame {frame}{tag}: pc9={:#010X} pc7={:#010X} dispcntA={:#010X} dispcntB={:#010X} vramcnt={:02X?} ie9={:#010X} if9={:#010X} ime9={} h9={} ie7={:#010X} if7={:#010X} h7={}",
                    c.cpu9.st.regs[15], c.cpu7.st.regs[15], d(0), d(1), mm.vramcnt,
                    mm.ie[0], mm.if_[0], mm.ime[0], c.cpu9.st.halted, mm.ie[1], mm.if_[1], c.cpu7.st.halted
                );
                eprintln!(
                    "  fifo to7={} to9={} ime7={} cnt7={:#06X}",
                    mm.fifo_to7.len(), mm.fifo_to9.len(), mm.ime[1], mm.ipcfifocnt[1]
                );
            }
        }
    }
    audio.finish();
    if let Ok(p) = std::env::var("NDS_STATE_SAVE") {
        let c = &consoles[0];
        if let Err(e) = state::save(&p, &c.m, &c.cpu9, &c.cpu7, &c.ppu, last_frame) {
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
    for c in consoles.iter() {
        let mm = c.m.borrow();
        if mm.save_dirty || (c.sav_path.exists() && !c.state_loaded) {
            let _ = std::fs::write(&c.sav_path, &mm.save);
        }
    }
    if trace {
        let mut v: Vec<_> = pc_hist.into_iter().collect();
        v.sort_by_key(|&(_, n)| std::cmp::Reverse(n));
        for (pc, n) in v.into_iter().take(16) {
            let (cpu, pc) = if pc & 0xF000_0000 == 0xF000_0000 { ("pc7", pc & 0x0FFF_FFFF) } else { ("pc9", pc) };
            eprintln!(
                "{cpu} {:#010X}: {} line-samples (halted9={} halted7={})",
                pc, n, consoles[0].cpu9.st.halted, consoles[0].cpu7.st.halted
            );
        }
    }
    // NDS_GXDUMP=1: dump every polygon in the displayed 3D frame (attr,
    // texture params, projected screen position) for debugging.
    if std::env::var("NDS_GXDUMP").is_ok() {
        let mm = consoles[0].m.borrow();
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
    if prof {
        let f = last_frame.max(1) as f64;
        eprintln!(
            "prof summary: {} frames | cpu {:.1}ms/f | render {:.1}ms/f | total {:.1}ms/f -> {:.0} fps",
            last_frame,
            t_cpu.as_secs_f64() * 1000.0 / f,
            t_render.as_secs_f64() * 1000.0 / f,
            (t_cpu + t_render).as_secs_f64() * 1000.0 / f,
            f / (t_cpu + t_render).as_secs_f64()
        );
    }
    if let Ok(path) = std::env::var("NDS_RAMDUMP") {
        // Write the full 4MB main RAM as raw binary at exit, for offline
        // pointer hunts / disassembly (address 0x02000000 = file offset 0).
        std::fs::write(&path, &consoles[0].m.borrow().main_ram).ok();
        eprintln!("ramdump -> {path}");
    }
    if let Ok(spec) = std::env::var("NDS_DUMPMEM") {
        // "7:addr:len" or "9:addr:len", hex addr/len — dump live memory words.
        let parts: Vec<&str> = spec.split(':').collect();
        if parts.len() == 3 {
            let addr = u32::from_str_radix(parts[1].trim_start_matches("0x"), 16).unwrap();
            let len = u32::from_str_radix(parts[2].trim_start_matches("0x"), 16).unwrap();
            for i in (0..len).step_by(4) {
                let c = &mut consoles[0];
                let v = if parts[0] == "7" {
                    c.cpu7.bus.read32(addr + i)
                } else {
                    c.cpu9.bus.read32(addr + i)
                };
                eprintln!("{:#010X}: {:#010X}", addr + i, v);
            }
        }
    }
    if let Some(log) = consoles[0].m.borrow().io_log.as_ref() {
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
    // NDS_OUT redirects the frame dump so concurrent runs do not race on it;
    // NDS_OUT2 does the same for the linked console.
    for (i, c) in consoles.iter().enumerate() {
        let var = if i == 0 { "NDS_OUT" } else { "NDS_OUT2" };
        let out = std::env::var(var)
            .unwrap_or_else(|_| if i == 0 { "frame.ppm".into() } else { format!("frame{}.ppm", i + 1) });
        dump_frame(&c.ppu, c.m.borrow().powcnt1, &out);
        eprintln!("dumped {out}");
    }
    if state_failed { ExitCode::FAILURE } else { ExitCode::SUCCESS }
}
