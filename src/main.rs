mod bus;
mod cpu;
mod ppu;

use bus::{Bus, Machine, View, IRQ_VBLANK};
use cpu::Cpu;
use ppu::Ppu;
use std::cell::RefCell;
use std::process::ExitCode;
use std::rc::Rc;

/// Stack both screens (A on top) into one PPM.
fn dump_frame(ppu: &Ppu, path: &str) {
    let mut out = format!("P6\n{} {}\n255\n", ppu::WIDTH, ppu::HEIGHT * 2).into_bytes();
    for fb in [&ppu.fb_a, &ppu.fb_b] {
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

fn main() -> ExitCode {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: nds <rom.nds>");
        return ExitCode::FAILURE;
    };
    let rom = std::fs::read(&path).expect("read rom");
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
    let mut cpu9 = Cpu::new(
        View { m: m.clone(), cpu: 0 },
        true,
        arm9_entry,
        0x0300_2F7C,
        0x0300_3F80,
        0x0300_3FC0,
    );
    let mut cpu7 = Cpu::new(
        View { m: m.clone(), cpu: 1 },
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
    cpu9.bus.write16(0x027F_F850, 0x5835);
    cpu9.bus.write16(0x027F_FC10, 0x5835);
    cpu9.bus.write32(0x027F_FC40, 1); // boot indicator: cart

    let frames: u32 = std::env::var("NDS_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(120);
    let script = std::env::var("NDS_INPUT").ok().map(|s| InputScript::parse(&s));
    let mut ppu = Ppu::new();
    let trace = std::env::var("NDS_TRACE").is_ok();
    let mut pc_hist: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();

    // Rough per-scanline instruction budgets (ARM9 66MHz, ARM7 33MHz).
    const LINES: u32 = 263;
    const INSTR9: u32 = 2000;
    const INSTR7: u32 = 500;

    for frame in 0..frames {
        if let Some(s) = &script {
            m.borrow_mut().keyinput = s.keys_at(frame);
        }
        for line in 0..LINES {
            {
                let mut mm = m.borrow_mut();
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
            }
            for _ in 0..INSTR9 {
                cpu9.step();
            }
            for _ in 0..INSTR7 {
                cpu7.step();
            }
            if trace {
                *pc_hist.entry(cpu9.regs[15]).or_insert(0u32) += 1;
            }
        }
        ppu.render_frame(&m.borrow());
        if frame % 30 == 0 {
            let mm = m.borrow();
            let d = |e: usize| u32::from_le_bytes(mm.io2d[e][0..4].try_into().unwrap());
            eprintln!(
                "frame {frame}: pc9={:#010X} pc7={:#010X} dispcntA={:#010X} dispcntB={:#010X} vramcnt={:02X?}",
                cpu9.regs[15], cpu7.regs[15], d(0), d(1), mm.vramcnt
            );
        }
    }
    if trace {
        let mut v: Vec<_> = pc_hist.into_iter().collect();
        v.sort_by_key(|&(_, n)| std::cmp::Reverse(n));
        for (pc, n) in v.into_iter().take(10) {
            eprintln!("pc9 {:#010X}: {} line-samples (halted={})", pc, n, cpu9.halted);
        }
    }
    dump_frame(&ppu, "frame.ppm");
    eprintln!("dumped frame.ppm");
    ExitCode::SUCCESS
}
