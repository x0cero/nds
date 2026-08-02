use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

/// One CPU's view of the machine. The two views share `Machine` and differ
/// in memory map, I/O block, and interrupt state.
pub trait Bus {
    fn read8(&mut self, a: u32) -> u8;
    fn read16(&mut self, a: u32) -> u16;
    fn read32(&mut self, a: u32) -> u32;
    fn write8(&mut self, a: u32, v: u8);
    fn write16(&mut self, a: u32, v: u16);
    fn write32(&mut self, a: u32, v: u32);
    fn irq_pending(&mut self) -> bool;
    fn ime(&mut self) -> bool;
    fn set_ime(&mut self, on: bool);
    /// Address of the BIOS interrupt-flag mirror word games' IRQ handlers
    /// update (top of DTCM on the ARM9, top of ARM7 WRAM on the ARM7).
    fn bios_flag_addr(&mut self) -> u32;
    fn set_dtcm(&mut self, _base: u32) {}
}

pub const LCDC_BASE: [u32; 9] = [
    0x0680_0000, // A
    0x0682_0000, // B
    0x0684_0000, // C
    0x0686_0000, // D
    0x0688_0000, // E
    0x0689_0000, // F
    0x0689_4000, // G
    0x0689_8000, // H
    0x068A_0000, // I
];
pub const BANK_SIZE: [usize; 9] = [
    0x20000, 0x20000, 0x20000, 0x20000, 0x10000, 0x4000, 0x4000, 0x8000, 0x4000,
];

/// IRQ bits (shared numbering across both CPUs).
pub const IRQ_VBLANK: u32 = 1 << 0;
pub const IRQ_IPC_SEND_EMPTY: u32 = 1 << 17;
pub const IRQ_IPC_RECV: u32 = 1 << 18;

pub struct Machine {
    pub main_ram: Vec<u8>, // 4MB at 0x02000000
    pub wram7: Vec<u8>,    // 64KB ARM7 at 0x03800000
    pub swram: Vec<u8>,    // 32KB shared at 0x03000000 per WRAMCNT
    pub wramcnt: u8,
    pub dtcm: Vec<u8>, // 16KB, ARM9, base from CP15
    pub dtcm_base: u32,
    pub itcm: Vec<u8>, // 32KB, ARM9, at 0
    pub vram: [Vec<u8>; 9],
    pub vramcnt: [u8; 9],
    pub pal: Vec<u8>, // 2KB: A-BG, A-OBJ, B-BG, B-OBJ
    pub oam: Vec<u8>, // 2KB: engine A then B
    /// 2D register blocks: engine A at 0x04000000, B at 0x04001000.
    pub io2d: [[u8; 0x70]; 2],
    pub vcount: u16,
    pub keyinput: u16, // 10 bits, active low
    pub extkeyin: u16,
    pub powcnt1: u32,
    // Per-CPU interrupt + display state. Index 0 = ARM9, 1 = ARM7.
    pub ime: [bool; 2],
    pub ie: [u32; 2],
    pub if_: [u32; 2],
    pub dispstat: [u16; 2],
    pub postflg: [u8; 2],
    // IPC.
    pub ipcsync_in: [u16; 2], // what each CPU reads in bits 0-3 (other's output)
    pub ipcfifocnt: [u16; 2],
    pub fifo_to7: VecDeque<u32>,
    pub fifo_to9: VecDeque<u32>,
    pub fifo_last: [u32; 2],
    // ARM9 math hardware.
    pub divcnt: u16,
    pub div_num: u64,
    pub div_den: u64,
    pub div_result: u64,
    pub div_rem: u64,
    pub sqrtcnt: u16,
    pub sqrt_param: u64,
    pub sqrt_result: u32,
    // HLE BIOS IRQ dispatch stubs.
    pub stub9: [u8; 0x40],
    pub stub7: [u8; 0x40],
}

/// The BIOS IRQ dispatcher both CPUs get: save regs, load the user handler
/// pointer from a literal-addressed word, call it, return from exception.
fn build_stub(handler_ptr_plus4: u32) -> [u8; 0x40] {
    let words: [u32; 7] = [
        0xE92D500F, // 0x18: stmfd sp!, {r0-r3, r12, lr}
        0xE59F000C, // 0x1C: ldr r0, [pc, #12]   ; -> literal at 0x30
        0xE28FE000, // 0x20: add lr, pc, #0      ; return to 0x28
        0xE510F004, // 0x24: ldr pc, [r0, #-4]
        0xE8BD500F, // 0x28: ldmfd sp!, {r0-r3, r12, lr}
        0xE25EF004, // 0x2C: subs pc, lr, #4
        handler_ptr_plus4, // 0x30: literal
    ];
    let mut stub = [0u8; 0x40];
    for (i, w) in words.iter().enumerate() {
        stub[0x18 + i * 4..0x18 + i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    stub
}

impl Machine {
    pub fn new() -> Self {
        Self {
            main_ram: vec![0; 0x40_0000],
            wram7: vec![0; 0x1_0000],
            swram: vec![0; 0x8000],
            wramcnt: 0, // all shared WRAM to the ARM9 (firmware default)
            dtcm: vec![0; 0x4000],
            dtcm_base: 0x0080_0000,
            itcm: vec![0; 0x8000],
            vram: [
                vec![0; 0x20000],
                vec![0; 0x20000],
                vec![0; 0x20000],
                vec![0; 0x20000],
                vec![0; 0x10000],
                vec![0; 0x4000],
                vec![0; 0x4000],
                vec![0; 0x8000],
                vec![0; 0x4000],
            ],
            vramcnt: [0; 9],
            pal: vec![0; 0x800],
            oam: vec![0; 0x800],
            io2d: [[0; 0x70]; 2],
            vcount: 0,
            keyinput: 0x3FF,
            extkeyin: 0x7F,
            powcnt1: 0x820F,
            ime: [false; 2],
            ie: [0; 2],
            if_: [0; 2],
            dispstat: [0; 2],
            postflg: [1; 2],
            ipcsync_in: [0; 2],
            ipcfifocnt: [0x0101; 2],
            fifo_to7: VecDeque::new(),
            fifo_to9: VecDeque::new(),
            fifo_last: [0; 2],
            divcnt: 0,
            div_num: 0,
            div_den: 0,
            div_result: 0,
            div_rem: 0,
            sqrtcnt: 0,
            sqrt_param: 0,
            sqrt_result: 0,
            stub9: build_stub(0x0080_4000),
            stub7: build_stub(0x0381_0000),
        }
    }

    pub fn request_irq(&mut self, cpu: usize, bit: u32) {
        self.if_[cpu] |= bit;
    }

    /// Resolve a 0x06xxxxxx address to (bank, offset) per VRAMCNT.
    pub fn vram_slot(&self, addr: u32) -> Option<(usize, usize)> {
        for i in 0..9 {
            let cnt = self.vramcnt[i];
            if cnt & 0x80 == 0 {
                continue;
            }
            let mst = (cnt & 7) as u32;
            let ofs = (cnt >> 3 & 3) as u32;
            let base = match (i, mst) {
                (_, 0) => LCDC_BASE[i],
                (0..=3, 1) => 0x0600_0000 + ofs * 0x20000,
                (0..=3, 2) => 0x0640_0000 + (ofs & 1) * 0x20000,
                (2, 4) => 0x0620_0000,
                (3, 4) => 0x0660_0000,
                (4, 1) => 0x0600_0000,
                (4, 2) => 0x0640_0000,
                (5 | 6, 1) => 0x0600_0000 + (ofs & 1) * 0x4000 + (ofs >> 1) * 0x10000,
                (5 | 6, 2) => 0x0640_0000 + (ofs & 1) * 0x4000 + (ofs >> 1) * 0x10000,
                (7, 1) => 0x0620_0000,
                (8, 1) => 0x0620_8000,
                (8, 2) => 0x0660_0000,
                _ => continue,
            };
            let size = BANK_SIZE[i] as u32;
            if addr >= base && addr < base + size {
                return Some((i, (addr - base) as usize));
            }
        }
        None
    }

    pub fn vram_read8(&self, addr: u32) -> u8 {
        match self.vram_slot(addr) {
            Some((b, off)) => self.vram[b][off],
            None => 0,
        }
    }

    fn div_execute(&mut self) {
        let mode = self.divcnt & 3;
        let (num, den): (i64, i64) = match mode {
            0 => (self.div_num as u32 as i32 as i64, self.div_den as u32 as i32 as i64),
            1 | 3 => (self.div_num as i64, self.div_den as u32 as i32 as i64),
            _ => (self.div_num as i64, self.div_den as i64),
        };
        self.divcnt &= !0x4000;
        if den == 0 {
            self.divcnt |= 0x4000; // div-by-zero flag
            self.div_rem = num as u64;
            let q = if num < 0 { 1i64 } else { -1i64 } as u64;
            self.div_result = if mode == 0 { q ^ 0xFFFF_FFFF_0000_0000 } else { q };
        } else {
            self.div_result = num.wrapping_div(den) as u64;
            self.div_rem = num.wrapping_rem(den) as u64;
        }
    }

    fn sqrt_execute(&mut self) {
        let v = if self.sqrtcnt & 1 != 0 { self.sqrt_param } else { self.sqrt_param as u32 as u64 };
        self.sqrt_result = (v as f64).sqrt() as u32;
        // Integer-exact fixup.
        while (self.sqrt_result as u64 + 1) * (self.sqrt_result as u64 + 1) <= v {
            self.sqrt_result += 1;
        }
        while (self.sqrt_result as u64) * (self.sqrt_result as u64) > v {
            self.sqrt_result -= 1;
        }
    }

    /// Shared WRAM mapping for a CPU; None = unmapped for that CPU.
    fn swram_off(&self, cpu: usize, addr: u32) -> Option<usize> {
        let a = addr as usize;
        match (self.wramcnt & 3, cpu) {
            (0, 0) => Some(a & 0x7FFF),
            (1, 0) => Some(0x4000 + (a & 0x3FFF)),
            (2, 0) => Some(a & 0x3FFF),
            (3, 0) => None,
            (0, _) => None,
            (1, _) => Some(a & 0x3FFF),
            (2, _) => Some(0x4000 + (a & 0x3FFF)),
            _ => Some(a & 0x7FFF),
        }
    }
}

pub struct View {
    pub m: Rc<RefCell<Machine>>,
    pub cpu: usize, // 0 = ARM9, 1 = ARM7
}

impl View {
    fn io_read16(&mut self, a: u32) -> u16 {
        let mut m = self.m.borrow_mut();
        let cpu = self.cpu;
        let off = a & 0xFFFF;
        match off {
            0x0004 | 0x0006 => match off {
                0x0004 => m.dispstat[cpu],
                _ => m.vcount,
            },
            0x0000..=0x006F if cpu == 0 => {
                let io = &m.io2d[0];
                u16::from_le_bytes([io[off as usize], io[off as usize + 1]])
            }
            0x1000..=0x106F if cpu == 0 => {
                let io = &m.io2d[1];
                let o = (off - 0x1000) as usize;
                u16::from_le_bytes([io[o], io[o + 1]])
            }
            0x0004 => m.dispstat[cpu],
            0x0006 => m.vcount,
            0x0130 => m.keyinput,
            0x0136 if cpu == 1 => m.extkeyin,
            0x0180 => {
                // Bits 0-3: the other CPU's output; bits 8-11: our own.
                (m.ipcsync_in[1 - cpu] & 0xF) | (m.ipcsync_in[cpu] & 0xF) << 8
            }
            0x0184 => {
                let (send, recv) = if cpu == 0 {
                    (&m.fifo_to7, &m.fifo_to9)
                } else {
                    (&m.fifo_to9, &m.fifo_to7)
                };
                let mut v = m.ipcfifocnt[cpu] & 0x8404; // enable, error, irq bits
                if send.is_empty() {
                    v |= 1;
                }
                if send.len() >= 16 {
                    v |= 2;
                }
                if recv.is_empty() {
                    v |= 0x100;
                }
                if recv.len() >= 16 {
                    v |= 0x200;
                }
                v
            }
            0x0208 => m.ime[cpu] as u16,
            0x0210 => m.ie[cpu] as u16,
            0x0212 => (m.ie[cpu] >> 16) as u16,
            0x0214 => m.if_[cpu] as u16,
            0x0216 => (m.if_[cpu] >> 16) as u16,
            0x0240 if cpu == 0 => u16::from_le_bytes([m.vramcnt[0], m.vramcnt[1]]),
            0x0242 if cpu == 0 => u16::from_le_bytes([m.vramcnt[2], m.vramcnt[3]]),
            0x0244 if cpu == 0 => u16::from_le_bytes([m.vramcnt[4], m.vramcnt[5]]),
            0x0246 if cpu == 0 => u16::from_le_bytes([m.vramcnt[6], m.wramcnt]),
            0x0248 if cpu == 0 => u16::from_le_bytes([m.vramcnt[7], m.vramcnt[8]]),
            0x0240 if cpu == 1 => {
                // VRAMSTAT | WRAMSTAT
                let c_arm7 = (m.vramcnt[2] & 0x87 == 0x82) as u16;
                let d_arm7 = (m.vramcnt[3] & 0x87 == 0x82) as u16;
                c_arm7 | d_arm7 << 1 | (m.wramcnt as u16 & 3) << 8
            }
            0x0280 if cpu == 0 => m.divcnt,
            0x0290..=0x0297 if cpu == 0 => (m.div_num >> ((off - 0x290) * 8)) as u16,
            0x0298..=0x029F if cpu == 0 => (m.div_den >> ((off - 0x298) * 8)) as u16,
            0x02A0..=0x02A7 if cpu == 0 => (m.div_result >> ((off - 0x2A0) * 8)) as u16,
            0x02A8..=0x02AF if cpu == 0 => (m.div_rem >> ((off - 0x2A8) * 8)) as u16,
            0x02B0 if cpu == 0 => m.sqrtcnt,
            0x02B4 if cpu == 0 => m.sqrt_result as u16,
            0x02B6 if cpu == 0 => (m.sqrt_result >> 16) as u16,
            0x02B8..=0x02BF if cpu == 0 => (m.sqrt_param >> ((off - 0x2B8) * 8)) as u16,
            0x0300 => m.postflg[cpu] as u16,
            0x0304 => m.powcnt1 as u16,
            0x0138 if cpu == 1 => 0, // RTC stub
            0x01C0 | 0x01C2 if cpu == 1 => 0, // SPI stub
            _ => 0,
        }
    }

    fn io_write16(&mut self, a: u32, v: u16) {
        let mut m = self.m.borrow_mut();
        let cpu = self.cpu;
        let off = a & 0xFFFF;
        match off {
            0x0004 => {
                m.dispstat[cpu] = (m.dispstat[cpu] & 0x0047) | (v & !0x0047);
            }
            0x0000..=0x006F if cpu == 0 => {
                let o = off as usize;
                m.io2d[0][o..o + 2].copy_from_slice(&v.to_le_bytes());
            }
            0x1000..=0x106F if cpu == 0 => {
                let o = (off - 0x1000) as usize;
                m.io2d[1][o..o + 2].copy_from_slice(&v.to_le_bytes());
            }
            0x0004 => {
                m.dispstat[cpu] = (m.dispstat[cpu] & 0x0047) | (v & !0x0047);
            }
            0x0180 => {
                m.ipcsync_in[cpu] = v >> 8 & 0xF; // our output bits, read by the other CPU
                if v & 0x2000 != 0 {
                    // remote IRQ request
                    let other = 1 - cpu;
                    m.request_irq(other, 1 << 16);
                }
            }
            0x0184 => {
                m.ipcfifocnt[cpu] = (m.ipcfifocnt[cpu] & !0x8404) | (v & 0x8404);
                if v & 0x4000 != 0 {
                    m.ipcfifocnt[cpu] &= !0x4000; // ack error
                }
                if v & 0x0008 != 0 {
                    if cpu == 0 {
                        m.fifo_to7.clear();
                    } else {
                        m.fifo_to9.clear();
                    }
                }
            }
            0x0208 => m.ime[cpu] = v & 1 != 0,
            0x0210 => m.ie[cpu] = (m.ie[cpu] & 0xFFFF_0000) | v as u32,
            0x0212 => m.ie[cpu] = (m.ie[cpu] & 0xFFFF) | (v as u32) << 16,
            0x0214 => m.if_[cpu] &= !(v as u32),
            0x0216 => m.if_[cpu] &= !((v as u32) << 16),
            0x0240 if cpu == 0 => {
                m.vramcnt[0] = v as u8;
                m.vramcnt[1] = (v >> 8) as u8;
            }
            0x0242 if cpu == 0 => {
                m.vramcnt[2] = v as u8;
                m.vramcnt[3] = (v >> 8) as u8;
            }
            0x0244 if cpu == 0 => {
                m.vramcnt[4] = v as u8;
                m.vramcnt[5] = (v >> 8) as u8;
            }
            0x0246 if cpu == 0 => {
                m.vramcnt[6] = v as u8;
                m.wramcnt = (v >> 8) as u8;
            }
            0x0248 if cpu == 0 => {
                m.vramcnt[7] = v as u8;
                m.vramcnt[8] = (v >> 8) as u8;
            }
            0x0280 if cpu == 0 => {
                m.divcnt = v & 3;
                m.div_execute();
            }
            0x02B0 if cpu == 0 => {
                m.sqrtcnt = v & 1;
                m.sqrt_execute();
            }
            0x0304 => m.powcnt1 = v as u32,
            _ => {}
        }
    }

    fn io_read32(&mut self, a: u32) -> u32 {
        if a & 0xFFFFF == 0x0184 {
            return self.io_read16(a) as u32;
        }
        if a & 0xFFFFF == 0x10_0000 {
            // IPCFIFORECV
            let mut m = self.m.borrow_mut();
            let cpu = self.cpu;
            let enabled = m.ipcfifocnt[cpu] & 0x8000 != 0;
            let (recv, last) = if cpu == 0 {
                (&mut m.fifo_to9, 0)
            } else {
                (&mut m.fifo_to7, 1)
            };
            if let Some(v) = if enabled { recv.pop_front() } else { recv.front().copied() } {
                m.fifo_last[last] = v;
                // Sender's "send empty" IRQ.
                let sender = 1 - cpu;
                let empty = if cpu == 0 { m.fifo_to9.is_empty() } else { m.fifo_to7.is_empty() };
                if empty && m.ipcfifocnt[sender] & 0x0004 != 0 {
                    m.request_irq(sender, IRQ_IPC_SEND_EMPTY);
                }
                v
            } else {
                let cpu = self.cpu;
                m.ipcfifocnt[cpu] |= 0x4000; // error: read on empty
                m.fifo_last[last]
            }
        } else {
            let lo = self.io_read16(a) as u32;
            let hi = self.io_read16(a + 2) as u32;
            lo | hi << 16
        }
    }

    fn io_write32(&mut self, a: u32, v: u32) {
        match a & 0xF_FFFF {
            0x0188 => {
                // IPCFIFOSEND
                let mut m = self.m.borrow_mut();
                let cpu = self.cpu;
                if m.ipcfifocnt[cpu] & 0x8000 == 0 {
                    return;
                }
                let receiver = 1 - cpu;
                let q = if cpu == 0 { &mut m.fifo_to7 } else { &mut m.fifo_to9 };
                let was_empty = q.is_empty();
                if q.len() >= 16 {
                    m.ipcfifocnt[cpu] |= 0x4000;
                } else {
                    q.push_back(v);
                    if was_empty && m.ipcfifocnt[receiver] & 0x0400 != 0 {
                        m.request_irq(receiver, IRQ_IPC_RECV);
                    }
                }
            }
            0x0208 => {
                let cpu = self.cpu;
                self.m.borrow_mut().ime[cpu] = v & 1 != 0;
            }
            0x0210 => {
                let cpu = self.cpu;
                self.m.borrow_mut().ie[cpu] = v;
            }
            0x0214 => {
                let cpu = self.cpu;
                self.m.borrow_mut().if_[cpu] &= !v;
            }
            0x0290 if self.cpu == 0 => {
                let mut m = self.m.borrow_mut();
                m.div_num = (m.div_num & 0xFFFF_FFFF_0000_0000) | v as u64;
                m.div_execute();
            }
            0x0294 if self.cpu == 0 => {
                let mut m = self.m.borrow_mut();
                m.div_num = (m.div_num & 0xFFFF_FFFF) | (v as u64) << 32;
                m.div_execute();
            }
            0x0298 if self.cpu == 0 => {
                let mut m = self.m.borrow_mut();
                m.div_den = (m.div_den & 0xFFFF_FFFF_0000_0000) | v as u64;
                m.div_execute();
            }
            0x029C if self.cpu == 0 => {
                let mut m = self.m.borrow_mut();
                m.div_den = (m.div_den & 0xFFFF_FFFF) | (v as u64) << 32;
                m.div_execute();
            }
            0x02B8 if self.cpu == 0 => {
                let mut m = self.m.borrow_mut();
                m.sqrt_param = (m.sqrt_param & 0xFFFF_FFFF_0000_0000) | v as u64;
                m.sqrt_execute();
            }
            0x02BC if self.cpu == 0 => {
                let mut m = self.m.borrow_mut();
                m.sqrt_param = (m.sqrt_param & 0xFFFF_FFFF) | (v as u64) << 32;
                m.sqrt_execute();
            }
            _ => {
                self.io_write16(a, v as u16);
                self.io_write16(a + 2, (v >> 16) as u16);
            }
        }
    }

    /// Byte access to plain memory regions; returns None when the address is
    /// I/O (handled at 16/32-bit granularity) or unmapped.
    fn mem_read8(&mut self, a: u32) -> u8 {
        let m = self.m.borrow();
        let cpu = self.cpu;
        if cpu == 0 {
            // TCMs first: they shadow everything else.
            if a >= m.dtcm_base && a < m.dtcm_base + 0x4000 {
                return m.dtcm[(a - m.dtcm_base) as usize];
            }
            if a < 0x0200_0000 {
                return m.itcm[(a & 0x7FFF) as usize];
            }
        }
        match a >> 24 {
            0x00 | 0x01 if cpu == 1 => {
                // ARM7 BIOS region: HLE stub only.
                *m.stub7.get((a & 0x3FFF) as usize).unwrap_or(&0)
            }
            0x02 => m.main_ram[(a & 0x3F_FFFF) as usize],
            0x03 => {
                if cpu == 1 && a >= 0x0380_0000 {
                    m.wram7[(a & 0xFFFF) as usize]
                } else {
                    match m.swram_off(cpu, a) {
                        Some(off) => m.swram[off],
                        None if cpu == 1 => m.wram7[(a & 0xFFFF) as usize],
                        None => 0,
                    }
                }
            }
            0x05 if cpu == 0 => m.pal[(a & 0x7FF) as usize],
            0x06 => {
                if cpu == 0 {
                    m.vram_read8(a)
                } else {
                    0 // ARM7 VRAM (banks C/D as WRAM): not yet mapped
                }
            }
            0x07 if cpu == 0 => m.oam[(a & 0x7FF) as usize],
            0x08 | 0x09 => 0xFF, // GBA slot, open
            0xFF if cpu == 0 => *m.stub9.get((a & 0x3FFF) as usize & 0x3F).unwrap_or(&0),
            _ => 0,
        }
    }

    fn mem_write8(&mut self, a: u32, v: u8) {
        let mut m = self.m.borrow_mut();
        let cpu = self.cpu;
        if cpu == 0 {
            if a >= m.dtcm_base && a < m.dtcm_base + 0x4000 {
                let off = (a - m.dtcm_base) as usize;
                m.dtcm[off] = v;
                return;
            }
            if a < 0x0200_0000 {
                let off = (a & 0x7FFF) as usize;
                m.itcm[off] = v;
                return;
            }
        }
        match a >> 24 {
            0x02 => m.main_ram[(a & 0x3F_FFFF) as usize] = v,
            0x03 => {
                if cpu == 1 && a >= 0x0380_0000 {
                    m.wram7[(a & 0xFFFF) as usize] = v;
                } else {
                    match m.swram_off(cpu, a) {
                        Some(off) => m.swram[off] = v,
                        None if cpu == 1 => m.wram7[(a & 0xFFFF) as usize] = v,
                        None => {}
                    }
                }
            }
            0x05 if cpu == 0 => m.pal[(a & 0x7FF) as usize] = v,
            0x06 if cpu == 0 => {
                if let Some((b, off)) = m.vram_slot(a) {
                    m.vram[b][off] = v;
                }
            }
            0x07 if cpu == 0 => m.oam[(a & 0x7FF) as usize] = v,
            _ => {}
        }
    }

    fn is_io(&self, a: u32) -> bool {
        a >> 24 == 0x04
    }
}

impl Bus for View {
    fn read8(&mut self, a: u32) -> u8 {
        if self.is_io(a) {
            let v = self.io_read16(a & !1);
            (v >> ((a & 1) * 8)) as u8
        } else {
            self.mem_read8(a)
        }
    }

    fn read16(&mut self, a: u32) -> u16 {
        let a = a & !1;
        if self.is_io(a) {
            self.io_read16(a)
        } else {
            u16::from_le_bytes([self.mem_read8(a), self.mem_read8(a + 1)])
        }
    }

    fn read32(&mut self, a: u32) -> u32 {
        let a = a & !3;
        if self.is_io(a) {
            self.io_read32(a)
        } else {
            u32::from_le_bytes([
                self.mem_read8(a),
                self.mem_read8(a + 1),
                self.mem_read8(a + 2),
                self.mem_read8(a + 3),
            ])
        }
    }

    fn write8(&mut self, a: u32, v: u8) {
        if self.is_io(a) {
            // Byte I/O writes: read-modify-write the 16-bit register.
            let cur = self.io_read16(a & !1);
            let nv = if a & 1 == 0 {
                (cur & 0xFF00) | v as u16
            } else {
                (cur & 0x00FF) | (v as u16) << 8
            };
            self.io_write16(a & !1, nv);
        } else {
            self.mem_write8(a, v);
        }
    }

    fn write16(&mut self, a: u32, v: u16) {
        let a = a & !1;
        if self.is_io(a) {
            self.io_write16(a, v);
        } else {
            self.mem_write8(a, v as u8);
            self.mem_write8(a + 1, (v >> 8) as u8);
        }
    }

    fn write32(&mut self, a: u32, v: u32) {
        let a = a & !3;
        if self.is_io(a) {
            self.io_write32(a, v);
        } else {
            self.mem_write8(a, v as u8);
            self.mem_write8(a + 1, (v >> 8) as u8);
            self.mem_write8(a + 2, (v >> 16) as u8);
            self.mem_write8(a + 3, (v >> 24) as u8);
        }
    }

    fn irq_pending(&mut self) -> bool {
        let m = self.m.borrow();
        m.ie[self.cpu] & m.if_[self.cpu] != 0
    }

    fn ime(&mut self) -> bool {
        self.m.borrow().ime[self.cpu]
    }

    fn set_ime(&mut self, on: bool) {
        let cpu = self.cpu;
        self.m.borrow_mut().ime[cpu] = on;
    }

    fn bios_flag_addr(&mut self) -> u32 {
        if self.cpu == 0 {
            self.m.borrow().dtcm_base + 0x3FF8
        } else {
            0x0380_FFF8
        }
    }

    fn set_dtcm(&mut self, base: u32) {
        let mut m = self.m.borrow_mut();
        m.dtcm_base = base;
        // Re-point the IRQ stub's handler literal at the new DTCM top.
        let lit = base.wrapping_add(0x4000);
        m.stub9[0x30..0x34].copy_from_slice(&lit.to_le_bytes());
    }
}
