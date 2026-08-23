use crate::bus::Bus;

/// Env-gated debug switches. Read ONCE: `std::env::var` scans the whole
/// environment and allocates, which is far too slow for the SWI/decode paths
/// these guard (they were costing real frame time).
static SWILOG: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_SWILOG").is_ok());
static STRICT: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_STRICT").is_ok());
static BREAKSTACK: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_BREAKSTACK").is_ok());

/// CPSR flag bits.
const N: u32 = 1 << 31;
const Z: u32 = 1 << 30;
const C: u32 = 1 << 29;
const V: u32 = 1 << 28;
const Q: u32 = 1 << 27;
const T: u32 = 1 << 5; // Thumb state

/// One ARM core, generic over its bus view. With `arm9` set it executes the
/// ARMv5TE extensions (BLX, CLZ, saturating ops, halfword multiplies,
/// LDRD/STRD, CP15) on top of the ARMv4T base; the NDS ARM7 uses the base
/// set only. No pipeline emulation: r15 reads as PC+8 (ARM) / PC+4 (Thumb).
/// Everything about a core that belongs in a savestate. Kept in its own
/// struct so the savestate can derive Serialize/Deserialize over it: the
/// remaining `Cpu` fields are the bus view (shared machine handle, not state)
/// and the debug breakpoint (environment config).
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct CpuState {
    pub regs: [u32; 16],
    pub cpsr: u32,
    spsr: u32,
    bank_usr: [u32; 2], // r13, r14
    bank_fiq: [u32; 7], // r8-r14
    bank_svc: [u32; 2],
    bank_abt: [u32; 2],
    bank_irq: [u32; 2],
    bank_und: [u32; 2],
    bank_fiq_usr: [u32; 5],
    spsr_fiq: u32,
    spsr_svc: u32,
    spsr_abt: u32,
    spsr_irq: u32,
    spsr_und: u32,
    pub halted: bool,
    /// Steps to idle (WaitByLoop HLE): real code relies on this delay for
    /// cross-CPU handshakes, so it must consume scheduler time.
    spin: u32,
    /// Active IntrWait target mask: re-halts until the BIOS flag word (top of
    /// DTCM / ARM7 WRAM) has one of these bits set by the game's IRQ handler.
    intr_wait: Option<u32>,
    pub arm9: bool,
    /// Exception vector base: 0xFFFF_0000 on the ARM9 (CP15 high vectors,
    /// the NDS default), 0 on the ARM7.
    vec_base: u32,
    // CP15 (ARM9 only).
    cp15_control: u32,
    pub cp15_dtcm: u32,
    cp15_itcm: u32,
}

pub struct Cpu<B: Bus> {
    pub st: CpuState,
    brk: Option<u32>,
    pub bus: B,
}

impl<B: Bus> Cpu<B> {
    pub fn new(bus: B, arm9: bool, entry: u32, sp: u32, sp_irq: u32, sp_svc: u32) -> Self {
        let mut regs = [0u32; 16];
        regs[13] = sp;
        regs[15] = entry;
        Self {
            st: CpuState {
                regs,
                cpsr: 0x1F, // system mode, ARM state
                spsr: 0,
                bank_usr: [sp, 0],
                bank_fiq: [0; 7],
                bank_svc: [sp_svc, 0],
                bank_abt: [0; 2],
                bank_irq: [sp_irq, 0],
                bank_und: [0; 2],
                bank_fiq_usr: [0; 5],
                spsr_fiq: 0,
                spsr_svc: 0,
                spsr_abt: 0,
                spsr_irq: 0,
                spsr_und: 0,
                halted: false,
                spin: 0,
                intr_wait: None,
                arm9,
                vec_base: if arm9 { 0xFFFF_0000 } else { 0 },
                cp15_control: 0x0005_2078,
                cp15_dtcm: 0,
                cp15_itcm: 0,
            },
            brk: std::env::var("NDS_BREAK")
                .ok()
                .and_then(|v| u32::from_str_radix(v.trim_start_matches("0x"), 16).ok()),
            bus,
        }
    }

    /// Overwrite the core with a savestate's registers. The bus view and the
    /// debug breakpoint stay as they are.
    pub fn restore(&mut self, st: CpuState) {
        self.st = st;
    }

    fn thumb(&self) -> bool {
        self.st.cpsr & T != 0
    }

    fn flag(&self, f: u32) -> bool {
        self.st.cpsr & f != 0
    }

    fn set_flag(&mut self, f: u32, on: bool) {
        if on { self.st.cpsr |= f } else { self.st.cpsr &= !f }
    }

    fn set_nz(&mut self, v: u32) {
        self.set_flag(N, v & 0x8000_0000 != 0);
        self.set_flag(Z, v == 0);
    }

    fn r(&self, i: u32) -> u32 {
        if i == 15 {
            self.st.regs[15].wrapping_add(if self.thumb() { 4 } else { 8 })
        } else {
            self.st.regs[i as usize]
        }
    }

    fn set_r(&mut self, i: u32, v: u32) {
        if i == 15 {
            self.st.regs[15] = v & if self.thumb() { !1 } else { !3 };
        } else {
            self.st.regs[i as usize] = v;
        }
    }

    fn mode(&self) -> u32 {
        self.st.cpsr & 0x1F
    }

    fn switch_mode(&mut self, old: u32) {
        let new = self.mode();
        if old == new || (old | new) & !0xF == 0x10 && (old & 0xF) == (new & 0xF) {
            return;
        }
        match old {
            0x11 => {
                self.st.bank_fiq.copy_from_slice(&self.st.regs[8..15]);
                self.st.regs[8..13].copy_from_slice(&self.st.bank_fiq_usr);
            }
            0x13 => self.st.bank_svc.copy_from_slice(&self.st.regs[13..15]),
            0x17 => self.st.bank_abt.copy_from_slice(&self.st.regs[13..15]),
            0x12 => self.st.bank_irq.copy_from_slice(&self.st.regs[13..15]),
            0x1B => self.st.bank_und.copy_from_slice(&self.st.regs[13..15]),
            _ => self.st.bank_usr.copy_from_slice(&self.st.regs[13..15]),
        }
        match new {
            0x11 => {
                self.st.bank_fiq_usr.copy_from_slice(&self.st.regs[8..13]);
                self.st.regs[8..15].copy_from_slice(&self.st.bank_fiq);
            }
            0x13 => self.st.regs[13..15].copy_from_slice(&self.st.bank_svc),
            0x17 => self.st.regs[13..15].copy_from_slice(&self.st.bank_abt),
            0x12 => self.st.regs[13..15].copy_from_slice(&self.st.bank_irq),
            0x1B => self.st.regs[13..15].copy_from_slice(&self.st.bank_und),
            _ => self.st.regs[13..15].copy_from_slice(&self.st.bank_usr),
        }
    }

    fn spsr_for_mode(&mut self) -> &mut u32 {
        match self.mode() {
            0x11 => &mut self.st.spsr_fiq,
            0x13 => &mut self.st.spsr_svc,
            0x17 => &mut self.st.spsr_abt,
            0x12 => &mut self.st.spsr_irq,
            0x1B => &mut self.st.spsr_und,
            _ => &mut self.st.spsr,
        }
    }

    fn user_reg(&self, i: u32) -> u32 {
        let m = self.mode();
        match i {
            8..=12 if m == 0x11 => self.st.bank_fiq_usr[i as usize - 8],
            13 | 14 if m != 0x10 && m != 0x1F => self.st.bank_usr[i as usize - 13],
            _ => self.st.regs[i as usize],
        }
    }

    fn set_user_reg(&mut self, i: u32, v: u32) {
        let m = self.mode();
        match i {
            8..=12 if m == 0x11 => self.st.bank_fiq_usr[i as usize - 8] = v,
            13 | 14 if m != 0x10 && m != 0x1F => self.st.bank_usr[i as usize - 13] = v,
            _ => self.st.regs[i as usize] = v,
        }
    }

    fn cond(&self, c: u32) -> bool {
        match c {
            0x0 => self.flag(Z),
            0x1 => !self.flag(Z),
            0x2 => self.flag(C),
            0x3 => !self.flag(C),
            0x4 => self.flag(N),
            0x5 => !self.flag(N),
            0x6 => self.flag(V),
            0x7 => !self.flag(V),
            0x8 => self.flag(C) && !self.flag(Z),
            0x9 => !self.flag(C) || self.flag(Z),
            0xA => self.flag(N) == self.flag(V),
            0xB => self.flag(N) != self.flag(V),
            0xC => !self.flag(Z) && self.flag(N) == self.flag(V),
            0xD => self.flag(Z) || self.flag(N) != self.flag(V),
            _ => true,
        }
    }

    fn shift(&self, ty: u32, val: u32, amount: u32, imm: bool) -> (u32, bool) {
        let c_in = self.flag(C);
        match ty {
            0 => {
                if amount == 0 {
                    (val, c_in)
                } else if amount < 32 {
                    (val << amount, val >> (32 - amount) & 1 != 0)
                } else if amount == 32 {
                    (0, val & 1 != 0)
                } else {
                    (0, false)
                }
            }
            1 => {
                let amount = if imm && amount == 0 { 32 } else { amount };
                if amount == 0 {
                    (val, c_in)
                } else if amount < 32 {
                    (val >> amount, val >> (amount - 1) & 1 != 0)
                } else if amount == 32 {
                    (0, val >> 31 != 0)
                } else {
                    (0, false)
                }
            }
            2 => {
                let amount = if imm && amount == 0 { 32 } else { amount };
                if amount == 0 {
                    (val, c_in)
                } else if amount < 32 {
                    (((val as i32) >> amount) as u32, val >> (amount - 1) & 1 != 0)
                } else {
                    let fill = if val >> 31 != 0 { u32::MAX } else { 0 };
                    (fill, val >> 31 != 0)
                }
            }
            _ => {
                if imm && amount == 0 {
                    ((val >> 1) | ((c_in as u32) << 31), val & 1 != 0)
                } else if amount == 0 {
                    (val, c_in)
                } else {
                    let a = amount & 31;
                    if a == 0 {
                        (val, val >> 31 != 0)
                    } else {
                        (val.rotate_right(a), val >> (a - 1) & 1 != 0)
                    }
                }
            }
        }
    }

    fn add_with_flags(&mut self, a: u32, b: u32, carry: u32, set: bool) -> u32 {
        let r64 = a as u64 + b as u64 + carry as u64;
        let r = r64 as u32;
        if set {
            self.set_nz(r);
            self.set_flag(C, r64 > 0xFFFF_FFFF);
            self.set_flag(V, (!(a ^ b) & (a ^ r)) >> 31 != 0);
        }
        r
    }

    fn sub_with_flags(&mut self, a: u32, b: u32, carry: u32, set: bool) -> u32 {
        self.add_with_flags(a, !b, carry, set)
    }

    /// Execute up to `n` instructions, collapsing idle time.
    ///
    /// A halted core with nothing pending, or one spinning out a WaitByLoop,
    /// has no work for the whole slice, yet `step()` still polls the bus
    /// (through RefCell) once per call. Platinum's ARM9 sits halted for most
    /// of every frame, so those polls dominated emulation time: skipping the
    /// slice wholesale is the single biggest speed win available. The slice
    /// length is unchanged, so cross-CPU wake latency is exactly as before.
    pub fn run_slice(&mut self, n: u32) {
        if self.st.spin >= n {
            self.st.spin -= n;
            return;
        }
        if self.st.spin == 0 && self.st.halted && !self.bus.irq_pending() {
            // The IntrWait flag is only ever set by this core's own IRQ
            // handler, which cannot run while nothing is pending.
            return;
        }
        for _ in 0..n {
            self.step();
        }
    }

    pub fn step(&mut self) {
        if self.st.spin > 0 {
            self.st.spin -= 1;
            return;
        }
        if let Some(mask) = self.st.intr_wait {
            if self.mode() != 0x12 {
                let flag_addr = self.bus.bios_flag_addr();
                let flags = self.bus.read32(flag_addr);
                if flags & mask != 0 {
                    self.bus.write32(flag_addr, flags & !mask);
                    self.st.intr_wait = None;
                } else {
                    self.st.halted = true;
                }
            }
        }
        let pending = self.bus.irq_pending();
        if pending {
            self.st.halted = false;
            if self.bus.ime() && self.st.cpsr & 0x80 == 0 {
                let old_cpsr = self.st.cpsr;
                let old_mode = self.mode();
                let ret = self.st.regs[15].wrapping_add(4);
                self.st.cpsr = (self.st.cpsr & !0x3F) | 0x12 | 0x80; // IRQ mode, ARM, I set
                self.switch_mode(old_mode);
                *self.spsr_for_mode() = old_cpsr;
                self.st.regs[14] = ret;
                self.st.regs[15] = self.st.vec_base + 0x18;
                return;
            }
        }
        if self.st.halted {
            return;
        }
        self.bus.note_pc(self.st.regs[15]);
        if self.brk == Some(self.st.regs[15]) {
            // NDS_BREAKSTACK=1 appends the top of the stack, which is how you
            // recover a call chain: the saved link registers sitting there
            // name the callers that the bare lr cannot.
            let stack = if *BREAKSTACK {
                let sp = self.st.regs[13];
                let mut s = String::from(" stack:");
                for i in 0..10 {
                    s += &format!(" {:08X}", self.bus.read32(sp + i * 4));
                }
                s
            } else {
                String::new()
            };
            eprintln!(
                "BREAK [{}] pc={:#010X} r0={:#010X} r1={:#010X} r2={:#010X} r3={:#010X} lr={:#010X} sp={:#010X}{}",
                if self.st.arm9 { "9" } else { "7" },
                self.st.regs[15], self.st.regs[0], self.st.regs[1], self.st.regs[2], self.st.regs[3],
                self.st.regs[14], self.st.regs[13], stack
            );
        }
        if self.thumb() {
            let op = self.bus.read16(self.st.regs[15]);
            let pc_before = self.st.regs[15];
            self.exec_thumb(op);
            if self.st.regs[15] == pc_before {
                self.st.regs[15] = self.st.regs[15].wrapping_add(2);
            }
        } else {
            let op = self.bus.read32(self.st.regs[15]);
            let pc_before = self.st.regs[15];
            let c = op >> 28;
            if c == 0xF {
                if self.st.arm9 {
                    self.exec_arm_uncond(op);
                }
            } else if self.cond(c) {
                self.exec_arm(op);
            }
            if self.st.regs[15] == pc_before {
                self.st.regs[15] = self.st.regs[15].wrapping_add(4);
            }
        }
    }

    // ===================== ARM =====================

    /// Condition field 0xF (ARMv5): BLX immediate and hint space.
    fn exec_arm_uncond(&mut self, op: u32) {
        if op & 0x0E00_0000 == 0x0A00_0000 {
            // BLX <imm>: branch, link, switch to Thumb; H bit is offset bit 1.
            let off = ((op << 8) as i32 >> 6) as u32;
            let h = (op >> 24 & 1) << 1;
            self.st.regs[14] = self.st.regs[15].wrapping_add(4);
            self.st.regs[15] = self.st.regs[15].wrapping_add(8).wrapping_add(off).wrapping_add(h);
            self.set_flag(T, true);
            self.st.regs[15] &= !1;
        }
        // PLD and other hints: no-op.
    }

    fn exec_arm(&mut self, op: u32) {
        if op & 0x0FFF_FFF0 == 0x012F_FF10 {
            // BX
            let v = self.r(op & 0xF);
            self.set_flag(T, v & 1 != 0);
            self.st.regs[15] = v & !1 & if v & 1 != 0 { !0 } else { !3 };
            return;
        }
        if self.st.arm9 && op & 0x0FFF_FFF0 == 0x012F_FF30 {
            // BLX <reg>
            let v = self.r(op & 0xF);
            self.st.regs[14] = self.st.regs[15].wrapping_add(4);
            self.set_flag(T, v & 1 != 0);
            self.st.regs[15] = v & !1 & if v & 1 != 0 { !0 } else { !3 };
            return;
        }
        if self.st.arm9 && op & 0x0FFF_0FF0 == 0x016F_0F10 {
            // CLZ
            let rm = self.r(op & 0xF);
            self.set_r(op >> 12 & 0xF, rm.leading_zeros());
            return;
        }
        if self.st.arm9 && op & 0x0F90_0FF0 == 0x0100_0050 {
            // QADD/QSUB/QDADD/QDSUB
            let rm = self.r(op & 0xF) as i32;
            let rn = self.r(op >> 16 & 0xF) as i32;
            let opc = op >> 21 & 3;
            let sat = |v: i64, cpu: &mut Self| -> i32 {
                if v > i32::MAX as i64 {
                    cpu.st.cpsr |= Q;
                    i32::MAX
                } else if v < i32::MIN as i64 {
                    cpu.st.cpsr |= Q;
                    i32::MIN
                } else {
                    v as i32
                }
            };
            let doubled = if opc >= 2 { sat(rn as i64 * 2, self) } else { rn };
            let r = if opc & 1 == 0 {
                sat(rm as i64 + doubled as i64, self)
            } else {
                sat(rm as i64 - doubled as i64, self)
            };
            self.set_r(op >> 12 & 0xF, r as u32);
            return;
        }
        if self.st.arm9 && op & 0x0F90_0090 == 0x0100_0080 {
            // SMLAxy / SMLAWy / SMULWy / SMLALxy / SMULxy
            let opc = op >> 21 & 3;
            let x = op & 0x20 != 0;
            let y = op & 0x40 != 0;
            let rm = self.r(op & 0xF);
            let rs = self.r(op >> 8 & 0xF);
            let half = |v: u32, top: bool| if top { (v >> 16) as i16 as i32 } else { v as i16 as i32 };
            match opc {
                0 => {
                    // SMLAxy
                    let p = half(rm, x).wrapping_mul(half(rs, y));
                    let acc = self.r(op >> 12 & 0xF) as i32;
                    let (r, ov) = p.overflowing_add(acc);
                    if ov {
                        self.st.cpsr |= Q;
                    }
                    self.set_r(op >> 16 & 0xF, r as u32);
                }
                1 => {
                    // SMULWy / SMLAWy
                    let p = ((rm as i32 as i64 * half(rs, y) as i64) >> 16) as i32;
                    if x {
                        self.set_r(op >> 16 & 0xF, p as u32); // SMULWy
                    } else {
                        let acc = self.r(op >> 12 & 0xF) as i32;
                        let (r, ov) = p.overflowing_add(acc);
                        if ov {
                            self.st.cpsr |= Q;
                        }
                        self.set_r(op >> 16 & 0xF, r as u32);
                    }
                }
                2 => {
                    // SMLALxy
                    let p = half(rm, x).wrapping_mul(half(rs, y)) as i64;
                    let rdhi = op >> 16 & 0xF;
                    let rdlo = op >> 12 & 0xF;
                    let acc = ((self.r(rdhi) as u64) << 32 | self.r(rdlo) as u64) as i64;
                    let r = acc.wrapping_add(p) as u64;
                    self.set_r(rdlo, r as u32);
                    self.set_r(rdhi, (r >> 32) as u32);
                }
                _ => {
                    // SMULxy
                    let p = half(rm, x).wrapping_mul(half(rs, y));
                    self.set_r(op >> 16 & 0xF, p as u32);
                }
            }
            return;
        }
        if op & 0x0E00_0000 == 0x0A00_0000 {
            // B/BL
            let off = ((op << 8) as i32 >> 6) as u32;
            if op & 0x0100_0000 != 0 {
                self.st.regs[14] = self.st.regs[15].wrapping_add(4);
            }
            self.st.regs[15] = self.st.regs[15].wrapping_add(8).wrapping_add(off);
            return;
        }
        if op & 0x0F00_0010 == 0x0E00_0010 {
            // MRC/MCR
            self.coprocessor(op);
            return;
        }
        if op & 0x0F00_0000 == 0x0C00_0000 || op & 0x0F00_0000 == 0x0D00_0000 {
            return; // LDC/STC: no coprocessor data transfers we care about
        }
        if op & 0x0FC0_00F0 == 0x0000_0090 {
            // MUL/MLA
            let rd = op >> 16 & 0xF;
            let rn = op >> 12 & 0xF;
            let rs = op >> 8 & 0xF;
            let rm = op & 0xF;
            let mut r = self.r(rm).wrapping_mul(self.r(rs));
            if op & 0x0020_0000 != 0 {
                r = r.wrapping_add(self.r(rn));
            }
            self.set_r(rd, r);
            if op & 0x0010_0000 != 0 {
                self.set_nz(r);
            }
            return;
        }
        if op & 0x0F80_00F0 == 0x0080_0090 {
            // UMULL/UMLAL/SMULL/SMLAL
            let rdhi = op >> 16 & 0xF;
            let rdlo = op >> 12 & 0xF;
            let rs = op >> 8 & 0xF;
            let rm = op & 0xF;
            let signed = op & 0x0040_0000 != 0;
            let acc = op & 0x0020_0000 != 0;
            let mut r: u64 = if signed {
                (self.r(rm) as i32 as i64).wrapping_mul(self.r(rs) as i32 as i64) as u64
            } else {
                (self.r(rm) as u64).wrapping_mul(self.r(rs) as u64)
            };
            if acc {
                r = r.wrapping_add((self.r(rdhi) as u64) << 32 | self.r(rdlo) as u64);
            }
            self.set_r(rdlo, r as u32);
            self.set_r(rdhi, (r >> 32) as u32);
            if op & 0x0010_0000 != 0 {
                self.set_flag(N, r >> 63 != 0);
                self.set_flag(Z, r == 0);
            }
            return;
        }
        if op & 0x0FB0_0FF0 == 0x0100_0090 {
            // SWP/SWPB
            let addr = self.r(op >> 16 & 0xF);
            let rm = self.r(op & 0xF);
            let rd = op >> 12 & 0xF;
            if op & 0x0040_0000 != 0 {
                let old = self.bus.read8(addr) as u32;
                self.bus.write8(addr, rm as u8);
                self.set_r(rd, old);
            } else {
                let old = self.bus.read32(addr).rotate_right((addr & 3) * 8);
                self.bus.write32(addr, rm);
                self.set_r(rd, old);
            }
            return;
        }
        if op & 0x0E00_0090 == 0x0000_0090 && op & 0x60 != 0 {
            self.arm_halfword(op);
            return;
        }
        if op & 0x0FBF_0FFF == 0x010F_0000 {
            // MRS
            let v = if op & 0x0040_0000 != 0 { *self.spsr_for_mode() } else { self.st.cpsr };
            self.set_r(op >> 12 & 0xF, v);
            return;
        }
        if op & 0x0DB0_F000 == 0x0120_F000 {
            // MSR
            let val = if op & 0x0200_0000 != 0 {
                let imm = op & 0xFF;
                imm.rotate_right((op >> 8 & 0xF) * 2)
            } else {
                self.r(op & 0xF)
            };
            let mut mask = 0u32;
            if op & 0x0008_0000 != 0 {
                mask |= 0xFF00_0000;
            }
            if op & 0x0001_0000 != 0 {
                mask |= 0x0000_00FF;
            }
            if op & 0x0040_0000 != 0 {
                let s = self.spsr_for_mode();
                *s = (*s & !mask) | (val & mask);
            } else {
                if self.mode() == 0x10 {
                    mask &= 0xFF00_0000;
                }
                let old = self.mode();
                self.st.cpsr = (self.st.cpsr & !mask) | (val & mask);
                self.switch_mode(old);
            }
            return;
        }
        if op & 0x0C00_0000 == 0x0000_0000 {
            self.arm_data_processing(op);
            return;
        }
        if op & 0x0C00_0000 == 0x0400_0000 {
            self.arm_single_transfer(op);
            return;
        }
        if op & 0x0E00_0000 == 0x0800_0000 {
            self.arm_block_transfer(op);
            return;
        }
        if op & 0x0F00_0000 == 0x0F00_0000 {
            self.hle_swi(op >> 16 & 0xFF);
            return;
        }
        if *STRICT {
            panic!("unimplemented ARM op {op:#010X} at {:#010X}", self.st.regs[15]);
        }
    }

    /// CP15 (system control) on the ARM9; everything else ignored.
    fn coprocessor(&mut self, op: u32) {
        let cp = op >> 8 & 0xF;
        let load = op & 0x0010_0000 != 0; // MRC
        let rd = op >> 12 & 0xF;
        if cp != 15 || !self.st.arm9 {
            if load {
                self.set_r(rd, 0);
            }
            return;
        }
        let crn = op >> 16 & 0xF;
        let crm = op & 0xF;
        let op2 = op >> 5 & 7;
        if load {
            let v = match (crn, crm, op2) {
                (0, 0, 0) => 0x4105_9461, // main ID: ARM946E-S
                (0, 0, 1) => 0x0F0D_2112, // cache type
                (1, 0, 0) => self.st.cp15_control,
                (9, 1, 0) => self.st.cp15_dtcm,
                (9, 1, 1) => self.st.cp15_itcm,
                _ => 0,
            };
            self.set_r(rd, v);
        } else {
            let v = self.r(rd);
            match (crn, crm, op2) {
                (1, 0, 0) => self.st.cp15_control = v,
                (9, 1, 0) => {
                    self.st.cp15_dtcm = v;
                    self.bus.set_dtcm(v & 0xFFFF_F000);
                }
                (9, 1, 1) => self.st.cp15_itcm = v,
                (7, 0, 4) | (7, 8, 2) => self.st.halted = true, // wait for interrupt
                _ => {} // cache/TLB maintenance: no-op
            }
        }
    }

    fn dp_operand2(&mut self, op: u32) -> (u32, bool) {
        if op & 0x0200_0000 != 0 {
            let imm = op & 0xFF;
            let rot = (op >> 8 & 0xF) * 2;
            if rot == 0 {
                (imm, self.flag(C))
            } else {
                let v = imm.rotate_right(rot);
                (v, v >> 31 != 0)
            }
        } else {
            let rm = op & 0xF;
            let ty = op >> 5 & 3;
            if op & 0x10 != 0 {
                let amount = self.r(op >> 8 & 0xF) & 0xFF;
                let val = if rm == 15 { self.r(15).wrapping_add(4) } else { self.r(rm) };
                self.shift(ty, val, amount, false)
            } else {
                let amount = op >> 7 & 0x1F;
                self.shift(ty, self.r(rm), amount, true)
            }
        }
    }

    fn arm_data_processing(&mut self, op: u32) {
        let opcode = op >> 21 & 0xF;
        let set = op & 0x0010_0000 != 0;
        let rn = op >> 16 & 0xF;
        let rd = op >> 12 & 0xF;
        if (0x8..=0xB).contains(&opcode) && rd == 15 && !set {
            return;
        }
        if (0x8..=0xB).contains(&opcode) && rd == 15 {
            let old = self.mode();
            self.st.cpsr = *self.spsr_for_mode();
            self.switch_mode(old);
            return;
        }
        let (op2, sh_carry) = self.dp_operand2(op);
        let a = if rn == 15 && op & 0x0200_0000 == 0 && op & 0x10 != 0 {
            self.r(15).wrapping_add(4)
        } else {
            self.r(rn)
        };
        let logical_flags = |cpu: &mut Self, r: u32| {
            cpu.set_nz(r);
            cpu.set_flag(C, sh_carry);
        };
        let c = self.flag(C) as u32;
        let result = match opcode {
            0x0 => { let r = a & op2; if set { logical_flags(self, r) } Some(r) } // AND
            0x1 => { let r = a ^ op2; if set { logical_flags(self, r) } Some(r) } // EOR
            0x2 => Some(self.sub_with_flags(a, op2, 1, set)),                     // SUB
            0x3 => Some(self.sub_with_flags(op2, a, 1, set)),                     // RSB
            0x4 => Some(self.add_with_flags(a, op2, 0, set)),                     // ADD
            0x5 => Some(self.add_with_flags(a, op2, c, set)),                     // ADC
            0x6 => Some(self.sub_with_flags(a, op2, c, set)),                     // SBC
            0x7 => Some(self.sub_with_flags(op2, a, c, set)),                     // RSC
            0x8 => { let r = a & op2; logical_flags(self, r); None }              // TST
            0x9 => { let r = a ^ op2; logical_flags(self, r); None }              // TEQ
            0xA => { self.sub_with_flags(a, op2, 1, true); None }                 // CMP
            0xB => { self.add_with_flags(a, op2, 0, true); None }                 // CMN
            0xC => { let r = a | op2; if set { logical_flags(self, r) } Some(r) } // ORR
            0xD => { let r = op2; if set { logical_flags(self, r) } Some(r) }     // MOV
            0xE => { let r = a & !op2; if set { logical_flags(self, r) } Some(r) }// BIC
            _ => { let r = !op2; if set { logical_flags(self, r) } Some(r) }      // MVN
        };
        if let Some(r) = result {
            if rd == 15 {
                if set {
                    let old = self.mode();
                    self.st.cpsr = *self.spsr_for_mode();
                    self.switch_mode(old);
                }
                self.st.regs[15] = r & if self.thumb() { !1 } else { !3 };
            } else {
                self.set_r(rd, r);
            }
        }
    }

    fn arm_single_transfer(&mut self, op: u32) {
        let rn = op >> 16 & 0xF;
        let rd = op >> 12 & 0xF;
        let offset = if op & 0x0200_0000 != 0 {
            let (v, _) = self.shift(op >> 5 & 3, self.r(op & 0xF), op >> 7 & 0x1F, true);
            v
        } else {
            op & 0xFFF
        };
        let base = self.r(rn);
        let up = op & 0x0080_0000 != 0;
        let pre = op & 0x0100_0000 != 0;
        let byte = op & 0x0040_0000 != 0;
        let load = op & 0x0010_0000 != 0;
        let wb = op & 0x0020_0000 != 0;
        let off_base = if up { base.wrapping_add(offset) } else { base.wrapping_sub(offset) };
        let addr = if pre { off_base } else { base };
        if load {
            let v = if byte {
                self.bus.read8(addr) as u32
            } else if self.st.arm9 {
                // ARM9 forces alignment (no rotate; the v5 way with U=0).
                self.bus.read32(addr & !3).rotate_right((addr & 3) * 8)
            } else {
                self.bus.read32(addr).rotate_right((addr & 3) * 8)
            };
            if !pre || wb {
                self.set_r(rn, off_base);
            }
            if rd == 15 && self.st.arm9 {
                // v5: LDR to PC switches to Thumb on bit 0.
                self.set_flag(T, v & 1 != 0);
                self.st.regs[15] = v & !1 & if v & 1 != 0 { !0 } else { !3 };
            } else {
                self.set_r(rd, v);
            }
        } else {
            let v = if rd == 15 { self.r(15).wrapping_add(4) } else { self.r(rd) };
            if byte {
                self.bus.write8(addr, v as u8);
            } else {
                self.bus.write32(addr, v);
            }
            if !pre || wb {
                self.set_r(rn, off_base);
            }
        }
    }

    fn arm_halfword(&mut self, op: u32) {
        let rn = op >> 16 & 0xF;
        let rd = op >> 12 & 0xF;
        let offset = if op & 0x0040_0000 != 0 {
            (op >> 4 & 0xF0) | (op & 0xF)
        } else {
            self.r(op & 0xF)
        };
        let base = self.r(rn);
        let up = op & 0x0080_0000 != 0;
        let pre = op & 0x0100_0000 != 0;
        let load = op & 0x0010_0000 != 0;
        let wb = op & 0x0020_0000 != 0;
        let off_base = if up { base.wrapping_add(offset) } else { base.wrapping_sub(offset) };
        let addr = if pre { off_base } else { base };
        let ty = op >> 5 & 3;
        if !load && ty >= 2 && self.st.arm9 {
            // v5TE LDRD/STRD (encoded in the store space).
            let rd2 = rd + 1;
            if ty == 2 {
                // LDRD
                let lo = self.bus.read32(addr & !3);
                let hi = self.bus.read32((addr & !3).wrapping_add(4));
                if !pre || wb {
                    self.set_r(rn, off_base);
                }
                self.set_r(rd, lo);
                self.set_r(rd2, hi);
            } else {
                // STRD
                self.bus.write32(addr & !3, self.r(rd));
                self.bus.write32((addr & !3).wrapping_add(4), self.r(rd2));
                if !pre || wb {
                    self.set_r(rn, off_base);
                }
            }
            return;
        }
        if load {
            let v = match ty {
                1 => {
                    let v = self.bus.read16(addr) as u32;
                    if self.st.arm9 { v } else { v.rotate_right((addr & 1) * 8) }
                }
                2 => self.bus.read8(addr) as i8 as i32 as u32, // LDRSB
                _ => {
                    if addr & 1 != 0 && !self.st.arm9 {
                        self.bus.read8(addr) as i8 as i32 as u32
                    } else {
                        self.bus.read16(addr) as i16 as i32 as u32
                    }
                }
            };
            if !pre || wb {
                self.set_r(rn, off_base);
            }
            self.set_r(rd, v);
        } else {
            let v = self.r(rd);
            self.bus.write16(addr, v as u16);
            if !pre || wb {
                self.set_r(rn, off_base);
            }
        }
    }

    fn arm_block_transfer(&mut self, op: u32) {
        let rn = op >> 16 & 0xF;
        let list = op & 0xFFFF;
        let load = op & 0x0010_0000 != 0;
        let wb = op & 0x0020_0000 != 0;
        let s_bit = op & 0x0040_0000 != 0;
        let up = op & 0x0080_0000 != 0;
        let pre = op & 0x0100_0000 != 0;
        let n = list.count_ones();
        let base = self.r(rn);

        if list == 0 {
            let addr = if up {
                if pre { base + 4 } else { base }
            } else if pre {
                base - 0x40
            } else {
                base - 0x3C
            };
            if load {
                self.st.regs[15] = self.bus.read32(addr) & !3;
            } else {
                self.bus.write32(addr, self.r(15).wrapping_add(4));
            }
            if wb {
                self.set_r(rn, if up { base + 0x40 } else { base - 0x40 });
            }
            return;
        }

        let start = if up {
            if pre { base.wrapping_add(4) } else { base }
        } else {
            let s = base.wrapping_sub(n * 4);
            if pre { s } else { s.wrapping_add(4) }
        };
        let new_base = if up { base.wrapping_add(n * 4) } else { base.wrapping_sub(n * 4) };
        let user_bank = s_bit && !(load && list & 0x8000 != 0);

        let mut addr = start;
        let first_reg = list.trailing_zeros();
        let in_list = list & (1 << rn) != 0;
        let last_reg = 31 - list.leading_zeros();
        if load {
            if wb {
                self.set_r(rn, new_base);
            }
            // v5 (ARM9): writeback beats the loaded value unless Rn is the
            // last (and not only) register in the list. Applied after loads.
            let v5_wb_wins = self.st.arm9 && wb && in_list && (n == 1 || rn != last_reg);
            for i in 0..16 {
                if list & (1 << i) != 0 {
                    let v = self.bus.read32(addr);
                    if user_bank && i < 15 {
                        self.set_user_reg(i, v);
                        addr = addr.wrapping_add(4);
                        continue;
                    }
                    if i == 15 {
                        if s_bit {
                            let old = self.mode();
                            self.st.cpsr = *self.spsr_for_mode();
                            self.switch_mode(old);
                        }
                        if self.st.arm9 && !s_bit {
                            self.set_flag(T, v & 1 != 0);
                        }
                        self.st.regs[15] = v & if self.thumb() { !1 } else { !3 };
                    } else {
                        self.st.regs[i as usize] = v;
                    }
                    addr = addr.wrapping_add(4);
                }
            }
            if v5_wb_wins {
                self.set_r(rn, new_base);
            }
        } else {
            for i in 0..16 {
                if list & (1 << i) != 0 {
                    let v = if i == 15 {
                        self.r(15).wrapping_add(4)
                    } else if i == rn && i != first_reg && wb {
                        // v4 stores the written-back base; v5 the old one.
                        if self.st.arm9 { base } else { new_base }
                    } else if user_bank {
                        self.user_reg(i)
                    } else {
                        self.st.regs[i as usize]
                    };
                    self.bus.write32(addr, v);
                    addr = addr.wrapping_add(4);
                }
            }
            if wb {
                self.set_r(rn, new_base);
            }
        }
    }

    /// High-level emulation of NDS BIOS calls (no BIOS images needed).
    /// SWI numbers follow the NDS BIOS table, which differs from GBA.
    fn hle_swi(&mut self, n: u32) {
        if *SWILOG {
            eprintln!(
                "[{}] swi {:#04X} r0={:#010X} r1={:#010X} r2={:#010X} lr={:#010X}",
                if self.st.arm9 { "9" } else { "7" },
                n, self.st.regs[0], self.st.regs[1], self.st.regs[2], self.st.regs[14]
            );
        }
        match n {
            0x03 => { // WaitByLoop: r0 = loop count (4 cycles each); idle for it
                self.st.spin = self.st.regs[0].min(1_000_000);
                self.st.regs[0] = 0;
            }
            0x04 => { // IntrWait(discard_old, mask)
                let mask = self.st.regs[1];
                let flag_addr = self.bus.bios_flag_addr();
                if self.st.regs[0] != 0 {
                    let flags = self.bus.read32(flag_addr);
                    self.bus.write32(flag_addr, flags & !mask);
                }
                self.st.intr_wait = Some(mask);
                self.bus.set_ime(true);
            }
            0x05 => { // VBlankIntrWait = IntrWait(1, vblank)
                let flag_addr = self.bus.bios_flag_addr();
                let flags = self.bus.read32(flag_addr);
                self.bus.write32(flag_addr, flags & !1);
                self.st.intr_wait = Some(1);
                self.bus.set_ime(true);
            }
            0x06 | 0x07 => self.st.halted = true, // Halt / Sleep(ARM7)
            0x09 => { // Div: r0/r1 -> r0=quot, r1=rem, r3=|quot|
                let num = self.st.regs[0] as i32;
                let den = self.st.regs[1] as i32;
                if den != 0 {
                    let q = num.wrapping_div(den);
                    self.st.regs[0] = q as u32;
                    self.st.regs[1] = num.wrapping_rem(den) as u32;
                    self.st.regs[3] = q.unsigned_abs();
                }
            }
            0x0B => { // CpuSet
                let src = self.st.regs[0];
                let dst = self.st.regs[1];
                let cnt = self.st.regs[2];
                let count = cnt & 0x1F_FFFF;
                let fill = cnt & 0x0100_0000 != 0;
                if cnt & 0x0400_0000 != 0 {
                    let v0 = self.bus.read32(src);
                    for i in 0..count {
                        let v = if fill { v0 } else { self.bus.read32(src + i * 4) };
                        self.bus.write32(dst + i * 4, v);
                    }
                } else {
                    let v0 = self.bus.read16(src);
                    for i in 0..count {
                        let v = if fill { v0 } else { self.bus.read16(src + i * 2) };
                        self.bus.write16(dst + i * 2, v);
                    }
                }
            }
            0x0C => { // CpuFastSet
                let src = self.st.regs[0];
                let dst = self.st.regs[1];
                let cnt = self.st.regs[2];
                let count = ((cnt & 0x1F_FFFF) + 7) & !7;
                let fill = cnt & 0x0100_0000 != 0;
                let v0 = self.bus.read32(src);
                for i in 0..count {
                    let v = if fill { v0 } else { self.bus.read32(src + i * 4) };
                    self.bus.write32(dst + i * 4, v);
                }
            }
            0x0D => self.st.regs[0] = (self.st.regs[0] as f64).sqrt() as u32, // Sqrt
            0x0E => { // GetCRC16(initial, addr, len)
                let mut crc = self.st.regs[0] as u16;
                let addr = self.st.regs[1];
                let len = self.st.regs[2];
                for i in 0..len {
                    crc ^= self.bus.read8(addr + i) as u16;
                    for _ in 0..8 {
                        crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
                    }
                }
                self.st.regs[0] = crc as u32;
            }
            0x11 | 0x12 => { // LZ77UnComp (Wram / Vram-callback form treated as flat)
                let mut src = self.st.regs[0];
                let dst = self.st.regs[1];
                let header = self.bus.read32(src);
                let size = header >> 8;
                src += 4;
                let mut written = 0u32;
                while written < size {
                    let flags = self.bus.read8(src);
                    src += 1;
                    for bit in (0..8).rev() {
                        if written >= size {
                            break;
                        }
                        if flags >> bit & 1 == 0 {
                            let b = self.bus.read8(src);
                            src += 1;
                            self.bus.write8(dst + written, b);
                            written += 1;
                        } else {
                            let b0 = self.bus.read8(src) as u32;
                            let b1 = self.bus.read8(src + 1) as u32;
                            src += 2;
                            let len = (b0 >> 4) + 3;
                            let disp = ((b0 & 0xF) << 8 | b1) + 1;
                            for _ in 0..len {
                                if written >= size {
                                    break;
                                }
                                let b = self.bus.read8(dst + written - disp);
                                self.bus.write8(dst + written, b);
                                written += 1;
                            }
                        }
                    }
                }
            }
            0x14 | 0x15 => { // RLUnComp
                let mut src = self.st.regs[0];
                let dst = self.st.regs[1];
                let size = self.bus.read32(src) >> 8;
                src += 4;
                let mut written = 0u32;
                while written < size {
                    let flag = self.bus.read8(src);
                    src += 1;
                    if flag & 0x80 != 0 {
                        let len = (flag as u32 & 0x7F) + 3;
                        let b = self.bus.read8(src);
                        src += 1;
                        for _ in 0..len.min(size - written) {
                            self.bus.write8(dst + written, b);
                            written += 1;
                        }
                    } else {
                        let len = (flag as u32 & 0x7F) + 1;
                        for _ in 0..len.min(size - written) {
                            let b = self.bus.read8(src);
                            src += 1;
                            self.bus.write8(dst + written, b);
                            written += 1;
                        }
                    }
                }
            }
            // ARM7 sound tables. The NitroSDK driver converts every note's
            // pitch and volume through these, so returning the index unchanged
            // (the old behaviour) detunes the music and mangles its levels.
            0x1A => {
                let i = self.st.regs[0] as usize;
                self.st.regs[0] = crate::soundtbl::SINE.get(i).map_or(0, |v| *v as u16 as u32);
            }
            0x1B => {
                let i = self.st.regs[0] as usize;
                self.st.regs[0] = crate::soundtbl::PITCH.get(i).map_or(0, |v| *v as u32);
            }
            0x1C => {
                let i = self.st.regs[0] as usize;
                self.st.regs[0] = crate::soundtbl::VOLUME.get(i).map_or(0, |v| *v as u32);
            }
            _ => {}
        }
    }

    // ===================== Thumb =====================

    fn exec_thumb(&mut self, op: u16) {
        let op = op as u32;
        match op >> 13 {
            0b000 => {
                if op >> 11 & 3 == 3 {
                    let rd = op & 7;
                    let rs = op >> 3 & 7;
                    let v = if op & 0x400 != 0 { op >> 6 & 7 } else { self.r(op >> 6 & 7) };
                    let a = self.r(rs);
                    let r = if op & 0x200 != 0 {
                        self.sub_with_flags(a, v, 1, true)
                    } else {
                        self.add_with_flags(a, v, 0, true)
                    };
                    self.set_r(rd, r);
                } else {
                    let rd = op & 7;
                    let rs = op >> 3 & 7;
                    let amount = op >> 6 & 0x1F;
                    let (r, c) = self.shift(op >> 11 & 3, self.r(rs), amount, true);
                    self.set_r(rd, r);
                    self.set_nz(r);
                    self.set_flag(C, c);
                }
            }
            0b001 => {
                let rd = op >> 8 & 7;
                let imm = op & 0xFF;
                let a = self.r(rd);
                match op >> 11 & 3 {
                    0 => {
                        self.set_r(rd, imm);
                        self.set_nz(imm);
                    }
                    1 => {
                        self.sub_with_flags(a, imm, 1, true);
                    }
                    2 => {
                        let r = self.add_with_flags(a, imm, 0, true);
                        self.set_r(rd, r);
                    }
                    _ => {
                        let r = self.sub_with_flags(a, imm, 1, true);
                        self.set_r(rd, r);
                    }
                }
            }
            0b010 => self.thumb_group_010(op),
            0b011 => {
                let rd = op & 7;
                let base = self.r(op >> 3 & 7);
                let imm = op >> 6 & 0x1F;
                let byte = op & 0x1000 != 0;
                let load = op & 0x0800 != 0;
                let addr = base.wrapping_add(if byte { imm } else { imm << 2 });
                match (load, byte) {
                    (false, false) => self.bus.write32(addr, self.r(rd)),
                    (false, true) => self.bus.write8(addr, self.r(rd) as u8),
                    (true, false) => {
                        let v = self.bus.read32(addr).rotate_right((addr & 3) * 8);
                        self.set_r(rd, v);
                    }
                    (true, true) => {
                        let v = self.bus.read8(addr) as u32;
                        self.set_r(rd, v);
                    }
                }
            }
            0b100 => {
                if op & 0x1000 == 0 {
                    let rd = op & 7;
                    let addr = self.r(op >> 3 & 7).wrapping_add((op >> 6 & 0x1F) << 1);
                    if op & 0x0800 != 0 {
                        let v = (self.bus.read16(addr) as u32).rotate_right((addr & 1) * 8);
                        self.set_r(rd, v);
                    } else {
                        self.bus.write16(addr, self.r(rd) as u16);
                    }
                } else {
                    let rd = op >> 8 & 7;
                    let addr = self.r(13).wrapping_add((op & 0xFF) << 2);
                    if op & 0x0800 != 0 {
                        let v = self.bus.read32(addr).rotate_right((addr & 3) * 8);
                        self.set_r(rd, v);
                    } else {
                        self.bus.write32(addr, self.r(rd));
                    }
                }
            }
            0b101 => {
                if op & 0x1000 == 0 {
                    let rd = op >> 8 & 7;
                    let imm = (op & 0xFF) << 2;
                    let base = if op & 0x0800 != 0 { self.r(13) } else { self.r(15) & !3 };
                    self.set_r(rd, base.wrapping_add(imm));
                } else if op & 0x0F00 == 0 {
                    let imm = (op & 0x7F) << 2;
                    let sp = self.r(13);
                    self.set_r(13, if op & 0x80 != 0 { sp.wrapping_sub(imm) } else { sp.wrapping_add(imm) });
                } else if op & 0x0600 == 0x0400 {
                    let load = op & 0x0800 != 0;
                    let r_bit = op & 0x0100 != 0;
                    let list = op & 0xFF;
                    let n = list.count_ones() + r_bit as u32;
                    if load {
                        let mut addr = self.r(13);
                        for i in 0..8 {
                            if list & (1 << i) != 0 {
                                self.st.regs[i as usize] = self.bus.read32(addr);
                                addr = addr.wrapping_add(4);
                            }
                        }
                        if r_bit {
                            let v = self.bus.read32(addr);
                            if self.st.arm9 {
                                // v5: POP {pc} honors bit 0 for interworking.
                                self.set_flag(T, v & 1 != 0);
                            }
                            self.st.regs[15] = v & !1;
                            addr = addr.wrapping_add(4);
                        }
                        self.set_r(13, addr);
                    } else {
                        let base = self.r(13).wrapping_sub(n * 4);
                        let mut addr = base;
                        for i in 0..8 {
                            if list & (1 << i) != 0 {
                                let v = self.st.regs[i as usize];
                                self.bus.write32(addr, v);
                                addr = addr.wrapping_add(4);
                            }
                        }
                        if r_bit {
                            self.bus.write32(addr, self.st.regs[14]);
                        }
                        self.set_r(13, base);
                    }
                } else if *STRICT {
                    panic!("unimplemented Thumb op {op:#06X} at {:#010X}", self.st.regs[15]);
                }
            }
            0b110 => {
                if op & 0x1000 == 0 {
                    let rb = op >> 8 & 7;
                    let list = op & 0xFF;
                    let mut addr = self.r(rb);
                    let load = op & 0x0800 != 0;
                    if list == 0 {
                        if load {
                            self.st.regs[15] = self.bus.read32(addr) & !1;
                        } else {
                            self.bus.write32(addr, self.r(15).wrapping_add(2));
                        }
                        self.set_r(rb, addr.wrapping_add(0x40));
                        return;
                    }
                    let first = list.trailing_zeros();
                    let new_base = addr.wrapping_add(list.count_ones() * 4);
                    for i in 0..8 {
                        if list & (1 << i) != 0 {
                            if load {
                                self.st.regs[i as usize] = self.bus.read32(addr);
                            } else {
                                let v = if i == rb && i != first {
                                    new_base
                                } else {
                                    self.st.regs[i as usize]
                                };
                                self.bus.write32(addr, v);
                            }
                            addr = addr.wrapping_add(4);
                        }
                    }
                    if !(load && list & (1 << rb) != 0) {
                        self.set_r(rb, addr);
                    }
                } else {
                    let cond = op >> 8 & 0xF;
                    if cond == 0xF {
                        self.hle_swi(op & 0xFF);
                    } else if self.cond(cond) {
                        let off = ((op & 0xFF) as i8 as i32) << 1;
                        self.st.regs[15] = self.r(15).wrapping_add(off as u32);
                    }
                }
            }
            _ => {
                if op & 0x1800 == 0x0000 {
                    // B unconditional
                    let off = ((op << 21) as i32 >> 20) as u32;
                    self.st.regs[15] = self.r(15).wrapping_add(off);
                } else if op & 0x1800 == 0x1000 {
                    // BL/BLX prefix: LR = PC + offset<<12
                    let off = ((op << 21) as i32 >> 9) as u32;
                    self.st.regs[14] = self.r(15).wrapping_add(off);
                } else if op & 0x1800 == 0x1800 {
                    // BL suffix
                    let lr = self.st.regs[14].wrapping_add((op & 0x7FF) << 1);
                    self.st.regs[14] = self.st.regs[15].wrapping_add(2) | 1;
                    self.st.regs[15] = lr & !1;
                } else {
                    // 0x0800: BLX suffix (v5): like BL but switch to ARM.
                    if self.st.arm9 {
                        let lr = self.st.regs[14].wrapping_add((op & 0x7FF) << 1);
                        self.st.regs[14] = self.st.regs[15].wrapping_add(2) | 1;
                        self.st.regs[15] = lr & !3;
                        self.set_flag(T, false);
                    } else if *STRICT {
                        panic!("unimplemented Thumb op {op:#06X} at {:#010X}", self.st.regs[15]);
                    }
                }
            }
        }
    }

    fn thumb_group_010(&mut self, op: u32) {
        if op & 0x1C00 == 0x0000 {
            let rd = op & 7;
            let rs = op >> 3 & 7;
            let a = self.r(rd);
            let b = self.r(rs);
            let c = self.flag(C) as u32;
            match op >> 6 & 0xF {
                0x0 => { let r = a & b; self.set_r(rd, r); self.set_nz(r); }
                0x1 => { let r = a ^ b; self.set_r(rd, r); self.set_nz(r); }
                0x2 => {
                    let (r, cy) = self.shift(0, a, b & 0xFF, false);
                    self.set_r(rd, r); self.set_nz(r); self.set_flag(C, cy);
                }
                0x3 => {
                    let (r, cy) = self.shift(1, a, b & 0xFF, false);
                    self.set_r(rd, r); self.set_nz(r); self.set_flag(C, cy);
                }
                0x4 => {
                    let (r, cy) = self.shift(2, a, b & 0xFF, false);
                    self.set_r(rd, r); self.set_nz(r); self.set_flag(C, cy);
                }
                0x5 => { let r = self.add_with_flags(a, b, c, true); self.set_r(rd, r); }
                0x6 => { let r = self.sub_with_flags(a, b, c, true); self.set_r(rd, r); }
                0x7 => {
                    let (r, cy) = self.shift(3, a, b & 0xFF, false);
                    self.set_r(rd, r); self.set_nz(r); self.set_flag(C, cy);
                }
                0x8 => { let r = a & b; self.set_nz(r); }
                0x9 => { let r = self.sub_with_flags(0, b, 1, true); self.set_r(rd, r); }
                0xA => { self.sub_with_flags(a, b, 1, true); }
                0xB => { self.add_with_flags(a, b, 0, true); }
                0xC => { let r = a | b; self.set_r(rd, r); self.set_nz(r); }
                0xD => { let r = a.wrapping_mul(b); self.set_r(rd, r); self.set_nz(r); }
                0xE => { let r = a & !b; self.set_r(rd, r); self.set_nz(r); }
                _ => { let r = !b; self.set_r(rd, r); self.set_nz(r); }
            }
        } else if op & 0x1C00 == 0x0400 {
            let rd = (op & 7) | (op >> 4 & 8);
            let rs = op >> 3 & 0xF;
            match op >> 8 & 3 {
                0 => {
                    let r = self.r(rd).wrapping_add(self.r(rs));
                    if rd == 15 {
                        self.st.regs[15] = r & !1;
                    } else {
                        self.set_r(rd, r);
                    }
                }
                1 => {
                    let a = self.r(rd);
                    let b = self.r(rs);
                    self.sub_with_flags(a, b, 1, true);
                }
                2 => {
                    let v = self.r(rs);
                    if rd == 15 {
                        self.st.regs[15] = v & !1;
                    } else {
                        self.set_r(rd, v);
                    }
                }
                _ => {
                    // BX / BLX (v5 when bit 7 set)
                    let v = self.r(rs);
                    if self.st.arm9 && op & 0x80 != 0 {
                        self.st.regs[14] = self.st.regs[15].wrapping_add(2) | 1;
                    }
                    self.set_flag(T, v & 1 != 0);
                    self.st.regs[15] = v & !1;
                    if v & 1 == 0 {
                        self.st.regs[15] &= !3;
                    }
                }
            }
        } else if op & 0x1800 == 0x0800 {
            let rd = op >> 8 & 7;
            let addr = (self.r(15) & !3).wrapping_add((op & 0xFF) << 2);
            let v = self.bus.read32(addr);
            self.set_r(rd, v);
        } else {
            let rd = op & 7;
            let addr = self.r(op >> 3 & 7).wrapping_add(self.r(op >> 6 & 7));
            if op & 0x0200 != 0 {
                match op >> 10 & 3 {
                    0 => self.bus.write16(addr, self.r(rd) as u16),
                    1 => { let v = self.bus.read8(addr) as i8 as i32 as u32; self.set_r(rd, v); }
                    2 => {
                        let v = (self.bus.read16(addr) as u32).rotate_right((addr & 1) * 8);
                        self.set_r(rd, v);
                    }
                    _ => {
                        let v = if addr & 1 != 0 && !self.st.arm9 {
                            self.bus.read8(addr) as i8 as i32 as u32
                        } else {
                            self.bus.read16(addr) as i16 as i32 as u32
                        };
                        self.set_r(rd, v);
                    }
                }
            } else {
                match op >> 10 & 3 {
                    0 => self.bus.write32(addr, self.r(rd)),
                    1 => self.bus.write8(addr, self.r(rd) as u8),
                    2 => {
                        let v = self.bus.read32(addr).rotate_right((addr & 3) * 8);
                        self.set_r(rd, v);
                    }
                    _ => { let v = self.bus.read8(addr) as u32; self.set_r(rd, v); }
                }
            }
        }
    }
}
