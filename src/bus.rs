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
    fn note_pc(&mut self, _pc: u32) {}
}

/// NDS_WATCH="addr[:len]" - log every read/write of that byte range with the
/// accessing cpu and pc. Length defaults to 4.
static WATCH_ADDR: std::sync::LazyLock<Option<(u32, u32)>> = std::sync::LazyLock::new(|| {
    let v = std::env::var("NDS_WATCH").ok()?;
    let (a, len) = v.split_once(':').unwrap_or((v.as_str(), "4"));
    let a = u32::from_str_radix(a.trim().trim_start_matches("0x"), 16).ok()?;
    let len = u32::from_str_radix(len.trim().trim_start_matches("0x"), 16).ok()?;
    Some((a, len))
});

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

/// NDS_RTC="HH:MM" or "YYYY-MM-DD HH:MM" (seconds optional: "HH:MM:SS" /
/// "YYYY-MM-DD HH:MM:SS") sets the emulated real-time clock's starting date
/// and time. Without it the clock starts at the built-in default below.
///
/// The clock then RUNS, but off the emulated scanline counter rather than the
/// host clock: one emulated second per 15734 scanlines. A game that waits for
/// the seconds digit to change (Pokemon does, during its start-up checks)
/// needs a clock that actually ticks, and driving it from emulated time keeps
/// every replay of the same script byte-identical, which is what makes
/// Pokemon's day and night tinting reproducible.
///
/// Stored as plain binary (not BCD): year 0-99, month, day, weekday, hour,
/// minute, second. The BCD conversion happens when a register is read.
static RTC_FIXED: std::sync::LazyLock<[u8; 7]> = std::sync::LazyLock::new(|| {
    // Default: 2026-08-02 (a Sunday), 12:30:00.
    let mut d: [u8; 7] = [26, 8, 2, 0, 12, 30, 0];
    let Ok(spec) = std::env::var("NDS_RTC") else { return d };
    let (date, time) = match spec.trim().split_once(' ') {
        Some((a, b)) => (Some(a.trim()), b.trim()),
        None => (None, spec.trim()),
    };
    if let Some(date) = date {
        let p: Vec<u32> = date.split('-').filter_map(|x| x.parse().ok()).collect();
        if p.len() == 3 {
            d[0] = (p[0] % 100) as u8;
            d[1] = p[1] as u8;
            d[2] = p[2] as u8;
            // Day of week (0 = Sunday) by Zeller's congruence, so a given date
            // reports a consistent weekday for games that branch on it.
            let (mut y, mut mo) = (p[0], p[1]);
            if mo < 3 {
                y -= 1;
                mo += 12;
            }
            let k = y % 100;
            let j = y / 100;
            let h = (p[2] + 13 * (mo + 1) / 5 + k + k / 4 + j / 4 + 5 * j) % 7;
            d[3] = ((h + 6) % 7) as u8; // Zeller: 0 = Saturday
        }
    }
    let t: Vec<u32> = time.split(':').filter_map(|x| x.parse().ok()).collect();
    if t.len() >= 2 {
        d[4] = t[0] as u8;
        d[5] = t[1] as u8;
        d[6] = *t.get(2).unwrap_or(&0) as u8;
    }
    eprintln!("rtc starts at 20{:02}-{:02}-{:02} (wd {}) {:02}:{:02}:{:02}",
        d[0], d[1], d[2], d[3], d[4], d[5], d[6]);
    d
});

/// Scanlines per emulated second: 263 lines per frame at the DS's 59.8261 Hz.
const RTC_LINES_PER_SEC: u64 = 15734;

/// Power management registers as the firmware leaves them: sound amplifier on,
/// both backlights on, battery healthy, microphone amplifier off.
const PMIC_BOOT: [u8; 8] = [0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

/// NDS_RTCLOG=1 traces raw traffic on the RTC bus (0x04000138): every value
/// the ARM7 writes and every value it reads back, capped so a long run still
/// leaves a readable log. This is the only way to see the actual bit-bang
/// waveform the game drives, which is what the protocol has to match.
static RTC_LOG: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_RTCLOG").is_ok());
static RTC_TRACE_N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
fn rtc_trace(tag: &str, v: u16) {
    if !*RTC_LOG {
        return;
    }
    let n = RTC_TRACE_N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    if n < 1200 {
        eprintln!("rtc{} {:04X}", tag, v);
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Machine {
    /// Cartridge image: re-injected by the loader after a savestate restore
    /// rather than stored in it (it is several megabytes and never changes).
    #[serde(skip)]
    pub rom: Vec<u8>,
    /// Debug-only I/O access counters (NDS_IOLOG); never part of a savestate.
    #[serde(skip)]
    pub io_log: Option<std::collections::HashMap<(usize, u32, bool), u64>>,
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
    /// 2D register blocks: engine A at 0x04000000, B at 0x04001000. Held as
    /// Vecs rather than [[u8; 0x70]; 2] because serde only derives array
    /// support up to 32 elements.
    pub io2d: [Vec<u8>; 2],
    pub vcount: u16,
    pub keyinput: u16, // 10 bits, active low
    pub extkeyin: u16,
    pub powcnt1: u32,
    pub exmemcnt: u16,
    // Per-CPU interrupt + display state. Index 0 = ARM9, 1 = ARM7.
    pub ime: [bool; 2],
    pub ie: [u32; 2],
    pub if_: [u32; 2],
    pub dispstat: [u16; 2],
    pub postflg: [u8; 2],
    // IPC.
    pub ipcsync: [u16; 2], // each CPU's last written IPCSYNC (out nibble + enables)
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
    // SPI bus (ARM7): firmware flash, touchscreen, power management.
    pub spicnt: u16,
    pub spi_out: u8,
    pub firmware: Vec<u8>,
    spi_phase: u8, // 0 = command, 1 = address, 2 = data
    spi_cmd: u8,
    spi_addr: u32,
    spi_addr_n: u8,
    tsc_cmd: u8,
    tsc_byte: u8,
    tsc_val: u16,
    /// Touchscreen state fed by the frontend (screen coords on the lower
    /// display; down = pen contact).
    pub touch_x: u32,
    pub touch_y: u32,
    pub touch_down: bool,
    // DMA: [cpu][ch]. cnt holds len (0-20) + CNT_H<<16. src/dst are the
    // internal latches (hardware never writes them back to SAD/DAD).
    pub dma_sad: [[u32; 4]; 2],
    pub dma_dad: [[u32; 4]; 2],
    pub dma_cnt: [[u32; 4]; 2],
    pub dma_src: [[u32; 4]; 2],
    pub dma_dst: [[u32; 4]; 2],
    // Timers: [cpu][n]. val is the live 16-bit counter, acc the sub-prescaler
    // remainder, driven in per-scanline lumps by tick_timers.
    pub timer_cnt: [[u16; 4]; 2],
    pub timer_reload: [[u16; 4]; 2],
    pub timer_val: [[u32; 4]; 2],
    pub timer_acc: [[u32; 4]; 2],
    // Cartridge interface (`rom` lives at the top of the struct).
    pub auxspicnt: u16,
    pub romctrl: u32,
    pub cart_cmd: [u8; 8],
    cart_addr: usize,
    pub cart_left: usize,
    cart_kind: u8, // 0 = rom data, 1 = chip id, 2 = ones
    // AUXSPI backup chip (serial flash FSM, 512KB space).
    pub save: Vec<u8>,
    pub auxspi_out: u8,
    aux_phase: u8, // 0 cmd, 1 addr, 2 data
    aux_cmd: u8,
    aux_addr: u32,
    aux_addr_n: u8,
    aux_wren: bool,
    pub save_dirty: bool,
    // RTC (bit-banged on 0x04000138): S-35180-style serial protocol.
    pub rtc_reg: u16,
    rtc_bit_n: u8,
    rtc_byte: u8,
    rtc_cmd: u8,
    rtc_data: Vec<u8>,
    rtc_pos: usize,
    rtc_reading: bool,
    /// RTC status registers. `None` means "never written", which reads back as
    /// 24-hour mode with the power-off and battery-low flags clear. They are
    /// deliberately kept OUT of savestates (adding bytes would invalidate
    /// every existing state file) and the default is what Platinum writes
    /// anyway, so a restored state behaves identically.
    #[serde(skip)]
    rtc_stat1: Option<u8>,
    #[serde(skip)]
    rtc_stat2: u8,
    /// Power management chip registers (SPI device 0). `None` means untouched,
    /// which reads back as the state the firmware leaves behind. Kept out of
    /// savestates for the same reason as the RTC status registers.
    #[serde(skip)]
    pmic: Option<[u8; 8]>,
    /// Firmware flash write-enable latch (set by WREN, cleared once a program
    /// or erase completes). Not in savestates: it is only ever set for the
    /// few transfers inside one write sequence.
    #[serde(skip)]
    fw_wel: bool,
    /// GXSTAT IRQ mode (bits 14-15 of the high half): with an always-empty
    /// geometry FIFO, any enabled mode means the IRQ line is held asserted.
    pub gxstat_irq: u8,
    /// 3D geometry engine (command processor; rasterizer is phase 2).
    pub gx: crate::gpu3d::Gpu3d,
    /// Geometry commands the game has submitted and we have thrown away. Only
    /// a diagnostic: it tells a blank screen "the 3D engine is missing" apart
    /// from "the game is stuck".
    pub gx_cmds: u64,
    /// WiFi register block 0x04800000-0x0480FFFF (ARM7): RAM-backed so init
    /// handshakes read back what they wrote; a few IDs/status special-cased.
    pub wifi: Vec<u8>,
    /// HLE BIOS IRQ dispatch stubs (0x40 bytes each, Vec for the same serde
    /// reason as io2d). The ARM9 one is patched in place whenever CP15 moves
    /// DTCM, so a savestate must carry the patched bytes, not rebuild them.
    pub stub9: Vec<u8>,
    pub stub7: Vec<u8>,
    pub last_pc: [u32; 2],
    /// Scanline counter since boot, for event timestamps in debug logs.
    pub now: u64,
    /// Wi-Fi baseband (BB) register file (0x69 regs). The wireless manager
    /// writes then reads these back to verify the RF chip during load; a bare
    /// stub returns 0 and fails the self-test, which is what made Pokémon
    /// report a "communication error" on Continue. #[serde(skip)]: lazily
    /// re-seeded, not part of savestates.
    #[serde(skip)]
    pub bb: Vec<u8>,
    #[serde(skip)]
    pub bb_read: u16,
    /// ARM7 sound register block 0x04000400-0x0400051F, RAM-backed so the
    /// sound driver's status polls read back what it wrote (no audio output).
    /// Kept out of savestates (older state files predate it); re-initialized
    /// lazily, which just means channels read idle right after a restore.
    #[serde(skip)]
    pub sound: Vec<u8>,
    /// Per-channel busy deadline in `now` scanline ticks: one-shot channels
    /// read SOUNDxCNT bit 31 as set until this passes, then it self-clears.
    #[serde(skip)]
    pub sound_end: Vec<u64>,
}

/// Firmware user-settings block (0x74 bytes incl. update counter + CRC).
pub fn user_settings_block() -> [u8; 0x74] {
    let mut us = [0u8; 0x74];
    us[0x00] = 5; // version
    us[0x03] = 1; // birth month
    us[0x04] = 1; // birth day
    us[0x06] = b'J'; // nickname "JO", UTF-16LE
    us[0x08] = b'O';
    us[0x1A] = 2; // nickname length
    // Touch calibration: ADC (0x02DF,0x032C)->(32,32), (0x0D3B,0x0CE7)->(224,160).
    us[0x58..0x5A].copy_from_slice(&0x02DFu16.to_le_bytes());
    us[0x5A..0x5C].copy_from_slice(&0x032Cu16.to_le_bytes());
    us[0x5C] = 32;
    us[0x5D] = 32;
    us[0x5E..0x60].copy_from_slice(&0x0D3Bu16.to_le_bytes());
    us[0x60..0x62].copy_from_slice(&0x0CE7u16.to_le_bytes());
    us[0x62] = 224;
    us[0x63] = 160;
    us[0x64..0x66].copy_from_slice(&0x0001u16.to_le_bytes()); // language: English
    let mut crc: u16 = 0xFFFF;
    for b in &us[..0x70] {
        crc ^= *b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
        }
    }
    us[0x72..0x74].copy_from_slice(&crc.to_le_bytes());
    us
}

/// The BIOS IRQ dispatcher both CPUs get: save regs, load the user handler
/// pointer from a literal-addressed word, call it, return from exception.
fn build_stub(handler_ptr_plus4: u32) -> Vec<u8> {
    let words: [u32; 7] = [
        0xE92D500F, // 0x18: stmfd sp!, {r0-r3, r12, lr}
        0xE59F000C, // 0x1C: ldr r0, [pc, #12]   ; -> literal at 0x30
        0xE28FE000, // 0x20: add lr, pc, #0      ; return to 0x28
        0xE510F004, // 0x24: ldr pc, [r0, #-4]
        0xE8BD500F, // 0x28: ldmfd sp!, {r0-r3, r12, lr}
        0xE25EF004, // 0x2C: subs pc, lr, #4
        handler_ptr_plus4, // 0x30: literal
    ];
    let mut stub = vec![0u8; 0x40];
    for (i, w) in words.iter().enumerate() {
        stub[0x18 + i * 4..0x18 + i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    stub
}

impl Machine {
    pub fn new() -> Self {
        Self {
            rom: Vec::new(),
            io_log: std::env::var("NDS_IOLOG").ok().map(|_| Default::default()),
            main_ram: vec![0; 0x40_0000],
            wram7: vec![0; 0x1_0000],
            swram: vec![0; 0x8000],
            wramcnt: 3, // all shared WRAM to the ARM7 (what firmware boot leaves)
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
            io2d: [vec![0; 0x70], vec![0; 0x70]],
            vcount: 0,
            keyinput: 0x3FF,
            // Bit 7 = hinge, GBATEK: 1 = lid CLOSED, so keep it 0 (open).
            // Faking "closed" suppresses Platinum's failure watchdog but
            // sends the game into sleep mode instead. Bits 0-6: X/Y/DEBUG
            // released, pen up.
            extkeyin: 0x7F,
            powcnt1: 0x820F,
            exmemcnt: 0xE880,
            ime: [false; 2],
            ie: [0; 2],
            if_: [0; 2],
            dispstat: [0; 2],
            postflg: [1; 2],
            ipcsync: [0; 2],
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
            spicnt: 0,
            spi_out: 0,
            firmware: {
                // Synthesized 256KB firmware image: header points at a valid
                // user-settings block (offset stored in 8-byte units at 0x20).
                let mut fw = vec![0u8; 0x4_0000];
                fw[0x08..0x0C].copy_from_slice(b"MACP");
                fw[0x20..0x22].copy_from_slice(&((0x3FE00u32 / 8) as u16).to_le_bytes());
                // WiFi calibration block: length at 0x2C, body 0x2C..0x2C+0x138,
                // CRC16 (init 0) at 0x2A. Pokemon's WM init validates this.
                fw[0x2C..0x2E].copy_from_slice(&0x0138u16.to_le_bytes());
                fw[0x36..0x3C].copy_from_slice(&[0x00, 0x09, 0xBF, 0x12, 0x34, 0x56]); // MAC
                fw[0x3C..0x3E].copy_from_slice(&0x3FFEu16.to_le_bytes()); // enabled channels
                fw[0x40] = 0xFF; // flags
                let mut crc: u16 = 0;
                for i in 0..0x138usize {
                    crc ^= fw[0x2C + i] as u16;
                    for _ in 0..8 {
                        crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
                    }
                }
                fw[0x2A..0x2C].copy_from_slice(&crc.to_le_bytes());
                let us = user_settings_block();
                fw[0x3FE00..0x3FE00 + us.len()].copy_from_slice(&us);
                fw[0x3FF00..0x3FF00 + us.len()].copy_from_slice(&us); // backup copy
                // WFC access-point connection blocks (3 x 0x100 at 0x3FA00):
                // unconfigured (status 0xFF) but with valid CRC16 over the
                // first 0xFE bytes - Pokemon re-reads these forever otherwise.
                for base in [0x3FA00usize, 0x3FB00, 0x3FC00] {
                    fw[base + 0xE7] = 0xFF; // status: not configured
                    let mut crc: u16 = 0;
                    for i in 0..0xFE {
                        crc ^= fw[base + i] as u16;
                        for _ in 0..8 {
                            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
                        }
                    }
                    fw[base + 0xFE..base + 0x100].copy_from_slice(&crc.to_le_bytes());
                }
                fw
            },
            spi_phase: 0,
            spi_cmd: 0,
            spi_addr: 0,
            spi_addr_n: 0,
            tsc_cmd: 0,
            tsc_byte: 0,
            tsc_val: 0,
            touch_x: 0,
            touch_y: 0,
            touch_down: false,
            dma_sad: [[0; 4]; 2],
            dma_dad: [[0; 4]; 2],
            dma_cnt: [[0; 4]; 2],
            dma_src: [[0; 4]; 2],
            dma_dst: [[0; 4]; 2],
            timer_cnt: [[0; 4]; 2],
            timer_reload: [[0; 4]; 2],
            timer_val: [[0; 4]; 2],
            timer_acc: [[0; 4]; 2],
            auxspicnt: 0,
            romctrl: 0,
            cart_cmd: [0; 8],
            cart_addr: 0,
            cart_left: 0,
            cart_kind: 0,
            save: vec![0xFF; 0x8_0000],
            auxspi_out: 0xFF,
            aux_phase: 0,
            aux_cmd: 0,
            aux_addr: 0,
            aux_addr_n: 0,
            aux_wren: false,
            save_dirty: false,
            gxstat_irq: 0,
            gx: crate::gpu3d::Gpu3d::new(),
            gx_cmds: 0,
            bb: Vec::new(),
            bb_read: 0,
            sound: vec![0; 0x120],
            sound_end: vec![0; 16],
            wifi: vec![0; 0x1_0000],
            rtc_reg: 0,
            rtc_bit_n: 0,
            rtc_byte: 0,
            rtc_cmd: 0,
            rtc_data: Vec::new(),
            rtc_pos: 0,
            rtc_reading: false,
            rtc_stat1: None,
            rtc_stat2: 0,
            pmic: None,
            fw_wel: false,
            stub9: build_stub(0x0080_4000),
            stub7: build_stub(0x0381_0000),
            last_pc: [0; 2],
            now: 0,
        }
    }

    /// Advance all timers by `cycles` bus clocks (33MHz domain, both CPUs).
    pub fn tick_timers(&mut self, cycles: u32) {
        for cpu in 0..2 {
            let mut overflows = 0u32;
            for n in 0..4 {
                let cnt = self.timer_cnt[cpu][n];
                if cnt & 0x80 == 0 {
                    overflows = 0;
                    continue;
                }
                let cascade = n > 0 && cnt & 4 != 0;
                let inc = if cascade {
                    overflows
                } else {
                    let shift = match cnt & 3 {
                        0 => 0,
                        1 => 6,
                        2 => 8,
                        _ => 10,
                    };
                    self.timer_acc[cpu][n] += cycles;
                    let i = self.timer_acc[cpu][n] >> shift;
                    self.timer_acc[cpu][n] &= (1 << shift) - 1;
                    i
                };
                let mut v = self.timer_val[cpu][n] + inc;
                overflows = 0;
                if v > 0xFFFF {
                    let reload = self.timer_reload[cpu][n] as u32;
                    let period = 0x1_0000 - reload;
                    let over = v - 0x1_0000;
                    overflows = 1 + over / period;
                    v = reload + over % period;
                    if cnt & 0x40 != 0 {
                        self.request_irq(cpu, 1 << (3 + n));
                    }
                }
                self.timer_val[cpu][n] = v;
            }
        }
    }

    /// Start a cartridge command per ROMCTRL bit 31.
    fn cart_start(&mut self, cpu: usize) {
        if self.io_log.is_some() {
            static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            let n = COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if n < 200_000 {
                eprintln!("[t={}] cart cmd={:02X?} romctrl={:#010X}", self.now, self.cart_cmd, self.romctrl);
            }
        }
        let n = (self.romctrl >> 24 & 7) as usize;
        let len = match n {
            0 => 0,
            7 => 4,
            _ => 0x100usize << n,
        };
        let log_first = self.io_log.is_some();
        let cmd = self.cart_cmd[0];
        if log_first && cmd == 0xB7 {
            let addr = u32::from_be_bytes([
                self.cart_cmd[1],
                self.cart_cmd[2],
                self.cart_cmd[3],
                self.cart_cmd[4],
            ]) as usize;
            let b = |i: usize| *self.rom.get(addr + i).unwrap_or(&0xFF);
            static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 60 {
                eprintln!(
                    "cartword @{:#010X} = {:#010X} (len code {})",
                    addr,
                    u32::from_le_bytes([b(0), b(1), b(2), b(3)]),
                    self.romctrl >> 24 & 7
                );
            }
        }
        match cmd {
            0xB7 => {
                // Main data read: address in command bytes 1-4 (big endian).
                let addr = u32::from_be_bytes([
                    self.cart_cmd[1],
                    self.cart_cmd[2],
                    self.cart_cmd[3],
                    self.cart_cmd[4],
                ]) as usize;
                self.cart_kind = 0;
                self.cart_addr = addr;
            }
            0xB8 | 0x90 => self.cart_kind = 1, // chip ID
            0x00 => {
                // Header read (unencrypted boot command).
                self.cart_kind = 0;
                self.cart_addr = 0;
            }
            _ => self.cart_kind = 2, // 0x9F dummy and everything else: 0xFF
        }
        self.cart_left = len;
        if len == 0 {
            self.romctrl &= !0x8000_0000; // nothing to transfer: done
            if self.auxspicnt & 0x4000 != 0 {
                self.request_irq(cpu, 1 << 19);
            }
        } else {
            self.romctrl |= 0x0080_0000; // data-ready
        }
    }

    /// Pop the next cart data word (reads of 0x04100010).
    pub fn cart_read_word(&mut self, cpu: usize) -> u32 {
        if self.cart_left == 0 {
            return 0xFFFF_FFFF;
        }
        let v = match self.cart_kind {
            0 => {
                let a = self.cart_addr;
                let b = |i: usize| *self.rom.get(a + i).unwrap_or(&0xFF);
                // Cart hardware quirk (GBATEK): a B7 transfer that crosses a
                // 4KB boundary wraps back to the start of the SAME 4KB block
                // instead of continuing. Platinum's filesystem reads rely on
                // it; advancing linearly returns wrong bytes for any block
                // read starting off-alignment (this broke Continue-from-save:
                // the FS re-read the same table entry forever).
                self.cart_addr = (a & !0xFFF) | ((a + 4) & 0xFFF);
                u32::from_le_bytes([b(0), b(1), b(2), b(3)])
            }
            1 => 0x0000_1FC2, // chip ID
            _ => 0xFFFF_FFFF,
        };
        self.cart_left = self.cart_left.saturating_sub(4);
        if self.cart_left == 0 {
            self.romctrl &= !0x8080_0000; // busy + data-ready clear
            if self.auxspicnt & 0x4000 != 0 {
                self.request_irq(cpu, 1 << 19);
            }
        }
        v
    }

    pub fn aux_deselect(&mut self) {
        self.aux_phase = 0;
    }

    /// One byte over AUXSPI to the backup chip (flash-style commands).
    pub fn auxspi_transfer(&mut self, v: u8) {
        if self.io_log.is_some() {
            static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 80 {
                eprintln!(
                    "auxspi in={:#04X} phase={} cmd={:#04X} cnt={:#06X} -> out={:#04X}",
                    v, self.aux_phase, self.aux_cmd, self.auxspicnt, self.auxspi_out
                );
            }
        }
        match self.aux_phase {
            0 => {
                self.aux_cmd = v;
                self.auxspi_out = 0xFF;
                match v {
                    0x03 | 0x0B | 0x02 | 0x0A | 0xDB | 0xD8 => {
                        self.aux_phase = 1;
                        self.aux_addr = 0;
                        self.aux_addr_n = 0;
                    }
                    0x06 => self.aux_wren = true,
                    0x04 => self.aux_wren = false,
                    0x05 => {
                        // RDSR: streams the status register on every
                        // subsequent clocked byte until deselect.
                        self.auxspi_out = if self.aux_wren { 2 } else { 0 };
                        self.aux_phase = 4;
                    }
                    0x9F => {
                        // JEDEC ID: ST 2Mbit flash (M45PE20), byte-streamed.
                        self.aux_phase = 3;
                        self.aux_addr = 0;
                    }
                    _ => {}
                }
            }
            1 => {
                self.aux_addr = self.aux_addr << 8 | v as u32;
                self.aux_addr_n += 1;
                self.auxspi_out = 0xFF;
                if self.aux_addr_n == 3 {
                    self.aux_phase = 2;
                }
            }
            4 => {
                self.auxspi_out = if self.aux_wren { 2 } else { 0 };
            }
            3 => {
                const ID: [u8; 3] = [0x20, 0x40, 0x13]; // ST M45PE40: 4Mbit, 512KB
                self.auxspi_out = ID[(self.aux_addr as usize).min(2)];
                self.aux_addr += 1;
            }
            _ => {
                let a = (self.aux_addr as usize) & 0x7_FFFF;
                match self.aux_cmd {
                    0x03 | 0x0B => {
                        self.auxspi_out = self.save[a];
                    }
                    0x02 | 0x0A => {
                        self.save[a] = v;
                        self.save_dirty = true;
                        self.auxspi_out = 0xFF;
                    }
                    _ => self.auxspi_out = 0xFF,
                }
                self.aux_addr = self.aux_addr.wrapping_add(1);
            }
        }
        // CS releases when the hold bit (AUXSPICNT bit 6) is clear.
        if self.auxspicnt & 0x0040 == 0 {
            self.aux_phase = 0;
        }
    }

    /// Current date and time as BCD bytes (year, month, day, weekday, hour,
    /// minute, second): the NDS_RTC starting point advanced by however much
    /// emulated time has passed, so it ticks like a real clock without ever
    /// consulting the host's.
    fn rtc_now(&self) -> [u8; 7] {
        let b = *RTC_FIXED;
        let mut s = b[6] as u64 + self.now / RTC_LINES_PER_SEC;
        let mut mi = b[5] as u64 + s / 60;
        s %= 60;
        let mut h = b[4] as u64 + mi / 60;
        mi %= 60;
        let elapsed_days = h / 24;
        h %= 24;
        let (mut y, mut mo) = (b[0] as u64, b[1] as u64);
        let mut day = b[2] as u64 + elapsed_days;
        let wd = (b[3] as u64 + elapsed_days) % 7;
        loop {
            let leap = y % 4 == 0; // 2000-2099: every fourth year is a leap year
            let dim = match mo {
                1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
                4 | 6 | 9 | 11 => 30,
                _ if leap => 29,
                _ => 28,
            };
            if day <= dim {
                break;
            }
            day -= dim;
            mo += 1;
            if mo > 12 {
                mo = 1;
                y = (y + 1) % 100;
            }
        }
        let bcd = |v: u64| ((v / 10) << 4 | v % 10) as u8;
        [bcd(y), bcd(mo), bcd(day), wd as u8, bcd(h), bcd(mi), bcd(s)]
    }

    /// Contents of one RTC register, LSB-first byte order, as the chip would
    /// clock them out.
    fn rtc_reg_data(&self, reg: u8) -> Vec<u8> {
        let d = self.rtc_now(); // year, month, day, weekday, hour, minute, second
        let stat1 = self.rtc_stat1.unwrap_or(0x02);
        // Hour byte: bit 6 is the AM/PM flag and hardware sets it for any hour
        // from 12 onwards EVEN IN 24-HOUR MODE. In 12-hour mode the hour
        // counts 0-11 and the flag is the only thing separating morning from
        // afternoon, so a game that reconstructs the hour needs both.
        let h24 = (d[4] >> 4) * 10 + (d[4] & 0xF);
        let pm = if h24 >= 12 { 0x40 } else { 0 };
        let hour = if stat1 & 0x02 != 0 {
            d[4] // 24-hour mode: plain BCD 00-23
        } else {
            let h12 = h24 % 12;
            (h12 / 10) << 4 | h12 % 10
        } | pm;
        match reg {
            0 => vec![stat1],
            1 => vec![self.rtc_stat2],
            2 => vec![d[0], d[1], d[2], d[3], hour, d[5], d[6]],
            3 => vec![hour, d[5], d[6]],
            4 => vec![0x00],             // alarm 1 / int 1 (int mode off: 1 byte)
            5 => vec![0x00, 0x00, 0x00], // alarm 2 / int 2
            6 => vec![0x00],             // clock adjust
            _ => vec![0x00],             // free register
        }
    }

    /// RTC register write (0x04000138): CS on bit 2, SCK on bit 1, data on
    /// bit 0, with the pin directions in bits 4-6. Bytes shift LSB-first.
    /// The ARM7 drives a written bit while the clock is low and the chip
    /// latches it on the RISING edge; for reads the chip presents the next
    /// bit on the FALLING edge and holds it until the following one, because
    /// the ARM7 samples the line while the clock is still low (confirmed
    /// against Platinum's own waveform with NDS_RTCLOG).
    pub fn rtc_write(&mut self, v: u16) {
        rtc_trace("w", v);
        let old = self.rtc_reg;
        if v & 4 == 0 {
            // Chip deselected: reset the transaction.
            self.rtc_bit_n = 0;
            self.rtc_byte = 0;
            self.rtc_cmd = 0;
            self.rtc_reading = false;
            self.rtc_reg = v;
            return;
        }
        let rising = old & 2 == 0 && v & 2 != 0;
        let falling = old & 2 != 0 && v & 2 == 0;
        let mut out = v;
        if self.rtc_reading {
            if falling {
                let byte = self.rtc_data.get(self.rtc_pos / 8).copied().unwrap_or(0);
                let bit = (byte >> (self.rtc_pos % 8)) & 1;
                self.rtc_pos += 1;
                out = (out & !1) | bit as u16;
            } else {
                // Hold the bit the chip is already driving (it lives in bit 0
                // of the last presented register value).
                out = (out & !1) | (old & 1);
            }
        } else if rising {
            self.rtc_byte |= ((v & 1) as u8) << self.rtc_bit_n;
            self.rtc_bit_n += 1;
            if self.rtc_bit_n == 8 {
                if self.rtc_cmd == 0 {
                    // Command byte: fixed 0110 pattern in the low nibble
                    // (LSB-first order), register in bits 4-6, read in 7.
                    // Some code sends it MSB-first; normalize.
                    if self.rtc_byte & 0xF != 6 && self.rtc_byte >> 4 == 6 {
                        self.rtc_byte = self.rtc_byte.reverse_bits();
                    }
                    self.rtc_cmd = self.rtc_byte;
                    if self.io_log.is_some() {
                        eprintln!("rtc cmd {:#04X}", self.rtc_byte);
                    }
                    let reg = self.rtc_byte >> 4 & 7;
                    if self.rtc_byte & 0x80 != 0 {
                        self.rtc_reading = true;
                        self.rtc_pos = 0;
                        self.rtc_data = self.rtc_reg_data(reg);
                    }
                } else {
                    // A data byte written to the selected register. The status
                    // registers must keep what the game puts in them: a game
                    // that sets 24-hour mode and reads it back has to see its
                    // own value or it decides the clock is broken. The clock
                    // itself is fixed by NDS_RTC, so date/time and alarm
                    // writes are accepted and discarded.
                    let reg = self.rtc_cmd >> 4 & 7;
                    match reg {
                        // Bit 0 is a write-only reset strobe and bits 4-7 are
                        // read-only flags, so only bits 1-3 stick.
                        0 => {
                            let s = self.rtc_stat1.unwrap_or(0x02);
                            self.rtc_stat1 = Some(s & !0x0E | self.rtc_byte & 0x0E);
                        }
                        1 => self.rtc_stat2 = self.rtc_byte,
                        _ => {}
                    }
                }
                self.rtc_bit_n = 0;
                self.rtc_byte = 0;
            }
        }
        self.rtc_reg = out;
    }

    /// One byte over the SPI bus; device per SPICNT bits 8-9.
    pub fn spi_transfer(&mut self, v: u8) {
        match self.spicnt >> 8 & 3 {
            1 => {
                // Firmware serial flash (an ST M25P40-style chip). Reads were
                // all this used to do, but the ARM7 also WRITES here: it saves
                // the user settings block, and the write sequence is
                // WREN -> poll RDSR until the write-enable latch shows up ->
                // program -> poll RDSR until the write finishes. With every
                // command but 0x03 answering zero, that poll never resolved
                // and the boot stalled with the screens still dark.
                match self.spi_phase {
                    0 => {
                        self.spi_cmd = v;
                        self.spi_out = 0;
                        self.spi_addr = 0;
                        self.spi_addr_n = 0;
                        match v {
                            0x06 => self.fw_wel = true,  // write enable
                            0x04 => self.fw_wel = false, // write disable
                            // Read, page program and the erases all take a
                            // three byte address first.
                            0x03 | 0x0B | 0x02 | 0x0A | 0xDB | 0xD8 => self.spi_phase = 1,
                            0x05 | 0x9F => self.spi_phase = 2, // status / JEDEC id
                            _ => {}
                        }
                    }
                    1 => {
                        self.spi_addr = self.spi_addr << 8 | v as u32;
                        self.spi_addr_n += 1;
                        self.spi_out = 0;
                        if self.spi_addr_n == 3 {
                            // 0x0B fast read inserts one dummy byte; treat it
                            // as part of the address run.
                            self.spi_phase = if self.spi_cmd == 0x0B { 1 } else { 2 };
                            // Page write (0x0A) erases its 256 byte page before
                            // programming it, unlike page program (0x02) which
                            // can only clear bits. Getting that wrong makes a
                            // rewritten block read back as the AND of old and
                            // new, which is what the ARM7's verify caught.
                            if matches!(self.spi_cmd, 0xDB | 0xD8 | 0x0A) {
                                let page = self.spi_addr as usize & 0x3_FF00;
                                let end = if self.spi_cmd == 0xD8 { 0x4_0000 } else { page + 0x100 };
                                for i in page..end.min(0x4_0000) {
                                    self.firmware[i] = 0xFF;
                                }
                                if self.spi_cmd != 0x0A {
                                    self.fw_wel = false;
                                }
                            }
                            if self.io_log.is_some() {
                                static COUNT: std::sync::atomic::AtomicU32 =
                                    std::sync::atomic::AtomicU32::new(0);
                                if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 2000 {
                                    eprintln!(
                                        "[t={}] fwspi cmd {:#04X} @{:#08X}",
                                        self.now, self.spi_cmd, self.spi_addr
                                    );
                                }
                            }
                        } else if self.spi_addr_n > 3 {
                            self.spi_phase = 2; // fast-read dummy byte consumed
                        }
                    }
                    _ => match self.spi_cmd {
                        // Status register: writes complete instantly here, so
                        // the write-in-progress bit is never set; bit 1 is the
                        // write-enable latch the ARM7 polls for.
                        0x05 => self.spi_out = if self.fw_wel { 0x02 } else { 0x00 },
                        0x9F => {
                            self.spi_out = match self.spi_addr_n {
                                0 => 0x20, // manufacturer: ST
                                1 => 0x40, // memory type
                                _ => 0x12, // capacity: 4 Mbit
                            };
                            self.spi_addr_n += 1;
                        }
                        0x02 | 0x0A => {
                            // Page program. Real flash can only clear bits, and
                            // programming wraps inside the 256 byte page.
                            let i = (self.spi_addr as usize) & 0x3_FFFF;
                            if self.fw_wel {
                                self.firmware[i] &= v;
                            }
                            let page = self.spi_addr & !0xFF;
                            self.spi_addr = page | (self.spi_addr.wrapping_add(1) & 0xFF);
                            self.spi_out = 0;
                        }
                        _ => {
                            self.spi_out = self.firmware[(self.spi_addr as usize) & 0x3_FFFF];
                            self.spi_addr = self.spi_addr.wrapping_add(1);
                        }
                    },
                }
            }
            2 => {
                // Touchscreen controller (TSC2046). A 12-bit conversion is
                // read over TWO transfers, MSB first, and the command byte
                // that starts the NEXT conversion also clocks out the low
                // byte of the previous one - the SDK sampler relies on that
                // overlap (write cmd, then loop: write 0 -> high byte,
                // write cmd -> low byte).
                if v & 0x80 != 0 {
                    self.spi_out = ((self.tsc_val & 0x1F) << 3) as u8;
                    let chan = v >> 4 & 7;
                    // Calibration must match user_settings_block():
                    // ADC (0x02DF,0x032C)->(32,32), (0x0D3B,0x0CE7)->(224,160).
                    // Released readings are not zero: per GBATEK the X channel
                    // reads 000h and the Y channel reads FFFh with the pen up,
                    // and games detect pen-lift with either one.
                    self.tsc_val = match (chan, self.touch_down) {
                        (5, true) => (0x02DF
                            + self.touch_x.saturating_sub(32) * (0x0D3B - 0x02DF) / 192)
                            .min(0xFFF) as u16,
                        (1, true) => (0x032C
                            + self.touch_y.saturating_sub(32) * (0x0CE7 - 0x032C) / 128)
                            .min(0xFFF) as u16,
                        (5, false) => 0x000,
                        (1, false) => 0xFFF,
                        _ => 0, // Z1/Z2 pressure, temperature, battery, mic: unimplemented
                    };
                } else {
                    self.spi_out = (self.tsc_val >> 5) as u8;
                }
            }
            0 => {
                // Power management chip. One command byte (bit 7 = read,
                // bits 0-6 = register index) then one data byte. Register 0
                // holds the sound amplifier and the two backlight enables,
                // register 1 the battery status, 2 and 3 the microphone
                // amplifier. Writes MUST stick: the ARM7's power-management
                // module reads a register back to confirm what it just set,
                // and a chip that always answers zero makes it retry forever
                // (which is exactly what kept Platinum's screens dark).
                match self.spi_phase {
                    0 => {
                        self.spi_cmd = v;
                        self.spi_out = 0;
                        self.spi_phase = 1;
                    }
                    _ => {
                        let idx = (self.spi_cmd & 7) as usize;
                        let mut regs = self.pmic.unwrap_or(PMIC_BOOT);
                        if self.spi_cmd & 0x80 == 0 {
                            // Battery status is read-only; bit 7 of the
                            // control register is unused.
                            if idx != 1 {
                                regs[idx] = if idx == 0 { v & 0x7F } else { v };
                            }
                            self.pmic = Some(regs);
                        }
                        self.spi_out = regs[idx];
                    }
                }
            }
            _ => self.spi_out = 0,
        }
        // Chip deselects unless the hold bit is set.
        if self.spicnt & 0x0800 == 0 {
            self.spi_phase = 0;
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
                // MST 2 = engine-A OBJ only for banks A/B; for C/D it hands
                // the bank to the ARM7 (vram_arm7_slot), not the ARM9.
                (0..=1, 2) => 0x0640_0000 + (ofs & 1) * 0x20000,
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

    /// Banks C and D with MST=2 are handed to the ARM7 as work RAM at
    /// 0x06000000 + OFS.0 * 0x20000 (GBATEK). Only the ARM7 sees them.
    pub fn vram_arm7_slot(&self, addr: u32) -> Option<(usize, usize)> {
        let a = 0x0600_0000 + (addr & 0x3_FFFF); // 256KB ARM7 VRAM region, mirrored
        for i in [2usize, 3] {
            let cnt = self.vramcnt[i];
            if cnt & 0x87 != 0x82 {
                continue;
            }
            let base = 0x0600_0000 + ((cnt >> 3 & 1) as u32) * 0x2_0000;
            if a >= base && a < base + BANK_SIZE[i] as u32 {
                return Some((i, (a - base) as usize));
            }
        }
        None
    }

    /// ARM7 sound block 0x400-0x51F, storage-only (no audio output): registers
    /// read back what was written so the driver's status polls see a coherent
    /// channel model. SOUNDxCNT bit 31 (start/busy) self-clears for one-shot
    /// channels once the sample's play time elapses on the scanline clock
    /// (~2130 CPU cycles per scanline); loop/manual channels stay busy until
    /// the CPU clears them. Capture busy bits clear on read.
    fn sound_ensure(&mut self) {
        if self.sound.len() < 0x120 {
            self.sound = vec![0; 0x120];
        }
        if self.sound_end.len() < 16 {
            self.sound_end = vec![0; 16];
        }
    }

    pub fn sound_read16(&mut self, off: u32) -> u16 {
        self.sound_ensure();
        let i = (off - 0x400) as usize & !1;
        let mut v = u16::from_le_bytes([self.sound[i], self.sound[i + 1]]);
        if off < 0x500 && off & 0xF == 2 {
            let ch = ((off - 0x400) >> 4) as usize;
            let repeat = v >> 11 & 3;
            if v & 0x8000 != 0 && repeat == 2 && self.now >= self.sound_end[ch] {
                v &= 0x7FFF;
                self.sound[i + 1] &= 0x7F;
            }
        }
        if off & !1 == 0x508 {
            // SNDCAP0/1CNT busy bits: capture "completes" immediately.
            v &= !0x8080;
            self.sound[i] &= 0x7F;
            self.sound[i + 1] &= 0x7F;
        }
        v
    }

    pub fn sound_write16(&mut self, off: u32, v: u16) {
        self.sound_ensure();
        let i = (off - 0x400) as usize & !1;
        self.sound[i] = v as u8;
        self.sound[i + 1] = (v >> 8) as u8;
        if off < 0x500 && off & 0xF == 2 && v & 0x8000 != 0 {
            let ch = ((off - 0x400) >> 4) as usize;
            let b = ch * 0x10;
            let tmr = u16::from_le_bytes([self.sound[b + 8], self.sound[b + 9]]) as u64;
            let pnt = u16::from_le_bytes([self.sound[b + 0xA], self.sound[b + 0xB]]) as u64;
            let len = u32::from_le_bytes([
                self.sound[b + 0xC], self.sound[b + 0xD], self.sound[b + 0xE], self.sound[b + 0xF],
            ]) as u64;
            // Words -> samples by format: PCM8 x4, PCM16 x2, ADPCM x8.
            let samples = (pnt + len) * match v >> 13 & 3 { 0 => 4, 1 => 2, 2 => 8, _ => 1 };
            let cycles = samples.max(1) * (0x1_0000 - tmr).max(1) * 2;
            self.sound_end[ch] = self.now + (cycles / 2130).max(1);
        }
    }

    /// Read the 3D texture-image space (4 x 128KB slots): banks A-D with
    /// MST 3 map to slot OFS.
    pub fn tex_read8(&self, off: u32) -> u8 {
        let slot = (off >> 17 & 3) as u8;
        for i in 0..4 {
            let cnt = self.vramcnt[i];
            if cnt & 0x87 == 0x83 && cnt >> 3 & 3 == slot {
                return self.vram[i][(off & 0x1_FFFF) as usize];
            }
        }
        0
    }

    /// Read the 3D texture-palette space (6 x 16KB slots): bank E with MST 3
    /// covers slots 0-3, banks F/G map to slot (OFS.0) + (OFS.1)*4.
    pub fn texpal_read16(&self, off: u32) -> u16 {
        let off = off & 0x1_FFFE;
        let chunk = (off >> 14) as u8;
        let rd = |v: &Vec<u8>, o: usize| u16::from_le_bytes([v[o], v[o + 1]]);
        if chunk < 4 && self.vramcnt[4] & 0x87 == 0x83 {
            return rd(&self.vram[4], off as usize);
        }
        for i in [5usize, 6] {
            let cnt = self.vramcnt[i];
            if cnt & 0x87 == 0x83 {
                let ofs = cnt >> 3 & 3;
                if (ofs & 1) + (ofs >> 1 & 1) * 4 == chunk {
                    return rd(&self.vram[i], (off & 0x3FFF) as usize);
                }
            }
        }
        0
    }

    /// Read a BG extended-palette color: four 8KB slots, each holding 16
    /// palettes of 256 colors. Engine A takes bank E (all four slots) or
    /// banks F/G (two slots each, picked by OFS bit 0); engine B takes H.
    pub fn bg_extpal(&self, eng: usize, slot: usize, palnum: u32, color: u32) -> u16 {
        let off = slot * 0x2000 + palnum as usize * 0x200 + color as usize * 2;
        let rd = |v: &[u8], o: usize| u16::from_le_bytes([v[o], v[o + 1]]);
        if eng == 1 {
            return if self.vramcnt[7] & 0x87 == 0x82 { rd(&self.vram[7], off) } else { 0 };
        }
        if self.vramcnt[4] & 0x87 == 0x84 {
            return rd(&self.vram[4], off);
        }
        for i in [5usize, 6] {
            let cnt = self.vramcnt[i];
            if cnt & 0x87 == 0x84 {
                let first = (cnt >> 3 & 1) as usize * 2;
                if slot == first || slot == first + 1 {
                    return rd(&self.vram[i], off - first * 0x2000);
                }
            }
        }
        0
    }

    /// Read an OBJ extended-palette color: one 8KB slot of 16 palettes.
    /// Engine A takes bank F or G (MST 5), engine B takes bank I (MST 3).
    pub fn obj_extpal(&self, eng: usize, palnum: u32, color: u32) -> u16 {
        let off = palnum as usize * 0x200 + color as usize * 2;
        let rd = |v: &[u8], o: usize| u16::from_le_bytes([v[o], v[o + 1]]);
        if eng == 1 {
            return if self.vramcnt[8] & 0x87 == 0x83 { rd(&self.vram[8], off) } else { 0 };
        }
        for i in [5usize, 6] {
            if self.vramcnt[i] & 0x87 == 0x85 {
                return rd(&self.vram[i], off);
            }
        }
        0
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
    pub in_dma: bool,
}

/// Normalized DMA trigger kinds.
pub const DMA_IMM: u8 = 0;
pub const DMA_VBLANK: u8 = 1;
pub const DMA_CART: u8 = 2;

impl View {
    fn chan_mode(&self, cnt: u32) -> u8 {
        let cnt_h = (cnt >> 16) as u16;
        if self.cpu == 0 {
            match cnt_h >> 11 & 7 {
                0 => DMA_IMM,
                1 => DMA_VBLANK,
                5 => DMA_CART,
                // Geometry-FIFO DMA fires whenever the FIFO is under half
                // full; ours executes commands instantly so it always is -
                // treat it as immediate.
                7 => DMA_IMM,
                _ => 3,
            }
        } else {
            match cnt_h >> 12 & 3 {
                0 => DMA_IMM,
                1 => DMA_VBLANK,
                2 => DMA_CART,
                _ => 3,
            }
        }
    }

    /// Run every enabled channel matching `kind`. Called after I/O writes
    /// (immediate + cart) and from the scheduler (vblank).
    pub fn dma_service(&mut self, kind: u8) {
        if self.in_dma {
            return;
        }
        for ch in 0..4 {
            let (cnt, cart_ready) = {
                let m = self.m.borrow();
                (m.dma_cnt[self.cpu][ch], m.romctrl & 0x8000_0000 != 0)
            };
            if cnt & 0x8000_0000 == 0 || self.chan_mode(cnt) != kind {
                continue;
            }
            if kind == DMA_CART && !cart_ready {
                continue;
            }
            self.in_dma = true;
            self.dma_exec(ch, kind);
            self.in_dma = false;
        }
    }

    fn dma_exec(&mut self, ch: usize, kind: u8) {
        let cpu = self.cpu;
        let (mut src, mut dst, cnt) = {
            let m = self.m.borrow();
            (m.dma_src[cpu][ch], m.dma_dst[cpu][ch], m.dma_cnt[cpu][ch])
        };
        let cnt_h = (cnt >> 16) as u16;
        let word = cnt_h & 0x0400 != 0 || kind == DMA_CART;
        let step: u32 = if word { 4 } else { 2 };
        let dst_ctl = cnt_h >> 5 & 3;
        let src_ctl = cnt_h >> 7 & 3;
        let mut len = cnt & 0x1F_FFFF;
        if len == 0 {
            len = 0x20_0000;
        }
        if kind == DMA_CART {
            len = {
                let m = self.m.borrow();
                (m.cart_left as u32 / 4).max(1)
            };
        }
        for _ in 0..len {
            if kind == DMA_CART {
                let v = { self.m.borrow_mut().cart_read_word(cpu) };
                self.write32(dst, v);
            } else if word {
                let v = self.read32(src);
                self.write32(dst, v);
            } else {
                let v = self.read16(src);
                self.write16(dst, v);
            }
            match src_ctl {
                0 => src = src.wrapping_add(step),
                1 => src = src.wrapping_sub(step),
                _ => {}
            }
            match dst_ctl {
                0 | 3 => dst = dst.wrapping_add(step),
                1 => dst = dst.wrapping_sub(step),
                _ => {}
            }
        }
        let mut m = self.m.borrow_mut();
        m.dma_src[cpu][ch] = src;
        m.dma_dst[cpu][ch] = if dst_ctl == 3 { m.dma_dad[cpu][ch] } else { dst };
        if cnt_h & 0x4000 != 0 {
            m.request_irq(cpu, 1 << (8 + ch));
        }
        let repeat = cnt_h & 0x0200 != 0 && kind != DMA_IMM;
        if !repeat {
            m.dma_cnt[cpu][ch] &= !0x8000_0000;
        }
    }
}

impl View {
    fn io_read16(&mut self, a: u32) -> u16 {
        let mut m = self.m.borrow_mut();
        let cpu = self.cpu;
        let off = a & 0xFFFF;
        if let Some(log) = m.io_log.as_mut() {
            *log.entry((cpu, off, false)).or_insert(0) += 1;
        }
        match off {
            0x0400..=0x051F if cpu == 1 => m.sound_read16(off),
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
                // Bits 0-3: the other CPU's output; 8-11 + 13-14: our own.
                (m.ipcsync[1 - cpu] >> 8 & 0xF) | (m.ipcsync[cpu] & 0x6F00)
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
            0x00B0..=0x00DF => {
                let ch = ((off - 0xB0) / 12) as usize;
                let f = (off - 0xB0) % 12;
                match f {
                    0 => m.dma_sad[cpu][ch] as u16,
                    2 => (m.dma_sad[cpu][ch] >> 16) as u16,
                    4 => m.dma_dad[cpu][ch] as u16,
                    6 => (m.dma_dad[cpu][ch] >> 16) as u16,
                    8 => m.dma_cnt[cpu][ch] as u16,
                    _ => (m.dma_cnt[cpu][ch] >> 16) as u16,
                }
            }
            0x0100 | 0x0104 | 0x0108 | 0x010C => m.timer_val[cpu][(off as usize - 0x100) / 4] as u16,
            0x0102 | 0x0106 | 0x010A | 0x010E => m.timer_cnt[cpu][(off as usize - 0x102) / 4],
            0x01A0 => m.auxspicnt,
            0x01A2 => m.auxspi_out as u16,
            0x01A4 => m.romctrl as u16,
            0x01A6 => (m.romctrl >> 16) as u16,
            0x01A8 => u16::from_le_bytes([m.cart_cmd[0], m.cart_cmd[1]]),
            0x01AA => u16::from_le_bytes([m.cart_cmd[2], m.cart_cmd[3]]),
            0x01AC => u16::from_le_bytes([m.cart_cmd[4], m.cart_cmd[5]]),
            0x01AE => u16::from_le_bytes([m.cart_cmd[6], m.cart_cmd[7]]),
            // GXSTAT low: stack levels + test results from the geometry
            // engine. High half: FIFO count 0 (commands execute instantly),
            // so empty + less-than-half are always set - that keeps the
            // level-IRQ property Platinum's title sequence waits on.
            0x0600 if cpu == 0 => m.gx.gxstat_lo(),
            0x0602 if cpu == 0 => 0x0600 | (m.gxstat_irq as u16) << 14,
            // RAM_COUNT: polygons in bits 0-11, vertices in 16-28.
            0x0604 if cpu == 0 => m.gx.polys.len() as u16,
            0x0606 if cpu == 0 => m.gx.verts.len() as u16,
            // POS_TEST result.
            0x0620 if cpu == 0 => m.gx.pos_result[0] as u16,
            0x0622 if cpu == 0 => (m.gx.pos_result[0] >> 16) as u16,
            0x0624 if cpu == 0 => m.gx.pos_result[1] as u16,
            0x0626 if cpu == 0 => (m.gx.pos_result[1] >> 16) as u16,
            0x0628 if cpu == 0 => m.gx.pos_result[2] as u16,
            0x062A if cpu == 0 => (m.gx.pos_result[2] >> 16) as u16,
            0x062C if cpu == 0 => m.gx.pos_result[3] as u16,
            0x062E if cpu == 0 => (m.gx.pos_result[3] >> 16) as u16,
            // Matrix read-back: current clip matrix and vector-matrix rotation.
            0x0640..=0x067F if cpu == 0 => {
                let w = m.gx.clip_word(((off - 0x640) / 4) as usize);
                if off & 2 == 0 { w as u16 } else { (w >> 16) as u16 }
            }
            0x0680..=0x06A3 if cpu == 0 => {
                let w = m.gx.vec_word(((off - 0x680) / 4) as usize);
                if off & 2 == 0 { w as u16 } else { (w >> 16) as u16 }
            }
            0x0204 => m.exmemcnt,
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
            0x0138 if cpu == 1 => {
                rtc_trace("r", m.rtc_reg);
                m.rtc_reg
            }
            0x01C0 if cpu == 1 => m.spicnt, // busy bit never set: instant transfers
            0x01C2 if cpu == 1 => m.spi_out as u16,
            _ => 0,
        }
    }

    fn io_write16(&mut self, a: u32, v: u16) {
        let mut m = self.m.borrow_mut();
        let cpu = self.cpu;
        let off = a & 0xFFFF;
        if let Some(log) = m.io_log.as_mut() {
            *log.entry((cpu, off, true)).or_insert(0) += 1;
        }
        match off {
            0x0400..=0x051F if cpu == 1 => m.sound_write16(off, v),
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
                if m.io_log.is_some() && cpu == 1 {
                    static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
                    let n = COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    if n < 64 {
                        eprintln!("sync 7 writes {:#06X} (arm9 side={:#06X})", v, m.ipcsync[1 - cpu]);
                    }
                }
                m.ipcsync[cpu] = v & 0x4F00;
                if v & 0x2000 != 0 {
                    // Remote IRQ, only if the remote enabled it (its bit 14).
                    let other = 1 - cpu;
                    if m.ipcsync[other] & 0x4000 != 0 {
                        m.request_irq(other, 1 << 16);
                    }
                }
            }
            0x0184 => {
                let old = m.ipcfifocnt[cpu];
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
                // Level semantics: enabling an IRQ whose condition already
                // holds raises it immediately.
                let recv_nonempty =
                    if cpu == 0 { !m.fifo_to9.is_empty() } else { !m.fifo_to7.is_empty() };
                let send_empty =
                    if cpu == 0 { m.fifo_to7.is_empty() } else { m.fifo_to9.is_empty() };
                if v & 0x0400 != 0 && old & 0x0400 == 0 && recv_nonempty {
                    m.request_irq(cpu, IRQ_IPC_RECV);
                }
                if v & 0x0004 != 0 && old & 0x0004 == 0 && send_empty {
                    m.request_irq(cpu, IRQ_IPC_SEND_EMPTY);
                }
            }
            0x0600 if cpu == 0 => {
                if v & 0x8000 != 0 {
                    m.gx.stack_error = false; // ack matrix stack error
                }
            }
            0x0602 if cpu == 0 => {
                m.gxstat_irq = (v >> 14 & 3) as u8;
            }
            0x0350 if cpu == 0 => m.gx.clear_color = (m.gx.clear_color & 0xFFFF_0000) | v as u32,
            0x0352 if cpu == 0 => {
                m.gx.clear_color = (m.gx.clear_color & 0xFFFF) | (v as u32) << 16
            }
            0x0354 if cpu == 0 => m.gx.clear_depth = v & 0x7FFF,
            0x0204 => {
                if cpu == 0 {
                    m.exmemcnt = v;
                } else {
                    // ARM7 may only touch its own low bits.
                    m.exmemcnt = (m.exmemcnt & 0xFF80) | (v & 0x007F);
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
            0x00B0..=0x00DF => {
                let ch = ((off - 0xB0) / 12) as usize;
                let f = (off - 0xB0) % 12;
                let set16 = |x: &mut u32, hi: bool| {
                    *x = if hi { (*x & 0xFFFF) | (v as u32) << 16 } else { (*x & 0xFFFF_0000) | v as u32 };
                };
                match f {
                    0 => set16(&mut m.dma_sad[cpu][ch], false),
                    2 => set16(&mut m.dma_sad[cpu][ch], true),
                    4 => set16(&mut m.dma_dad[cpu][ch], false),
                    6 => set16(&mut m.dma_dad[cpu][ch], true),
                    8 => set16(&mut m.dma_cnt[cpu][ch], false),
                    _ => {
                        let was_on = m.dma_cnt[cpu][ch] & 0x8000_0000 != 0;
                        set16(&mut m.dma_cnt[cpu][ch], true);
                        if m.dma_cnt[cpu][ch] & 0x8000_0000 != 0 && !was_on {
                            m.dma_src[cpu][ch] = m.dma_sad[cpu][ch];
                            m.dma_dst[cpu][ch] = m.dma_dad[cpu][ch];
                        }
                    }
                }
            }
            0x0100 | 0x0104 | 0x0108 | 0x010C => {
                m.timer_reload[cpu][(off as usize - 0x100) / 4] = v;
            }
            0x0102 | 0x0106 | 0x010A | 0x010E => {
                let n = (off as usize - 0x102) / 4;
                let was_on = m.timer_cnt[cpu][n] & 0x80 != 0;
                m.timer_cnt[cpu][n] = v;
                if v & 0x80 != 0 && !was_on {
                    m.timer_val[cpu][n] = m.timer_reload[cpu][n] as u32;
                    m.timer_acc[cpu][n] = 0;
                }
            }
            0x01A0 => {
                m.auxspicnt = v;
                // Disabling AUXSPI deselects immediately; a cleared hold bit
                // only takes effect after the in-flight byte (handled in
                // auxspi_transfer).
                if v & 0x8000 == 0 {
                    m.aux_deselect();
                }
            }
            0x01A2 => {
                if m.auxspicnt & 0x8000 != 0 {
                    m.auxspi_transfer(v as u8);
                }
            }
            0x01A4 => m.romctrl = (m.romctrl & 0xFFFF_0000) | v as u32,
            0x01A6 => {
                m.romctrl = (m.romctrl & 0xFFFF) | (v as u32) << 16;
                if v & 0x8000 != 0 {
                    m.cart_start(cpu);
                }
            }
            0x01A8 => {
                m.cart_cmd[0] = v as u8;
                m.cart_cmd[1] = (v >> 8) as u8;
            }
            0x01AA => {
                m.cart_cmd[2] = v as u8;
                m.cart_cmd[3] = (v >> 8) as u8;
            }
            0x01AC => {
                m.cart_cmd[4] = v as u8;
                m.cart_cmd[5] = (v >> 8) as u8;
            }
            0x01AE => {
                m.cart_cmd[6] = v as u8;
                m.cart_cmd[7] = (v >> 8) as u8;
            }
            0x0138 if cpu == 1 => m.rtc_write(v),
            0x01C0 if cpu == 1 => m.spicnt = v,
            0x01C2 if cpu == 1 => {
                if m.spicnt & 0x8000 != 0 {
                    m.spi_transfer(v as u8);
                    if m.spicnt & 0x4000 != 0 {
                        m.request_irq(1, 1 << 23); // SPI transfer-done IRQ
                    }
                }
            }
            0x0304 => m.powcnt1 = v as u32,
            _ => {}
        }
    }

    fn io_read32(&mut self, a: u32) -> u32 {
        if a & 0xFF_FFFF == 0x0184 {
            return self.io_read16(a) as u32;
        }
        if a & 0xFF_FFFF == 0x10_0010 {
            // Cartridge data window.
            let cpu = self.cpu;
            return self.m.borrow_mut().cart_read_word(cpu);
        }
        if a & 0xFF_FFFF == 0x10_0000 {
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
                if m.io_log.is_some() {
                    static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
                    if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 500_000 {
                        eprintln!("[t={}] fifo {} recvs {:#010X} pc={:#010X}", m.now, cpu, v, m.last_pc[cpu]);
                    }
                }
                m.fifo_last[last] = v;
                // Sender's "send empty" IRQ.
                let sender = 1 - cpu;
                let empty = if cpu == 0 { m.fifo_to9.is_empty() } else { m.fifo_to7.is_empty() };
                if empty && m.ipcfifocnt[sender] & 0x0004 != 0 {
                    m.request_irq(sender, IRQ_IPC_SEND_EMPTY);
                }
                // Still non-empty: keep the reader's recv IRQ asserted.
                if !empty && m.ipcfifocnt[cpu] & 0x0400 != 0 {
                    m.request_irq(cpu, IRQ_IPC_RECV);
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
        match a & 0xFF_FFFF {
            0x01A4 => {
                let mut m = self.m.borrow_mut();
                m.romctrl = v & !0x0080_0000;
                if v & 0x8000_0000 != 0 {
                    let cpu = self.cpu;
                    m.cart_start(cpu);
                }
            }
            // Geometry engine: GXFIFO packed port at 0x400 (mirrored to
            // 0x43F) and per-command register ports at 0x440-0x5FF where
            // cmd = (addr >> 2) & 0x7F. Executes instantly; gx_cmds stays as
            // the NDS_VIDLOG diagnostic counter.
            off @ 0x0400..=0x05FF if self.cpu == 0 => {
                let mut m = self.m.borrow_mut();
                m.gx_cmds += 1;
                if off < 0x0440 {
                    m.gx.write_fifo(v);
                } else {
                    m.gx.write_port((off >> 2 & 0x7F) as u8, v);
                }
            }
            0x01A8 => {
                self.m.borrow_mut().cart_cmd[0..4].copy_from_slice(&v.to_le_bytes());
            }
            0x01AC => {
                self.m.borrow_mut().cart_cmd[4..8].copy_from_slice(&v.to_le_bytes());
            }
            0x0188 => {
                // IPCFIFOSEND
                let mut m = self.m.borrow_mut();
                let cpu = self.cpu;
                if m.io_log.is_some() {
                    static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
                    if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 500_000 {
                        eprintln!("[t={}] fifo {} sends {:#010X} pc={:#010X}", m.now, cpu, v, m.last_pc[cpu]);
                    }
                }
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
        if a >> 16 == 0x027F && self.cpu == 1 {
            let mut m = self.m.borrow_mut();
            if let Some(log) = m.io_log.as_mut() {
                *log.entry((9, a & !3, false)).or_insert(0) += 1; // pseudo-cpu 9 = RAM watch
            }
        }
        if let Some((watch, len)) = *WATCH_ADDR {
            if a >= watch && a < watch + len {
                let m = self.m.borrow();
                eprintln!(
                    "[t={}] watch READ {:#010X} by cpu{} pc={:#010X}",
                    m.now, a, if self.cpu == 0 { 9 } else { 7 }, m.last_pc[self.cpu]
                );
            }
        }
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
                    m.vram_arm7_slot(a).map_or(0, |(b, off)| m.vram[b][off])
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
        if let Some((watch, len)) = *WATCH_ADDR {
            if a >= watch && a < watch + len {
                eprintln!(
                    "[t={}] watch WRITE {:#010X} <- {:#04X} by cpu{} pc={:#010X}",
                    m.now, a, v, if cpu == 0 { 9 } else { 7 }, m.last_pc[cpu]
                );
            }
        }
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
            0x05 if cpu == 0 => {
                if a & 0x7FF < 2 && m.io_log.is_some() {
                    eprintln!("pal0 write {:#04X} from pc9={:#010X}", v, m.last_pc[0]);
                }
                m.pal[(a & 0x7FF) as usize] = v;
            }
            0x06 => {
                let slot = if cpu == 0 { m.vram_slot(a) } else { m.vram_arm7_slot(a) };
                if let Some((b, off)) = slot {
                    m.vram[b][off] = v;
                }
            }
            0x07 if cpu == 0 => m.oam[(a & 0x7FF) as usize] = v,
            _ => {}
        }
    }

    fn is_io(&self, a: u32) -> bool {
        a >> 24 == 0x04 && a & 0x0080_0000 == 0
    }

    fn is_wifi(&self, a: u32) -> bool {
        self.cpu == 1 && a >> 24 == 0x04 && a & 0x0080_0000 != 0
    }

    fn wifi_read16(&mut self, a: u32) -> u16 {
        let m = self.m.borrow();
        let off = (a & 0xFFFF) as usize;
        match off {
            0x8000 => 0x1440, // W_ID: DS wifi chipset
            0x815C => m.bb_read, // W_BB_READ: last baseband register read back
            0x815E => 0,      // W_BB_BUSY: never busy
            0x8180 | 0x8184 => 0, // W_RF_BUSY / RF control: ready
            // Wireless-manager transfer poll: after the driver kicks a command
            // (0x8001 -> 0x8040) it waits for status 0x803C high byte == 2 and
            // 0x8214 in {0,9}. With no real wireless engine, report the
            // transfer as immediately complete so the manager proceeds.
            0x803C => 0x0200,
            0x8214 => 0,
            _ => u16::from_le_bytes([m.wifi[off], m.wifi[off + 1 & 0xFFFF]]),
        }
    }

    fn wifi_write16(&mut self, a: u32, v: u16) {
        let mut m = self.m.borrow_mut();
        let off = (a & 0xFFFF) as usize;
        m.wifi[off] = v as u8;
        m.wifi[(off + 1) & 0xFFFF] = (v >> 8) as u8;
        // W_BB_CNT (0x8158): a baseband register access. Bits 0-7 = index,
        // bits 12-14 = direction (5 = write W_BB_WRITE into the reg, 6 = read
        // the reg into W_BB_READ). The wireless manager writes a reg then reads
        // it back to confirm the chip is alive, so the file must remember what
        // was written. BB[0] is the chip version the driver checks (0x6D).
        if off == 0x8158 {
            if m.bb.len() < 0x100 {
                m.bb = vec![0; 0x100];
                m.bb[0] = 0x6D;
            }
            let idx = (v & 0xFF) as usize;
            match (v >> 12) & 0xF {
                5 => m.bb[idx] = m.wifi[0x815A], // W_BB_WRITE low byte
                6 => m.bb_read = m.bb[idx] as u16,
                _ => {}
            }
        }
    }
}

impl Bus for View {
    fn read8(&mut self, a: u32) -> u8 {
        if self.is_wifi(a) {
            let v = self.wifi_read16(a & !1);
            return (v >> ((a & 1) * 8)) as u8;
        }
        if self.is_io(a) {
            let v = self.io_read16(a & !1);
            (v >> ((a & 1) * 8)) as u8
        } else {
            self.mem_read8(a)
        }
    }

    fn read16(&mut self, a: u32) -> u16 {
        let a = a & !1;
        if self.is_wifi(a) {
            self.wifi_read16(a)
        } else if self.is_io(a) {
            self.io_read16(a)
        } else {
            u16::from_le_bytes([self.mem_read8(a), self.mem_read8(a + 1)])
        }
    }

    fn read32(&mut self, a: u32) -> u32 {
        let a = a & !3;
        if self.is_wifi(a) {
            return self.wifi_read16(a) as u32 | (self.wifi_read16(a + 2) as u32) << 16;
        }
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
        if self.is_wifi(a) {
            let cur = self.wifi_read16(a & !1);
            let nv = if a & 1 == 0 { (cur & 0xFF00) | v as u16 } else { (cur & 0xFF) | (v as u16) << 8 };
            self.wifi_write16(a & !1, nv);
            return;
        }
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
        if self.is_wifi(a) {
            self.wifi_write16(a, v);
        } else if self.is_io(a) {
            self.io_write16(a, v);
            self.dma_service(DMA_IMM);
            self.dma_service(DMA_CART);
        } else {
            self.mem_write8(a, v as u8);
            self.mem_write8(a + 1, (v >> 8) as u8);
        }
    }

    fn write32(&mut self, a: u32, v: u32) {
        let a = a & !3;
        if self.is_wifi(a) {
            self.wifi_write16(a, v as u16);
            self.wifi_write16(a + 2, (v >> 16) as u16);
            return;
        }
        if self.is_io(a) {
            self.io_write32(a, v);
            self.dma_service(DMA_IMM);
            self.dma_service(DMA_CART);
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

    fn note_pc(&mut self, pc: u32) {
        let cpu = self.cpu;
        self.m.borrow_mut().last_pc[cpu] = pc;
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
