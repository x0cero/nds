//! DS wireless hardware (the "NiFi" 802.11b MAC at 0x04808000 plus its 8KB of
//! packet RAM at 0x04804000).
//!
//! Register map, buffer layouts and multiplay semantics follow GBATEK's DS
//! Wireless Communications chapters. Only what Nintendo's own wireless library
//! uses is implemented; the many W_INTERNAL diagnostic ports stay plain RAM.

use crate::bus::Machine;

/// Packet RAM occupies 4804000h..4805FFFh, i.e. offsets 0x4000..0x6000 of the
/// 64KB wifi window that `Machine::wifi` backs.
const RAM: usize = 0x4000;

/// ARM7 cycles per microsecond, scaled by 1000 (33.513982 MHz).
const CYCLES_PER_US_K: u64 = 33514;

/// W_IF bit numbers (GBATEK "DS Wifi Interrupts").
const IRQ_RX_END: u32 = 0;
const IRQ_TX_END: u32 = 1;
const IRQ_RX_START: u32 = 6;
const IRQ_TX_START: u32 = 7;
const IRQ_TXBUF_COUNT: u32 = 8;
const IRQ_RXBUF_COUNT: u32 = 9;
const IRQ_RF_WAKEUP: u32 = 11;
const IRQ_CMD_DONE: u32 = 12;
const IRQ_POST_BEACON: u32 = 13;
const IRQ_BEACON: u32 = 14;
const IRQ_PRE_BEACON: u32 = 15;

/// Wireless state that is derived rather than register-backed. The registers
/// themselves live in `Machine::wifi` so that savestates written before this
/// existed still load; this struct is `#[serde(skip)]` for the same reason,
/// and the driver re-initialises the whole chip on every use anyway.
pub struct Wifi {
    /// ARM7 cycles (x1000) not yet converted into whole microseconds.
    us_acc: u64,
    /// Baseband register file. The wireless manager writes each register and
    /// reads it back to prove the radio chip answers; BB[0] is the chip
    /// version it checks.
    pub bb: Vec<u8>,
    pub bb_read: u16,
    /// RF chip (RF9008) registers, written as 18-bit words via W_RF_DATA1/2.
    rf: [u32; 32],
    /// W_RANDOM shift register.
    lfsr: u16,
    /// Frames received from the air, waiting to be written into the RX buffer.
    rx_queue: std::collections::VecDeque<(Vec<u8>, u8)>,
    /// The multiplay command this console (as host) is waiting on replies for.
    cmd: Option<Cmd>,
    /// The shared medium, when this console is linked to another one, plus
    /// which console we are on it.
    pub air: Option<std::rc::Rc<std::cell::RefCell<Air>>>,
    pub id: usize,
}

/// The air between linked consoles. Two DS machines running in one process
/// share one of these; transmitting drops the frame into every other
/// console's inbox, which is as close to a radio as this needs to be.
#[derive(Default)]
pub struct Air {
    /// Per-console inbox. Each frame carries the client number of whoever
    /// sent it: on real hardware the host identifies a reply by WHEN it
    /// arrives in the slot sequence, and that timing is not reproducible once
    /// the frame is a byte array in a queue.
    inbox: Vec<std::collections::VecDeque<(Vec<u8>, u8)>>,
}

impl Air {
    pub fn new(consoles: usize) -> Self {
        Self { inbox: (0..consoles).map(|_| Default::default()).collect() }
    }
}

impl Default for Wifi {
    fn default() -> Self {
        let mut bb = vec![0u8; 0x100];
        bb[0] = 0x6D; // chip version the driver's self-test compares against
        Self {
            us_acc: 0,
            bb,
            bb_read: 0,
            rf: [0; 32],
            lfsr: 1,
            rx_queue: Default::default(),
            cmd: None,
            air: None,
            id: 0,
        }
    }
}

/// Power-on register defaults (GBATEK's [init] column). Only the ports whose
/// initial value the driver can actually observe before writing them.
pub fn reset(m: &mut Machine) {
    set(m, 0x803C, 0x0200); // W_POWERSTATE: powered down
    set(m, 0x819C, 0x0004); // W_RF_PINS: RFU pin3 high, RX and TX off
    set(m, 0x8214, 0x0000); // W_RF_STATUS: pre-W_MODE_RST value
    set(m, 0x80B0, 0x0010); // W_TXREQ_READ: beacon slot always enabled
    set(m, 0x802C, 0x0707); // W_TX_RETRYLIMIT
    set(m, 0x808C, 0x0064); // W_BEACONINT
    set(m, 0x80F0, 0xFC00); // W_US_COMPARE0
    for off in [0x80F2, 0x80F4, 0x80F6] {
        set(m, off, 0xFFFF); // W_US_COMPARE1-3: practically never
    }
    set(m, 0x8134, 0xFFFF); // W_POST_BEACON
}

fn get(m: &Machine, off: usize) -> u16 {
    u16::from_le_bytes([m.wifi[off], m.wifi[off + 1]])
}

fn set(m: &mut Machine, off: usize, v: u16) {
    m.wifi[off] = v as u8;
    m.wifi[off + 1] = (v >> 8) as u8;
}

/// Byte offset within packet RAM of a buffer-limit register (BEGIN/END/GAP and
/// the read/write cursors that carry the 0x4000 base in bit 14).
fn buf_off(v: u16) -> usize {
    v as usize & 0x1FFE
}

/// Byte offset within packet RAM of a halfword-address register (the WRCSR /
/// READCSR / WR_ADDR cursors and the W_TXBUF_* transmit locations).
fn hw_off(v: u16) -> usize {
    (v as usize & 0xFFF) * 2
}

/// Raise a wifi interrupt. The ARM7's IF bit 24 is edge-triggered on
/// (W_IF AND W_IE) going from zero to non-zero, so a second wifi IRQ raised
/// while the handler has not yet cleared the first one does NOT re-signal the
/// CPU. Nintendo's handler loops until W_IF AND W_IE is zero for that reason.
fn raise(m: &mut Machine, bit: u32) {
    let ie = get(m, 0x8012);
    let old = get(m, 0x8010) & ie;
    let new = get(m, 0x8010) | (1 << bit) as u16;
    set(m, 0x8010, new);
    let signalled = old == 0 && new & ie != 0;
    if signalled {
        m.request_irq(1, 1 << 24);
    }
    if *crate::bus::WIFILOG {
        const NAMES: [&str; 16] = [
            "rx-done", "tx-done", "rx-count", "tx-error", "rx-overflow", "tx-overflow",
            "rx-start", "tx-start", "txbuf-empty", "rxbuf-empty", "?", "rf-wakeup",
            "cmd-done", "post-beacon", "beacon", "pre-beacon",
        ];
        eprintln!(
            "[wifi] c{} {:>8} IRQ {} ({}), wifi ie={ie:04X} arm7 ie={:08X}",
            m.wf.id,
            m.now,
            NAMES[bit as usize],
            if signalled { "signalled the cpu" } else if ie >> bit & 1 == 0 {
                "masked in wifi ie"
            } else {
                "cpu already had an unacknowledged wifi irq"
            },
            m.ie[1],
        );
    }
}

/// Human-readable name for a frame, from its 802.11 frame-control field, so
/// a link trace reads as a conversation rather than as hex.
pub fn frame_name(fc: u16) -> &'static str {
    match fc & 0x0FFF {
        0x0228 => "MP-CMD",
        0x0218 => "MP-ACK",
        0x0118 => "MP-REPLY",
        0x0158 => "MP-REPLY(empty)",
        _ => match (fc >> 2 & 3, fc >> 4 & 0xF) {
            (0, 0) => "assoc-req",
            (0, 1) => "assoc-resp",
            (0, 2) => "reassoc-req",
            (0, 3) => "reassoc-resp",
            (0, 4) => "probe-req",
            (0, 5) => "probe-resp",
            (0, 8) => "beacon",
            (0, 10) => "disassoc",
            (0, 11) => "auth",
            (0, 12) => "deauth",
            (0, _) => "mgmt",
            (1, 10) => "ps-poll",
            (1, _) => "control",
            (2, _) => "data",
            _ => "?",
        },
    }
}

/// Human-readable name for a wifi-region offset (address & 0xFFFF), used by
/// NDS_WIFILOG. Unknown ports print as their offset so a log line is still
/// greppable.
pub fn reg_name(off: u32) -> String {
    if (0x4000..0x6000).contains(&off) {
        return format!("RAM[{off:04X}]");
    }
    let n = match off {
        0x8000 => "W_ID",
        0x8004 => "W_MODE_RST",
        0x8006 => "W_MODE_WEP",
        0x8008 => "W_TXSTATCNT",
        0x800A => "W_X_00A",
        0x8010 => "W_IF",
        0x8012 => "W_IE",
        0x8018 => "W_MACADDR_0",
        0x801A => "W_MACADDR_1",
        0x801C => "W_MACADDR_2",
        0x8020 => "W_BSSID_0",
        0x8022 => "W_BSSID_1",
        0x8024 => "W_BSSID_2",
        0x8028 => "W_AID_LOW",
        0x802A => "W_AID_FULL",
        0x802C => "W_TX_RETRYLIMIT",
        0x8030 => "W_RXCNT",
        0x8032 => "W_WEP_CNT",
        0x8036 => "W_POWER_US",
        0x8038 => "W_POWER_TX",
        0x803C => "W_POWERSTATE",
        0x8040 => "W_POWERFORCE",
        0x8044 => "W_RANDOM",
        0x8048 => "W_POWER_048",
        0x8050 => "W_RXBUF_BEGIN",
        0x8052 => "W_RXBUF_END",
        0x8054 => "W_RXBUF_WRCSR",
        0x8056 => "W_RXBUF_WR_ADDR",
        0x8058 => "W_RXBUF_RD_ADDR",
        0x805A => "W_RXBUF_READCSR",
        0x805C => "W_RXBUF_COUNT",
        0x8060 => "W_RXBUF_RD_DATA",
        0x8062 => "W_RXBUF_GAP",
        0x8064 => "W_RXBUF_GAPDISP",
        0x8068 => "W_TXBUF_WR_ADDR",
        0x806C => "W_TXBUF_COUNT",
        0x8070 => "W_TXBUF_WR_DATA",
        0x8074 => "W_TXBUF_GAP",
        0x8076 => "W_TXBUF_GAPDISP",
        0x8080 => "W_TXBUF_BEACON",
        0x8084 => "W_TXBUF_TIM",
        0x8088 => "W_LISTENCOUNT",
        0x808C => "W_BEACONINT",
        0x808E => "W_LISTENINT",
        0x8090 => "W_TXBUF_CMD",
        0x8094 => "W_TXBUF_REPLY1",
        0x8098 => "W_TXBUF_REPLY2",
        0x80A0 => "W_TXBUF_LOC1",
        0x80A4 => "W_TXBUF_LOC2",
        0x80A8 => "W_TXBUF_LOC3",
        0x80AC => "W_TXREQ_RESET",
        0x80AE => "W_TXREQ_SET",
        0x80B0 => "W_TXREQ_READ",
        0x80B4 => "W_TXBUF_RESET",
        0x80B6 => "W_TXBUSY",
        0x80B8 => "W_TXSTAT",
        0x80BC => "W_PREAMBLE",
        0x80C0 => "W_CMD_TOTALTIME",
        0x80C4 => "W_CMD_REPLYTIME",
        0x80D0 => "W_RXFILTER",
        0x80D4 => "W_CONFIG_0D4",
        0x80D8 => "W_CONFIG_0D8",
        0x80DA => "W_RX_LEN_CROP",
        0x80E0 => "W_RXFILTER2",
        0x80E8 => "W_US_COUNTCNT",
        0x80EA => "W_US_COMPARECNT",
        0x80EC => "W_CONFIG_0EC",
        0x80EE => "W_CMD_COUNTCNT",
        0x80F0 => "W_US_COMPARE0",
        0x80F2 => "W_US_COMPARE1",
        0x80F4 => "W_US_COMPARE2",
        0x80F6 => "W_US_COMPARE3",
        0x80F8 => "W_US_COUNT0",
        0x80FA => "W_US_COUNT1",
        0x80FC => "W_US_COUNT2",
        0x80FE => "W_US_COUNT3",
        0x810C => "W_CONTENTFREE",
        0x8110 => "W_PRE_BEACON",
        0x8118 => "W_CMD_COUNT",
        0x811C => "W_BEACON_COUNT",
        0x8134 => "W_POST_BEACON",
        0x8158 => "W_BB_CNT",
        0x815A => "W_BB_WRITE",
        0x815C => "W_BB_READ",
        0x815E => "W_BB_BUSY",
        0x8160 => "W_BB_MODE",
        0x8168 => "W_BB_POWER",
        0x817C => "W_RF_DATA2",
        0x817E => "W_RF_DATA1",
        0x8180 => "W_RF_BUSY",
        0x8184 => "W_RF_CNT",
        0x8194 => "W_TX_HDR_CNT",
        0x819C => "W_RF_PINS",
        0x81A0 => "W_X_1A0",
        0x81A2 => "W_X_1A2",
        0x81A4 => "W_X_1A4",
        0x81A8 => "W_RXSTAT_INC_IF",
        0x81AA => "W_RXSTAT_INC_IE",
        0x81AC => "W_RXSTAT_OVF_IF",
        0x81AE => "W_RXSTAT_OVF_IE",
        0x81C0 => "W_TX_ERR_COUNT",
        0x81C4 => "W_RX_COUNT",
        0x8210 => "W_TX_SEQNO",
        0x8214 => "W_RF_STATUS",
        0x821C => "W_IF_SET",
        0x8220 => "W_RAM_DISABLE",
        0x8268 => "W_RXTX_ADDR",
        _ => return format!("[{off:04X}]"),
    };
    n.to_string()
}

// ---------------------------------------------------------------------------
// Packet RAM helpers. Offsets are relative to the start of packet RAM.
// ---------------------------------------------------------------------------

fn ram16(m: &Machine, off: usize) -> u16 {
    let o = RAM + (off & 0x1FFF);
    u16::from_le_bytes([m.wifi[o], m.wifi[o + 1]])
}

fn set_ram16(m: &mut Machine, off: usize, v: u16) {
    let o = RAM + (off & 0x1FFF);
    m.wifi[o] = v as u8;
    m.wifi[o + 1] = (v >> 8) as u8;
}

fn ram8(m: &Machine, off: usize) -> u8 {
    m.wifi[RAM + (off & 0x1FFF)]
}

fn set_ram8(m: &mut Machine, off: usize, v: u8) {
    m.wifi[RAM + (off & 0x1FFF)] = v;
}

// ---------------------------------------------------------------------------
// Power state
// ---------------------------------------------------------------------------

/// The radio comes up: receive mode on, and the wakeup interrupt the wireless
/// manager waits for. Nintendo's driver then polls W_RF_PINS until RX.ON reads
/// back high, so this is the write that unblocks the whole stack.
fn power_up(m: &mut Machine) {
    if get(m, 0x803C) & 0x0200 == 0 {
        return; // already enabled
    }
    set(m, 0x803C, get(m, 0x803C) & !0x0300);
    set(m, 0x819C, 0x0084); // W_RF_PINS: RX.ON high
    set(m, 0x8214, 1); // W_RF_STATUS: RX mode
    raise(m, IRQ_RF_WAKEUP);
}

fn power_down(m: &mut Machine) {
    set(m, 0x803C, (get(m, 0x803C) & !0x0100) | 0x0200);
    set(m, 0x80B0, 0); // W_TXREQ_READ
    set(m, 0x819C, 0x0046); // W_RF_PINS: RX.ON low
    set(m, 0x8214, 9); // W_RF_STATUS: idle
}

fn powered(m: &Machine) -> bool {
    get(m, 0x803C) & 0x0200 == 0 && get(m, 0x8004) & 1 != 0
}

// ---------------------------------------------------------------------------
// Register reads and writes
// ---------------------------------------------------------------------------

pub fn read16(m: &mut Machine, off: usize) -> u16 {
    let off = off & 0xFFFE;
    match off {
        0x8000 => 0x1440, // W_ID: original DS chipset
        0x8044 => {
            // W_RANDOM: 11-bit shift register the driver uses to scatter its
            // beacon interval. Any decent sequence will do.
            let l = m.wf.lfsr;
            m.wf.lfsr = ((l >> 1) ^ if l & 1 != 0 { 0x0420 } else { 0 }) & 0x7FF;
            m.wf.lfsr
        }
        0x8060 => rxbuf_read_data(m),
        0x8078 => get(m, 0x8068), // mirror of W_TXBUF_WR_ADDR
        0x815C => m.wf.bb_read,
        0x815E => 0, // W_BB_BUSY: transfers complete instantly
        0x8180 => 0, // W_RF_BUSY
        _ => get(m, off),
    }
}

pub fn write16(m: &mut Machine, off: usize, v: u16) {
    let off = off & 0xFFFE;
    // Packet RAM and the many diagnostic ports are plain memory.
    if !(0x8000..0x9000).contains(&off) {
        set(m, off, v);
        return;
    }
    match off {
        0x8010 => {
            // W_IF: writing a 1 acknowledges that interrupt.
            let cur = get(m, 0x8010);
            set(m, 0x8010, cur & !v);
        }
        0x821C => {
            // W_IF_SET: force interrupt flags (bit 10 does not exist).
            let ie = get(m, 0x8012);
            let old = get(m, 0x8010) & ie;
            let new = get(m, 0x8010) | (v & !0x0400);
            set(m, 0x8010, new);
            if old == 0 && new & ie != 0 {
                m.request_irq(1, 1 << 24);
            }
        }
        0x8012 => {
            // W_IE: enabling an already-pending flag signals the CPU.
            let pending = get(m, 0x8010);
            let old = pending & get(m, 0x8012);
            set(m, 0x8012, v);
            if old == 0 && pending & v != 0 {
                m.request_irq(1, 1 << 24);
            }
        }
        0x8004 => {
            // W_MODE_RST: bit0 enables the MAC.
            let was = get(m, 0x8004) & 1;
            set(m, 0x8004, v);
            if was == 0 && v & 1 != 0 {
                set(m, 0x8214, 9); // W_RF_STATUS: idle
            } else if v & 1 == 0 {
                set(m, 0x8214, 0);
                set(m, 0x80B6, 0); // W_TXBUSY
                m.wf.rx_queue.clear();
            }
        }
        0x803C => {
            // W_POWERSTATE: bit1 queues "power enable".
            set(m, 0x803C, (get(m, 0x803C) & 0xFF00) | (v & 0x03));
            if v & 2 != 0 && get(m, 0x8036) & 1 == 0 && get(m, 0x8004) & 1 != 0 {
                power_up(m);
            }
        }
        0x8040 => {
            // W_POWERFORCE: bit15 applies bit0 to the power state at once.
            set(m, 0x8040, v);
            if v & 0x8000 != 0 {
                if v & 1 != 0 {
                    power_down(m);
                } else {
                    power_up(m);
                }
            }
        }
        0x8030 => {
            // W_RXCNT: bit0 latches the write cursor, bit7 hand-forwards the
            // queued multiplay reply.
            set(m, 0x8030, v & 0xFF0E);
            if v & 1 != 0 {
                let wr = get(m, 0x8056);
                set(m, 0x8054, wr); // W_RXBUF_WRCSR
            }
            if v & 0x80 != 0 {
                let r1 = get(m, 0x8094);
                set(m, 0x8098, r1); // W_TXBUF_REPLY2
                set(m, 0x8094, 0);
            }
        }
        0x8070 => {
            // W_TXBUF_WR_DATA: writes through to packet RAM, advancing the
            // cursor and wrapping at the software-defined gap.
            let addr = get(m, 0x8068);
            set_ram16(m, buf_off(addr), v);
            let mut next = (buf_off(addr) + 2) & 0x1FFE;
            let gap = buf_off(get(m, 0x8074));
            if gap != 0 && next == gap {
                next = (next + (get(m, 0x8076) as usize & 0xFFF) * 2) & 0x1FFE;
            }
            set(m, 0x8068, next as u16);
            let count = get(m, 0x806C) & 0xFFF;
            if count > 0 {
                set(m, 0x806C, count - 1);
                if count == 1 {
                    raise(m, IRQ_TXBUF_COUNT);
                }
            }
        }
        0x80AC => {
            // W_TXREQ_RESET / W_TXREQ_SET only touch bits 0-3 of the readable
            // request register.
            let cur = get(m, 0x80B0);
            set(m, 0x80B0, cur & !(v & 0xF));
        }
        0x80AE => {
            let cur = get(m, 0x80B0);
            set(m, 0x80B0, cur | (v & 0xF));
        }
        0x80B4 => {
            // W_TXBUF_RESET: clear the enable bit of the named slots.
            for (bit, loc) in [(0, 0x80A0), (1, 0x8090), (2, 0x80A4), (3, 0x80A8), (6, 0x8098), (7, 0x8094)] {
                if v >> bit & 1 != 0 {
                    let cur = get(m, loc);
                    set(m, loc, cur & 0x7FFF);
                }
            }
        }
        0x8158 => {
            // W_BB_CNT: bits 0-7 index, bits 12-14 direction (5 = write the
            // byte in W_BB_WRITE, 6 = latch the register into W_BB_READ).
            set(m, 0x8158, v);
            let idx = (v & 0xFF) as usize;
            match (v >> 12) & 0xF {
                5 => m.wf.bb[idx] = m.wifi[0x815A],
                6 => m.wf.bb_read = m.wf.bb[idx] as u16,
                _ => {}
            }
        }
        0x8184 => {
            // W_RF_CNT: bit5 selects a read; the RF chip's registers are
            // write-only in practice, so a read just returns what was written.
            set(m, 0x8184, v);
            let data = (get(m, 0x817E) as u32) | (get(m, 0x817C) as u32) << 16;
            let idx = (data >> 18) as usize & 0x1F;
            if data & 0x0080_0000 == 0 {
                m.wf.rf[idx] = data & 0x3FFFF;
            }
        }
        _ => set(m, off, v),
    }
}

/// W_RXBUF_RD_DATA: hand the CPU the halfword under the read cursor and step
/// the cursor, honouring both the buffer end and the software gap.
fn rxbuf_read_data(m: &mut Machine) -> u16 {
    let addr = buf_off(get(m, 0x8058));
    let v = ram16(m, addr);
    let mut next = (addr + 2) & 0x1FFE;
    if next == buf_off(get(m, 0x8052)) {
        next = buf_off(get(m, 0x8050));
    }
    let gap = buf_off(get(m, 0x8062));
    if gap != 0 && next == gap {
        next += (get(m, 0x8064) as usize & 0xFFF) * 2;
        let end = buf_off(get(m, 0x8052));
        if next >= end {
            next = next + buf_off(get(m, 0x8050)) - end;
        }
    }
    set(m, 0x8058, next as u16);
    let count = get(m, 0x805C) & 0xFFF;
    if count > 0 {
        set(m, 0x805C, count - 1);
        if count == 1 {
            raise(m, IRQ_RXBUF_COUNT);
        }
    }
    v
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

/// Transmit slots in hardware priority order: LOC3 goes out first, LOC1 last.
/// Each entry is (W_TXREQ_READ bit, W_TXBUF_* register, W_TXSTAT packet code).
/// The multiplay command slot is not here: it starts an exchange rather than a
/// single transfer, and is handled by `start_cmd`.
const TX_SLOTS: [(u16, usize, u16); 3] = [(3, 0x80A8, 2), (2, 0x80A4, 1), (0, 0x80A0, 0)];

/// Nintendo's fixed addresses for the multiplay flows (GBATEK "Special NDS
/// related Addresses"). The hardware, not the game, puts these on the CMD
/// acknowledge and on empty replies.
const ADDR_HOST_ACK: [u8; 6] = [0x03, 0x09, 0xBF, 0x00, 0x00, 0x03];
const ADDR_CLIENT_REPLY: [u8; 6] = [0x03, 0x09, 0xBF, 0x00, 0x00, 0x10];

/// Build the frame a transmit location points at: the 12-byte hardware header
/// is stripped, the IEEE header gets the adjustments hardware makes, and the
/// trailing FCS is dropped (the receiving MAC strips it again anyway).
fn build_frame(m: &mut Machine, loc: u16, seq_from_hw: bool) -> Option<Vec<u8>> {
    let base = hw_off(loc);
    let len = ram16(m, base + 0x0A) as usize & 0x3FFF;
    if len < 4 + 12 {
        return None; // shorter than an IEEE header plus checksum
    }
    let body = len - 4; // hardware appends the FCS itself
    let mut f = Vec::with_capacity(body);
    for i in 0..body {
        f.push(ram8(m, base + 12 + i));
    }
    // Hardware forces the protocol-version bits to zero and tends to set the
    // power-management bit on the copy that goes out.
    let fc = u16::from_le_bytes([f[0], f[1]]) & !0x0003;
    f[0..2].copy_from_slice(&(fc | 0x1000).to_le_bytes());
    if seq_from_hw && f.len() >= 24 {
        let seq = get(m, 0x8210).wrapping_add(1) & 0xFFF;
        set(m, 0x8210, seq);
        f[22..24].copy_from_slice(&(seq << 4).to_le_bytes());
    }
    Some(f)
}

/// Run one pending transmit, if any. Returns whether something was sent.
fn service_tx(m: &mut Machine) -> bool {
    if !powered(m) || m.wf.cmd.is_some() {
        return false; // a command exchange owns the radio until it finishes
    }
    let req = get(m, 0x80B0);
    if req & 2 != 0 && get(m, 0x8090) & 0x8000 != 0 && get(m, 0x8118) != 0 {
        start_cmd(m);
        return true;
    }
    for (bit, loc_reg, code) in TX_SLOTS {
        if req >> bit & 1 == 0 {
            continue;
        }
        let loc = get(m, loc_reg);
        if loc & 0x8000 == 0 {
            continue;
        }
        set(m, 0x80B6, 1 << bit); // W_TXBUSY
        set(m, 0x8214, 3); // W_RF_STATUS: transmitting
        raise(m, IRQ_TX_START);
        let frame = build_frame(m, loc, loc & 0x2000 != 0);
        if let Some(f) = frame {
            if *crate::bus::WIFILOG {
                let fc = u16::from_le_bytes([f[0], f[1]]);
                eprintln!(
                    "[wifi] c{} {:>8} TX slot{bit} {} len={} fc={fc:04X}",
                    m.wf.id, m.now, frame_name(fc), f.len()
                );
            }
            send(m, f);
        }
        // Hardware writes the result back into the frame's own header.
        set_ram16(m, hw_off(loc), 0x0001);
        set_ram8(m, hw_off(loc) + 5, 0);
        set(m, loc_reg, loc & 0x7FFF);
        set(m, 0x80B6, 0);
        set(m, 0x8214, 1); // back to RX mode
        set(m, 0x819C, 0x0084);
        set(m, 0x80B8, 0x0001 | code << 12); // W_TXSTAT
        raise(m, IRQ_TX_END);
        return true;
    }
    false
}

// ---------------------------------------------------------------------------
// Multiplay: the host polls, every client answers in its own slot, the host
// acknowledges. This is the exchange DS local wireless games are built on, and
// the one the Pokemon Union Room uses.
// ---------------------------------------------------------------------------

/// A command the host has sent and is waiting on replies for.
pub struct Cmd {
    /// Bitmask of the clients this command asked to answer.
    slaves: u16,
    /// Which of them have answered so far.
    replied: u16,
    /// Microsecond clock reading at which the host gives up waiting.
    deadline: u64,
}

/// Send the multiplay command frame and open the reply window. The clients it
/// polls are named by a bitmask in the frame body, which is also where the
/// hardware reads them from (the copy in the transmit header is ignored).
fn start_cmd(m: &mut Machine) {
    let loc = get(m, 0x8090);
    let Some(f) = build_frame(m, loc, loc & 0x2000 != 0) else { return };
    let slaves = if f.len() >= 28 { u16::from_le_bytes([f[26], f[27]]) } else { 0 } & !1;
    set(m, 0x80B6, 2); // W_TXBUSY: the command slot
    set(m, 0x8214, 3); // W_RF_STATUS: transmitting
    set(m, 0x819C, 0x0046);
    raise(m, IRQ_TX_START);
    if *crate::bus::WIFILOG {
        eprintln!(
            "[wifi] c{} {:>8} TX cmd len={} slaves={slaves:04X}",
            m.wf.id,
            m.now,
            f.len()
        );
    }
    send(m, f);
    if get(m, 0x8008) & 0x4000 != 0 {
        set(m, 0x80B8, 0x0800); // W_TXSTAT: command data sent
        raise(m, IRQ_TX_END);
    }
    set(m, 0x8214, 5); // W_RF_STATUS: waiting for replies
    set(m, 0x819C, 0x0084);
    // How long hardware waits: a fixed turnaround plus one reply slot per
    // client that was polled.
    let per = 10 + get(m, 0x80C4) as u64;
    let wait = 16 + per * slaves.count_ones() as u64;
    m.wf.cmd = Some(Cmd { slaves, replied: 0, deadline: us_count(m) + wait });
}

/// Close the command exchange: acknowledge the replies, tell the driver which
/// clients answered, and raise the "command done" interrupt it waits on.
fn finish_cmd(m: &mut Machine) {
    let Some(cmd) = m.wf.cmd.take() else { return };
    let missing = cmd.slaves & !cmd.replied;
    set(m, 0x8214, 7); // switching from reply to acknowledge
    set(m, 0x8214, 8);
    raise(m, IRQ_TX_START);
    let mut ack = vec![0u8; 24];
    ack[0..2].copy_from_slice(&0x0218u16.to_le_bytes()); // data, FromDS, CF-Ack
    ack[4..10].copy_from_slice(&ADDR_HOST_ACK);
    // From the host: address 2 is the network, address 3 the sender.
    for i in 0..3 {
        ack[10 + i * 2..12 + i * 2].copy_from_slice(&get(m, 0x8020 + i * 2).to_le_bytes());
        ack[16 + i * 2..18 + i * 2].copy_from_slice(&get(m, 0x8018 + i * 2).to_le_bytes());
    }
    let seq = get(m, 0x8210).wrapping_add(1) & 0xFFF;
    set(m, 0x8210, seq);
    ack[22..24].copy_from_slice(&(seq << 4).to_le_bytes());
    send(m, ack);
    if get(m, 0x8008) & 0x2000 != 0 {
        set(m, 0x80B8, 0x0B01); // W_TXSTAT: acknowledge sent
        raise(m, IRQ_TX_END);
    }
    // The command's own transmit header reports the outcome: the bits left set
    // in entry [02h] are the clients that failed to answer. Nintendo's code
    // checks exactly this.
    let loc = get(m, 0x8090);
    set_ram16(m, hw_off(loc), if missing == 0 { 0x0001 } else { 0x0005 });
    set_ram16(m, hw_off(loc) + 2, missing);
    set(m, 0x8090, loc & 0x7FFF);
    set(m, 0x80B6, 0); // W_TXBUSY
    set(m, 0x8214, 1); // back to receive
    if *crate::bus::WIFILOG {
        eprintln!(
            "[wifi] c{} {:>8} cmd done: replied={:04X} missing={missing:04X}",
            m.wf.id, m.now, cmd.replied
        );
    }
    raise(m, IRQ_CMD_DONE);
}

/// A client answering a command it was polled by. The queued reply is latched
/// into the "current" slot, and if the game has not queued one, hardware still
/// answers with an empty frame so the host knows the client is alive.
fn send_reply(m: &mut Machine, aid: u8) {
    set(m, 0x8214, 5); // preparing the reply
    // The reply that was current until now is marked as consumed.
    let old = get(m, 0x8098);
    if old & 0x8000 != 0 {
        let h = ram8(m, hw_off(old));
        set_ram8(m, hw_off(old) + 1, h);
        set_ram8(m, hw_off(old), 0x01);
    }
    let next = get(m, 0x8094);
    set(m, 0x8098, next);
    set(m, 0x8094, 0);
    if next & 0x8000 != 0 {
        let base = hw_off(next);
        let n = ram8(m, base + 4);
        set_ram8(m, base + 4, n.saturating_add(1));
        set_ram8(m, base + 5, 0);
    }
    let seq = get(m, 0x8210).wrapping_add(1) & 0xFFF;
    set(m, 0x8210, seq);
    set(m, 0x8214, 8); // sending the reply
    raise(m, IRQ_TX_START);
    let frame = if next & 0x8000 != 0 {
        build_frame(m, next, false)
    } else {
        // Empty reply: a bare header with the "CF-Ack only" frame control.
        let mut f = vec![0u8; 24];
        f[0..2].copy_from_slice(&0x0158u16.to_le_bytes());
        f[4..10].copy_from_slice(&ADDR_CLIENT_REPLY);
        for i in 0..3 {
            f[10 + i * 2..12 + i * 2].copy_from_slice(&get(m, 0x8018 + i * 2).to_le_bytes());
            f[16 + i * 2..18 + i * 2].copy_from_slice(&get(m, 0x8020 + i * 2).to_le_bytes());
        }
        f[22..24].copy_from_slice(&(seq << 4).to_le_bytes());
        Some(f)
    };
    if let Some(f) = frame {
        if *crate::bus::WIFILOG {
            eprintln!(
                "[wifi] c{} {:>8} TX reply aid={aid} len={} fc={:04X}",
                m.wf.id,
                m.now,
                f.len(),
                u16::from_le_bytes([f[0], f[1]])
            );
        }
        send_as(m, f, aid);
    }
    set(m, 0x8214, 1); // back to receive
    if get(m, 0x8008) & 0x1000 != 0 {
        set(m, 0x80B8, 0x0401); // W_TXSTAT: reply sent
        raise(m, IRQ_TX_END);
    }
}

/// Send the periodic beacon, which announces a hosted game. Unlike the other
/// slots, its enable bit stays set: beacons repeat every beacon interval.
fn send_beacon(m: &mut Machine) {
    let loc = get(m, 0x8080);
    if loc & 0x8000 == 0 || !powered(m) {
        return;
    }
    raise(m, IRQ_TX_START);
    if let Some(mut f) = build_frame(m, loc, true) {
        // Beacons carry the sender's microsecond clock as their timestamp.
        if f.len() >= 32 {
            let us = us_count(m);
            f[24..32].copy_from_slice(&us.to_le_bytes());
        }
        send(m, f);
    }
    set_ram16(m, hw_off(loc), 0x0001);
    if get(m, 0x8008) & 0x8000 != 0 {
        set(m, 0x80B8, 0x0301); // W_TXSTAT: beacon done
    }
    raise(m, IRQ_TX_END);
}

/// Hand a finished frame to every other console on the air. Unlinked, a
/// transmission simply leaves the console and is heard by nobody.
fn send(m: &mut Machine, frame: Vec<u8>) {
    send_as(m, frame, 0);
}

fn send_as(m: &mut Machine, frame: Vec<u8>, aid: u8) {
    let Some(air) = m.wf.air.clone() else { return };
    let id = m.wf.id;
    let mut air = air.borrow_mut();
    for (i, inbox) in air.inbox.iter_mut().enumerate() {
        if i != id {
            inbox.push_back((frame.clone(), aid));
        }
    }
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

/// RXHDR[0] frame-type nibble, derived from the IEEE frame-control field. The
/// four multiplay frames have fixed frame-control values; bit 12 is masked off
/// because the transmitting hardware sets it on the copy that goes out.
fn rx_type(fc: u16, body_len: usize) -> u16 {
    let ftype = fc >> 2 & 3;
    let subtype = fc >> 4 & 0xF;
    match fc & 0x0FFF {
        0x0228 => return 0x0C, // multiplay CMD
        0x0218 => return 0x0D, // multiplay CMD acknowledge
        0x0118 => return 0x0E, // multiplay REPLY carrying data
        0x0158 => return 0x0F, // multiplay REPLY with an empty body
        _ => {}
    }
    match (ftype, subtype) {
        (0, 8) => 0x01,  // management / beacon
        (0, _) => 0x00,  // management / anything else
        (1, 10) => 0x05, // control / ps-poll
        (2, _) if body_len == 0 => 0x0F,
        (2, _) => 0x08,
        _ => 0x00,
    }
}

/// Should this frame reach the RX buffer at all? Hardware filters on the
/// destination and BSSID addresses: anything addressed to us is taken, a
/// broadcast is taken when it carries our BSSID, and broadcasts from a
/// different network need W_RXFILTER bit0, which is what a console sets while
/// it is scanning for hosts to join.
fn accepted(m: &Machine, frame: &[u8]) -> bool {
    if !powered(m) || get(m, 0x8030) & 0x8000 == 0 {
        if *crate::bus::WIFILOG {
            eprintln!(
                "[wifi] c{} {:>8} RX dropped (radio off): powerstate={:04X} mode_rst={:04X} rxcnt={:04X}",
                m.wf.id, m.now,
                get(m, 0x803C),
                get(m, 0x8004),
                get(m, 0x8030)
            );
        }
        return false; // receive queuing disabled
    }
    if frame.len() < 24 {
        return false;
    }
    let fc0 = u16::from_le_bytes([frame[0], frame[1]]) & 0x0FFF;
    if matches!(fc0, 0x0228 | 0x0218 | 0x0118 | 0x0158) {
        // Multiplay command, acknowledge and reply frames belong to an
        // exchange the hardware runs itself, and carry Nintendo's fixed flow
        // addresses rather than a BSSID. They are never address-filtered.
        return true;
    }
    let addr = |n: usize| &frame[4 + n * 6..10 + n * 6];
    let mac: Vec<u8> = (0..3).flat_map(|i| get(m, 0x8018 + i * 2).to_le_bytes()).collect();
    let bssid: Vec<u8> = (0..3).flat_map(|i| get(m, 0x8020 + i * 2).to_le_bytes()).collect();
    if addr(0) == mac.as_slice() {
        return true;
    }
    if addr(0)[0] & 1 == 0 {
        return false; // unicast to somebody else
    }
    let fc = u16::from_le_bytes([frame[0], frame[1]]);
    let frame_bssid = match (fc >> 8 & 1, fc >> 9 & 1) {
        (0, 0) => addr(2),
        (0, 1) => addr(1),
        (1, 0) => addr(0),
        _ => return false, // four-address frames have no BSSID
    };
    let ok = frame_bssid == bssid.as_slice() || get(m, 0x80D0) & 1 != 0;
    if !ok && *crate::bus::WIFILOG {
        eprintln!(
            "[wifi] RX dropped: frame bssid {frame_bssid:02X?} != {bssid:02X?}, rxfilter={:04X}",
            get(m, 0x80D0)
        );
    }
    ok
}

/// Write one received frame into the circular RX buffer and interrupt.
fn deliver_rx(m: &mut Machine, frame: &[u8], from_aid: u8) {
    if !accepted(m, frame) {
        return;
    }
    let begin = buf_off(get(m, 0x8050));
    let end = buf_off(get(m, 0x8052));
    if end <= begin {
        return; // buffer not configured
    }
    let mut wr = hw_off(get(m, 0x8054));
    if wr < begin || wr >= end {
        wr = begin;
    }
    let fc = u16::from_le_bytes([frame[0], frame[1]]);
    let total = 12 + ((frame.len() + 3) & !3);
    // Never overwrite data the CPU has not consumed yet.
    let read = hw_off(get(m, 0x805A));
    let free = if read > wr { read - wr } else { (end - wr) + (read - begin) };
    if read != wr && free < total {
        return; // buffer full: the frame is simply lost, as on hardware
    }
    let mut hdr = [0u16; 6];
    hdr[0] = rx_type(fc, frame.len().saturating_sub(24)) | 0x0010;
    if frame.len() >= 16 {
        let bssid = [
            u16::from_le_bytes([frame[16], frame[17]]),
            u16::from_le_bytes([frame[18], frame[19]]),
            u16::from_le_bytes([frame[20], frame[21]]),
        ];
        if bssid[0] == get(m, 0x8020) && bssid[1] == get(m, 0x8022) && bssid[2] == get(m, 0x8024) {
            hdr[0] |= 0x8000;
        }
    }
    hdr[1] = 0x0040;
    hdr[3] = 0x0014; // 2 Mbit/s
    hdr[4] = frame.len() as u16;
    hdr[5] = 0x00FF; // max/min RSSI: a perfect signal
    // Lay the header and the frame out contiguously, then copy it into the
    // ring, which may wrap partway through.
    let mut bytes = Vec::with_capacity(total);
    for h in hdr {
        bytes.push(h as u8);
        bytes.push((h >> 8) as u8);
    }
    bytes.extend_from_slice(frame);
    while bytes.len() % 4 != 0 {
        bytes.push(0);
    }
    raise(m, IRQ_RX_START);
    let mut p = wr;
    for b in bytes {
        set_ram8(m, p, b);
        p += 1;
        if p >= end {
            p = begin;
        }
    }
    set(m, 0x8054, (p / 2) as u16); // W_RXBUF_WRCSR
    if *crate::bus::WIFILOG {
        eprintln!(
            "[wifi] c{} {:>8} RX {} len={} fc={fc:04X} -> wrcsr={:04X}",
            m.wf.id, m.now, frame_name(fc), frame.len(), p / 2
        );
    }
    raise(m, IRQ_RX_END);

    match hdr[0] & 0xF {
        // A command addressed to us: answer it in our slot. The clients being
        // polled are named by the bitmask in the frame body, and our slot
        // number is whatever the host assigned us in W_AID_LOW.
        0x0C => {
            let aid = (get(m, 0x8028) & 0xF) as u8;
            let slaves = if frame.len() >= 28 {
                u16::from_le_bytes([frame[26], frame[27]])
            } else {
                0
            };
            if aid != 0 && slaves >> aid & 1 != 0 {
                send_reply(m, aid);
            }
        }
        // A reply to the command we are hosting.
        0x0E | 0x0F => {
            if let Some(cmd) = &mut m.wf.cmd {
                cmd.replied |= 1 << from_aid;
            }
        }
        _ => {}
    }
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

fn us_count(m: &Machine) -> u64 {
    (get(m, 0x80F8) as u64)
        | (get(m, 0x80FA) as u64) << 16
        | (get(m, 0x80FC) as u64) << 32
        | (get(m, 0x80FE) as u64) << 48
}

fn set_us_count(m: &mut Machine, v: u64) {
    set(m, 0x80F8, v as u16);
    set(m, 0x80FA, (v >> 16) as u16);
    set(m, 0x80FC, (v >> 32) as u16);
    set(m, 0x80FE, (v >> 48) as u16);
}

fn us_compare(m: &Machine) -> u64 {
    (get(m, 0x80F0) as u64)
        | (get(m, 0x80F2) as u64) << 16
        | (get(m, 0x80F4) as u64) << 32
        | (get(m, 0x80F6) as u64) << 48
}

/// Advance the wireless hardware by a slice of ARM7 cycles (one scanline's
/// worth). This is where the microsecond clock, the beacon timeslots and any
/// pending transfers happen.
pub fn step(m: &mut Machine, cycles: u32) {
    m.wf.us_acc += cycles as u64 * 1000;
    let ticks = m.wf.us_acc / CYCLES_PER_US_K;
    m.wf.us_acc -= ticks * CYCLES_PER_US_K;

    let counting = get(m, 0x80E8) & 1 != 0 && get(m, 0x8036) & 1 == 0;
    if counting && ticks > 0 {
        let old = us_count(m);
        let new = old + ticks;
        set_us_count(m, new);
        // W_US_COMPARE is usually parked at "never"; a match is a beacon
        // timeslot just like the beacon counter reaching zero.
        let cmp = us_compare(m);
        if old < cmp && new >= cmp && get(m, 0x80EA) & 1 != 0 {
            beacon_timeslot(m);
        }
        // The beacon and post-beacon counters tick once per 1024 microseconds.
        for _ in 0..((old & 0x3FF) + ticks) / 1024 {
            millisecond(m);
        }
    }

    // W_CMD_COUNT bounds the multiplay command timeslot: one step per 10us.
    if get(m, 0x80EE) & 1 != 0 {
        let cmd = get(m, 0x8118);
        if cmd > 0 {
            set(m, 0x8118, cmd.saturating_sub((ticks / 10) as u16));
        }
    }

    // Anything the other console put on the air since the last step.
    if let Some(air) = m.wf.air.clone() {
        let id = m.wf.id;
        let mut air = air.borrow_mut();
        while let Some(f) = air.inbox[id].pop_front() {
            m.wf.rx_queue.push_back(f);
        }
    }
    while let Some((f, aid)) = m.wf.rx_queue.pop_front() {
        deliver_rx(m, &f, aid);
    }
    // A command exchange ends as soon as every polled client has answered, or
    // when the reply window runs out.
    if let Some(cmd) = &m.wf.cmd {
        if cmd.replied & cmd.slaves == cmd.slaves || us_count(m) >= cmd.deadline {
            finish_cmd(m);
        }
    }
    service_tx(m);
}

/// One millisecond of beacon bookkeeping.
fn millisecond(m: &mut Machine) {
    let pre = get(m, 0x8110);
    let count = get(m, 0x811C);
    if pre != 0 && count == 1 && get(m, 0x80EA) & 1 != 0 {
        raise(m, IRQ_PRE_BEACON);
    }
    if count <= 1 {
        beacon_timeslot(m);
    } else {
        set(m, 0x811C, count - 1);
    }
    let post = get(m, 0x8134);
    if post > 0 {
        set(m, 0x8134, post - 1);
        if post == 1 {
            raise(m, IRQ_POST_BEACON);
            if get(m, 0x8038) & 2 == 0 {
                power_down(m);
            }
        }
    }
}

/// A beacon timeslot: reload the counters, hand the transmit slots back to
/// software, send our beacon if one is configured, and interrupt.
fn beacon_timeslot(m: &mut Machine) {
    let interval = get(m, 0x808C) & 0x3FF;
    set(m, 0x811C, interval.max(1));
    set(m, 0x8134, 0xFFFF);
    let listen = get(m, 0x8088) & 0xFF;
    if listen == 0 {
        set(m, 0x8088, get(m, 0x808E) & 0xFF);
    } else {
        set(m, 0x8088, listen - 1);
    }
    let req = get(m, 0x80B0);
    set(m, 0x80B0, req & !0x0D); // bits 0,2,3 are cleared by hardware here
    send_beacon(m);
    if get(m, 0x80EA) & 1 != 0 {
        raise(m, IRQ_BEACON);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bring a console's radio up the way the driver does, and point its
    /// receive ring at the second half of packet RAM.
    fn radio_on(m: &mut Machine, mac_last: u8) {
        write16(m, 0x8004, 1); // W_MODE_RST: enable
        write16(m, 0x8012, 0xFFFF); // W_IE: every interrupt
        write16(m, 0x8030, 0x8000); // W_RXCNT: queue received frames
        write16(m, 0x8050, 0x5000); // W_RXBUF_BEGIN
        write16(m, 0x8052, 0x5F00); // W_RXBUF_END
        write16(m, 0x8056, 0x0800); // W_RXBUF_WR_ADDR
        write16(m, 0x8030, 0x8001); // latch it into W_RXBUF_WRCSR
        write16(m, 0x805A, 0x0800); // W_RXBUF_READCSR
        for (i, v) in [0x0900u16, 0xBF00, (mac_last as u16) << 8 | 0x12].iter().enumerate() {
            write16(m, 0x8018 + i * 2, *v); // W_MACADDR
            write16(m, 0x8020 + i * 2, 0xABCD); // W_BSSID: one shared network
        }
        write16(m, 0x80E8, 1); // W_US_COUNTCNT: run the microsecond clock
        write16(m, 0x803C, 2); // W_POWERSTATE: power up
        assert!(powered(m), "radio should be enabled and powered");
    }

    /// Lay a transmit header plus an IEEE frame into packet RAM, and return
    /// the halfword address a W_TXBUF_* register would name it by.
    fn stage_frame(m: &mut Machine, at: usize, fc: u16, body: &[u8]) -> u16 {
        let len = 24 + body.len() + 4; // header + body + the FCS hardware adds
        set_ram16(m, at + 0x0A, len as u16);
        set_ram8(m, at + 8, 0x14); // 2 Mbit/s
        for i in 0..24 + body.len() {
            set_ram8(m, at + 12 + i, 0);
        }
        set_ram16(m, at + 12, fc);
        set_ram8(m, at + 12 + 4, 0x03); // a group address, so the filter passes it
        for (i, b) in body.iter().enumerate() {
            set_ram8(m, at + 12 + 24 + i, *b);
        }
        // BSSID field (address 3 for a from-the-host data frame is address 2).
        for i in 0..3 {
            set_ram16(m, at + 12 + 10 + i * 2, 0xABCD);
        }
        0x8000 | (at / 2) as u16
    }

    /// The newest frame in a console's receive ring: its hardware header type
    /// nibble and its length.
    fn last_rx(m: &Machine, from: usize) -> Vec<(u16, u16)> {
        let mut out = Vec::new();
        let mut p = from;
        let wr = hw_off(get(m, 0x8054));
        while p < wr {
            let ty = ram16(m, p) & 0xF;
            let len = ram16(m, p + 8);
            out.push((ty, len));
            p += 12 + ((len as usize + 3) & !3);
        }
        out
    }

    #[test]
    fn multiplay_command_and_reply() {
        let air = std::rc::Rc::new(std::cell::RefCell::new(Air::new(2)));
        let mut host = Machine::new();
        let mut client = Machine::new();
        reset(&mut host);
        reset(&mut client);
        host.wf.air = Some(air.clone());
        client.wf.air = Some(air.clone());
        client.wf.id = 1;
        radio_on(&mut host, 1);
        radio_on(&mut client, 2);

        // The host polls client 1: the bitmask lives in the frame body, and
        // the reply window is sized from W_CMD_REPLYTIME.
        let cmd = stage_frame(&mut host, 0x0100, 0x0228, &[0x40, 0x00, 0x02, 0x00]);
        write16(&mut host, 0x8090, cmd); // W_TXBUF_CMD
        write16(&mut host, 0x80C4, 100); // W_CMD_REPLYTIME
        write16(&mut host, 0x80EE, 1); // W_CMD_COUNTCNT
        write16(&mut host, 0x8118, 0x1000); // W_CMD_COUNT: time available
        write16(&mut host, 0x8008, 0xF000); // W_TXSTATCNT: report every stage
        write16(&mut host, 0x80AE, 2); // W_TXREQ_SET: the command slot

        // The client has an answer queued.
        let reply = stage_frame(&mut client, 0x0200, 0x0118, &[0xAA, 0xBB]);
        write16(&mut client, 0x8094, reply); // W_TXBUF_REPLY1
        write16(&mut client, 0x8028, 1); // W_AID_LOW: we are client 1
        write16(&mut client, 0x8008, 0xF000);

        // Run both consoles for a while, a microsecond at a time.
        for _ in 0..400 {
            step(&mut host, 34);
            step(&mut client, 34);
        }

        let client_rx = last_rx(&client, 0x1000);
        let host_rx = last_rx(&host, 0x1000);
        assert!(
            client_rx.iter().any(|&(t, _)| t == 0x0C),
            "client should have received the command frame, got {client_rx:?}"
        );
        assert!(
            host_rx.iter().any(|&(t, l)| t == 0x0E && l == 26),
            "host should have received the client's reply, got {host_rx:?}"
        );
        assert!(
            client_rx.iter().any(|&(t, _)| t == 0x0D),
            "client should have received the command acknowledge, got {client_rx:?}"
        );
        // The command is finished, reported successful, and every polled
        // client answered (no bits left set in transmit header entry 02h).
        assert_eq!(get(&host, 0x8090) & 0x8000, 0, "command enable bit should be cleared");
        assert_eq!(ram16(&host, 0x0100), 0x0001, "command header should report success");
        assert_eq!(ram16(&host, 0x0102), 0x0000, "no client should be marked missing");
        assert_ne!(get(&host, 0x8010) & 1 << IRQ_CMD_DONE, 0, "IRQ12 should have fired");
        // The queued reply was consumed: REPLY1 empties into REPLY2.
        assert_eq!(get(&client, 0x8094), 0, "REPLY1 should have been latched away");
        assert_eq!(get(&client, 0x8098), reply, "REPLY2 should hold the sent reply");
    }

    /// With nothing queued, a polled client still answers, so the host can
    /// tell the difference between "no data" and "gone".
    #[test]
    fn client_answers_even_with_nothing_to_say() {
        let air = std::rc::Rc::new(std::cell::RefCell::new(Air::new(2)));
        let mut host = Machine::new();
        let mut client = Machine::new();
        reset(&mut host);
        reset(&mut client);
        host.wf.air = Some(air.clone());
        client.wf.air = Some(air.clone());
        client.wf.id = 1;
        radio_on(&mut host, 1);
        radio_on(&mut client, 2);
        let cmd = stage_frame(&mut host, 0x0100, 0x0228, &[0x40, 0x00, 0x02, 0x00]);
        write16(&mut host, 0x8090, cmd);
        write16(&mut host, 0x80C4, 100);
        write16(&mut host, 0x80EE, 1);
        write16(&mut host, 0x8118, 0x1000);
        write16(&mut host, 0x80AE, 2);
        write16(&mut client, 0x8028, 1);

        for _ in 0..400 {
            step(&mut host, 34);
            step(&mut client, 34);
        }
        let host_rx = last_rx(&host, 0x1000);
        assert!(
            host_rx.iter().any(|&(t, _)| t == 0x0F),
            "host should have received an empty reply, got {host_rx:?}"
        );
        assert_eq!(ram16(&host, 0x0102), 0x0000, "the client did answer");
    }
}
