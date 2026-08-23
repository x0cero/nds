//! Sound Processing Unit: the ARM7's 16 mixer channels at 0x04000400-0x0400051F.
//!
//! The DS sound hardware is a DMA-driven mixer, not a synthesiser. Each of the
//! 16 channels walks a buffer in main RAM on its own clock and hands the mixer
//! one sample at a time; the mixer sums them, applies a master volume, and
//! feeds a 10-bit DAC. Channels 8-13 can instead run a PSG square wave and
//! 14-15 a noise generator, neither of which touches memory.
//!
//! Everything here is driven from one number: the mixer produces one stereo
//! sample every 1024 system cycles (33.513982 MHz / 1024 = 32728.5 Hz), and a
//! channel's own clock ticks at half the system rate, so 512 channel ticks
//! pass per output sample. A channel's 16-bit timer counts up from its reload
//! value and fetches the next source sample every time it wraps, which is what
//! makes SOUNDxTMR a period rather than a frequency.
//!
//! Scaling is chosen to match the hardware's clipping point rather than to be
//! loud: one channel at full volume panned centre reaches half of full scale,
//! so two of them saturate the DAC, exactly as on a DS. Games mix themselves
//! well below that, so the clamp almost never fires in practice.

use crate::bus::Machine;

/// NDS_SPULOG=1 reports every note the driver starts: which channel, what
/// format, the sample rate the timer works out to, and where the data is.
/// Read once - this sits on the register-write path.
static SPULOG: std::sync::LazyLock<bool> =
    std::sync::LazyLock::new(|| std::env::var("NDS_SPULOG").is_ok());

/// System cycles per output sample. The whole SPU clock derives from this.
pub const CYCLES_PER_SAMPLE: u32 = 1024;
/// 33513982 / 1024, rounded. Only the frontend cares (WAV headers, resampling).
pub const SAMPLE_RATE: u32 = 32_728;

/// Channel ticks per output sample: the channel clock is half the system clock.
const TICKS_PER_SAMPLE: u32 = CYCLES_PER_SAMPLE / 2;

/// SOUNDxCNT bits 8-9 divide the channel volume by 1, 2, 4 or 16.
const DIV_SHIFT: [u32; 4] = [0, 1, 2, 4];

/// IMA-ADPCM step sizes and the per-nibble index adjustment.
const STEP: [u16; 89] = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
];
const INDEX_ADJ: [i8; 8] = [-1, -1, -1, -1, 2, 4, 6, 8];

/// Live playback state for one channel. None of it is addressable by the
/// game: the registers hold what the driver wrote, this holds where the
/// channel has got to.
#[derive(Clone, Copy, Default)]
struct Channel {
    /// 16-bit up-counter; wrapping past 0xFFFF fetches the next sample.
    timer: u32,
    /// Position in the source: sample index for PCM8/PCM16, nibble index
    /// (past the 4-byte header) for ADPCM, unused for PSG and noise.
    pos: u32,
    /// The sample the channel is currently presenting to the mixer.
    cur: i16,
    /// ADPCM decoder state, plus the copy taken when the loop point goes by
    /// so a looping sample restarts from the right predictor instead of
    /// drifting into noise.
    adpcm: i32,
    adpcm_idx: i32,
    loop_adpcm: i32,
    loop_idx: i32,
    loop_saved: bool,
    /// PSG duty position (0-7) and the 15-bit noise LFSR.
    psg: u8,
    lfsr: u16,
    /// Cleared at key-on; the first run of the mixer loads the ADPCM header
    /// from memory, which cannot happen during the register write itself.
    primed: bool,
}

pub struct Spu {
    /// The register block verbatim, offset 0 = 0x04000400. Kept because the
    /// driver reads back what it wrote (and polls SOUNDxCNT bit 31 to find out
    /// when a one-shot sample has finished).
    regs: [u8; 0x120],
    ch: [Channel; 16],
    /// Byte offset into each capture destination buffer.
    cap_pos: [u32; 2],
    /// System cycles banked toward the next output sample.
    acc: u32,
    /// Interleaved stereo output, drained by the frontend every frame.
    pub out: Vec<i16>,
}

impl Default for Spu {
    fn default() -> Self {
        // Power on with the master enabled at full volume, which is what the
        // driver leaves behind once it has started. It matters because the
        // driver only writes SOUNDCNT when it initialises: a savestate taken
        // mid-game (SPU state is not snapshotted) would otherwise restore with
        // the mixer switched off and stay silent forever. Every channel is
        // idle regardless, so nothing can play until the game keys one on.
        let mut regs = [0; 0x120];
        regs[0x100..0x102].copy_from_slice(&0x807Fu16.to_le_bytes());
        Self {
            regs,
            ch: [Channel::default(); 16],
            cap_pos: [0; 2],
            acc: 0,
            out: Vec::new(),
        }
    }
}

impl Spu {
    fn r16(&self, off: usize) -> u16 {
        u16::from_le_bytes([self.regs[off], self.regs[off + 1]])
    }

    fn r32(&self, off: usize) -> u32 {
        u32::from_le_bytes(self.regs[off..off + 4].try_into().unwrap())
    }

    /// `off` is the raw I/O offset (0x400-0x51F).
    pub fn read16(&self, off: u32) -> u16 {
        self.r16((off - 0x400) as usize & !1)
    }

    pub fn write16(&mut self, off: u32, v: u16) {
        let i = (off - 0x400) as usize & !1;
        let old = self.r16(i);
        self.regs[i] = v as u8;
        self.regs[i + 1] = (v >> 8) as u8;
        // SOUNDxCNT high half: bit 31 is start/busy. A 0 -> 1 edge is a key-on
        // and rewinds the channel; the driver also writes the low half alone
        // (volume and pan tweaks), which must not restart anything.
        if off < 0x500 && off & 0xF == 2 && v & 0x8000 != 0 && old & 0x8000 == 0 {
            self.key_on(((off - 0x400) >> 4) as usize);
        }
        // Capture control: both SNDCAPxCNT bytes share one halfword at 0x508.
        if off & !1 == 0x508 {
            for c in 0..2 {
                let started = v >> (c * 8) & 0x80 != 0;
                if started && old >> (c * 8) & 0x80 == 0 {
                    self.cap_pos[c as usize] = 0;
                    if *SPULOG {
                        eprintln!(
                            "spu capture{c} start -> {:#010X} {} words",
                            self.r32(0x110 + c as usize * 8),
                            self.r16(0x114 + c as usize * 8)
                        );
                    }
                }
            }
        }
    }

    fn key_on(&mut self, i: usize) {
        let tmr = self.r16(i * 0x10 + 8) as u32;
        self.ch[i] = Channel {
            timer: tmr,
            lfsr: 0x7FFF,
            ..Channel::default()
        };
        if *SPULOG {
            let cnt = self.r32(i * 0x10);
            let fmt = ["pcm8", "pcm16", "adpcm", "psg/noise"][(cnt >> 29 & 3) as usize];
            // The timer is a period, so the rate it produces is the channel
            // clock divided by how far the counter has left to run.
            let rate = 16_756_991.0 / (0x1_0000 - tmr).max(1) as f64;
            eprintln!(
                "spu ch{i:<2} on  {fmt:<9} {rate:8.1} Hz  vol {:3}/{:<2} pan {:3} repeat {}  \
                 src {:#010X} pnt {} len {}",
                cnt & 0x7F,
                1 << DIV_SHIFT[(cnt >> 8 & 3) as usize],
                cnt >> 16 & 0x7F,
                cnt >> 27 & 3,
                self.r32(i * 0x10 + 4),
                self.r16(i * 0x10 + 0xA),
                self.r32(i * 0x10 + 0xC) & 0x003F_FFFF,
            );
        }
    }

    /// Stop a channel: clear SOUNDxCNT bit 31, which is what the driver polls
    /// to find out that a one-shot sound has finished.
    ///
    /// SOUNDxCNT bit 15 (hold the last sample after a one-shot) is deliberately
    /// not implemented: the mixer drops a stopped channel entirely, so holding
    /// would only park a constant level on the output, and a constant level is
    /// not something anyone can hear.
    fn key_off(&mut self, i: usize) {
        self.regs[i * 0x10 + 3] &= 0x7F;
        self.ch[i].cur = 0;
    }

    /// Advance the mixer by `cycles` system cycles, appending stereo samples.
    pub fn run(&mut self, m: &mut Machine, cycles: u32) {
        self.acc += cycles;
        while self.acc >= CYCLES_PER_SAMPLE {
            self.acc -= CYCLES_PER_SAMPLE;
            self.mix_one(m);
        }
        // Safety net: the frontend drains `out` every frame, but a caller that
        // forgets must not grow it without bound.
        if self.out.len() > 1 << 20 {
            self.out.clear();
        }
    }

    fn mix_one(&mut self, m: &mut Machine) {
        let soundcnt = self.r32(0x100);
        if soundcnt & 0x8000 == 0 {
            // Master disable: the mixer is off, but the stream keeps running
            // so the frontend's timing does not depend on the game's volume.
            self.out.extend_from_slice(&[0, 0]);
            return;
        }
        let mut mix = [0i32; 2];
        // Channels 1 and 3 can be pulled out of the mixer and routed straight
        // to an output or to capture, which is how the SDK builds echo.
        let mut side = [[0i32; 2]; 4];
        for i in 0..16 {
            let cnt = self.r32(i * 0x10);
            if cnt & 0x8000_0000 == 0 {
                continue;
            }
            self.step_channel(i, m, cnt);
            // step_channel can end the sample; re-read so a channel that just
            // stopped contributes its final (or held) value and nothing more.
            let s = self.ch[i].cur as i32;
            let v = (s * (cnt & 0x7F) as i32) >> (7 + DIV_SHIFT[(cnt >> 8 & 3) as usize]);
            let pan = (cnt >> 16 & 0x7F) as i32;
            let lr = [(v * (128 - pan)) >> 7, (v * pan) >> 7];
            if i < 4 {
                side[i] = lr;
            }
            // SOUNDCNT bits 12/13 keep channel 1 / channel 3 out of the mixer.
            let muted = (i == 1 && soundcnt & 0x1000 != 0) || (i == 3 && soundcnt & 0x2000 != 0);
            if !muted {
                mix[0] += lr[0];
                mix[1] += lr[1];
            }
        }
        // Capture takes the mixer output, before the master volume.
        let cap_src = [mix[0], mix[1]];
        let master = (soundcnt & 0x7F) as i32;
        let mut out = [0i16; 2];
        for c in 0..2 {
            // SOUNDCNT bits 8-11 pick what each output actually plays.
            let sel = soundcnt >> (8 + c * 2) & 3;
            let v = match sel {
                1 => side[1][c],
                2 => side[3][c],
                3 => side[1][c] + side[3][c],
                _ => mix[c],
            };
            out[c] = ((v * master) >> 7).clamp(-0x8000, 0x7FFF) as i16;
        }
        self.out.extend_from_slice(&out);
        self.capture(m, cap_src, side);
    }

    /// Run one channel's clock forward by one output sample. The timer counts
    /// channel ticks up to 0x10000 and reloads with SOUNDxTMR, so a small
    /// reload is a long period and a large one is a high pitch.
    fn step_channel(&mut self, i: usize, m: &mut Machine, cnt: u32) {
        if !self.ch[i].primed {
            self.ch[i].primed = true;
            if cnt >> 29 & 3 == 2 {
                // ADPCM: the first word of the sample is the initial predictor
                // and step index, not audio.
                let sad = self.r32(i * 0x10 + 4) & 0x07FF_FFFF;
                let hdr = m.spu_read32(sad);
                self.ch[i].adpcm = hdr as i16 as i32;
                self.ch[i].adpcm_idx = (hdr >> 16 & 0x7F).min(88) as i32;
            }
        }
        let tmr = self.r16(i * 0x10 + 8) as u32;
        self.ch[i].timer += TICKS_PER_SAMPLE;
        // A reload near 0xFFFF is a legal (ultrasonic) rate that would fetch
        // hundreds of samples per output sample; bound the work either way.
        let mut guard = 0;
        while self.ch[i].timer > 0xFFFF {
            self.ch[i].timer = tmr + (self.ch[i].timer - 0x1_0000);
            self.advance(i, m, cnt);
            guard += 1;
            if guard >= 1024 || self.regs[i * 0x10 + 3] & 0x80 == 0 {
                break;
            }
        }
    }

    /// Fetch the next source sample for one channel, handling the loop point
    /// and the end of the sample.
    fn advance(&mut self, i: usize, m: &mut Machine, cnt: u32) {
        let fmt = cnt >> 29 & 3;
        // Channels 8-13 run a PSG square wave and 14-15 a noise generator when
        // the format field says 3; neither reads memory.
        if fmt == 3 {
            if i >= 14 {
                let c = &mut self.ch[i];
                if c.lfsr & 1 != 0 {
                    c.lfsr = (c.lfsr >> 1) ^ 0x6000;
                    c.cur = -0x7FFF;
                } else {
                    c.lfsr >>= 1;
                    c.cur = 0x7FFF;
                }
            } else if i >= 8 {
                let duty = (cnt >> 24 & 7) as u8;
                let c = &mut self.ch[i];
                // Duty 0 is 12.5% high through duty 6 at 87.5%; duty 7 is
                // silent (permanently low).
                c.cur = if duty == 7 || c.psg < 7 - duty { -0x7FFF } else { 0x7FFF };
                c.psg = (c.psg + 1) & 7;
            } else {
                // Channels 0-7 have no generator, so this setting is a driver
                // bug; on hardware it produces nothing rather than freezing
                // whatever sample the channel last held.
                self.key_off(i);
            }
            return;
        }
        let sad = self.r32(i * 0x10 + 4) & 0x07FF_FFFF;
        let pnt = self.r16(i * 0x10 + 0xA) as u32;
        let len = self.r32(i * 0x10 + 0xC) & 0x003F_FFFF;
        // SOUNDxPNT and SOUNDxLEN are both in 4-byte words: PNT is where the
        // loop starts and LEN is how much follows it.
        let (total, loop_at) = match fmt {
            0 => ((pnt + len) * 4, pnt * 4),
            1 => ((pnt + len) * 2, pnt * 2),
            // ADPCM counts nibbles, and the header word is not audio, so both
            // the total and the loop point lose its 8 nibbles.
            _ => (((pnt + len) * 8).saturating_sub(8), (pnt * 8).saturating_sub(8)),
        };
        if total == 0 {
            self.key_off(i);
            return;
        }
        // Passing the loop point banks the ADPCM predictor, so a repeat does
        // not have to re-derive it from the start of the sample.
        if fmt == 2 && !self.ch[i].loop_saved && self.ch[i].pos >= loop_at {
            self.ch[i].loop_saved = true;
            self.ch[i].loop_adpcm = self.ch[i].adpcm;
            self.ch[i].loop_idx = self.ch[i].adpcm_idx;
        }
        if self.ch[i].pos >= total {
            // Repeat mode 1 loops; manual and one-shot both stop here.
            if cnt >> 27 & 3 == 1 {
                self.ch[i].pos = loop_at;
                if fmt == 2 && self.ch[i].loop_saved {
                    self.ch[i].adpcm = self.ch[i].loop_adpcm;
                    self.ch[i].adpcm_idx = self.ch[i].loop_idx;
                }
            } else {
                self.key_off(i);
                return;
            }
        }
        let pos = self.ch[i].pos;
        match fmt {
            0 => self.ch[i].cur = (m.spu_read8(sad + pos) as i8 as i16) << 8,
            1 => self.ch[i].cur = m.spu_read16(sad + pos * 2) as i16,
            _ => {
                let byte = m.spu_read8(sad + 4 + pos / 2);
                let nib = if pos & 1 == 0 { byte & 0xF } else { byte >> 4 } as u32;
                let c = &mut self.ch[i];
                let step = STEP[c.adpcm_idx as usize] as i32;
                // GBATEK's exact reconstruction: eighth of a step, plus a
                // quarter, a half and a whole for each set magnitude bit.
                let mut diff = step / 8;
                if nib & 1 != 0 {
                    diff += step / 4;
                }
                if nib & 2 != 0 {
                    diff += step / 2;
                }
                if nib & 4 != 0 {
                    diff += step;
                }
                c.adpcm = if nib & 8 != 0 {
                    (c.adpcm - diff).max(-0x7FFF)
                } else {
                    (c.adpcm + diff).min(0x7FFF)
                };
                c.adpcm_idx = (c.adpcm_idx + INDEX_ADJ[(nib & 7) as usize] as i32).clamp(0, 88);
                c.cur = c.adpcm as i16;
            }
        }
        self.ch[i].pos += 1;
    }

    /// Sound capture: two units that write the mixer output (or channel 0 / 2)
    /// back into memory, which is how the SDK implements echo and reverb.
    fn capture(&mut self, m: &mut Machine, mixer: [i32; 2], side: [[i32; 2]; 4]) {
        for c in 0..2 {
            let cnt = self.regs[0x108 + c];
            if cnt & 0x80 == 0 {
                continue;
            }
            let dad = self.r32(0x110 + c * 8) & 0x07FF_FFFF;
            let words = self.r16(0x114 + c * 8) as u32;
            if words == 0 {
                continue;
            }
            // Bit 1 picks the source: the mixer output, or channel 0 (unit 0)
            // / channel 2 (unit 1) on its own.
            let v = if cnt & 2 != 0 { side[c * 2][c] } else { mixer[c] };
            let v = v.clamp(-0x8000, 0x7FFF) as i16;
            let pos = self.cap_pos[c];
            // Bit 3 selects 8-bit capture, which keeps only the high byte.
            if cnt & 8 != 0 {
                m.spu_write8(dad + pos, (v >> 8) as u8);
                self.cap_pos[c] = pos + 1;
            } else {
                m.spu_write16(dad + pos, v as u16);
                self.cap_pos[c] = pos + 2;
            }
            if self.cap_pos[c] >= words * 4 {
                // Bit 2 set is one-shot: stop and report not-busy.
                if cnt & 4 != 0 {
                    self.regs[0x108 + c] &= 0x7F;
                } else {
                    self.cap_pos[c] = 0;
                }
            }
        }
    }
}
