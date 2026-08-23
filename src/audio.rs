//! Host audio: gets the mixer's samples out of the emulator, either to the
//! speakers or to a WAV file.
//!
//! The awkward part is that two clocks are involved and neither is willing to
//! move. The emulator produces samples at whatever speed it happens to be
//! running (59.8 frames a second if it keeps up, less if it does not), while
//! the sound card asks for a fixed number of samples per second forever. Feed
//! it directly and every shortfall is an audible gap.
//!
//! So the callback reads through a queue at a rate it adjusts continuously:
//! when the queue is draining it reads a little slower, when the queue is
//! filling it reads a little faster, and the correction is capped at 5% so it
//! never becomes an audible pitch bend. This also does the resampling from the
//! DS's 32728 Hz to whatever rate the sound card wants, since both are the
//! same operation: read the queue at a fractional step.

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::collections::VecDeque;
use std::io::{Seek, SeekFrom, Write};
use std::sync::{Arc, Mutex};

use crate::spu::SAMPLE_RATE;

/// How much audio to keep buffered, in stereo frames. About 60 ms: enough to
/// absorb a slow frame, short enough that input still feels attached to sound.
const TARGET_FILL: usize = 2048;
/// Hard cap; if the device stalls entirely, drop the oldest audio rather than
/// grow without bound.
const MAX_FILL: usize = 8192;

struct Ring {
    q: VecDeque<i16>,
    /// The two frames the interpolator is currently between, and where it sits.
    prev: [f32; 2],
    next: [f32; 2],
    frac: f32,
    /// Frames the callback wanted and did not have. Reported on exit, since a
    /// steady stream of them means the emulator is simply running too slowly.
    underruns: u64,
    /// The device starts asking for audio before the emulator has produced
    /// any. Stay quiet until there is a buffer's worth rather than playing a
    /// burst of nothing and calling it a fault.
    primed: bool,
}

impl Ring {
    fn pop(&mut self) -> Option<[f32; 2]> {
        if self.q.len() < 2 {
            return None;
        }
        let l = self.q.pop_front().unwrap() as f32 / 32768.0;
        let r = self.q.pop_front().unwrap() as f32 / 32768.0;
        Some([l, r])
    }
}

pub struct Audio {
    ring: Option<Arc<Mutex<Ring>>>,
    stream: Option<cpal::Stream>,
    wav: Option<std::fs::File>,
    wav_samples: u32,
}

impl Audio {
    /// `speakers` opens the default output device; `wav_path` additionally
    /// records everything to a file. Either may be absent.
    pub fn new(speakers: bool, wav_path: Option<String>) -> Self {
        let mut a = Audio { ring: None, stream: None, wav: None, wav_samples: 0 };
        if speakers {
            match a.open_device() {
                Ok(()) => {}
                Err(e) => eprintln!("audio: no output device ({e}); running silent"),
            }
        }
        if let Some(p) = wav_path {
            match a.open_wav(&p) {
                Ok(()) => eprintln!("audio: recording to {p}"),
                Err(e) => eprintln!("audio: cannot write {p}: {e}"),
            }
        }
        a
    }

    fn open_device(&mut self) -> Result<(), String> {
        let host = cpal::default_host();
        let device = host.default_output_device().ok_or("no default output device")?;
        let config = device.default_output_config().map_err(|e| e.to_string())?;
        if config.sample_format() != cpal::SampleFormat::F32 {
            return Err(format!("unsupported sample format {:?}", config.sample_format()));
        }
        let channels = config.channels() as usize;
        let host_rate = config.sample_rate() as f32;
        // One DS sample per this many host samples. Everything else is a
        // correction applied on top of it.
        let base_ratio = SAMPLE_RATE as f32 / host_rate;
        let ring = Arc::new(Mutex::new(Ring {
            q: VecDeque::with_capacity(MAX_FILL * 2),
            prev: [0.0; 2],
            next: [0.0; 2],
            frac: 0.0,
            underruns: 0,
            primed: false,
        }));
        let cb_ring = ring.clone();
        let stream = device
            .build_output_stream(
                config.into(),
                move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    let mut r = cb_ring.lock().unwrap();
                    if !r.primed {
                        if r.q.len() / 2 < TARGET_FILL {
                            data.fill(0.0);
                            return;
                        }
                        r.primed = true;
                    }
                    // Steer the read rate by how full the queue is. Positive
                    // error means we are ahead and should read faster.
                    let fill = (r.q.len() / 2) as f32;
                    let err = ((fill - TARGET_FILL as f32) / TARGET_FILL as f32).clamp(-1.0, 1.0);
                    let ratio = base_ratio * (1.0 + err * 0.05);
                    for frame in data.chunks_mut(channels) {
                        while r.frac >= 1.0 {
                            r.prev = r.next;
                            match r.pop() {
                                Some(s) => r.next = s,
                                // Underrun: hold the last sample rather than
                                // drop to silence, which clicks far louder.
                                None => r.underruns += 1,
                            }
                            r.frac -= 1.0;
                        }
                        let t = r.frac;
                        let (l, rr) = (
                            r.prev[0] + (r.next[0] - r.prev[0]) * t,
                            r.prev[1] + (r.next[1] - r.prev[1]) * t,
                        );
                        for (i, out) in frame.iter_mut().enumerate() {
                            *out = if i % 2 == 0 { l } else { rr };
                        }
                        r.frac += ratio;
                    }
                },
                move |e| eprintln!("audio stream error: {e}"),
                None,
            )
            .map_err(|e| e.to_string())?;
        stream.play().map_err(|e| e.to_string())?;
        eprintln!("audio: {host_rate:.0} Hz, {channels} channels");
        self.ring = Some(ring);
        self.stream = Some(stream);
        Ok(())
    }

    fn open_wav(&mut self, path: &str) -> Result<(), String> {
        let mut f = std::fs::File::create(path).map_err(|e| e.to_string())?;
        // Placeholder header; the two length fields are patched in finish().
        f.write_all(&wav_header(0)).map_err(|e| e.to_string())?;
        self.wav = Some(f);
        Ok(())
    }

    /// Hand the mixer's output for one frame to whichever sinks are open.
    pub fn push(&mut self, samples: &[i16]) {
        if samples.is_empty() {
            return;
        }
        if let Some(ring) = &self.ring {
            let mut r = ring.lock().unwrap();
            r.q.extend(samples.iter().copied());
            while r.q.len() > MAX_FILL * 2 {
                r.q.pop_front();
            }
        }
        if let Some(f) = &mut self.wav {
            let mut bytes = Vec::with_capacity(samples.len() * 2);
            for s in samples {
                bytes.extend_from_slice(&s.to_le_bytes());
            }
            if f.write_all(&bytes).is_ok() {
                self.wav_samples += samples.len() as u32 / 2;
            }
        }
    }

    /// Block until the queue has drained to its target, and report whether
    /// there was a device to wait on at all.
    ///
    /// This is what paces the emulator when the speakers are running, and it
    /// is deliberately the sound card's clock rather than a wall-clock timer:
    /// the card consumes an exact number of samples per second, so waiting on
    /// it makes the emulator produce exactly that many and the queue never
    /// drifts. A timer would be independently right and still slowly diverge.
    /// If the emulator is too slow to keep the queue full this returns at once
    /// and lets it run flat out, which is the correct response to being slow.
    pub fn pace(&self) -> bool {
        let Some(ring) = &self.ring else { return false };
        let deadline = std::time::Instant::now() + std::time::Duration::from_millis(100);
        while ring.lock().unwrap().q.len() / 2 > TARGET_FILL {
            // A device that has stopped consuming (unplugged, say) must not
            // freeze the emulator, so give up waiting after a short while.
            if std::time::Instant::now() > deadline {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        true
    }

    /// Output frames the device asked for and the emulator had not produced
    /// yet, and how much audio is queued. Both are the honest measure of
    /// whether emulation is keeping ahead of the sound card.
    pub fn health(&self) -> (u64, usize) {
        match &self.ring {
            Some(r) => {
                let r = r.lock().unwrap();
                (r.underruns, r.q.len() / 2)
            }
            None => (0, 0),
        }
    }

    /// Patch the WAV header now that the length is known, and report whether
    /// the speakers ever ran dry.
    pub fn finish(&mut self) {
        if let Some(f) = &mut self.wav {
            let _ = f.seek(SeekFrom::Start(0));
            let _ = f.write_all(&wav_header(self.wav_samples));
            let _ = f.flush();
            eprintln!(
                "audio: wrote {:.1}s ({} stereo frames)",
                self.wav_samples as f64 / SAMPLE_RATE as f64,
                self.wav_samples
            );
        }
        if let Some(ring) = &self.ring {
            let n = ring.lock().unwrap().underruns;
            if n > 0 {
                eprintln!("audio: {n} underrun frames (emulator running below full speed)");
            }
        }
    }
}

/// 44-byte canonical WAV header for 16-bit stereo at the DS mixer rate.
fn wav_header(frames: u32) -> [u8; 44] {
    let data_bytes = frames * 4;
    let mut h = [0u8; 44];
    h[0..4].copy_from_slice(b"RIFF");
    h[4..8].copy_from_slice(&(36 + data_bytes).to_le_bytes());
    h[8..12].copy_from_slice(b"WAVE");
    h[12..16].copy_from_slice(b"fmt ");
    h[16..20].copy_from_slice(&16u32.to_le_bytes()); // PCM chunk size
    h[20..22].copy_from_slice(&1u16.to_le_bytes()); // format: PCM
    h[22..24].copy_from_slice(&2u16.to_le_bytes()); // channels
    h[24..28].copy_from_slice(&SAMPLE_RATE.to_le_bytes());
    h[28..32].copy_from_slice(&(SAMPLE_RATE * 4).to_le_bytes()); // byte rate
    h[32..34].copy_from_slice(&4u16.to_le_bytes()); // block align
    h[34..36].copy_from_slice(&16u16.to_le_bytes()); // bits per sample
    h[36..40].copy_from_slice(b"data");
    h[40..44].copy_from_slice(&data_bytes.to_le_bytes());
    h
}
