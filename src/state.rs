//! Savestates: dump the entire machine to a file and restore it later, so a
//! long scripted replay does not have to be re-run to reach an interesting
//! screen.
//!
//! The snapshot is produced by serde derives on the emulator's own structs
//! rather than a hand-written field list. That matters: a hand-written writer
//! silently goes stale the moment someone adds a field to `Machine`, and the
//! symptom (a resumed run that diverges a few thousand frames later) is nearly
//! impossible to trace back. With derives, a new field is snapshotted the day
//! it is added.
//!
//! Two things are deliberately left out and re-injected by the caller: the
//! cartridge image (megabytes, never changes) and the NDS_IOLOG counters
//! (debug only). Both are `#[serde(skip)]` on `Machine`.
//!
//! File layout: b"NDSSTATE" + u32 version + bincode payload (about 6 MB).

use crate::bus::{Bus, Machine};
use crate::cpu::{Cpu, CpuState};
use crate::ppu::Ppu;
use std::cell::RefCell;
use std::rc::Rc;

const MAGIC: &[u8; 8] = b"NDSSTATE";
const VERSION: u32 = 1;

/// The snapshot. Generic over ownership purely so that saving and loading can
/// share one field list: saving serializes `SaveState<&Machine, &CpuState,
/// &Ppu>`, loading deserializes the owned default.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct SaveState<M = Machine, C = CpuState, P = Ppu> {
    pub machine: M,
    pub cpu9: C,
    pub cpu7: C,
    pub ppu: P,
    /// Frame the resumed run should start on (the one AFTER the last frame
    /// rendered into this snapshot), so a resumed run keeps NDS_FRAMES /
    /// NDS_INPUT / NDS_TOUCH on the original timeline.
    pub frame: u32,
}

fn cfg() -> impl bincode::config::Config {
    bincode::config::standard()
}

/// Serialize the machine at a frame boundary. Must not be called from inside
/// the scanline loop: that loop keeps live state in local variables.
pub fn save<B: Bus>(
    path: &str,
    m: &Rc<RefCell<Machine>>,
    cpu9: &Cpu<B>,
    cpu7: &Cpu<B>,
    ppu: &Ppu,
    frame: u32,
) -> Result<(), String> {
    let mm = m.borrow();
    let snap = SaveState {
        machine: &*mm,
        cpu9: &cpu9.st,
        cpu7: &cpu7.st,
        ppu,
        frame,
    };
    let body = bincode::serde::encode_to_vec(&snap, cfg()).map_err(|e| e.to_string())?;
    let mut out = Vec::with_capacity(body.len() + 12);
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&body);
    std::fs::write(path, &out).map_err(|e| e.to_string())?;
    eprintln!("saved state (resumes at frame {frame}): {path} ({} bytes)", out.len());
    Ok(())
}

/// Read a snapshot file, refusing anything that is not this exact format and
/// version (a state from a different build would restore garbage).
pub fn read_file(path: &str) -> Result<SaveState, String> {
    let data = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if data.len() < 12 || &data[..8] != MAGIC {
        return Err(format!("{path}: not an nds savestate"));
    }
    let ver = u32::from_le_bytes(data[8..12].try_into().unwrap());
    if ver != VERSION {
        return Err(format!("{path}: savestate version {ver}, this build wants {VERSION}"));
    }
    let (st, _) = bincode::serde::decode_from_slice::<SaveState, _>(&data[12..], cfg())
        .map_err(|e| format!("{path}: {e}"))?;
    Ok(st)
}

/// Install a snapshot. Returns the frame index it was taken at.
///
/// The existing `Rc<RefCell<Machine>>` is assigned into, never replaced: both
/// CPU views already hold clones of that Rc, and a fresh Rc would leave them
/// driving the old machine while the frontend rendered the new one.
pub fn apply<B: Bus>(
    st: SaveState,
    m: &Rc<RefCell<Machine>>,
    cpu9: &mut Cpu<B>,
    cpu7: &mut Cpu<B>,
    ppu: &mut Ppu,
    rom: &[u8],
) -> u32 {
    let SaveState { machine, cpu9: c9, cpu7: c7, ppu: p, frame } = st;
    {
        let mut mm = m.borrow_mut();
        let io_log = mm.io_log.take();
        *mm = machine;
        mm.rom = rom.to_vec();
        mm.io_log = io_log;
        // The backup chip's contents now come from the snapshot and no longer
        // match the .sav on disk. Clearing the dirty flag stops a loaded state
        // from flushing itself over a real playthrough's save file.
        mm.save_dirty = false;
    }
    cpu9.restore(c9);
    cpu7.restore(c7);
    *ppu = p;
    // The DTCM base exists in three places: Machine::dtcm_base, the CP15
    // register, and a literal patched into the ARM9 IRQ stub. All three are in
    // the snapshot, so this only re-derives what is already there - but it
    // keeps them provably in step if any one of them is ever dropped.
    let cp15 = cpu9.st.cp15_dtcm;
    if cp15 != 0 {
        cpu9.bus.set_dtcm(cp15 & 0xFFFF_F000);
    }
    eprintln!("loaded state, resuming at frame {frame}");
    frame
}
