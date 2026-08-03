"""Ground-truth probe: run a ROM in DeSmuME, dump state at checkpoints.

Usage: .venv/bin/python tools/refprobe.py <rom> <frames> [addr:len ...]
Writes ref_frame_<n>.png screenshots and prints IO/memory state.
"""
import sys

from desmume.emulator import DeSmuME

rom = sys.argv[1]
frames = int(sys.argv[2])
watches = []
for spec in sys.argv[3:]:
    a, l = spec.split(":")
    watches.append((int(a, 16), int(l, 16)))

emu = DeSmuME()
emu.open(rom)

checkpoints = sorted({frames // 4, frames // 2, frames - 1, 13, 60, 150} | {frames - 1})

for f in range(frames):
    emu.cycle()
    if f in checkpoints:
        img = emu.screenshot()
        img.save(f"ref_frame_{f}.png")
        mem = emu.memory.unsigned
        ie = mem.read_long(0x04000210)
        if_ = mem.read_long(0x04000214)
        ime = mem.read_long(0x04000208)
        dispa = mem.read_long(0x04000000)
        pal0 = mem.read_short(0x05000000)
        print(f"f{f}: ie9={ie:#010x} if9={if_:#010x} ime={ime} dispA={dispa:#010x} pal0={pal0:#06x}")
        for addr, ln in watches:
            words = [f"{mem.read_long(addr + i):#010x}" for i in range(0, ln, 4)]
            print(f"  {addr:#010x}: {' '.join(words)}")

emu.destroy()
print("done")
