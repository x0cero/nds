"""Hook game call-return sites in DeSmuME and print r0 there.

Usage: .venv/bin/python tools/refhook.py <rom> <frames> <addr> [addr ...]
Addresses are thumb return sites (even address of the instruction).
"""
import sys

from desmume.emulator import DeSmuME

rom = sys.argv[1]
frames = int(sys.argv[2])
addrs = [int(a, 16) for a in sys.argv[3:]]

emu = DeSmuME()
emu.open(rom)

hits = {}

def make_cb(addr):
    def cb(a, size):
        n = hits.get(addr, 0)
        if n < 5:
            r = emu.memory.register_arm9
            print(
                f"HOOK {addr:#010x}: r0={r.r0:#010x} r1={r.r1:#010x} "
                f"r2={r.r2:#010x} r3={r.r3:#010x} lr={r.lr:#010x}"
            )
        hits[addr] = n + 1
        return True
    return cb

for a in addrs:
    emu.memory.register_exec(a, make_cb(a))

for _ in range(frames):
    emu.cycle()

for a, n in hits.items():
    print(f"total {a:#010x}: {n} hits")
emu.destroy()
