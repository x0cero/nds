"""Log ARM9 IPCFIFOSEND writes in DeSmuME."""
import sys

from desmume.emulator import DeSmuME

emu = DeSmuME()
emu.open(sys.argv[1])
frames = int(sys.argv[2])

count = [0]

def cb(addr, size):
    if count[0] < 40:
        r = emu.memory.register_arm9
        # The value being stored is in a register; easiest reliable signal
        # is r0-r3 at the store site. Print all four.
        print(f"fifo9 write@{r.pc:#010x} r0={r.r0:#010x} r1={r.r1:#010x} r2={r.r2:#010x}")
    count[0] += 1

emu.memory.register_write(0x04000188, cb, 4)

for _ in range(frames):
    emu.cycle()
print("total arm9 fifo writes:", count[0])
emu.destroy()
