#!/bin/sh
# Two linked consoles for wireless work.
#   console 1: a scratch copy of the real save (the real one is never touched)
#   console 2: tests/platinum.sav2, the same save edited to a different trainer
# Frame-level wireless trace lands in /tmp/wifi.log. Tab moves the keyboard
# between consoles; F5 snapshots whichever console has it.
cd "$(dirname "$0")/.." || exit 1
[ -f /tmp/link1.sav ] || cp tests/platinum.sav /tmp/link1.sav
NDS_LINK=1 NDS_WIFILOG=1 NDS_SAV=/tmp/link1.sav NDS_SAV2=tests/platinum.sav2 \
  ./target/release/nds tests/platinum.nds 2>/tmp/wifi.log
echo "wireless trace: /tmp/wifi.log ($(wc -l < /tmp/wifi.log) lines)"
