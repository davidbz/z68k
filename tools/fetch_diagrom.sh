#!/usr/bin/env bash
# Fetch DiagROM into roms/, for the Amiga boot example.
#
# DiagROM (diagrom.com, by John Hertell) is freely distributable and narrates
# its power-on tests over the serial port, which examples/amiga.zig prints.
# Roughly 750 KB; roms/ is gitignored.
set -euo pipefail

cd "$(dirname "$0")/.."

url=https://www.diagrom.com/files/daily/DiagROMV2.zip

mkdir -p roms
echo "downloading $url"
curl -fL --progress-bar -o roms/DiagROMV2.zip "$url"
unzip -oq roms/DiagROMV2.zip -d roms
mv -f roms/DiagROMV2/diagrom.rom roms/diagrom.rom
rm -rf roms/DiagROMV2 roms/DiagROMV2.zip

echo "roms/diagrom.rom ready ($(wc -c < roms/diagrom.rom) bytes)"
echo "run: zig build amiga"
