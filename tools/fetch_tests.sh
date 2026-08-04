#!/usr/bin/env bash
# Fetch the SingleStepTests/m68000 conformance data into testdata/.
#
# The harness reads the upstream .json.bin files directly, so no decode step is
# needed. Roughly 2 GB after clone; testdata/ is gitignored.
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -d testdata/.git ]; then
    echo "updating testdata/"
    git -C testdata pull --ff-only
else
    echo "cloning SingleStepTests/m68000 into testdata/ (this is large)"
    git clone --depth 1 https://github.com/SingleStepTests/m68000.git testdata
fi

echo "$(find testdata/v1 -name '*.json.bin' | wc -l) test files ready"
echo "run: zig build sst"
