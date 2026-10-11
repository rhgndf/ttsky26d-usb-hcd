#!/bin/bash
# Rough area estimate: synthesize src/*.v with yosys against the sky130_fd_sc_hd
# tt liberty from the ciel PDK and print total cell area in um^2.
# Uses yosys from the LibreLane docker image (same version as the harden flow).
#
# Usage: scripts/area.sh            # whole design
#        scripts/area.sh usb_sie    # single module (must still read all sources)
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PDK_VER=8afc8346a57fe1ab7934ba5a6056ea8b43078e71
PDK_ROOT="${PDK_ROOT:-$HOME/.ciel}"
LIB="$PDK_ROOT/ciel/sky130/versions/$PDK_VER/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
TOP="${1:-$(grep -E '^\s*top_module' "$REPO/info.yaml" | sed 's/.*"\(.*\)".*/\1/')}"
IMAGE=ghcr.io/librelane/librelane:3.0.14

docker run --rm -i -v "$REPO/src":/src -v "$LIB":/liberty.lib:ro "$IMAGE" \
    yosys -Q -T -p "
        read_verilog /src/*.v;
        synth -top $TOP -flatten;
        dfflibmap -liberty /liberty.lib;
        abc -liberty /liberty.lib;
        stat -liberty /liberty.lib;
    " 2>&1 | grep -v "^$" | tail -40
