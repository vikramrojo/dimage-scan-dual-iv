#!/bin/bash
# End-to-end tests of the CLI against the protocol simulator.
set -u
cd "$(dirname "$0")/.."
BIN=./build/dsdual4
OUT=$(mktemp -d)
export DSD4_TIME_SCALE=0.002 DSD4_NO_CACHE=1
FW=${DSD4_FIRMWARE:-}
pass=0; failn=0

run() { # name expected-size command...
    local name=$1 size=$2; shift 2
    local trace=$OUT/$name.trace
    if DSD4_SIM_TRACE=$trace "$BIN" --sim ${FW:+--firmware "$FW"} -q "$@" >"$OUT/$name.log" 2>&1; then
        local v; v=$(grep -c VIOLATION "$trace" || true)
        if [ "$v" != 0 ]; then echo "FAIL $name: $v protocol violations"; failn=$((failn+1)); return; fi
        if [ "$size" != - ]; then
            local f; f=$(ls "$OUT"/$name*.tiff 2>/dev/null | head -1)
            local got; got=$(sips -g pixelWidth -g pixelHeight "$f" 2>/dev/null | awk '/pixel/{printf "%s%s", s, $2; s="x"}')
            if [ "$got" != "$size" ]; then echo "FAIL $name: size $got, expected $size"; failn=$((failn+1)); return; fi
        fi
        echo "ok   $name"; pass=$((pass+1))
    else
        echo "FAIL $name (exit $?):"; sed 's/^/     /' "$OUT/$name.log"; failn=$((failn+1))
    fi
}

run info        -          info
run init        -          init
run preview     780x1170   preview --focus 0 -o "$OUT/preview.tiff"
run full3200    3120x4680  scan --dpi 3200 --focus 37 -o "$OUT/full3200.tiff"
run crop1600    800x600    scan --dpi 1600 --focus 37 --area 100,100,800,600 -o "$OUT/crop1600.tiff"
run dpi1000     975x1462   scan --dpi 1000 --focus 37 -o "$OUT/dpi1000.tiff"
run nega400     390x585    scan --dpi 400 --film nega --invert --bits 8 --focus 37 -o "$OUT/nega400.tiff"
run bw1600      1560x2340  scan --dpi 1600 --film bw-nega --focus 37 -o "$OUT/bw1600.tiff"
run ms4         780x1170   scan --dpi 800 --samples 4 --focus 37 -o "$OUT/ms4.tiff"
run rotate      1170x780   scan --dpi 800 --focus 37 --rotate 90 -o "$OUT/rotate.tiff"
run frame3      780x1170   scan --frame 3 --dpi 800 --focus 37 -o "$OUT/frame3.tiff"
run autofocus   -          autofocus --frame 2
run scanall     780x1170   scan-all --dpi 800 --focus keep -o "$OUT/scanall"
run eject       -          eject
DSD4_SIM_APS=1 run aps800 544x948 scan --dpi 800 --focus 37 -o "$OUT/aps800.tiff"
run crop700     97x61      scan --dpi 700 --focus 37 --area 3,3,97,61 -o "$OUT/crop700.tiff"
if DSD4_SIM_TRACE=/dev/null "$BIN" --sim -q scan --dpi 200 --focus 37 --area=-8,0,40,40 -o "$OUT/neg.tiff" >/dev/null 2>&1; then
    echo "FAIL negative-area accepted"; failn=$((failn+1)); else echo "ok   negative-area-rejected"; pass=$((pass+1)); fi
if DSD4_SIM_TRACE=/dev/null "$BIN" --sim -q scan --dpi 200 --focus 37 --area 1,0,1073741823,1 -o "$OUT/big.tiff" >/dev/null 2>&1; then
    echo "FAIL oversized-area accepted"; failn=$((failn+1)); else echo "ok   oversized-area-rejected"; pass=$((pass+1)); fi

run minolta-bw   1560x2340  scan --film bw-nega --minolta --dpi 1600 --focus 37 -o "$OUT/minolta-bw.tiff"
run minolta-col  1560x2340  scan --film nega --minolta --dpi 1600 --focus 37 -o "$OUT/minolta-col.tiff"
# offline conversion needs a calibration cache: make one in a private HOME
mkdir -p "$OUT/home"
if HOME="$OUT/home" DSD4_NO_CACHE= "$BIN" --sim -q init >/dev/null 2>&1 &&
   HOME="$OUT/home" "$BIN" -q convert "$OUT/bw1600.tiff" --film bw-nega -o "$OUT/converted.tiff" >/dev/null 2>&1 &&
   [ "$(sips -g pixelWidth "$OUT/converted.tiff" | awk '/pixel/{print $2}')" = 1560 ]; then
    echo "ok   offline-convert"; pass=$((pass+1))
else echo "FAIL offline-convert"; failn=$((failn+1)); fi
if [ -f "../DS_Dual4_extracted/DS Dual4/Profile/MLTF3200p.icc" ]; then
    if sips -g profile "$OUT/full3200.tiff" | grep -q "Posi Linear" && ! sips -g profile "$OUT/nega400.tiff" | grep -q "Posi Linear"; then
        echo "ok   icc-profile-embedding"; pass=$((pass+1))
    else echo "FAIL icc-profile-embedding"; failn=$((failn+1)); fi
fi
if grep -q "best focus position: 3[5-9]" "$OUT/autofocus.log"; then echo "ok   autofocus-converges"; pass=$((pass+1));
else echo "FAIL autofocus did not converge:"; cat "$OUT/autofocus.log"; failn=$((failn+1)); fi

echo "$pass passed, $failn failed (artifacts in $OUT)"
[ $failn = 0 ]
