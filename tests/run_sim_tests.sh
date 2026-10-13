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
            local f; f=$(ls "$OUT"/$name*.tiff "$OUT"/$name*.dng 2>/dev/null | head -1)
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
if [ -f profiles/MLTF3200p.icc ]; then
    if sips -g profile "$OUT/full3200.tiff" | grep -q "Posi Linear" && ! sips -g profile "$OUT/nega400.tiff" | grep -q "Posi Linear"; then
        echo "ok   icc-profile-embedding"; pass=$((pass+1))
    else echo "FAIL icc-profile-embedding"; failn=$((failn+1)); fi
fi
if grep -q "best focus position: 3[5-9]" "$OUT/autofocus.log"; then echo "ok   autofocus-converges"; pass=$((pass+1));
else echo "FAIL autofocus did not converge:"; cat "$OUT/autofocus.log"; failn=$((failn+1)); fi

check() { # name condition-command...
    local name=$1; shift
    if "$@"; then echo "ok   $name"; pass=$((pass+1)); else echo "FAIL $name"; failn=$((failn+1)); fi
}
PY=${PYTHON:-python3}
IC="$PY tests/imgcheck.py"
gt() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a > b) }'; }

# --- fast autofocus: same result as the original sequence, far fewer lens moves
run af-fast     -          autofocus --frame 2 --af fast
run af-classic  -          autofocus --frame 2 --af classic
fb=$(awk '/best focus/{print $4}' "$OUT/af-fast.log"); cb=$(awk '/best focus/{print $4}' "$OUT/af-classic.log")
fm=$(grep -c '^CDB 2a 00 88' "$OUT/af-fast.trace"); cm=$(grep -c '^CDB 2a 00 88' "$OUT/af-classic.trace")
echo "     autofocus fast $fb ($fm lens moves), classic $cb ($cm lens moves)"
check af-fast-matches-classic [ -n "$fb" ] && [ $(( fb > cb ? fb - cb : cb - fb )) -le 2 ] && [ "$fm" -lt 40 ]

# --- negatives: density-space inversion of an orange-mask negative is neutral
DSD4_SIM_FILM=nega run invert-nega 390x585 scan --dpi 400 --film nega --invert --focus 37 -o "$OUT/invert-nega.tiff"
read -r dr db <<< "$($IC neutral "$OUT/invert-nega.tiff")"
echo "     inverted neutral scene: |R-G| $dr |B-G| $db"
check invert-neutral gt 0.02 "$dr" && gt 0.02 "$db"
DSD4_SIM_FILM=nega run minolta-preview 390x585 preview --film nega --minolta --dpi 400 --focus 37 -o "$OUT/minolta-preview.tiff"
if clang -O1 -Iinclude tests/minolta_cache.c build/libdsdual4.a -framework IOKit -framework CoreFoundation \
       -framework ApplicationServices -framework ImageIO -o "$OUT/minolta_cache" 2>"$OUT/mc.log"; then
    if "$OUT/minolta_cache"; then pass=$((pass+1)); else failn=$((failn+1)); fi
else echo "FAIL minolta-preview-reuse (build)"; failn=$((failn+1)); fi

# --- raw DNG, multi-sample and HDR
run dng-raw     780x1170   scan --dpi 800 --focus 37 --format dng -o "$OUT/dng-raw.dng"
info=$($IC dng "$OUT/dng-raw.dng"); echo "     $info"
check dng-structure grep -q "DNGVersion 1.4.0.0 photometric 34892 780x1170 spp 3 bps 16 sampleformat 1 colormatrix yes" <<< "$info"
DSD4_SIM_NOISE=0.01 run dng-noisy 780x1170 scan --dpi 800 --focus 37 --format dng -o "$OUT/dng-noisy.dng"
DSD4_SIM_NOISE=0.01 run dng-ms8   780x1170 scan --dpi 800 --focus 37 --samples 8 --format dng -o "$OUT/dng-ms8.dng"
check dng-float-format grep -q "bps 16 sampleformat 3" <<< "$($IC dng "$OUT/dng-ms8.dng")"
DSD4_SIM_NOISE=0.01 DSD4_SIM_OUTLIER=5 run dng-ms8-outlier 780x1170 scan --dpi 800 --focus 37 --samples 8 --format dng -o "$OUT/dng-ms8-outlier.dng"
n1=$($IC noise "$OUT/dng-raw.dng" "$OUT/dng-noisy.dng" | cut -d' ' -f1)
n8=$($IC noise "$OUT/dng-raw.dng" "$OUT/dng-ms8.dng" | cut -d' ' -f1)
no=$($IC noise "$OUT/dng-raw.dng" "$OUT/dng-ms8-outlier.dng" | cut -d' ' -f1)
echo "     noise: 1 sample $n1, 8 samples $n8, 8 samples with outlier reads $no"
check multisample-8x-noise gt "$n1" "$(awk -v a="$n8" 'BEGIN{print a*2.5}')"
check multisample-outliers-rejected gt "$(awk -v a="$n8" 'BEGIN{print a*1.3}')" "$no"
DSD4_SIM_NOISE=0.01 DSD4_SIM_SHIFT=0.5 run dng-hdr 780x1170 -v scan --dpi 800 --focus 37 --format dng --hdr -o "$OUT/dng-hdr.dng"
sh=$(sed -n 's/.*offset \(-*[0-9.]*\) lines.*/\1/p' "$OUT/dng-hdr.log")
nh=$($IC noise "$OUT/dng-raw.dng" "$OUT/dng-hdr.dng" 0.2 | cut -d' ' -f1)
nd=$($IC noise "$OUT/dng-raw.dng" "$OUT/dng-noisy.dng" 0.2 | cut -d' ' -f1)
echo "     HDR: pass offset found $sh lines (simulated 0.5); shadow noise $nd -> $nh"
check hdr-alignment awk -v s="$sh" 'BEGIN { s = s < 0 ? -s : s; exit !(s > 0.4 && s < 0.6) }'
check hdr-shadow-noise gt "$nd" "$(awk -v a="$nh" 'BEGIN{print a*3}')"
# noiseless, unshifted passes: the merge must reproduce the single pass
# (no bias or seam where the long exposure hands over to the short one)
run dng-hdr-clean 780x1170 scan --dpi 800 --focus 37 --format dng --hdr -o "$OUT/dng-hdr-clean.dng"
ns=$($IC noise "$OUT/dng-raw.dng" "$OUT/dng-hdr-clean.dng" | cut -d' ' -f1)
echo "     HDR merge of noiseless passes vs single pass: rms $ns"
check hdr-no-seams gt 0.0005 "$ns"
# macOS must render both DNG flavours with real content (a float DNG without
# WhiteLevel decodes, but black)
for f in dng-raw dng-hdr; do
    m=-1
    if [ "$(sips -g pixelWidth "$OUT/$f.dng" 2>/dev/null | awk '/pixel/{print $2}')" = 780 ] &&
       sips -s format bmp "$OUT/$f.dng" --out "$OUT/$f-decoded.bmp" >/dev/null 2>&1; then
        m=$($IC mean "$OUT/$f-decoded.bmp")
    fi
    echo "     $f decoded by macOS: mean level $m"
    check $f-decodes-on-macos gt "$m" 0.2
done

# --- super-resolution research: the measurements recover the simulated optics
export DSD4_SIM_CHART=star
DSD4_SIM_NOISE=0.01 run research-nyquist - research nyquist --focus 37
DSD4_SIM_NOISE=0.01 DSD4_SIM_PSF=1.2 run research-nyquist-blur - research nyquist --focus 37
check research-detects-detail grep -q "G1 detail at the sampling limit: *yes" "$OUT/research-nyquist.log"
check research-detects-blur grep -q "G1 detail at the sampling limit: *no" "$OUT/research-nyquist-blur.log"
DSD4_SIM_BREATHING=60 DSD4_SIM_JITTER=0.2 run research-focus - research focus --focus 37
ppm=$(sed -n 's/.*scale change: \([-0-9.]*\) ppm.*/\1/p' "$OUT/research-focus.log")
echo "     lens breathing: simulated 60 ppm/step, measured $ppm"
check research-breathing awk -v p="$ppm" 'BEGIN { exit !(p > 57 && p < 63) }'
DSD4_SIM_JITTER=0.3 run research-jitter - research jitter --focus 37
$PY - "$OUT/research-jitter.log" "$OUT/research-jitter.trace" > "$OUT/jitter-check.txt" <<'PYEOF'
import sys, re
meas = [float(v) for v in re.search(r'dy:\s+(.*)', open(sys.argv[1]).read()).group(1).split()]
off = [float(l.split()[1]) for l in open(sys.argv[2]) if l.startswith('SCANOFF')][-len(meas):]
err = max(abs(m - (off[0] - o)) for m, o in zip(meas, off))
print('%.4f' % err)
PYEOF
je=$(cat "$OUT/jitter-check.txt"); echo "     pass offsets: worst error against the simulated truth $je lines"
check research-jitter-accuracy gt 0.03 "$je"
run research-yres-refused - research yres --focus 37
check research-yres-refused grep -q "G2 vertical oversampling via window: *no" "$OUT/research-yres-refused.log"
DSD4_SIM_YRES=1 DSD4_SIM_NOISE=0.01 run research-yres - research yres --focus 37
check research-yres-oversampled grep -q "6400 dpi: accepted.*really oversampled" "$OUT/research-yres.log"
DSD4_SIM_YRES=2 DSD4_SIM_NOISE=0.01 run research-yres-pitch - research yres --focus 37
check research-yres-same-pitch grep -q "6400 dpi: accepted.*NOT finer" "$OUT/research-yres-pitch.log"
unset DSD4_SIM_CHART

echo "$pass passed, $failn failed (artifacts in $OUT)"
[ $failn = 0 ]
