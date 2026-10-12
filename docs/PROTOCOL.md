# Konica Minolta DiMAGE Scan Dual IV — USB protocol

Recovered from the PowerPC Mac OS driver (version for Mac OS 9 / Mac OS X
10.2–10.4): `MFSIOUsb2891.bundle` (Mach-O USB transport), `MFSIFLib2891`
(CFM bus layer), `MFSBaseLib2891` (scanner control, contains Konica
Minolta's port of the Avision "AVI" scanner library), `MFSLib2891` (scan
sequencing, exposure, autofocus) and `DS Dual4 Driver` (Mac OS 9 USB class
driver).  Function names below are the ones in the original binaries'
traceback tables.

The scanner is an Avision-built device: the command set is the Avision
SCSI-over-USB dialect also used by SANE's `avision` backend, with a few
Minolta-specific data types.  Everything documented here is implemented in
`src/` and was verified byte-for-byte against the original code running in a
PowerPC emulator (`tests/run_diff.sh`).

## 1. Device

| | |
|---|---|
| USB vendor / product | `0x132b` / `0x000a` |
| INQUIRY vendor/product | `KONICA MINOLTA  FS-V31  ` (firmware 1.04) |
| Optical resolution | 3200 dpi, 3 channels (no infrared channel) |
| Frame (35 mm) | 3120 × 4680 px at 3200 dpi |
| Frame (APS) | 2184 × 3792 px at 3200 dpi, horizontal offset 648 px |
| Controller | 8051-family firmware (`DS_Dual4.BIN`, 51 838 bytes) |

The firmware image embeds the scanner's 96-byte INQUIRY answer at file offset
48246 (search for `FS-V31` and step back 24 bytes).

## 2. Transport

One USB interface with a bulk-OUT and a bulk-IN pipe (an interrupt pipe may
exist and is unused).  Every command is three phases
(`MFSIOUsbExecCommand`):

1. bulk-OUT: the CDB, **always 10 bytes**, zero padded (6-byte CDBs too);
2. optional data phase on bulk-OUT (host→scanner) or bulk-IN;
3. bulk-IN: **one status byte** (`TakeFeedBack`).

Status: `0` GOOD, `2` CHECK CONDITION (issue REQUEST SENSE), `8` BUSY.
The library waits one Mac tick (1/60 s) after each command.  When a RELEASE
UNIT failed, the next command is preceded by another RELEASE UNIT.

REQUEST SENSE (`03 00 00 00 16 00`, 22 bytes) is mapped to driver error
codes (`CmdRequestSense`): response code 0x70/0x7f required; only sense key 3
is interpreted — ASC 0x81 → `0xC8 + ASCQ` (ASCQ 0..8, mechanical errors),
ASC 0x80 → ASCQ 1: `0x67`, 2: `0x68`, 3: `0x69`, 5: `0xD1`; otherwise
`0x7B`.  Status 8 maps to `0xA2` (busy).

`IsScannerReady(t)` polls TEST UNIT READY while the error is `0xA2`, for at
most `t` ticks (0x168 = 6 s in most places, 600 ticks before focus moves).

## 3. Commands

| opcode | CDB | direction |
|---|---|---|
| `00` TEST UNIT READY | `00 00 00 00 00 00` | – |
| `03` REQUEST SENSE | `03 00 00 00 16 00` | in 22 |
| `12` INQUIRY | `12 00 00 00 60 00` | in 96 |
| `16` RESERVE UNIT | `16 …` | – |
| `17` RELEASE UNIT | `17 …` | – |
| `1B` SCAN | `1B 00 00 00 01 ff` — `ff`: 0x80 = speed mode (always set), 0x40 = special mode | – |
| `24` SET WINDOW | `24 00 00 00 00 00 LL LL LL 00` | out |
| `28` READ | `28 00 dt 00 0A 0D LL LL LL 00` (qualifier always `0x0A0D`) | in |
| `2A` SEND | `2A 00 dt 00 qq qq LL LL LL 00` | out |
| `31` OBJECT POSITION | `31 op …`; op 2 = return carriage home ("paper feed") | – |

### READ / SEND data types

| dt | dir | size | meaning |
|---|---|---|---|
| `00` | READ | n | image data |
| `60` | READ | 32 | calibration format (§5) |
| `61`/`62` | READ | lines·ppl·bpc | white shading data, gray/colour |
| `66` | READ | lines·ppl·bpc | dark shading data |
| `69` | READ | 128 | NVM data (only if INQUIRY byte 52 bit 0) |
| `6C` | READ/SEND | 24 / n | acceleration table request / table (§8) |
| `70` | READ | 32 | raw line info (lamp warm-up) |
| `71` | READ | n | raw line (lamp warm-up) |
| `81` | SEND | 512 | gamma table, qualifier = channel−1, 256 LE words |
| `82` | SEND | 2·px | shading table: qualifier 0,1,2 white R,G,B; 3,4,5 dark R,G,B; LE words |
| `87` | READ/SEND | 8 | film holder (§9) |
| `88` | SEND | 8 | focus: BE word, `pos|0x8000` for pos > 0, `-pos` otherwise |
| `90` | READ/SEND | 34 / image | firmware status / firmware image (§11) |
| `92` | SEND | 8 | reset to initial state, byte 0 = mode (0) |
| `A0` | READ | 1 | lamp status: 0 ready, 2/3 warming up, 4/5 lamp error |

## 4. INQUIRY fields used (`GetScannerAbilityEx`)

| byte | value on Dual IV | use |
|---|---|---|
| 36 | `20` | bit 3 → gamma capabilities, bits 0-2 → caps 0x800 |
| 37 / 38 | `03` / `06` | optical / max res ÷ 100 (overridden by 89-90) |
| 39 | `67` | bit 2: extended inquiry; bit 5: max res words at 40-47 present |
| 50 | `D9` | bit 5: host must re-align colour lines (not set → scanner aligns) |
| 51 | `96` | bit 7: exposure in SET WINDOW supported |
| 52 | `FA` | bit 0: NVM present |
| 54-59 | `04 04 02 08 08 08` | line pixel alignment: colour 4, gray 4, lineart 8 |
| 62 | `A8` | bit 3: focus motor present |
| 75-76 | `0FFF` | default shading target |
| 81-84 | `0C30 1248` | max pixels / lines (3120 × 4680) |
| 89-90 | `0C80` | optical resolution 3200 |
| 91 | `06` | scanner type → 16-bit calibration mode |

## 5. Calibration format (READ 0x60, 32 bytes)

| bytes | endianness | meaning |
|---|---|---|
| 0-1 | BE | pixels per line of calibration data |
| 2 | | bytes per channel |
| 3 | | lines (counts R, G and B lines: per-colour lines = value / 3) |
| 4 | | state: 1 = needs initialisation, 2 = needs calibration, other = ready |
| 5 | | ability: bit 2 dark data available, bit 3 colour |
| 9, 11, 13 | BE | white shading targets R, G, B |
| 15, 17, 19 | BE | dark shading targets R, G, B (`FFFF` = none) |
| 21, 23, 25 | **LE** | base exposure time R, G, B |
| 27 / 29 | **LE** | maximum / minimum exposure time |

## 6. SET WINDOW

Transfer length `0x3B` (standard) or `0x46` (extended exposure block), +1
when multi-sampling.  8 byte header (bytes 6-7: descriptor length `0x33` /
`0x3E`, BE) followed by the descriptor (offsets relative to the descriptor):

| off | size | field |
|---|---|---|
| 2 / 4 | 2 | x / y resolution |
| 6 / 10 | 4 | upper-left x / y in **3200 dpi units** |
| 14 / 18 | 4 | width / length (pixels at the scan resolution in the final-scan path) |
| 22 | 1 | brightness + 0x80 |
| 23 | 1 | threshold 0x80 |
| 24 | 1 | contrast + 0x80 |
| 25 | 1 | composition: 0 lineart, 1 halftone, 2 gray, 5 colour |
| 26 | 1 | bits per channel: 8, or 12 for every 16-bit mode |
| 27 | 2 | halftone pattern − 1 (the original stores contrast+0x7F here in gray/colour) |
| 29 | 1 | padding: 0x03 (0x83 inverted) |
| 40 | 1 | vendor 0xFF |
| 41 | 1 | parameter length 9 / 0x14 |
| 42 | 1 | bitset 1: 0x40, +0x20 colour, gray filter bits |
| 43 / 44 | 1 | highlight 0xFF / shadow 0 |
| 45 | 2 | bytes per line (pixels × bits / 8) |
| 47 | 2 | line count |
| 49 | 1 | bitset 2 |
| 50 | 1 | IR exposure (100) |
| 51, 53, 55 | 2 | exposure R, G, B in percent of the base exposure (`100 × factor`) |
| 57 | 1 | bitset 3: 0x10 always; 0x02 multi-sample; 0x04 multi-scan; 0x08; 0x40 index |
| 58 | 1 | multi-scan count |
| 61 | 1 | multi-sample count |

Final scans use y resolutions 800, 1600 or 3200 only (other values are
scanned at the next step and resampled by the host); previews also allow 100,
200 and 400.  Widths are aligned up to 4 pixels.

Image data (READ 0x00) is pixel-interleaved RGB; 16-bit samples are
**little-endian**.  The scanner already registers the R, G, B CCD lines.  With
multi-sampling every line is sent *n* times and averaged by the host.

## 7. Bring-up and calibration

`InitializeBaseScanner` → `InitializeDriver`, GET CALIBRATION FORMAT (state),
INQUIRY.  If the state says "needs initialisation" (`DoInitializeScanner`):

1. SEND 0x92 (reset), TEST UNIT READY;
2. lamp warm-up (`WarmUpCheck`): poll READ 0xA0 until not 3 (max 120 s), then
   READ 0x70/0x71 raw lines every 10 s until two averages agree within 97 %;
3. SEND 0x92, TEST UNIT READY, wait 1 s;
4. calibration window (`SETTempScanParameter`): one 3120 px line at 3200 dpi,
   colour, 48 bit, flags 0x9008, exposures 100 %; no shading upload;
5. `CalibrationData`: GET CALIBRATION FORMAT, READ 0x62 white data, OBJECT
   POSITION 2, READ 0x66 dark data (if ability bit 2);
6. per sample position: partially sort the lines, drop the darkest third and
   average the rest (`BubbleSort`);
7. white −= dark; dark −= dark target (clamped); dark smoothed with a 600 px
   running mean where it deviates < 35 (`DarkSmoothProcess`);
8. white gain = `round(target × 0x4000 / white)` (or `target` when
   white < 0x4000); targets come from the calibration format (shifted << 8
   in 16-bit mode);
9. tables are stored per channel (3200 entries), shifted one pixel left.

Before every scan the tables are box-averaged by `3200 / y-res` from
`ulx (+648 for APS)` and sent with SEND 0x82 (dark 3,4,5 then white 0,1,2),
repeated ×2 (2 samples) or ×4 when multi-sampling.  For non-standard
exposures (anything but 1/0.7 on all channels) the dark data is re-measured
first at that exposure (`DarkCalibrationData`).

## 8. Starting a scan and the acceleration table

`StartScan`: RESERVE UNIT, SCAN (flags 0x80), then loop:
READ 0x6C (24 bytes, BE fields):

| bytes | field |
|---|---|
| 0-1 | total table entries N |
| 2-3 | constant tail entries |
| 4-7 / 8-11 | a / b (step count constraint) |
| 12-13 / 14-15 | start / end speed value |
| 16 | 0 = host must compute the table |
| 17 | number of tables requested |

Table (`GetAccTable`): bisection for k in [0.001, 1] (single precision, stop
at Δ ≤ 1e-4) such that the ramp `v₀ = start; vᵢ₊₁ = v − k(v − end)` (fused
multiply-subtract) rounded to bytes reaches `end` after exactly
`N − tail + 1` entries; copy the last entry over the tail; pad entries so that
`a × Σ` is a multiple of `b` (`FixTable`); decrement every entry; SEND 0x6C
(qualifier = table index, N bytes).

Image data is read with READ 0x00 in blocks of whole lines (≤ 64 KiB).  The
scan ends with RELEASE UNIT and OBJECT POSITION 2.

## 9. Film holder

READ 0x87 (8 bytes): byte 0 type (0 none, 1 APS adapter, 2 35 mm strip
holder (6 frames), 3 slide holder (4), 0xFF not ready/none), byte 1 current
frame, byte 2 frame count (APS), byte 3 APS film type, byte 4 = 1 while
loading.  SEND 0x87 `[type, frame]` moves to frame 1..N; frame 0xFF ejects.

## 10. Focus

SEND 0x88 sets the focus motor (range ±200, the driver approaches targets
from +50 to take up backlash).  Autofocus (`AutoFocus`): exposure for focus
from one line at position 0 (max sample → 50 %, factor ≤ 7); then a 256 px ×
1 line window at 3200 dpi in the frame centre in *multi-scan* mode (SET WINDOW
flags | 0x10A, bitset3 0x04); coarse sweep +200…−200 step 10, fine sweep ±50
around the best step 2; three reads averaged per position; sharpness
`√((3·ΣR + 6·ΣG + ΣB)/10)` with Σ = Σ|2vᵢ − vᵢ₋₁ − vᵢ₊₁| on 8-bit values.

## 11. Exposure

Exposure factors multiply the per-channel base exposure from the calibration
format.  Standard factors (`GetStdGain`): positive 1/0.7 on all channels;
colour negative 1/0.65, 1/0.3, 1/0.2; black-and-white negative 1.0.
`CheckGainLimit` clamps `factor × base` to [min + 10, max] (for colour
negatives and previews a lower limit from `GetMaxExpTime`) and to
3.0 / 2.2 / 3.2 × the standard factor for final scans; the remainder is
applied by the host as digital gain.

## 12. Firmware

On the first connection `MFSBaseGetScannerInfo` runs `DownFirmware`: READ
0x90 (34 bytes); if byte 0 ≠ 0 and the file's version (after `FS-V31`) is
not older than INQUIRY bytes 32-35, SEND 0x90 with the file followed by the
32-bit byte sum (little-endian), then TEST UNIT READY and wait 3 s.
