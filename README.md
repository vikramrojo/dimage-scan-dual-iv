# dsdual4 — DiMAGE Scan Dual IV driver for modern macOS

A native, user-space macOS driver for the **Konica Minolta DiMAGE Scan Dual IV**
film scanner (USB `132b:000a`, firmware "FS-V31"), whose official software only
exists for PowerPC Macs (Mac OS 9 / Mac OS X 10.2–10.4).

It was produced by reverse engineering the PowerPC driver in this folder's
`DS_Dual4_extracted/` (CFM/PEF libraries `MFSBaseLib2891`, `MFSLib2891`,
`MFSIFLib2891`, the Mach-O USB bundle `MFSIOUsb2891` and the Mac OS 9 class
driver). The scanner protocol, calibration, motor and exposure logic were
recovered and reimplemented in portable C. **The port is byte-identical to the
original driver**: running the original PowerPC code in a CPU emulator side by
side with this library produces the same scanner traffic, down to every
command, window block, shading table, motor table and firmware byte (see
[Verification](#verification)).

What you get:

* **`dsdual4`**: a command line tool to initialise and calibrate the scanner,
  move between frames, eject, focus and autofocus, preview and scan (16-bit
  TIFF, up to 3200 dpi).
* **`dsdual4 serve`**: an **AirScan (eSCL) bridge**. The scanner then shows up
  in **Image Capture**, **Preview → File → Import from Scanner** and any other
  app that uses macOS's scanner support. Film-specific options (film type,
  frame, autofocus, inversion) are set on a small settings page.
* **`libdsdual4`**: a C library (`include/dsdual4.h`) for your own front ends.

No kernel extension is needed. The scanner is a vendor-specific USB device, so
a normal user process can open it (Apple silicon and Intel; macOS 11 or later).

## Build

```sh
xcode-select --install        # if needed
make                          # universal arm64 + x86_64 binary in build/
make test                     # end-to-end tests against the built-in simulator
sudo make install FIRMWARE="../DS_Dual4_extracted/DS Dual4/Firmware/DS_Dual4.BIN"
```

## Interactive menu

Run `dsdual4` with no arguments for a guided menu:

```
 Scanner    KONICA MINOLTA FS-V31, firmware 1.02
 Calibrated yes
 Holder     35 mm film strip, 6 frames (at frame 1)
 Settings   colour slide (positive), 3200 dpi, 16-bit linear, focus auto
  i  Initialize (warm up lamp and calibrate)
  p  Preview a frame (800 dpi, opens in Preview)
  s  Scan a frame
  ...
```

It waits for the scanner to be connected, shows its status, and greys out
options until they can be used (scanning needs Initialize and a detected
holder). Previews are saved display-ready and opened in Preview. Scans go to
`~/Pictures/dsdual4`. `h` prints step-by-step instructions, and Settings
changes the film type, resolution, focus, multi-sampling, flip and output
folder.

## Using the command line tool

```sh
dsdual4 info                               # identification, holder, frames
dsdual4 init                               # lamp warm-up + calibration (~1-2 min)
dsdual4 preview -o preview.tiff            # 800 dpi preview of the current frame
dsdual4 scan --frame 3 --dpi 3200 -o slide3.tiff
dsdual4 scan --frame 2 --film nega --invert -o neg2.tiff
dsdual4 scan --dpi 1600 --area 200,300,1000,800 --samples 4 -o crop.tiff
dsdual4 scan-all --dpi 3200 -o roll        # roll-1.tiff ... roll-6.tiff
dsdual4 autofocus --frame 1                # prints the best focus position
dsdual4 scan --focus 37 -o fixed.tiff      # manual focus (−200 … 200)
dsdual4 eject
```

Scan options: `--film posi|nega|bw-posi|bw-nega`, `--dpi 50..3200`,
`--bits 8|16`, `--exposure r,g,b` (factors) or `--ev ±stops`,
`--focus auto|keep|N`, `--samples 1|2|4|8` (multi-sample noise reduction),
`--invert`, `--raw` (no host digital gain), `--flip-x`, `--flip-y`,
`--rotate 90|180|270`. Add `-v` for progress details, `-vvv` for a protocol
trace, `-vvvv` for hex dumps.

Each command opens the scanner and initialises it if needed. The first run
after power-on warms up the lamp and calibrates, and the shading tables are
cached in `~/Library/Application Support/dsdual4/`.

## Using it from Image Capture / Preview

```sh
dsdual4 serve --local          # keep running; Ctrl-C to stop
```

Then open **Image Capture** (or **Preview → File → Import from Scanner**) and
pick *DiMAGE Scan Dual IV*. The scan area is one 35 mm frame (0.97 × 1.46 in).
Open <http://localhost:8090/> to choose the film type, the frame, autofocus,
exposure correction, multi-sampling and negative inversion, or to eject the
holder.

To have it start automatically at login and whenever the scanner is plugged in:

```sh
make install-agent             # installs ~/Library/LaunchAgents/org.dsdual4.airscan.plist
make uninstall-agent
```

`--local` advertises the scanner only on this Mac. Without it, other Macs and
iPads on the network can use it too.

## Images and colour

Scans are the scanner's linear 16-bit data after the original driver's
shading correction, exposure model and digital gain. The scanner records
**device RGB**. Konica Minolta characterised it with three ICC profiles that
ship with the original software (`DS Dual4/Profile/`):

| profile | curve | use |
|---|---|---|
| `MLTF3200p.icc` "Posi Linear" | linear | linear slide scans (what dsdual4 produces) |
| `MLTF3200.icc` | gamma ≈ 2.1 | gamma-encoded slide output of the original |
| `MLTF3200n.icc` "Nega Input" | gamma ≈ 2.6 | negatives after the original's own inversion |

dsdual4 **embeds `MLTF3200p.icc` in colour slide scans** (16-bit, or 8-bit
linear), so Photoshop, Affinity, darktable, Preview etc. interpret the colours
correctly and you can convert to your working space. The profile is read from
your copy of the original software, `--profile`, `$DSD4_PROFILE` or the
install prefix (`make install PROFILE=…`); `--no-profile` turns it off. Its
gamut is wider than sRGB. Negatives, B&W and the display-ready 8-bit
previews are not tagged with it, because the profile does not describe them.

The AirScan bridge colour-manages slides through the same profile and
delivers sRGB JPEG/PDF (or profile-tagged PNG/TIFF) to Image Capture and
Preview. Other film types are made display-ready.

Negatives: the scanner never inverts. Three ways to get a positive:

* **Minolta processing** (`--minolta`, the menu's "Negatives → Minolta
  processing", or the settings page of `serve`): the original software's own
  pipeline. Autofocus, an 800 dpi auto-exposure prescan analysed by the
  original AE algorithm, a final scan at the exposure it chooses, and the
  original film curves (inversion, base and orange-mask removal, B&W density
  curve). Both algorithms are ports of the original PowerPC code, bit-identical
  to it under emulation (`tests/diff/ae_diff.py`, `tests/diff/lut_diff.py`).
  Colour results embed `MLTF3200n.icc` ("Nega Input"), the profile the
  original used for exactly this data.
* `dsdual4 convert raw.tiff --film bw-nega|nega -o out.tiff` applies the same
  processing offline to a raw negative you already scanned (pass
  `--scan-exposure` if it was not scanned at the film's standard exposure;
  a scan made as "slide" used 1.4286). The exposure step is approximated by
  scaling the existing data instead of rescanning.
* `--invert`: a quick per-channel inversion. Or keep the raw 16-bit negative
  and use a dedicated tool (Negative Lab Pro, darktable negadoctor, ColorPerfect).

The original software's image processing (Pixel Polish, Auto Dust Brush,
Grain Dissolver) is not included. The Dual IV has no infrared channel, so no ICE.

Orientation: lines are delivered in the scanner's order. If your scans come
out mirrored, use `--flip-x`.

## Verification

There was no scanner available while writing this. Three independent checks
were used instead:

1. **Differential test against the original binary** (`make difftest`): the
   original PowerPC `MFSBaseLib2891` code runs under the Unicorn emulator (as a
   PowerPC G4) with its Mac OS Toolbox imports stubbed. Its SCSI commands go to
   the same protocol simulator the port uses, and a scenario covering the
   following is executed on both:
   * initialisation (reset, lamp warm-up, calibration)
   * 3200, 1600, 800, 400 and 200 dpi colour and gray scans, 8-bit and 16-bit,
     crops and APS offsets
   * multi-sampling
   * dark recalibration for negatives
   * autofocus multi-scan sessions
   * holder control and range errors
   * the firmware update

   **All 327 scanner commands and 108 image reads are identical.**
   ```sh
   pip3 install unicorn
   ORIGINAL_LIB=../DS_Dual4_extracted/MFSBaseLib2891 \
   DSD4_FIRMWARE="../DS_Dual4_extracted/DS Dual4/Firmware/DS_Dual4.BIN" make difftest
   ```
2. **Protocol simulator** (`make test`, `dsdual4 --sim …`): a model of the
   scanner, using the real INQUIRY block from the firmware image, that checks
   everything the host sends and returns a focus-dependent test chart. It runs
   15 end-to-end tests, including autofocus convergence.
3. **macOS integration**: `tests/ica/ica_probe.swift` uses Apple's
   ImageCaptureCore (the framework behind Image Capture and Preview) to
   discover the AirScan bridge and complete a scan.

What is **not** verified yet is behaviour on real hardware (timing, the exact
meaning of a few device-reported values, image orientation). Bug reports with
`dsdual4 -vvv …` traces are the fastest way to fix such issues.

## Firmware

Firmware updates are **off by default**. `dsdual4` never writes to the
scanner's flash unless you pass `--firmware-update`. In that case, like the
original driver, it asks the scanner whether it wants an image (READ 0x90)
and sends `DS_Dual4.BIN` only if the scanner requests it and the file is not
older than the running firmware. This path is byte-identical to the original
but has not been tried on hardware. Scanning works on the installed firmware.

## How it is organised

| file | original counterpart |
|---|---|
| `src/usb_macos.c` | `MFSIOUsb2891.bundle` (IOKit USB transport) |
| `src/avision.c` | `AVIOScanCommand`, `Cmd*` (SCSI command layer, sense mapping) |
| `src/avlib.c` | the "AVI" library in `MFSBaseLib2891`: ability, calibration, SET WINDOW, shading upload, acceleration tables, data unpacking, firmware |
| `src/calib.c` | `GetAverageData`, `BubbleSort`, `Calculate{Dark,White}ShadingData`, `DarkSmoothProcess` |
| `src/scanner.c` | `CBaseScannerAVI`, `CBaseMethod2889` and `MFSLib` exposure/focus logic (`GetStdGain`, `CheckGainLimit`, `AutoFocus`, …) |
| `src/escl.c` | new: AirScan/eSCL server for Image Capture |
| `src/sim.c` | new: protocol-level scanner simulator |
| `docs/PROTOCOL.md` | the recovered USB protocol |
| `docs/re-tools/` | the reverse-engineering tools (PEF loader, Ghidra scripts) |
| `tests/diff/` | differential test harness (original PowerPC code under emulation) |

## Limitations

* Index (thumbnail) scans, the original's automatic exposure, and its image
  processing filters are not implemented. Exposure uses the original's standard
  film factors plus your `--exposure`/`--ev` correction.
* Final scans run at 800, 1600 or 3200 dpi on the scanner, as in the
  original. Other resolutions are resampled on the Mac with an area filter
  (the original used its own resampler).
* APS (IX240) adapter support follows the original code but is untested.

## Legal

This is an independent, interoperability-driven reimplementation. It contains
no code or firmware from Konica Minolta. "DiMAGE" and "Konica Minolta" are
trademarks of their owners.
