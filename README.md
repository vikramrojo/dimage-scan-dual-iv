# dsd4: DiMAGE Scan Dual IV driver for modern macOS

A native driver for the **Konica Minolta DiMAGE Scan Dual IV** film scanner
(USB `132b:000a`). The official software only runs on PowerPC Macs. This
driver runs on Apple silicon and Intel Macs with macOS 11 or later, entirely
in user space. There is no kernel extension.

* **Image Capture and Preview**: the scanner appears there while it is
  plugged in.
* **`dsd4`**: an interactive Terminal menu, plus commands for scripting.
* **`libdsd4`**: a C library (`include/dsd4.h`).

## Installation

### Simple

1. Download `dsd4-<version>.pkg` from
   [Releases](https://github.com/vikramrojo/minolta-scan-dual-iv/releases/latest).
2. Open it and follow the installer.
3. Connect and power on the scanner. Insert a film holder and close the front
   door.
4. Open **Image Capture**, or **Preview → File → Import from Scanner**, and
   choose **DiMAGE Scan Dual IV**. Set the film type and negative handling at
   <http://localhost:8090/>.

For more control, type `dsd4` in Terminal to open the menu.

* **Stop or start the background service:** `dsd4 service stop|start|status`.
* **Uninstall:** `sudo /usr/local/share/dsd4/uninstall.sh`.

### Build from source

```sh
xcode-select --install                 # once
make && make test                      # universal binary in build/
sudo make install                      # /usr/local/{bin,lib,include,share/dsd4}
make install-agent                     # optional: Image Capture service for this user
make pkg                               # optional: build/dsd4-<version>.pkg
```

The version number is `1.1.<commit count>`; `dsd4 --version` shows it.

To make a signed release, set `SIGN_APP`, `SIGN_INSTALLER` (your Developer ID
certificate names) and `NOTARY_PROFILE` (from `xcrun notarytool
store-credentials`), then run `make release`. It builds, signs, notarises,
tags and publishes the package to GitHub Releases.

## Usage

Running `dsd4` with no arguments opens a guided menu:

* Options stay greyed out until they can be used.
* `h` shows instructions.
* Scan settings (film, size, resolution, negatives) and device settings
  (focus, preview, multi-sampling, output folder) are remembered between runs.

Commands:

```sh
dsd4 init                                 # warm up lamp + calibrate (first time ~1-2 min)
dsd4 preview -o p.tiff                    # quick preview of the current frame
dsd4 scan --frame 3 --dpi 3200 -o s.tiff  # 16-bit TIFF
dsd4 scan --film nega --minolta -o n.tiff # negative, original Minolta processing
dsd4 scan --format dng --samples 8 -o r.dng   # raw DNG, 8x multi-sampled
dsd4 scan --format dng --hdr -o h.dng     # raw DNG, two exposures merged
dsd4 scan --film nega --minolta --format dng -o m.dng  # Minolta positive as a DNG
dsd4 scan --film nega --invert --format dng -o p.dng   # simple-invert positive DNG
dsd4 scan-all -o roll                     # roll-1.tiff ... roll-6.tiff
dsd4 scan-all --frames 4 -o roll          # a strip of 4: roll-1 ... roll-4
dsd4 scan --experimental 4000dpi,denoise-low -o s.tiff  # experimental, see below
dsd4 convert raw.tiff --film bw-nega -o out.tiff   # process an existing raw negative
dsd4 eject | home | reset | autofocus | info
```

* **Options:** `--film posi|nega|bw-posi|bw-nega`, `--dpi 50..3200`,
  `--bits 8|16`, `--ev ±stops`, `--focus auto|keep|N`, `--samples 1..8`,
  `--invert`, `--minolta`, `--format tiff|dng`, `--hdr[=ev]`,
  `--af fast|classic`, `--flip-x`, `--rotate`.
* **Diagnostics:** `-v` to `-vvvv` for progress, a protocol trace or hex dumps.
* **`dsd4 --help`** lists everything.

**If you see "holder position error":** close the front door and push the
holder fully in, then run `dsd4 reset`.

## Colour and negatives

Scans are linear 16-bit device RGB. Colour slide scans embed Konica Minolta's
`MLTF3200p.icc` profile, so colour-managed apps show them correctly.

The scanner itself never inverts negatives. There are three ways to handle
them:

* **Minolta processing** (the default in the menu and Image Capture): the
  original software's pipeline. It runs autofocus, an auto-exposure prescan,
  a rescan at the chosen exposure, and the original film curves. Colour
  results embed `MLTF3200n.icc`.
* **Simple invert:** works in density, like the film itself. It removes the
  orange mask and balances the channels' contrast.
* **Raw:** keeps the negative as scanned, for Negative Lab Pro, darktable and
  similar tools.

Previews can use Minolta processing too. It needs only the one 800 dpi preview
scan, and a full scan of that frame afterwards reuses its exposure.

## Raw DNG, multi-sampling and HDR

`--format dng`, or **Output → DNG raw** in the menu's scan settings, saves the
scanner's linear data as a DNG. Lightroom, darktable, Negative Lab Pro and
macOS all open it.

* **Colour:** Konica Minolta's own calibration (the `MLTF3200p.icc` matrix
  and per-channel curves), so raw converters show the same colour as the
  profiled TIFF.
* **Multi-sampling** (`--samples 2/4/8`): about √N less noise. Single bad
  reads are discarded. The averaged result is kept as a 16-bit float DNG,
  rather than rounded.
* **HDR** (`--hdr`): a second pass 2 stops brighter, aligned to the first
  to within a fraction of a line and merged. This gives about 4× less noise
  in the dense areas: slide shadows, and negative highlights.
* **Negatives** stay negative, for Negative Lab Pro and other raw converters
  that invert them, unless you ask for a positive DNG:
  * **Minolta processing** (`--minolta`, or **Negatives → Minolta
    processing**): the Minolta curves as a linear DNG. In Preview and Photos
    it looks the same as the Minolta TIFF, to within about 0.5 ΔE.
  * **Simple invert** (`--invert`): the simple invert above, as linear sRGB.

  Positive DNGs are tagged to show as they are: no extra contrast curve or
  black clip from Apple's raw engine or Lightroom. Multi-sampling keeps their
  float precision, and HDR works with both.

These don't add resolution. The Dual IV's sensor and line steps are fixed,
so every pass samples the same points. Multi-sampling and HDR improve the
cleanliness and precision of each pixel. Raw DNGs are made from the menu or
the command line; Image Capture can't receive them.

**Autofocus** runs a fast search by default: about a third of the original's
lens moves, with a peak fit. It also tries other parts of the frame when the
centre is featureless. `--af classic` (or the device setting) runs the
original sequence.

Not included: the original software's Digital ICE, Pixel Polish and Grain
Dissolver, and index scans.

## Experimental: 4000 dpi sharpness

After Norman Koren's film-scanner MTF studies: a scanner's own optics and
sensor cost a 3200 dpi scan a good part of its sharpness, and compensating
it gains more than anything else, provided the grain is not boosted with it.

1. **Calibrate (optional).** Without it, the experimental options use a
   typical Dual IV, measured on one unit, and sharpen at 70 % strength so as
   not to over-correct another unit's optics. Calibrating gives the full,
   exact correction for your own scanner:
   * `dsd4 research mtf`: tape a razor blade into a slide mount, its edge
     turned 2–20° from the mount's sides, and insert it as a slide. It finds
     the blade and its best focus and measures the MTF along the carriage
     (ISO 12233 slanted edge), single and multi-sampled.
   * `dsd4 research grain`: a film frame with flat areas (sky, blank
     film). Grain is the same in every direction, so its spectrum across
     against along gives the MTF across, which a blade in a mount can't show.
   * `dsd4 research noise`: blank film or an empty mount. The sensor's
     noise is measured from repeated reads of each pixel.
   * `dsd4 mtf scan.dng` measures any scan of the blade, for comparing.
2. **Scan with `--experimental`** (or Device & output settings → e):
   * `sharpen` undoes the measured blur, on brightness only. It is boosted
     only where the picture is stronger than the sensor's noise, mostly at
     edges, and never past Nyquist or into halos.
   * `denoise-low` removes sensor and colour noise and keeps the grain;
     `denoise-high` also reduces the grain. The grain is profiled from the
     flattest parts of each scan, like Neat Image's auto-profile.
   * `4000dpi` sharpens a 3200 dpi scan and resamples it to 4000 dpi.

It works on the raw scan, before inversion or film curves, so it applies to
TIFF and DNG output alike; DNGs stay raw. In the simulator, a blade's MTF50
goes from 35.6 to 53.9 lp/mm on film. Your scanner's numbers will differ, so
measure them.

## How it was made and verified

The driver was reverse engineered from the original PowerPC software, using
Ghidra and the PEF/Mach-O tools in `docs/re-tools/`. The recovered USB
protocol is documented in `docs/PROTOCOL.md`.

* **`make test`:** 44 end-to-end tests against a built-in scanner simulator
  (`dsd4 --sim`).
* **`make difftest`:** runs the original PowerPC code in the Unicorn emulator,
  side by side with the port.
  * All 327 scanner commands and 108 image reads are identical.
  * The auto-exposure and film-curve code produce bit-identical output
    (`tests/diff/`).
* **Real hardware:** initialisation, preview and scanning have been tested on
  a real Dual IV.
* **Firmware:** never written. `--firmware-update` is opt-in and untested.

## Licence and provenance

* **MIT** ([LICENSE](LICENSE)) covers this project's own code. The USB
  transport, command layer, calibration and scanning logic are
  reimplementations based on the original's behaviour.
* **Konica Minolta's code:** `src/minolta_ae.c` and `src/minolta_lut.c` are
  direct translations of Konica Minolta's auto-exposure and film-curve
  routines.
* **Konica Minolta's profiles:** `profiles/*.icc` are their colour profiles,
  redistributed unmodified ([profiles/NOTICE](profiles/NOTICE)).
* **Not MIT:** neither the translated routines nor the profiles are covered
  by the MIT licence.
* **Firmware:** no firmware is included.
* **Trademarks:** "DiMAGE" and "Konica Minolta" are trademarks of their
  owners.
