# espSynth — ESP32-S3 AMOLED Watch Synthesizer

Building a touchscreen synthesizer on the **Waveshare ESP32-S3-Touch-AMOLED-1.75** smartwatch
(466×466 AMOLED, capacitive touch, dual mic array, built-in speaker). The end goal is an
ocarina-style instrument and a mic→speaker resonator, built up from the stock spectrum-analyzer demo.

---

## What you need up front

| Requirement | Detail |
|---|---|
| **Disk space** | **~15 GB free** recommended before starting. A v5.5 ESP-IDF install is ~3–3.5 GB (esp32s3-only); builds add ~1–2 GB each. Don't start below ~10 GB free. |
| **ESP-IDF version** | **v5.5.x** (we used 5.5.4). **Do NOT use v6.x** — the demos target 5.5 and won't compile on 6.0. |
| **Toolchain** | Xtensa GCC (auto-installed by ESP-IDF). RISC-V toolchain is **not** needed — skip it to save ~2.7 GB. |
| **OS** | macOS (Apple Silicon here). Linux/Windows steps are similar. |
| **Board** | ESP32-S3, **16 MB flash**, 8 MB octal PSRAM, USB-Serial/JTAG over USB-C. |

---

## One-time setup

### 1. Install ESP-IDF v5.5 (Xtensa / esp32s3 only)
```bash
mkdir -p ~/esp
git clone -b release/v5.5 --depth 1 --recursive --shallow-submodules \
  https://github.com/espressif/esp-idf.git ~/esp/esp-idf-v5.5
cd ~/esp/esp-idf-v5.5
./install.sh esp32s3        # esp32s3 only = Xtensa toolchain, no RISC-V
```
> Tip: the standalone **EIM** installer (`/Applications/eim.app`) or the **VSCode ESP-IDF extension**
> can do the same — just pick **only ESP32-S3** so it doesn't pull the 2.7 GB RISC-V toolchain.

### 2. Activate the environment (every new terminal)
```bash
. ~/esp/esp-idf-v5.5/export.sh
```

### 3. Clone this board's examples
```bash
git clone https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.75.git
```
Examples live in `examples/ESP-IDF-v5.5/`:
- `01_AXP2101` – power management chip
- `02_lvgl_demo_v9` – LVGL graphics demo
- `03_esp-brookesia` – **the factory launcher UI** (music + image players)
- `04_Immersive_block` – animation demo
- `05_Spec_Analyzer` – **mic → FFT → spectrum display** (our synth starting point)

---

## Build & flash any example
```bash
. ~/esp/esp-idf-v5.5/export.sh
cd examples/ESP-IDF-v5.5/05_Spec_Analyzer    # or whichever example
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem101 flash          # adjust port to your board
```
Find your port with `ls /dev/cu.usbmodem*`. First build is slow (~10–20 min: it downloads
LVGL/esp-dsp/BSP). Later builds are fast.

---

## Gotchas we hit (so you don't)

- **"Port is busy / Resource busy"** — the VSCode ESP-IDF **serial monitor** holds the port open.
  Close it in VSCode, or free it from a terminal:
  ```bash
  P=$(lsof -t /dev/cu.usbmodem101); [ -n "$P" ] && kill $P
  ```
- **Wrong flash size** — `05_Spec_Analyzer` ships with `CONFIG_ESPTOOLPY_FLASHSIZE_32MB`, but our
  board is **16 MB**. Set `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` in `sdkconfig.defaults`
  (its partitions total ~15.7 MB, so they still fit). Verify your chip with `esptool.py flash_id`.
- **`idf.py monitor` needs a real terminal (TTY)** — it fails when run from a script/non-interactive
  shell. Run it in your own terminal, or read the serial port with a small pyserial script.
- **Incomplete IDF install** — symptoms are `idf.py: command not found` after `export.sh`, or a
  missing Python venv. Re-run `./install.sh esp32s3`.

---

## Status

- ✅ ESP-IDF v5.5.4 toolchain installed (esp32s3 only)
- ✅ `03_esp-brookesia` factory launcher — built, flashed, runs on device
- 🔧 `05_Spec_Analyzer` — building, to become the synth base
- ⏳ Next: audio **out** via `bsp_extra_i2s_write()` → ocarina + resonator pages

### Audio API (for the synth)
The `bsp_extra` component (`examples/ESP-IDF-v5.5/05_Spec_Analyzer/components/bsp_extra`) provides:
- `bsp_extra_codec_init()`, `bsp_extra_codec_set_fs(rate, bits, ch)`
- `bsp_extra_i2s_read(...)` — mic capture
- `bsp_extra_i2s_write(...)` — **speaker output** (the core of tone/synth playback)
- volume / mute / file-player helpers
