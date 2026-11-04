# Open Test Drive Unlimited for AARCH64 (based on OpenTDU)
Original repo: https://github.com/opentestdriveunlimited/OpenTestDriveUnlimited

**WORK IN PROGRESS**

The goal of this project is to bring TDU to handheld consoles running rocknix as a port.

**This project is not a replacement for the game (you still need to own a legal copy); and does not provide any game assets!**

<img src="/screenshots/carshowcase_exige.jpg" alt="GMCarShowcase" width="256" height="144"> <img src="/screenshots/worldviewer_3.jpg" alt="GMSandbox" width="256" height="144"> <img src="/screenshots/freeride.jpg" alt="GMFreeride" width="256" height="144"> <img src="/screenshots/login.jpg" alt="GMLogin" width="256" height="144"> <img src="/screenshots/cinviewer.jpg" alt="GMCinViewer" width="256" height="144"> <img src="/screenshots/boattour.jpg" alt="GMBoatTour" width="256" height="144"> <img src="/screenshots/living.jpg" alt="GMLiving" width="256" height="144"> <img src="/screenshots/garage.jpg" alt="GMGarage" width="256" height="144"> <img src="/screenshots/cockpit.jpg" alt="GMFreeride (cockpit)" width="256" height="144"> 

Updates of original project (updates/screenshots) can be found in this [thread](https://github.com/opentestdriveunlimited/OpenTestDriveUnlimited/discussions/175).



## Installation
1. Copy `ports/OpenTDU.sh` and the `ports/tdu/` folder from this package into `/storage/roms/ports/`.
2. Copy your own *Test Drive Unlimited* PC game files into `/storage/roms/ports/tdu/` so that
   `TestDriveUnlimited.exe` and the original `Euro` folder end up there next to `opentdu_unix_vulkan`.
   The launcher merges `Euro` into `EURO` (and `Bnk` into `BNK`) on first start.
3. Start **OpenTDU** from the Ports menu (refresh the game list first), or over SSH:
   `/storage/roms/ports/OpenTDU.sh`
4. The first start extracts shaders and configuration files from your `TestDriveUnlimited.exe`. This can take a while.
5. Plug in a USB keyboard once and create a player profile and enter activation code.

## Command Line Arguments
| Command | Description |
| ----------- |  ----------- |
| -w | Enable windowed mode |
| -vbl (value) | Set target vsync blank |
| -fps | Enable FPS counter |
| -pos | Enable debug position text overlay |
| -orb | Override ORB address for online login |
| -nopub | Disable GSPub |
| -gm (id) | Launch game with a specific game mode (see eGameMode) |
| -worldviewer | Launch world viewer |
| -carshowcase | Launch car showcase (alias for '-gm 7') |
| -cinviewer | Launch cinematic viewer (alias for '-gm 20') |
| -freeride | Launch cinematic viewer (alias for '-gm 1') |
| -login | Launch login screen (alias for '-gm 24') |
| -vk_enable_validation_layer | Create Vulkan render device with validation layer enabled |
| -break_on_assert_failure | Trigger debug breakpoint on assert failure |
| -disable_assert | Disable assert log/debug break |
| -norunlock | Disable single game instance run lock |
| -profile (name) | Launch game with a specific profile (must match savegame name) |
| -car (id) | Override the local player car in world game modes (e.g. freeride) with a car database ID |
| -silent | Disable logging (stdout, outpudebug and file) | 
| -log (path) | Redirect game logs to a specific file (default: ./opentdu.log) |
| -nolog | Disable logging to file (note: does not disable stdout/outputdebug) |
| -logtransient | Write transient log to file |
| -verbose | Enable verbose (aka Debug) logging. Adds significant runtime overhead |
| -maxfps (fps) | Set framecap (0: unlimited) (default: 0) |

## Debug Shortcuts
- F1 - Engine debug menu (services + managers)
- F2 - Pause simulation
- F3 - Active game mode debug menu

## Status

- Runs in FreeRide (`-gm 1`) at roughly 13-18 fps with the render resolution reduced to about 640 px on the long
  edge (see *Tweaks*). The Adreno 610 is the bottleneck, so the launcher disables several costly effects by default.
- Controls: gamepad is mapped to the keyboard through an InputPlumber profile that is loaded when the game starts and
  restored when it exits.
- Launching from the ROCKNIX *Ports* menu: **TODO: confirm**.
- Only tested on the Mangmi Air X. Other ROCKNIX devices may need `OUTPUT` / `SCALE` adjustments.

## Requirements

- ROCKNIX (Sway / Wayland, InputPlumber), aarch64
- Vulkan 1.3 capable driver (Mesa Turnip on the Air X)
- glibc 2.41 or newer (the binary is built on Debian 13 "trixie")
- A USB keyboard for the very first start (to type activation code)

## Controls

| Console | Key sent to the game | Function |
|---|---|---|
| Left stick | Steering, menu navigation |
| Right trigger | Up | Accelerate |
| Left trigger | Down | Brake / reverse |
| A | Validate, headlights |
| B | Back |
| X | Handbrake |
| Y | Change camera |
| Right bumper | Look back |
| Left bumper | Horn |
| L3 , Up Dpad | Driving help |
| R3 | Map |
| Start | Pause / quit |
| Select | Menu sort / insert |

The profile is `tdu/opentdu.yaml` (InputPlumber device profile). Edit it to change the layout.

## Tweaks

The launcher sets these by default; override them on the command line, e.g. `SCALE=2 NO_BOATS=0 ./OpenTDU.sh -gm 1 -fps`.

| Variable | Default | Effect |
|---|---|---|
| `OUTPUT` | first sway output | Sway output used for scaling |
| `SCALE` | auto (long edge ~640 px) | Integer Sway output scale while the game runs; the game renders at the reduced size and Sway upscales |
| `MAXFPS` | `13` | Frame cap (`-maxfps`), `0` = unlimited |
| `OTDU_PHYSICS_THREADS` | `1` | Number of physics worker threads |
| `OTDU_QUALITY` | `low` | `low` forces LOD 0 and disables AA and HDR |
| `OTDU_PARTICLE_QUALITY` | `low` | Fewer particle instances, lower coverage budget, higher alpha reject |
| `NO_AIRCRAFT`, `NO_BOATS` | `1` | `1` removes aircraft / boats |
| `NO_PAPERS`, `NO_BIRDS` | `1` | `1` removes flying papers and leaves / birds |
| `NO_PARTICLES` | `1` | `1` disables the world particle detail (leaves and ambient particles) |
| `NO_SUNFLARE` | `1` | `1` disables the sun flare |
| `NO_MOTIONBLUR` | `1` | `1` disables radial (motion) blur |

Any other arguments are passed to the game (`-gm 1` starts FreeRide, `-fps` shows the frame rate).

## Troubleshooting

- **Log:** `tdu/opentdu-run.log` is rewritten on every start.
- **Gamepad still acts like a keyboard after a crash:** restore the default InputPlumber profile:

  ```
  busctl call org.shadowblip.InputPlumber /org/shadowblip/InputPlumber/CompositeDevice0 \
      org.shadowblip.Input.CompositeDevice LoadProfilePath s /usr/share/inputplumber/profiles/default.yaml
  ```

- **Screen stays scaled after a crash:** `swaymsg output <name> scale 1` (find the name with `swaymsg -t get_outputs`).
- **Game window not visible (EmulationStation on top):** the launcher adds a Sway rule that makes the game window
  fullscreen and focused; if it is missing, check that `swaymsg` works for your session.

## Building from source

The port is cross-compiled from Debian 13 (x86_64) for aarch64. The source changes are in `patches/` and apply to
OpenTestDriveUnlimited commit `573d3d59`.

1. Packages:

   ```
   sudo dpkg --add-architecture arm64 && sudo apt update
   sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu cmake pkg-config glslang-tools \
     autoconf automake libtool \
     libwayland-dev:arm64 libxkbcommon-dev:arm64 libdrm-dev:arm64 libgbm-dev:arm64 libasound2-dev:arm64 \
     libudev-dev:arm64 libx11-dev:arm64 libxext-dev:arm64 libegl-dev:arm64 libvulkan-dev:arm64 \
     libxcursor-dev:arm64 libxi-dev:arm64 libxfixes-dev:arm64 libxrandr-dev:arm64 libxss-dev:arm64 \
     libxtst-dev:arm64 libxinerama-dev:arm64 libdbus-1-dev:arm64 libdecor-0-dev:arm64 \
     libpulse-dev:arm64 libpipewire-0.3-dev:arm64
   ```

2. CMake toolchain file (`aarch64.cmake`):

   ```
   set(CMAKE_SYSTEM_NAME Linux)
   set(CMAKE_SYSTEM_PROCESSOR aarch64)
   set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
   set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
   set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
   set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu /usr)
   set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
   set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
   set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
   set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
   ```

3. Environment, submodules (the `stb` submodule uses an SSH URL, rewrite it to HTTPS) and third-party libraries:

   ```
   export CMAKE_TOOLCHAIN_FILE=$HOME/aarch64.cmake
   export PKG_CONFIG_LIBDIR=/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/share/pkgconfig
   export PKG_CONFIG_PATH=
   git config --global url."https://github.com/".insteadOf "git@github.com:"
   git submodule update --init --recursive
   # cityhash: refresh config.sub/config.guess (or run autoreconf -fi) and configure with
   # --host=aarch64-linux-gnu in third_party/build_thirdparty.sh, then:
   (cd third_party && ./build_thirdparty.sh)
   ```

4. Apply the patches and build (`-fsigned-char` is required: `char` is unsigned on aarch64 Linux and the bank
   file name decoding depends on it):

   ```
   git apply patches/*.patch
   cmake -S . -B build-arm64 -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_C_FLAGS=-fsigned-char -DCMAKE_CXX_FLAGS=-fsigned-char
   cmake --build build-arm64 --parallel $(nproc) --target opentdu
   aarch64-linux-gnu-strip -o opentdu_unix_vulkan build/bin/opentdu_unix_vulkan
   ```

   Build only the `opentdu` target: the other targets try to run aarch64 tools on the x86_64 host.

Summary of the source changes:

- Vulkan swapchain on Wayland: handle the undefined `currentExtent` (use the window size) and respect `minImageCount`.
- Crash fix in `BankCommonFlash::initialize` (unchecked `strrchr` result).
- Optional performance switches controlled by environment variables (`OTDU_*`, see *Tweaks*).

## Credits and legal

- [OpenTestDriveUnlimited](https://github.com/opentestdriveunlimited/OpenTestDriveUnlimited) by its authors; see that repository for its license.
- *Test Drive Unlimited* is a trademark of its respective owners. This package contains no game assets and
  requires your own legally obtained copy of the PC game.
