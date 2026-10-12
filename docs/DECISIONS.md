# libfpv design decisions

This is the log of decisions not fixed by the original brief. Each entry
gives the decision, why it was made and what it would take to reverse it.
Newest entries go at the bottom of their section.

## Dependencies and repository

### D1. Flattened HTTPS submodules, SimITL pinned, patches as a series

SimITL's own submodules use `git@github.com:` URLs and nest Betaflight,
libwebsockets and fmt inside SimITL. libfpv adds each of them as a top-level
submodule under `third_party/` with an HTTPS URL, pinned to exactly the commit
SimITL `c881d55` (2026-09-07) references:

| Submodule | Commit / tag |
|---|---|
| `third_party/SimITL` | `c881d55` |
| `third_party/betaflight` (AJ92 fork, 2025.12.0 pre-release) | `7bd98d8` |
| `third_party/libwebsockets` | `f69835e` |
| `third_party/fmt` | `9869971` |
| `third_party/SDL` | `release-3.4.18` |
| `third_party/imgui` | `v1.92.9` |
| `third_party/nlohmann/json.hpp` (vendored single header) | `v3.12.0` |

libfpv does not use SimITL's CMake files; `cmake/SimITLCore.cmake`
reproduces SimITL's Betaflight source list against the flattened paths. It
builds only a static, position-independent `simitl_core` library with hidden
symbols.

Changes to SimITL and Betaflight are kept as a patch series in
`patches/simitl/` and `patches/betaflight/`. CMake applies them at configure
time, and a patch that already reverse-applies is skipped. Patched
submodules are marked `ignore = dirty`. We use a patch series rather than a
GitHub fork so the changes stay reviewable in this repository and can be
offered upstream one at a time.

Betaflight upstream is at 2026.6.2 while SimITL's fork tracks 2025.12.0. The
brief fixes SimITL as the flight core, so libfpv stays on SimITL's fork.
Moving to a newer Betaflight means rebasing that fork, which is out of scope
for v1.

### D2. SimITL patches

* `0001-lifecycle-logging-failsafe.patch`
  * Root cause of "stopping corrupts memory": `wsStop()` destroyed the
    libwebsockets contexts while the WebSocket thread was still inside
    `lws_service()`. The new order is: signal, `lws_cancel_service()`,
    join, then destroy. Contexts can be re-created (`wsRestart`), so the
    Configurator service can be stopped and restarted without
    re-initialising Betaflight.
  * Root cause of "game exit crashes": the `Sim` singleton owns a joinable
    `std::thread` and its destructor neither joined nor detached it, so
    process exit ran `std::terminate` (reproduced: "terminate called without
    an active exception"). The destructor now detaches. It never joins,
    because on Windows the thread is already killed by `ExitProcess` and
    joining under the loader lock can deadlock.
  * The WebSocket base port is now configurable (`wsSetBasePort`; UART1 is
    base + 1, so the default stays at 5761). `lws_service` timeout was
    lowered from 1000 ms to 50 ms so shutdown is prompt.
  * Failsafe: `rxRcFrameStatus` can report `RX_FRAME_PENDING`, which makes
    Betaflight's own RX-loss and failsafe logic run when the radio
    disappears.
  * SimITL's `fmt::print` and Betaflight's `printf`/`fprintf` are routed to
    libfpv's log callback. Betaflight C files are compiled with a
    force-included header, `src/core/bf_log_redirect.h`. A game's stdout is
    never written to.
* `0002-exact-contact-velocity.patch`: `StateInput::contact == 2` makes
  SimITL take the host's linear and angular velocity exactly. The existing
  value 1 blends 75 %/50 %. libfpv resolves collisions with impulses and
  needs the result applied exactly, otherwise the drone sinks into the
  ground or tunnels.

## Simulation

### D3. SimITL coordinate frame

Verified from `physics.cpp`, `bf.cpp` and the presets: X right, Y up,
Z forward, left-handed (Unity's convention), SI units, gravity −Y. Motor 1
(rear right in Betaflight's Quad X) is at (+x, −z), and motor 2 (front
right) is at (+x, +z). SimITL integrates velocities and orientation, but
**not position**. The host must feed the rotation back each update, because
`StateInput` overwrites it.

### D4. Betaflight configuration goes through Betaflight's own CLI, in-process

`src/core/bf_bridge.c` implements an in-memory `serialPort_t` and drives
`cliEnter`/`cliProcess` on the simulation thread. Preset defaults, overlay
edits and `diff all` imports are therefore just CLI commands. They go
through the same parameter-group code as the Configurator (which uses MSP
over the WebSocket), so there is one source of truth and no copy of any
Betaflight setting inside libfpv. `save` is always issued as
`save noreboot`. When the Configurator owns the CLI (`cliMode`), libfpv
reports "busy" instead of interleaving.

### D5. Default EEPROM settings

Measured: a fresh EEPROM does not arm. There is no ARM mode, there is a 5 s
power-on grace (`BOOTGRACE`), and gyro calibration (`CALIB`) takes about
4 s of sim time. libfpv's defaults set `pwr_on_arm_grace = 0`, PWM, and
`aux 0 0 0 1700 2100 0 0` (ARM on AUX1).

### D6. Betaflight patches

* `0001-cli-array-set-null-deref.patch`: `cliSet` parsed array settings
  with `valPtr = strchr(valPtr, ',') + 1`. When a diff gives fewer values
  than the array length (common in diffs from older firmware), this turns
  NULL into `(char *)1` and the next iteration reads address 1.
  Found by importing the 4.0.6 corpus diff under AddressSanitizer.

### D7. One EEPROM per quad preset

Each preset has its own Betaflight EEPROM at
`<data_dir>/eeprom/<preset>.eeprom`, like one flight controller per real
quad. Switching presets saves pending changes and then re-reads the other
file (`initEEPROM` → `ensureEEPROMStructureIsValid` → `readEEPROM`). A
missing file is first reset to Betaflight defaults, so it never inherits
the previous quad's memory image. Then libfpv's first-run script and the
preset's own tune (`<preset>.diff.txt` next to the JSON, if present) are
applied. `diff all` imports go to the active preset's EEPROM.

### D8. Props follow `yaw_motors_reversed`

Measured: with SimITL's preset motor directions (−1, 1, 1, −1) and
Betaflight's default "props in" mixer, the yaw loop has positive feedback.
The drone spins up to 18 rad/s and runaway-takeoff prevention disarms it.
SimITL's own tools work because pr0p's EEPROMs set
`yaw_motors_reversed = ON`. libfpv does not force that setting, because an
imported real "props in" diff would then spin the wrong way. Instead the
simulated prop directions follow Betaflight's `yaw_motors_reversed`, read
every 64 steps so Configurator edits apply live. It flies stably both ways
(tested).

### D9. Collision shape

The collision shape is a sphere at the centre of mass plus one under each
motor bell, with level bottoms. With only a body sphere the drone rested
on a single point and ground friction could not stop yaw spin (measured:
12.8 rad/s forever). Propeller discs are not collided with; they are tested
separately for prop strikes, which damage the props (`durability` from the
prop JSON). `FPV_MATERIAL_KILLS_PROPS` destroys them on contact.

### D10. Warm-up, so a fresh drone can arm immediately

Betaflight needs about 3.7 s of simulated time after boot for gyro, acc
and baro calibration, and RX is suspended briefly whenever an EEPROM is
read. `fpv_global_init`, EEPROM switches and the reboot after an import
step the parked simulation until `CALIBRATING`, `RXLOSS`, `FAILSAFE` and
`BOOTGRACE` clear. This costs about 40 ms of wall time.

### D11. Timing model: dedicated simulation thread at 1 kHz

Measured: one 1 ms SimITL step (8 Betaflight PID loops) costs about 10 µs
on a desktop CPU, so a 1 kHz thread uses about 1 % of a core. Collision
substeps keep every sphere moving less than half its radius, so a 30 m/s
drone takes about 4 substeps per step.

* `FPV_TIMING_THREAD` (default): a libfpv thread steps the simulation at
  `sim_rate_hz` and polls the radio at the same rate. Pacing uses a
  high-resolution waitable timer on Windows and `sleep_until` elsewhere. A
  fixed-step accumulator gives frame-rate independence and is capped at
  50 ms, so a host hitch cannot cause a spiral. Each step carries the wall
  time it represents. `fpv_drone_tick` renders one step (1 ms) in the past
  and interpolates (pos lerp, rot slerp), so there is no visible jitter at
  any host frame rate and no extrapolation error.
* `FPV_TIMING_MANUAL`: the host calls `fpv_step` itself. This is
  deterministic and used by the tests and lockstep engines.
* Auto-pause: when the host stops calling `fpv_drone_tick` for 0.5 s (a
  pause menu, loading screen or debugger), the simulation stops, unless the
  drone is disarmed on the ground. In that case Betaflight keeps running so
  the Configurator stays responsive.
* All Betaflight access (CLI, EEPROM, presets) is queued to the simulation
  thread. The public API is serialised by one recursive mutex.

The alternative, stepping inside the host's tick, would tie radio sampling
and PID timing to the game's frame rate (16 ms at 60 FPS) and break the
"radio polled at sim rate" requirement.

### D12. libfpv does not retune SimITL's aerodynamics

Measured with PropEater 5" (6S, 1900 kV): hover at 14 % throttle, and a
vertical punch-out reaches about 275 km/h after 2 s (thrust-to-weight
about 15:1 from the prop JSON). That is high but within what real 6S
racing quads do, and the model is SimITL's by design. Tests check
plausibility ranges, not exact values.

### D13. `diff all` import rules

* The import starts from `defaults nosave`, then applies the diff, clears
  all 20 mode slots, applies libfpv's base settings, re-adds the modes and
  saves without reboot. If Betaflight then reports
  `REBOOT_REQUIRED`, libfpv reboots it in-process (Betaflight's own
  `systemReset` → `init()`, the same path as the Configurator's
  "Save and Reboot") and warms it up again.
* Dropped as hardware-specific, each with its reason in the report:
  resources, timers, DMA, serial ports, board identity, LED strip,
  VTX/vtxtable, beacon, receiver settings, sensor hardware and
  calibration, OSD chip, battery/current sensor scaling, and `map` and
  `rxrange` (libfpv's radio wizard owns channel order and endpoints).
* `profile`/`rateprofile` indices this Betaflight does not have (newer
  firmware has 6 rate profiles, the embedded one has 4) are skipped
  together with every `set` that follows them, so those settings cannot
  land in the wrong profile.
* ARM is moved to AUX1 with range 1700–2100, the wizard's "switch on". The
  other mode channels are packed into AUX2–AUX4 in order of appearance. A
  fifth distinct switch channel is dropped and reported.
* Settings Betaflight rejects (renamed since older firmware) are listed
  under "Rejected by Betaflight". They are not an error.
* Corpus: six real diffs from Betaflight 4.0.6 to 4.5.1
  (`tests/diffs/README.md`). Every one imports, arms and flies in the test
  suite.

## Presentation

### D14. Overlay: libfpv is a "legacy" ImGui backend

Dear ImGui 1.92 supports dynamic font atlases (`RendererHasTextures`), but
that would push texture create/update/destroy requests onto every host
renderer. libfpv does not set the flag. It bakes ImGui's scalable default
font once at 32 px into one RGBA atlas (`FPV_TEXTURE_OVERLAY_FONT`) and
draws it at `16 px × ui_scale`. The host uploads at most two static
textures (overlay font, OSD font) and draws plain indexed triangles, so its
renderer snippet stays about 60 lines. The overlay draws its own mouse
cursor (`io.MouseDrawCursor`), because games usually hide the OS cursor.
No `imgui.ini` is written. ImGui is compiled inside libfpv with hidden
symbols, so it cannot clash with a game's own ImGui.

Overlay values for Betaflight settings are read and written with CLI
`get`/`set` on the simulation thread (`BfSettingsCache`) and refreshed
every second while the overlay is open. Configurator edits therefore show
up, and overlay edits show up in the Configurator after its next read.
Sliders write live and `save` on release.

### D15. OSD

Betaflight Configurator's MAX7456 fonts (`default.mcm` for the analog look,
`bold.mcm` for the HD look) are embedded and parsed at runtime into one
384×288 atlas. `fpv_osd_draw_data` returns quads in the same draw-data
format as the overlay, so a host needs only one renderer.
`fpv_osd_render_rgba` rasterises on the CPU for hosts that prefer a
texture. SimITL's OSD canvas is the 30×16 analog grid. "HD look" uses the
bold font in a 16:9 viewport; it does not add an HD (53×20) canvas.

### D16. Audio

`fpv_audio_read` is called from the host's audio thread. It never takes
the API lock: parameters (RPM, load from current, prop damage, pan from
motor position) are copied under a small mutex once per host frame and
glided over 15 ms. Each motor mixes a band-limited blade-pass saw
(harmonics above Nyquist dropped), the motor's electrical whine (7 pole
pairs), RPM-tracking low-passed noise scaled by load, and a
once-per-revolution wobble from prop damage. Betaflight's beeper is a
2.7 kHz tone. The output is soft-clipped with `tanh`.

### D17. Motor state is cleared on spawn and despawn

SimITL's `Reset`/`Repair` commands keep the motor model (RPM, ESC filters,
2 ms delay line). A drone deleted at hover throttle came back with
spinning props, popped up 5 cm and tripped ANGLE (found by test ordering).
`Flight::stopMotors` clears that state.

### D18. Shipped presets use Betaflight's stock tune

SimITL's per-quad `.bin` EEPROMs load fine (version 178), but differ from
defaults only in `feature -GPS -TELEMETRY`: there are no modes and no
tune. So libfpv ships no per-preset tune. The `<preset>.diff.txt`
mechanism (D7) is there for preset authors.

## Platforms

### D19. Windows: MinGW-w64 cross build, fully static runtimes

`cmake/toolchains/mingw-w64-x86_64.cmake` uses the `-posix` GCC
(winpthreads, needed for `std::thread`) and links libgcc, libstdc++ and
winpthreads statically into every binary. Without that, executables need
`libgcc_s_seh-1.dll`, `libstdc++-6.dll` and `libwinpthread-1.dll` and fail
to start silently: found when `fpv_tests.exe` exited with code 53. Now
`libfpv.dll` imports only Windows system DLLs. `libfpv.def` is emitted so
MSVC hosts can build an import library with `lib /def:libfpv.def
/machine:x64`. They can also `LoadLibrary` it, since the API is pure C.
The full test suite passes on Windows 11 (run from WSL interop).

SimITL's `string_light.c` re-implements libc's ctype and `strcasecmp`
functions. With whole-archive linking (D20) these collide with msvcrt, so
it is excluded on every platform and the platform libc is used. MinGW's
missing `strcasestr` comes from `src/core/bf_compat.c`. Betaflight C files
are compiled with `_GNU_SOURCE`, so `strcasestr` has a real prototype
rather than an implicit `int` that would truncate the pointer. Betaflight's
structs use GCC bitfield layout (`-mno-ms-bitfields`), applied to C and
C++ alike inside the core.

### D20. Whole-archive linking of the core

Betaflight's parameter groups register themselves in a linker section that
no code references by name. `simitl_core` is therefore linked with
`$<LINK_LIBRARY:WHOLE_ARCHIVE,...>`, so no PG can be dropped by the linker.
