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
