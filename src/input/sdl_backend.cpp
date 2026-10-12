// libfpv - SDL3 device backend (radios in USB joystick mode, gamepads).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "input/input.h"

namespace fpv {
#if !defined(LIBFPV_WITH_SDL)
std::unique_ptr<DeviceBackend> createSdlBackend() { return nullptr; }
#endif
}  // namespace fpv
