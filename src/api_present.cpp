// libfpv - C API: overlay, OSD, audio, textures (temporary stubs).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "lib.h"

using namespace fpv;

extern "C" {
FPV_API fpv_result fpv_texture_get(uint64_t, fpv_texture *) { return setError(FPV_ERROR_NOT_FOUND, "not implemented yet"); }
FPV_API void fpv_overlay_set_open(int) {}
FPV_API int fpv_overlay_is_open(void) { return 0; }
FPV_API int fpv_overlay_wants_input(void) { return 0; }
FPV_API void fpv_overlay_set_toggle_key(int) {}
FPV_API void fpv_overlay_set_display(float, float, float) {}
FPV_API void fpv_overlay_mouse_move(float, float) {}
FPV_API void fpv_overlay_mouse_button(int, int) {}
FPV_API void fpv_overlay_mouse_wheel(float, float) {}
FPV_API void fpv_overlay_text(const char *) {}
FPV_API const fpv_draw_data *fpv_overlay_render(float) { return nullptr; }
FPV_API const fpv_draw_data *fpv_osd_draw_data(const fpv_frame *, float, float, fpv_osd_style) { return nullptr; }
FPV_API fpv_result fpv_osd_render_rgba(const fpv_frame *, uint8_t *, uint32_t, uint32_t, uint32_t, fpv_osd_style) {
    return setError(FPV_ERROR_NOT_FOUND, "not implemented yet");
}
FPV_API fpv_result fpv_audio_configure(uint32_t, uint32_t) { return FPV_OK; }
FPV_API uint32_t fpv_audio_read(float *, uint32_t) { return 0; }
FPV_API void fpv_audio_set_volume(float) {}
FPV_API void fpv_input_key(int key, int down) { lib().input.key(key, down != 0); }
}
