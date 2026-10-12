// libfpv - OS specifics: pinning the module in memory, process exit hooks.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace fpv {

// Prevents the host from unmapping libfpv (FreeLibrary/dlclose) while its
// threads exist. Returns false if pinning is not possible.
bool pinLibraryInMemory();
// True once pinLibraryInMemory() succeeded (for tests).
bool libraryIsPinned();

// Runs fn at process exit on platforms where libfpv's threads are still
// alive at that point (POSIX). On Windows, ExitProcess has already killed
// them before DLL detach, so nothing is registered there.
void installExitHandler(void (*fn)());

}  // namespace fpv
