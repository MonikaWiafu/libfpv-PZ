/* libfpv-PZ: force-included into every C file by mingw-w64-x86_64.cmake.
 * SimITL's target.c defines strsep() (MinGW has none) but no header declares it, so
 * Betaflight's cli.c calls it through an implicit int declaration: GCC only warns, and the
 * 64-bit pointer it returns is cut to 32 bits. Declaring it here fixes that. */
#ifndef LIBFPV_PZ_MINGW_COMPAT_H
#define LIBFPV_PZ_MINGW_COMPAT_H
#ifndef __cplusplus
char *strsep(char **stringp, const char *delim);
#endif
#endif
