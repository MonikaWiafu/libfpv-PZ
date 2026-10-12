/* Force-included into every Betaflight / SimITL C translation unit.
 * Routes their console output to libfpv's log callback instead of the
 * host game's stdout. */
#ifndef LIBFPV_BF_LOG_REDIRECT_H
#define LIBFPV_BF_LOG_REDIRECT_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
int libfpv_bf_printf(const char *fmt, ...);
int libfpv_bf_fprintf(FILE *stream, const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#if defined(_WIN32)
/* not in msvcrt; provided by src/core/bf_compat.c */
char *strcasestr(const char *haystack, const char *needle);
#endif

/* Function-like macros so `__attribute__((format(printf, 1, 2)))` is untouched. */
#define printf(...) libfpv_bf_printf(__VA_ARGS__)
#define fprintf(...) libfpv_bf_fprintf(__VA_ARGS__)

#endif
