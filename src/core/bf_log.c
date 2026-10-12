#include <stdarg.h>
#include <stdio.h>
int libfpv_bf_printf(const char *fmt, ...) { va_list a; va_start(a, fmt); int r = vprintf(fmt, a); va_end(a); return r; }
int libfpv_bf_fprintf(FILE *f, const char *fmt, ...) { va_list a; va_start(a, fmt); int r = vfprintf(f, fmt, a); va_end(a); return r; }
