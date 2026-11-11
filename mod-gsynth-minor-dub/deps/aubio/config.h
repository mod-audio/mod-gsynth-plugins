/* Build configuration for the vendored aubio subset (yinfast only).
 * FFTW is required: the analysis window is 1408 samples, which aubio's
 * built-in Ooura FFT (power-of-two sizes only) cannot handle. */
#ifndef AUBIO_CONFIG_H
#define AUBIO_CONFIG_H
#define HAVE_STDLIB_H 1
#define HAVE_STDIO_H 1
#define HAVE_MATH_H 1
#define HAVE_STRING_H 1
#define HAVE_LIMITS_H 1
#define HAVE_STDARG_H 1
#define HAVE_C99_VARARGS_MACROS 1
#define HAVE_MEMCPY_HACKS 1
#define HAVE_FFTW3
#define HAVE_FFTW3F
#define HAVE_COMPLEX_H
#endif
