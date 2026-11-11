/*
 * MOD GSynth - flush-to-zero guard
 * Copyright (C) 2026 MOD Audio
 * GPL-3.0-or-later, see LICENSE.
 *
 * The filter, envelope and reverb tails all decay towards zero; denormal
 * arithmetic there is orders of magnitude slower and shows up as xruns on the
 * devices. The host owns the audio thread, so the previous FPU state is
 * restored on exit.
 */

#ifndef GSYNTH_DENORMALS_H
#define GSYNTH_DENORMALS_H

#if defined(__SSE2__)
  #include <xmmintrin.h>
  #include <pmmintrin.h>
#endif

namespace gsynth {

class ScopedFlushDenormals
{
public:
    ScopedFlushDenormals() noexcept
    {
#if defined(__SSE2__)
        saved = _mm_getcsr();
        _mm_setcsr(saved | 0x8040); // FTZ | DAZ
#elif defined(__aarch64__)
        __asm__ __volatile__("mrs %0, fpcr" : "=r"(saved));
        const unsigned long long fz = saved | (1ull << 24); // FPCR.FZ
        __asm__ __volatile__("msr fpcr, %0" : : "r"(fz));
#elif defined(__arm__) && defined(__VFP_FP__) && !defined(__SOFTFP__)
        __asm__ __volatile__("vmrs %0, fpscr" : "=r"(saved));
        const unsigned int fz = saved | (1u << 24); // FPSCR.FZ
        __asm__ __volatile__("vmsr fpscr, %0" : : "r"(fz));
#endif
    }

    ~ScopedFlushDenormals() noexcept
    {
#if defined(__SSE2__)
        _mm_setcsr(saved);
#elif defined(__aarch64__)
        __asm__ __volatile__("msr fpcr, %0" : : "r"(saved));
#elif defined(__arm__) && defined(__VFP_FP__) && !defined(__SOFTFP__)
        __asm__ __volatile__("vmsr fpscr, %0" : : "r"(saved));
#endif
    }

    ScopedFlushDenormals(const ScopedFlushDenormals&) = delete;
    ScopedFlushDenormals& operator=(const ScopedFlushDenormals&) = delete;

private:
#if defined(__aarch64__)
    unsigned long long saved = 0;
#else
    unsigned int saved = 0;
#endif
};

} // namespace gsynth

#endif
