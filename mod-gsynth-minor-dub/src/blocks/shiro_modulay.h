/*
 * MOD GSynth Creator - block library: SHIRO Modulay
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* SHIRO Modulay (Nino de Wit, GPLv3): plugins/modulay/gen_exported.cpp, a
 * Max gen~ export; the delay line, DC blocker and cosine oscillator follow
 * Cycling '74's genlib (MIT), plugins/common/gen_dsp/genlib_ops.h.
 * A delay with a tone filter in the feedback path and a short modulated
 * second delay; Morph moves between chorus-like and vibrato-like modulation.
 * As in the original: the delay memory is 44100 samples whatever the rate
 * (919 ms at 48 kHz, longer Time settings clamp), and the tone filter's
 * coefficient assumes 48 kHz. Output is dry + wet. */

#ifndef GSYNTH_BLOCKS_SHIRO_MODULAY_H
#define GSYNTH_BLOCKS_SHIRO_MODULAY_H

#include "common.h"

namespace gsynth {

struct ShiroModulay {
    struct Settings { float timeMs, repeatsPct, mixPct, rateHz, depthMs, morph, toneHz; };

    struct Delay {
        static constexpr long kMax = 44100, kSize = 65536, kWrap = kSize - 1;
        float* memory = nullptr;
        long reader = 0, writer = 0;

        bool init() { memory = (float*)std::calloc(kSize, sizeof(float)); return memory != nullptr; }
        void destroy() { std::free(memory); memory = nullptr; }
        void reset() { if (memory) std::memset(memory, 0, kSize * sizeof(float)); reader = writer = 0; }
        inline void step() { if (++reader >= kSize) reader = 0; }
        inline void write(float x) { writer = reader; memory[writer] = x; }
        inline float readLinear(double d)
        {
            const double lo = reader != writer ? 1.0 : 0.0;
            const double c = d < lo ? lo : (d > kMax ? (double)kMax : d);
            const double r = (double)(kSize + reader) - c;
            const long r1 = (long)r;
            const double a = r - (double)r1;
            const float x = memory[r1 & kWrap], y = memory[(r1 + 1) & kWrap];
            return (float)(x + a * (y - x));
        }
    };

    Delay main, mod;
    double smoothTime = 0, smoothMod = 0, tone = 0, dcX = 0, dcY = 0;
    uint32_t phase = 0;
    float sr = 48000.f;

    bool init(float sampleRate) { sr = sampleRate; return main.init() && mod.init(); }
    void destroy() { main.destroy(); mod.destroy(); }
    void reset() { main.reset(); mod.reset(); smoothTime = smoothMod = tone = dcX = dcY = 0; phase = 0; }

    inline float process(float x, const Settings& s)
    {
        const double mix = s.mixPct * 0.01, repeats = s.repeatsPct * 0.01;
        const double target = s.timeMs * (sr * 0.001);
        double coef = std::sin(s.toneHz * 2 * M_PI / 48000.0);
        coef = coef <= 1e-05 ? 1e-05 : (coef >= 0.99999 ? 0.99999 : coef);
        const double fold = std::fabs(s.morph * -0.8 + 40);
        const double throughGain = fold * 0.01;                 // main delay straight to the output
        const double modGain = (80 - fold) * 0.01;              // modulated tap to the output
        const double crossGain = (s.morph * 0.8 - 40) * 0.01;   // modulated tap back into itself
        double over = (s.morph > 50 ? s.morph : 0) - 50;
        over = over <= 0 ? 0 : (over >= 50 ? 50 : over);
        const double baseMs = (s.morph < 50 ? s.morph * -0.4 + 20 : 0) + over * 0.04;

        smoothTime = target + 0.999 * (smoothTime - target);
        const double tap = main.readLinear(smoothTime);
        tone = tone + coef * (tap - tone);

        const double fb = tone * repeats;
        double dc = fb - dcX + dcY * 0.9997;
        dcX = fb; dcY = dc;
        dc = dc <= -1 ? -1 : (dc >= 1 ? 1 : dc);

        // genlib's cycle~ is a cosine
        const double lfo = (std::cos(phase * (2.0 * M_PI / 4294967296.0)) + 1) * 0.5;
        phase += (uint32_t)(s.rateHz * (4294967296.0 / sr));
        const double modTarget = (s.depthMs * lfo + baseMs) * (sr * 0.001);
        smoothMod = modTarget + 0.999 * (smoothMod - modTarget);
        const double modTap = mod.readLinear(smoothMod);

        const float out = (float)((modTap * modGain + tone * throughGain) * mix + x);

        main.write((float)(dc + x));
        mod.write((float)(tone + modTap * crossGain));
        mod.step();
        main.step();
        return out;
    }
};

} // namespace gsynth

#endif
