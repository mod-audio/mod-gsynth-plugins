/*
 * MOD GSynth Creator - block library: x42 fil4 high-pass and low-pass
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* x42 fil4 (Robin Gareus, GPLv2+): src/hip.h, src/lop.h, src/iir.h, src/uris.h.
 * Only the high-pass and low-pass stages; the pedalboards leave the shelves and
 * the four bands at 0 dB. Coefficients are set directly to their targets
 * instead of fading in. */

#ifndef GSYNTH_BLOCKS_X42_FIL4_H
#define GSYNTH_BLOCKS_X42_FIL4_H

#include "common.h"

namespace gsynth {

struct Fil4HighPass {
    float a = 1, q = 0, g = 1, z1 = 0, z2 = 0, y2 = 0;

    void setup(float rate, float freq, float qual)
    {
        float q2 = 0.7f + 0.78f * std::tanh(1.82f * (qual - .8f));
        if (q2 < 0.f) q2 = 0.f;
        if (q2 > 1.6f) q2 = 1.6f;
        if (freq > rate / 12.f) freq = rate / 12.f;
        if (freq < 5.f) freq = 5.f;
        const float omega = freq / rate;
        a = std::exp(-2.0 * M_PI * omega);
        q = q2;
        g = 1.f + omega + 2.f * q * omega;
        z1 = z2 = y2 = 0;
    }

    inline float process(float x)
    {
        const float pz1 = z1, pz2 = z2;
        z1 = (g / a) * x - (g * q) * (y2 - z2);
        z2 = a * (z2 + z1 - pz1);
        y2 = a * (y2 + z2 - pz2);
        return y2;
    }

    void endBlock() { z1 += 1e-12f; z2 += 1e-12f; }
};

struct Fil4LowPass {
    float a = 1, b = 1, r = 0, z1 = 0, z2 = 0, z3 = 0, z4 = 0;
    /* fil4 follows its low-pass with a fixed -6 dB high shelf at rate/3 */
    bool shelf = true;
    float sb0 = 1, sb1 = 0, sb2 = 0, sa1 = 0, sa2 = 0, sy1 = 0, sy2 = 0;

    static float alphaFor(float rate, float freq)
    {
        float fr = freq / rate;
        if (fr < 0.0002f) fr = 0.0002f;
        if (fr > 0.4998f) fr = 0.4998f;
        return 1.0 - std::exp(-2.0 * M_PI * fr);
    }

    /* withShelf = false is only for signals nobody hears (the tracker's input) */
    void setup(float rate, float freq, float res, bool withShelf)
    {
        float fb = 3.f * std::pow(res, 3.20772f);
        if (fb < 0) fb = 0;
        if (fb > 9) fb = 9;
        const float fs = freq / std::sqrt(1 + fb);
        a = alphaFor(rate, fs);
        b = alphaFor(rate, .25f * rate + .5f * fs);
        const float w2 = 4 * freq / rate;
        const float w3 = freq / (.25f * rate + .5f + freq);
        const float tg = (1 + w3 * w3) / (1 + w2 * w2);
        r = fb * tg;
        z1 = z2 = z3 = z4 = 0;

        shelf = withShelf;
        if (shelf) {
            const double w0 = 2. * M_PI * (1. / 3.);
            const double cosW = std::cos(w0);
            const double A = std::sqrt(.5), As = std::sqrt(A);
            const double al = std::sin(w0) / 2 * (1 / .444);
            const double a0 = (A + 1) - (A - 1) * cosW + 2 * As * al;
            sb0 = A * ((A + 1) + (A - 1) * cosW + 2 * As * al) / a0;
            sb1 = -2 * A * ((A - 1) + (A + 1) * cosW) / a0;
            sb2 = A * ((A + 1) + (A - 1) * cosW - 2 * As * al) / a0;
            sa1 = 2 * ((A - 1) - (A + 1) * cosW) / a0;
            sa2 = ((A + 1) - (A - 1) * cosW - 2 * As * al) / a0;
            sy1 = sy2 = 0;
        }
    }

    inline float process(float x)
    {
        const float in = (1 + r) * x - z2 * r;
        z1 += a * (in - z1);
        z2 += a * (z1 - z2);
        z3 += b * (z2 - z3);
        z4 += b * (z3 - z4);
        if (!shelf)
            return z4;
        const float y = sb0 * z4 + sy1;
        sy1 = sb1 * z4 - sa1 * y + sy2;
        sy2 = sb2 * z4 - sa2 * y;
        return y;
    }

    void endBlock() { z1 += 1e-12f; z2 += 1e-12f; z3 += 1e-12f; z4 += 1e-12f; }
};

} // namespace gsynth

#endif
