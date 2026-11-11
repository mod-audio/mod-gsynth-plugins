/*
 * MOD GSynth Creator - block library: MOD LowPassFilter
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* mod-utilities LowPassFilter, third order (MOD Devices, GPLv2+):
 * LowPassFilter/src/ModFilters.cpp. The plugin's Order port value 2 selects
 * this one (round(v) + 1). Fixed cutoff, coefficients computed once. */

#ifndef GSYNTH_BLOCKS_MOD_LOWPASS_H
#define GSYNTH_BLOCKS_MOD_LOWPASS_H

#include "common.h"

namespace gsynth {

struct ModLowPass3 {
    double K = 0, B = 0, C = 0, D = 0, invA = 1;
    double y1 = 0, y2 = 0, y3 = 0, u1 = 0, u2 = 0, u3 = 0;

    void setup(float sr, float freq)
    {
        const double w = 2 * M_PI * freq / sr;
        const double A = w * w * w + 4 * w * w + 8 * w + 8;
        K = w * w * w;
        B = 3 * w * w * w + 4 * w * w - 8 * w - 24;
        C = 3 * w * w * w - 4 * w * w - 8 * w + 24;
        D = w * w * w - 4 * w * w + 8 * w - 8;
        invA = 1.0 / A;
        y1 = y2 = y3 = u1 = u2 = u3 = 0;
    }

    inline float process(float u)
    {
        const double y = (-B * y1 - C * y2 - D * y3 + K * (u + 3 * u1 + 3 * u2 + u3)) * invA;
        y3 = y2; y2 = y1; y1 = y;
        u3 = u2; u2 = u1; u1 = u;
        return (float)y;
    }

    void endBlock()
    {
        if (std::fabs(y1) < 1e-20) y1 = 0;
        if (std::fabs(y2) < 1e-20) y2 = 0;
        if (std::fabs(y3) < 1e-20) y3 = 0;
    }
};

} // namespace gsynth

#endif
