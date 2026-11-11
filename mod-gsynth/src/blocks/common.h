/*
 * MOD GSynth Creator - block library: shared helpers
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

#ifndef GSYNTH_BLOCKS_COMMON_H
#define GSYNTH_BLOCKS_COMMON_H

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace gsynth {

/* ams-lv2's SynthData::exp2_table is a table approximation of 2^x */
static inline float exp2_cv(float x) { return x < -16.f ? 0.f : std::exp2(x); }

/* One-pole smoother for parameters the original plugins did not smooth, or
 * smoothed once per block. */
struct Smooth {
    float z = 0.f, a = 1.f;
    void setup(float sr, float ms, float initial) { a = 1.f - std::exp(-1.f / (0.001f * ms * sr)); z = initial; }
    inline float process(float target) { return z += a * (target - z); }
};

} // namespace gsynth

#endif
