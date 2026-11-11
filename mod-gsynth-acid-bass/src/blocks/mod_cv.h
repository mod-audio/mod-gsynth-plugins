/*
 * MOD GSynth Creator - block library: MOD CV utilities
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* mod-cv-plugins (MOD Devices, GPLv2+): source/mod-cv-slew, mod-cv-gate,
 * mod-cv-attenuverter, mod-cv-range. The trivial ones (audio-to-cv, abs,
 * control) are a multiply, a fabs and a constant where they are used. */

#ifndef GSYNTH_BLOCKS_MOD_CV_H
#define GSYNTH_BLOCKS_MOD_CV_H

#include "common.h"

namespace gsynth {

/* mod-cv-slew */
struct Slew {
    float out = 0.f, riseDiv = 1.f, fallDiv = 1.f;
    bool rise = true, fall = true;

    void setTimes(float riseMs, float fallMs, float sr)
    {
        rise = riseMs != 0.f;
        fall = fallMs != 0.f;
        riseDiv = std::fmax(1.f, (riseMs / 1000) * sr);
        fallDiv = std::fmax(1.f, (fallMs / 1000) * sr);
    }

    inline float process(float in)
    {
        if (in > out && rise)
            out += (in - out) / riseDiv;
        else if (in < out && fall)
            out += (in - out) / fallDiv;
        return out;
    }
};

/* mod-cv-attenuverter, "booster" mode (Mode = 1): gain is level^2 / 10, signed.
 * In "attenuverter" mode (Mode = 0) the gain is the level itself. */
static inline float boosterCoef(float level)
{
    return (level >= 0 ? 1.f : -1.f) * level * level / 10.f;
}

/* mod-cv-gate: passes the signal once the key has crossed the opening
 * threshold, until the key falls to the closing threshold */
struct KeyedGate {
    bool open = false;
    inline float process(float in, float key, float openAt, float closeAt)
    {
        if (key >= openAt) open = true;
        if (key <= closeAt) open = false;
        return open ? in : 0.f;
    }
};

/* mod-cv-range, first output: the input while it is inside [lo, hi];
 * outside, zero ("Reset to zero") or the last inside value ("Sample and hold") */
struct CvRange {
    float held = 0.f;
    inline float process(float in, float lo, float hi, bool resetToZero)
    {
        if (resetToZero) held = 0.f;
        if (in >= lo && in <= hi) held = in;
        return held;
    }
};

} // namespace gsynth

#endif
