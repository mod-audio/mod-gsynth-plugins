/*
 * MOD GSynth Creator - block library: Invada compressor
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* Invada Compressor, mono (Fraser Stuart, GPLv2+): plugin/inv_compressor.c,
 * plugin/library/common.c. The original converts Ratio with the same dB
 * formula as Threshold and Gain; kept as is, so pass the knob values. */

#ifndef GSYNTH_BLOCKS_INVADA_COMPRESSOR_H
#define GSYNTH_BLOCKS_INVADA_COMPRESSOR_H

#include "common.h"

namespace gsynth {

struct InvadaCompressor {
    struct Settings { float rms, attack, release, threshDb, ratio, gainDb; bool noClip; };

    float rmsSize = 1, attack = 1, release = 1, thresh = 1, ratio = 1, gain = 1;
    bool clip = true;
    float rms = 0, env = 0;

    static float dbParam(float v)
    {
        if (v < -36) return std::pow(10., -1.8);
        if (v < 36) return std::pow(10., v / 20.0);
        return std::pow(10., 1.8);
    }

    void setup(float sr, const Settings& s)
    {
        rmsSize = s.rms < 0 ? 1 : (s.rms < 1 ? std::pow(s.rms, 3) * sr / 20 + 1 : sr / 20 + 1);
        const double att = s.attack < 0.00001 ? 0.01 : (s.attack < 0.750 ? s.attack * 1000.0 : 750.0);
        const double rel = s.release < 0.001 ? 1.0 : (s.release < 5 ? s.release * 1000.0 : 5000.0);
        attack = 1 - std::pow(10., -301.0301 / (sr * att));
        release = 1 - std::pow(10., -301.0301 / (sr * rel));
        thresh = dbParam(s.threshDb);
        ratio = dbParam(s.ratio);
        gain = dbParam(s.gainDb);
        clip = s.noClip;
        rms = env = 0;
    }

    static inline float noClip(float in)
    {
        if (std::fabs(in) < 0.7f)
            return in;
        return in > 0 ?  (0.7f + 0.3f * (1 - std::exp(3.33333333f * (0.7f - in))))
                      : (-0.7f - 0.3f * (1 - std::exp(3.33333333f * (0.7f + in))));
    }

    inline float process(float x)
    {
        rms = std::sqrt(((rmsSize - 1) * rms * rms + x * x) / rmsSize);
        env += (rms > env ? attack : release) * (rms - env);
        const float g = env > thresh ? std::pow(env / thresh, 1.f / ratio - 1.f) : 1.f;
        const float y = x * g * gain;
        return clip ? noClip(y) : y;
    }

    void endBlock()
    {
        if (std::fabs(env) < 1.0e-10f) env = 0.f;
        if (std::fabs(rms) < 1.0e-10f) rms = 0.f;
    }
};

} // namespace gsynth

#endif
