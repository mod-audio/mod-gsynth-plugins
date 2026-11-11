/*
 * MOD GSynth Creator - block library: blocks written for this project
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* Unlike the rest of the library these are not ports of a plugin from the
 * GSYNTH pedalboards: the pedalboards have no triggered envelope, no noise
 * source and no chorus. Original code. */

#ifndef GSYNTH_BLOCKS_EXTRAS_H
#define GSYNTH_BLOCKS_EXTRAS_H

#include "common.h"

namespace gsynth {

/* white noise, +-1 */
struct WhiteNoise {
    uint32_t state = 0x9E3779B9u;
    inline float process()
    {
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        return (int32_t)state * (1.f / 2147483648.f);
    }
};

/* Note onsets from the input signal itself. The pitch tracker's gate cannot
 * do this: it updates once per analysis and stays up through repeated notes.
 * A fast and a slow envelope follower run side by side; a pick attack makes
 * the fast one jump ahead of the slow one. */
struct OnsetDetector {
    float fast = 0.f, slow = 0.f, fastUp = 1.f, fastDown = 1.f, slowUp = 1.f, slowDown = 1.f;
    uint32_t holdoff = 0, holdoffSamples = 0;

    void setup(float sr)
    {
        auto coef = [sr](float ms) { return 1.f - std::exp(-1.f / (0.001f * ms * sr)); };
        fastUp = coef(0.5f); fastDown = coef(25.f);
        slowUp = coef(35.f); slowDown = coef(80.f);
        holdoffSamples = (uint32_t)(0.07f * sr);
        fast = slow = 0.f;
        holdoff = 0;
    }

    /* floor: level below which nothing is an onset. Returns true on the sample an onset is seen. */
    inline bool process(float x, float floor)
    {
        const float a = std::fabs(x);
        fast += (a > fast ? fastUp : fastDown) * (a - fast);
        slow += (fast > slow ? slowUp : slowDown) * (fast - slow);
        if (holdoff > 0) { --holdoff; return false; }
        if (fast > floor && fast > 1.8f * slow) { holdoff = holdoffSamples; return true; }
        return false;
    }
};

/* attack/decay envelope, 0..1, retriggered from wherever it is */
struct AdEnvelope {
    float value = 0.f, sr = 48000.f;
    bool rising = false;

    void setup(float sampleRate) { sr = sampleRate; value = 0.f; rising = false; }
    inline void trigger() { rising = true; }

    inline float process(float attackMs, float decayMs)
    {
        if (rising) {
            value += 1.f / std::fmax(1.f, 0.001f * attackMs * sr);
            if (value >= 1.f) { value = 1.f; rising = false; }
        } else {
            value -= value / std::fmax(1.f, 0.001f * decayMs * sr * 0.3f);   // reaches -60 dB in about decayMs * 2
            if (value < 1e-5f) value = 0.f;
        }
        return value;
    }
};

/* One modulated delay tap per side, a quarter cycle apart. Returns the wet pair. */
struct StereoChorus {
    float* memory = nullptr;
    uint32_t mask = 0, writer = 0;
    float sr = 48000.f, phase = 0.f;

    bool init(float sampleRate)
    {
        sr = sampleRate;
        uint32_t size = 1;
        while (size < (uint32_t)(0.05f * sr)) size <<= 1;
        memory = (float*)std::calloc(size, sizeof(float));
        mask = size - 1;
        return memory != nullptr;
    }
    void destroy() { std::free(memory); memory = nullptr; }
    void reset() { if (memory) std::memset(memory, 0, (mask + 1) * sizeof(float)); writer = 0; phase = 0.f; }

    /* parabolic sine, phase in cycles */
    static inline float sine(float p)
    {
        p -= std::floor(p);
        const float x = 2.f * p - 1.f;
        return -4.f * x * (1.f - std::fabs(x));
    }

    inline float tap(float delaySamples) const
    {
        const float r = (float)writer - delaySamples;
        const int32_t i = (int32_t)std::floor(r);
        const float f = r - (float)i;
        return memory[(uint32_t)i & mask] * (1.f - f) + memory[(uint32_t)(i + 1) & mask] * f;
    }

    inline void process(float x, float rateHz, float baseMs, float depthMs, float* wl, float* wr)
    {
        memory[writer] = x;
        const float k = 0.001f * sr;
        *wl = tap((baseMs + depthMs * sine(phase)) * k);
        *wr = tap((baseMs + depthMs * sine(phase + 0.25f)) * k);
        writer = (writer + 1) & mask;
        phase += rateHz / sr;
        if (phase >= 1.f) phase -= 1.f;
    }
};

} // namespace gsynth

#endif
