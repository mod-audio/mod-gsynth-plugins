/*
 * MOD GSynth Creator - block library: pitch tracker
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* DISTRHO PitchTracking Series, AudioToCVPitch (Bram Giesen, falkTX, GPLv3+),
 * on aubio's yinfast (Paul Brossier, GPLv3+). The original runs aubio_pitch
 * with a hop equal to the window, which reduces to: silence gate, yinfast on
 * the raw window, period to Hz. Needs deps/aubio and fftw3f.
 *
 * With the default configuration this is that plugin: 1408 samples at the full
 * rate, analysed once per window. yinfast finds periods up to half the window,
 * so that reaches down to 68 Hz at 48 kHz: a guitar, not a bass. For lower
 * notes the block can decimate its input (it must already be low-passed well
 * below the decimated Nyquist) and slide the window by a hop instead of
 * waiting for a full one: 1024 samples at 12 kHz span 85 ms, reach 23 Hz, and
 * cost a smaller FFT than the original's, every 10.7 ms. */

#ifndef GSYNTH_BLOCKS_PITCH_TRACKER_H
#define GSYNTH_BLOCKS_PITCH_TRACKER_H

#include "common.h"

extern "C" {
#include "types.h"
#include "fvec.h"
#include "pitch/pitchyinfast.h"
#include "musicutils.h"
}

namespace gsynth {

struct PitchTracker {
    static constexpr uint32_t kWindow = 1024 + 256 + 128;
    static constexpr float kSilenceDb = -30.f;

    uint32_t size = kWindow, hop = kWindow, decimation = 1, phase = 0;

    fvec_t* window = nullptr;
    fvec_t* result = nullptr;
    aubio_pitchyinfast_t* yin = nullptr;
    uint32_t pos = 0;
    float rate = 48000.f;
    float tolerance = -1.f;

    float cvPitch = 0.f;    // volts, 1 V/oct, MIDI note / 12
    float cvGate = 0.f;     // 0 or 10
    float pitchHz = 0.f;
    float confidence = 0.f;

    /* windowSize and hopSize count decimated samples; hopSize divides windowSize */
    bool init(float sampleRate, uint32_t windowSize = kWindow, uint32_t hopSize = kWindow, uint32_t decimate = 1)
    {
        size = windowSize;
        hop = hopSize;
        decimation = decimate;
        rate = sampleRate / decimate;
        window = new_fvec(size);
        result = new_fvec(1);
        yin = new_aubio_pitchyinfast(size);
        pos = size - hop;
        return window && result && yin;
    }

    void destroy()
    {
        if (yin) del_aubio_pitchyinfast(yin);
        if (result) del_fvec(result);
        if (window) del_fvec(window);
        yin = nullptr; result = nullptr; window = nullptr;
    }

    void reset()
    {
        pos = size - hop;
        phase = 0;
        cvPitch = cvGate = pitchHz = confidence = 0.f;
        if (window) fvec_zeros(window);
    }

    /* tolerancePct as on the plugin's knob, 0..100 */
    void setTolerance(float tolerancePct)
    {
        if (tolerancePct != tolerance) {
            tolerance = tolerancePct;
            aubio_pitchyinfast_set_tolerance(yin, tolerancePct * 0.01f);
        }
    }

    /* x is already scaled by Sensitivity; confidencePct as on the knob */
    inline void process(float x, int octave, float confidencePct, bool holdPitch)
    {
        if (decimation > 1 && ++phase != decimation)
            return;
        phase = 0;
        window->data[pos] = x;
        if (++pos != size)
            return;

        aubio_pitchyinfast_do(yin, window, result);
        float hz = result->data[0] > 0.f ? rate / result->data[0] : 0.f;
        if (aubio_silence_detection(window, kSilenceDb) == 1)
            hz = 0.f;
        confidence = aubio_pitchyinfast_get_confidence(yin);

        if (hz > 0.f && confidence >= confidencePct * 0.01f) {
            const float note = 12.f * (std::log2(hz / 440.f) + octave - 1) + 69.f;
            cvPitch = std::fmax(0.f, std::fmin(10.f, note * (1.f / 12.f)));
            pitchHz = hz;
            cvGate = 10.f;
        } else {
            if (!holdPitch)
                pitchHz = cvPitch = 0.f;
            cvGate = 0.f;
        }

        // slide: keep the newest (size - hop) samples; with hop == size nothing is kept
        pos = size - hop;
        if (pos != 0)
            std::memmove(window->data, window->data + hop, pos * sizeof(smpl_t));
    }
};

} // namespace gsynth

#endif
