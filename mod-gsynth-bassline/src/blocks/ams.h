/*
 * MOD GSynth Creator - block library: AMS oscillator, filter, LFO
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* ams-lv2 (Matthias Nagorni, Aurelien Leblond, GPLv2), MOD's fork:
 * src/vco3.cpp, src/vcf.cpp, src/lfo2_freq.cpp, src/synthdata.cpp.
 * The VCA (src/vcalin.cpp) is one line and lives where it is used:
 *   out = (gain1 + 0.2 * gainCv) * outputLevel * in */

#ifndef GSYNTH_BLOCKS_AMS_H
#define GSYNTH_BLOCKS_AMS_H

#include "common.h"

namespace gsynth {

/* ------------------------------------------------------------------------ */
/* VCO3. The 262,144-point wavetables are evaluated in closed form; phase is
 * in [0, 1). Hard sync is not ported yet. */

struct AmsVco3 {
    enum Wave { Sine, Triangle, Sawtooth, Rectangle, AuxSaw1, AuxSaw2, AuxSaw3 };

    float phi = 0.f;

    static inline float tri(float x)
    {
        if (x < .25f) return 4.f * x;
        if (x < .75f) return 1.f - 4.f * (x - .25f);
        return -1.f + 4.f * (x - .75f);
    }

    /* SynthData::wave_saw2: a falling ramp with a 1/16 period flyback */
    static inline float saw2(float x)
    {
        const float y = 1.f - x;
        const float ramp = (1.f - 1.f / 16.f) * .5f;
        if (y < ramp) return y * (2.f / (1.f - 1.f / 16.f));
        if (y < ramp + 1.f / 16.f) return 1.f - (y - ramp) * 32.f;
        return -1.f + (y - ramp - 1.f / 16.f) * (2.f / (1.f - 1.f / 16.f));
    }

    /* The knobs octave + tune + semitone/12 and the freq CV add up to `volts`:
     * the oscillator plays 2^(4.0313842 + volts) Hz. */
    static inline float hz(float volts) { return exp2_cv(4.0313842f + volts); }

    /* edge: knob 0..1; pw: pw0 + pwGain * 0.2 * pwCv, clamped here as the original does */
    inline float process(int wave, float freqHz, float sr, float edge, float pw)
    {
        float dphi = freqHz / sr;
        if (dphi > 32330.f / 262144.f) dphi = 32330.f / 262144.f;

        float out;
        switch (wave) {
        default:
        case Sine:     out = std::sin(2.f * (float)M_PI * phi); break;
        case Triangle: out = tri(phi); break;
        case AuxSaw1:  out = saw2(phi); break;
        case AuxSaw2:  out = phi < .5f ? saw2(2.f * phi) : 0.f; break;
        case AuxSaw3:  out = phi < 1.f / 3.f ? saw2(3.f * phi) : 0.f; break;
        case Sawtooth: {
            const float d = (0.01f + 1.8f * edge) * dphi;
            if (phi <= d)            out = phi / d;
            else if (phi <= 1.f - d) out = 1.f - (phi - d) * (2.f / (1.f - 2.f * d));
            else                     out = (phi - 1.f) / d;
            break;
        }
        case Rectangle: {
            const float d = (0.01f + 1.8f * edge) * dphi;
            if (pw < 0.1f) pw = 0.1f;
            else if (pw > 0.9f) pw = 0.9f;
            if (phi <= d)            out = phi / d;
            else if (phi <= pw - d)  out = 1.f;
            else if (phi <= pw + d)  out = (pw - phi) / d;
            else if (phi <= 1.f - d) out = -1.f;
            else                     out = (phi - 1.f) / d;
            break;
        }
        }

        phi += dphi;
        while (phi >= 1.f) phi -= 1.f;
        return out;
    }

    /* a bypassed oscillator keeps running on the pedalboard; only its output is dropped */
    inline void advance(float freqHz, float sr)
    {
        phi += std::fmin(freqHz / sr, 32330.f / 262144.f);
        while (phi >= 1.f) phi -= 1.f;
    }
};

/* ------------------------------------------------------------------------ */
/* VCF, all eight types. Cutoff in the original is
 *   2^(5.0313842 + freq + 0.2*keyCv + expFMGain*0.2*expCv) + 1000*linFMGain*0.2*linCv   [Hz]
 * The caller passes that sum as `hz`. "Resonant Lowpass" uses it through a
 * coefficient with no sample-rate term; it is scaled here so the tuning at any
 * rate matches what the pedalboards do at 48 kHz. The biquad types recompute
 * their coefficients every 16 samples instead of every sample. */

struct AmsVcf {
    enum Type { ResonantLowpass, Lowpass, Highpass, BandpassI, BandpassII, Notch, Lowpass24I, Lowpass24II };

    double in[5] = {}, buf[5] = {};
    double noise = 19.1919191919191919191919191919191919191919;
    double rateComp = 1.0, pi2Rate = 0, inv2Rate = 0;
    double b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0, invA0 = 1;
    int type = -1, coefAge = 0;

    void setup(float sr)
    {
        rateComp = 48000.0 / sr;
        pi2Rate = 2.0 * M_PI / sr;
        inv2Rate = 2.0 / sr;
        type = -1;
        clear();
    }

    void clear()
    {
        for (int i = 0; i < 5; ++i) in[i] = buf[i] = 0;
        coefAge = 0;
    }

    static inline double cutoffHz(float freq, float keyCv, float expFmGain, float expCv, float linFmGain, float linCv)
    {
        return exp2_cv(5.0313842f + freq + keyCv * 0.2f + expFmGain * expCv * 0.2f)
             + 1000.0 * linFmGain * linCv * 0.2;
    }

    inline double dither()
    {
        noise = noise * noise;
        noise -= (int)noise;
        const double out = noise - 1.5;
        noise += 19.0;
        return out;
    }

    inline float process(int newType, float x, double hz, double q)
    {
        if (newType != type) { clear(); type = newType; }
        if (q < 0.01) q = 0.01;
        else if (q > 1.0) q = 1.0;

        switch (type) {
        default:
        case ResonantLowpass: {
            double f = (2.85 / 20000.0) * rateComp * hz;
            if (f < 0.0) f = 0.0;
            else if (f > 0.99) f = 0.99;
            const double fa = 1.0 - f;
            const double fb = q * (1.0 + (1.0 / fa));
            buf[0] = fa * buf[0] + f * (x + fb * (buf[0] - buf[1]) + 0.00001 * dither());
            buf[1] = fa * buf[1] + f * buf[0];
            return (float)buf[1];
        }
        case Lowpass: case Highpass: case BandpassI: case BandpassII: case Notch: {
            if (coefAge == 0) {
                double f = hz;
                if (f < 20) f = 20;
                else if (f > 20000) f = 20000;
                const double s = std::sin(pi2Rate * f), c = std::cos(pi2Rate * f);
                const double alpha = s / (64.0 * q);
                switch (type) {
                case Lowpass:    b0 = (1.0 - c) * 0.5; b1 = 1.0 - c;  b2 = b0; break;
                case Highpass:   b0 = (1.0 + c) * 0.5; b1 = -1.0 - c; b2 = b0; break;
                case BandpassI:  b0 = q * alpha;       b1 = 0.0;      b2 = -q * alpha; break;
                case BandpassII: b0 = alpha;           b1 = 0.0;      b2 = -alpha; break;
                default:         b0 = 1.0;             b1 = -2.0 * c; b2 = 1.0; break;
                }
                invA0 = 1.0 / (1.0 + alpha);
                a1 = -2.0 * c;
                a2 = 1.0 - alpha;
            }
            coefAge = (coefAge + 1) & 15;
            const double y = invA0 * (b0 * x + b1 * buf[0] + b2 * buf[1] - a1 * buf[2] - a2 * buf[3]);
            buf[1] = buf[0]; buf[0] = x;
            buf[3] = buf[2]; buf[2] = y;
            return (float)y;
        }
        case Lowpass24I: {          // Timo Tossavainen
            double f = hz;
            if (f < 20) f = 20;
            else if (f > 22000) f = 22000;
            const double moogF = inv2Rate * f * 1.16;
            const double rev = 1.0 - moogF, f2 = moogF * moogF;
            const double fb = 4.1 * q * (1.0 - 0.15 * f2);
            in[0] = x + 0.000001 * dither();
            in[0] -= fb * buf[4];
            in[0] *= 0.35013 * (f2 * f2);
            buf[1] = in[0] + 0.3 * in[1] + rev * buf[1]; in[1] = in[0];
            buf[2] = buf[1] + 0.3 * in[2] + rev * buf[2]; in[2] = buf[1];
            buf[3] = buf[2] + 0.3 * in[3] + rev * buf[3]; in[3] = buf[2];
            buf[4] = buf[3] + 0.3 * in[4] + rev * buf[4]; in[4] = buf[3];
            return (float)buf[4];
        }
        case Lowpass24II: {         // Paul Kellet
            double f = hz;
            if (f < 20) f = 20;
            else if (f > 22000) f = 22000;
            const double fb = inv2Rate * f;
            double k = 1.0 - fb;
            const double p = fb + 0.8 * fb * k;
            const double fa = p + p - 1.0;
            k = q * (1.0 + 0.5 * k * (1.0 - k + 5.6 * k * k));
            in[0] = x + 0.000001 * dither();
            in[0] -= k * buf[4];
            if (in[0] < -1.0) in[0] = -1.0;
            if (in[0] > 1.0) in[0] = 1.0;
            double t1 = buf[1];
            buf[1] = (in[0] + buf[0]) * p - buf[1] * fa;
            double t2 = buf[2];
            buf[2] = (buf[1] + t1) * p - buf[2] * fa;
            t1 = buf[3];
            buf[3] = (buf[2] + t2) * p - buf[3] * fa;
            buf[4] = (buf[3] + t1) * p - buf[4] * fa;
            buf[4] -= buf[4] * buf[4] * buf[4] * 0.166667;
            buf[0] = in[0];
            return (float)buf[4];
        }
        }
    }
};

/* ------------------------------------------------------------------------ */
/* LFO2 (freq). The original advances its state once per 16-sample chunk and
 * interpolates linearly in between; the chunking depends on the host's block
 * size there, here it is always 16. Output is +-5 V. A rising reset CV (the
 * tracker's gate) restarts the cycle. */

struct AmsLfo2 {
    enum Wave { Sine, Triangle, SawUp, SawDown, Rectangle, SampleHold };

    double si = 0, oldSi = 0, sa = -1, oldSa = 0, t = 0, oldT = 0, r = 0, oldR = 0, sh = 0, oldSh = 0;
    double dSi = 0, dSa = 0, dT = 0, dR = 0, dSh = 0, dt = 0;
    int state = 0, left = 0;
    bool trigger = false;
    uint32_t rng = 0x2545F491u;
    float rate = 48000.f;

    void setup(float sr) { *this = AmsLfo2(); rate = sr; }

    inline double random()
    {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        return 2.0 * (rng / 4294967295.0) - 1.0;
    }

    inline float process(int wave, float freq, float resetCv)
    {
        if (left == 0) {
            const double period = rate / (16.0 * (freq < 0.0001f ? 0.0001f : freq));
            const double dsa = 2.0 / period, dt0 = 4.0 / period;
            if (!trigger && resetCv > 0.2f) {
                trigger = true;
                t = 0; state = 0; dt = dt0; r = -1; si = 0; sa = 0;
            }
            if (trigger && resetCv < 0.5f)
                trigger = false;
            if (t >= 1.0)       { state = 1; dt = -dt0; }
            else if (t <= -1.0) { state = 3; dt = dt0; }
            else if (state == 1 && t < 0) { state = 2; r = 1;  sh = random(); }
            else if (state == 3 && t > 0) { state = 0; r = -1; sh = random(); sa = -1; }
            si = (state < 2) ? t * (2.0 - t) : t * (2.0 + t);
            sa += dsa;
            t += dt;
            dt = (state == 1 || state == 2) ? -dt0 : dt0;
            dSi = (si - oldSi) / 16.0; dSa = (sa - oldSa) / 16.0; dSh = (sh - oldSh) / 16.0;
            dT = (t - oldT) / 16.0;    dR = (r - oldR) / 16.0;
            left = 16;
        }
        --left;
        oldSi += dSi; oldSa += dSa; oldSh += dSh; oldT += dT; oldR += dR;
        switch (wave) {
        default:
        case Sine:       return (float)(oldSi * 5.0);
        case Triangle:   return (float)(oldT * 5.0);
        case SawUp:      return (float)(oldSa * 5.0);
        case SawDown:    return (float)(-oldSa * 5.0);
        case Rectangle:  return (float)(-oldR * 5.0);
        case SampleHold: return (float)(oldSh * 5.0);
        }
    }
};

} // namespace gsynth

#endif
