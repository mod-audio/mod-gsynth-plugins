/*
 * MOD GSynth Creator - block library: CAPS Plate reverb
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* CAPS Plate (Tim Goetze, GPLv3+): Reverb.cc, Reverb.h, dsp/Delay.h,
 * dsp/IIR1.h, dsp/Sine.h. process() returns the wet pair; the plugin's Blend
 * is `dry * (1 - blend^1.6) + wet * blend^1.6`, done by the caller. */

#ifndef GSYNTH_BLOCKS_CAPS_PLATE_H
#define GSYNTH_BLOCKS_CAPS_PLATE_H

#include "common.h"

namespace gsynth {

struct CapsPlate {
    struct Delay {
        uint32_t mask = 0, read = 0, write = 0;
        float* data = nullptr;

        bool init(uint32_t n)
        {
            uint32_t size = 1;
            while (size < n) size <<= 1;
            if (size < 2) size = 2;
            data = (float*)std::calloc(size, sizeof(float));
            mask = size - 1;
            read = 0;
            write = n & mask;
            return data != nullptr;
        }
        void destroy() { std::free(data); data = nullptr; }
        void reset() { if (data) std::memset(data, 0, (mask + 1) * sizeof(float)); }

        inline float& tap(int i) { return data[(write - i) & mask]; }
        inline void put(float x) { data[write] = x; write = (write + 1) & mask; }
        inline float get() { const float x = data[read]; read = (read + 1) & mask; return x; }
        inline float putget(float x) { put(x); return get(); }
        inline float getLinear(float f)
        {
            const int n = (int)f;
            f -= n;
            return (1 - f) * tap(n) + f * tap(n + 1);
        }
    };

    struct Lattice : Delay {
        inline float process(float x, double d)
        {
            const float y = get();
            x -= d * y;
            put(x);
            return d * x + y;
        }
    };

    struct Sine {
        int z = 0;
        double y[2] = {0, 0}, b = 0;
        void set(double w, double phase)
        {
            b = 2 * std::cos(w);
            y[0] = std::sin(phase - w);
            y[1] = std::sin(phase - 2 * w);
            z = 0;
        }
        inline double get()
        {
            double s = b * y[z];
            z ^= 1;
            s -= y[z];
            return y[z] = s;
        }
    };

    struct ModLattice {
        float n0 = 0, width = 0;
        Delay delay;
        Sine lfo;
        bool init(int n, int w) { n0 = n; width = w; return delay.init(n + w); }
        inline float process(float x, double d)
        {
            const float y = delay.getLinear(n0 + width * lfo.get());
            x += d * y;
            delay.put(x);
            return y - d * x;
        }
    };

    struct LP1 {
        float a0 = 1, b1 = 0, y1 = 0;
        void set(float d) { a0 = d; b1 = 1 - d; }
        inline float process(float x) { return y1 = a0 * x + b1 * y1; }
    };

    static constexpr double indiff1 = .742, indiff2 = .712, dediff1 = .723, dediff2 = .729;

    float fs = 48000.f;
    float normal = 1e-20f;
    float lastBandwidth = -1.f, lastDamping = -1.f;
    LP1 bandwidth, damping[2];
    Lattice inLattice[4], tankLattice[2];
    ModLattice tankMod[2];
    Delay tankDelay[4];
    int taps[12] = {};

    bool init(float sampleRate)
    {
        fs = sampleRate;
        static const float l[] = {
            0.004771345048889486, 0.0035953092974026408,
            0.01273478713752898, 0.0093074829474816042,
            0.022579886428547427, 0.030509727495715868,
            0.14962534861059779, 0.060481838647894894, 0.12499579987231611,
            0.14169550754342933, 0.089244313027116023, 0.10628003091293972
        };
        static const float t[] = {
            0.0089378717113000241, 0.099929437854910791, 0.064278754074123853,
            0.067067638856221232, 0.066866032727394914, 0.006283391015086859,
            0.01186116057928161, 0.12187090487550822, 0.041262054366452743,
            0.089815530392123921, 0.070931756325392295, 0.011256342192802662
        };
#define GSYNTH_PLATE_L(i) ((int)(l[i] * fs))
        bool ok = true;
        for (int i = 0; i < 4; ++i)
            ok = inLattice[i].init(GSYNTH_PLATE_L(i)) && ok;
        ok = tankMod[0].init(GSYNTH_PLATE_L(4), (int)(0.000403221 * fs)) && ok;
        ok = tankMod[1].init(GSYNTH_PLATE_L(5), (int)(0.000403221 * fs)) && ok;
        ok = tankDelay[0].init(GSYNTH_PLATE_L(6)) && ok;
        ok = tankLattice[0].init(GSYNTH_PLATE_L(7)) && ok;
        ok = tankDelay[1].init(GSYNTH_PLATE_L(8)) && ok;
        ok = tankDelay[2].init(GSYNTH_PLATE_L(9)) && ok;
        ok = tankLattice[1].init(GSYNTH_PLATE_L(10)) && ok;
        ok = tankDelay[3].init(GSYNTH_PLATE_L(11)) && ok;
#undef GSYNTH_PLATE_L
        for (int i = 0; i < 12; ++i)
            taps[i] = (int)(t[i] * fs);

        setTone(1.f, 0.f);
        reset();
        return ok;
    }

    /* the Bandwidth and Damping knobs, 0..1 each; cheap enough to call once per block */
    void setTone(float bandwidthKnob, float dampingKnob)
    {
        if (bandwidthKnob == lastBandwidth && dampingKnob == lastDamping)
            return;
        lastBandwidth = bandwidthKnob;
        lastDamping = dampingKnob;
        const float bw = .005f + .994f * bandwidthKnob;
        bandwidth.set(std::exp(-M_PI * (1. - bw)));
        const float damp = std::exp(-M_PI * (.0005 + .9995 * dampingKnob));
        damping[0].set(damp);
        damping[1].set(damp);
    }

    void destroy()
    {
        for (auto& d : inLattice) d.destroy();
        for (auto& d : tankLattice) d.destroy();
        for (auto& d : tankDelay) d.destroy();
        for (auto& m : tankMod) m.delay.destroy();
    }

    void reset()
    {
        bandwidth.y1 = damping[0].y1 = damping[1].y1 = 0;
        for (auto& d : inLattice) d.reset();
        for (auto& d : tankLattice) d.reset();
        for (auto& d : tankDelay) d.reset();
        for (auto& m : tankMod) m.delay.reset();
        tankMod[0].lfo.set(1.2 * 2 * M_PI / fs, 0);
        tankMod[1].lfo.set(1.2 * 2 * M_PI / fs, .5 * M_PI);
    }

    /* tail: the "Decay" knob, 0..1. Returns the wet pair. */
    inline void process(float x, float tail, float* wl, float* wr)
    {
        const float decay = .749f * tail;
        normal = -normal;
        x = bandwidth.process(x + normal);

        x = inLattice[0].process(x, indiff1);
        x = inLattice[1].process(x, indiff1);
        x = inLattice[2].process(x, indiff2);
        x = inLattice[3].process(x, indiff2);

        double xl = x + decay * tankDelay[3].get();
        double xr = x + decay * tankDelay[1].get();

        xl = tankMod[0].process(xl, dediff1);
        xl = tankDelay[0].putget(xl);
        xl = damping[0].process(xl);
        xl *= decay;
        xl = tankLattice[0].process(xl, dediff2);
        tankDelay[1].put(xl);

        xr = tankMod[1].process(xr, dediff1);
        xr = tankDelay[2].putget(xr);
        xr = damping[1].process(xr);
        xr *= decay;
        xr = tankLattice[1].process(xr, dediff2);
        tankDelay[3].put(xr);

        xl  = .6 * tankDelay[2].tap(taps[0]);
        xl += .6 * tankDelay[2].tap(taps[1]);
        xl -= .6 * tankLattice[1].tap(taps[2]);
        xl += .6 * tankDelay[3].tap(taps[3]);
        xl -= .6 * tankDelay[0].tap(taps[4]);
        xl += .6 * tankLattice[0].tap(taps[5]);

        xr  = .6 * tankDelay[0].tap(taps[6]);
        xr += .6 * tankDelay[0].tap(taps[7]);
        xr -= .6 * tankLattice[0].tap(taps[8]);
        xr += .6 * tankDelay[1].tap(taps[9]);
        xr -= .6 * tankDelay[2].tap(taps[10]);
        xr += .6 * tankLattice[1].tap(taps[11]);

        *wl = xl;
        *wr = xr;
    }
};

} // namespace gsynth

#endif
