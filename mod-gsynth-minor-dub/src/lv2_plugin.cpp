/*
 * MOD GSynth Creator - LV2 plugin wrapper, shared by every generated synth
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* Port layout: 0 audio in, 1 and 2 audio out, then the synth's controls in the
 * order of its spec, then the detected pitch output. Everything that differs
 * between synths comes from the generated synth_config.h. */

#include "synth_config.h"
#include "blocks/denormals.h"

#include <lv2/core/lv2.h>

#include <new>

namespace {

using namespace gsynth;

enum { kPortIn = 0, kPortOutL, kPortOutR, kPortFirstControl,
       kPortDetectedPitch = kPortFirstControl + synth::kControlCount, kPortCount };

struct Plugin {
    synth::Engine engine;
    const float* in = nullptr;
    float* outL = nullptr;
    float* outR = nullptr;
    const float* control[synth::kControlCount] = {};
    float* detectedPitch = nullptr;
};

LV2_Handle instantiate(const LV2_Descriptor*, double rate, const char*, const LV2_Feature* const*)
{
    Plugin* const self = new (std::nothrow) Plugin();
    if (self == nullptr)
        return nullptr;
    if (!self->engine.init((float)rate)) {
        self->engine.destroy();
        delete self;
        return nullptr;
    }
    return self;
}

void connect_port(LV2_Handle instance, uint32_t port, void* data)
{
    Plugin* const self = (Plugin*)instance;
    if (port == kPortIn) self->in = (const float*)data;
    else if (port == kPortOutL) self->outL = (float*)data;
    else if (port == kPortOutR) self->outR = (float*)data;
    else if (port == kPortDetectedPitch) self->detectedPitch = (float*)data;
    else if (port < kPortDetectedPitch) self->control[port - kPortFirstControl] = (const float*)data;
}

void activate(LV2_Handle instance)
{
    ((Plugin*)instance)->engine.reset();
}

void run(LV2_Handle instance, uint32_t n)
{
    Plugin* const self = (Plugin*)instance;
    const ScopedFlushDenormals flush;

    float c[synth::kControlCount];
    for (int i = 0; i < synth::kControlCount; ++i) {
        const float v = self->control[i] ? *self->control[i] : synth::kDefaults[i];
        c[i] = v < synth::kMinimum[i] ? synth::kMinimum[i] : (v > synth::kMaximum[i] ? synth::kMaximum[i] : v);
    }

    synth::Params p;
    synth::voicing(p);
    synth::controls(p, c);
    self->engine.process(self->in, self->outL, self->outR, n, p);

    if (self->detectedPitch != nullptr)
        *self->detectedPitch = self->engine.detectedHz();
}

void cleanup(LV2_Handle instance)
{
    Plugin* const self = (Plugin*)instance;
    self->engine.destroy();
    delete self;
}

const LV2_Descriptor descriptor = {
    GSYNTH_URI, instantiate, connect_port, activate, run, nullptr, cleanup, nullptr
};

} // namespace

LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor(uint32_t index)
{
    return index == 0 ? &descriptor : nullptr;
}
