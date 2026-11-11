/*
 * MOD GSynth Creator - engine: GSYNTH:Minimal
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* The GSYNTH:Minimal pedalboard (pedalboards/factory/GSYNTH_Minimal.pedalboard):
 *
 *   in - fil4 HP 80 / LP 1k - tracker -+- pitch ---------------- VCO3
 *    |                                 +- pitch, slew ---------- VCF key tracking
 *    |                                 +- gate, slew, boost ---+
 *    +- compressor - x10 - |x| - gate - slew - boost -+--------+- VCA gain
 *                                                     +- DYN>VCF - VCF cutoff (linear)
 *                                  FLTR > - slew ------------------ VCF cutoff (exponential)
 *   VCO3 - VCF - VCA - low-pass 7 kHz - Plate - level - out
 */

#ifndef GSYNTH_ENGINE_MINIMAL_H
#define GSYNTH_ENGINE_MINIMAL_H

#include "../blocks/ams.h"
#include "../blocks/caps_plate.h"
#include "../blocks/invada_compressor.h"
#include "../blocks/mod_cv.h"
#include "../blocks/mod_lowpass.h"
#include "../blocks/pitch_tracker.h"
#include "../blocks/x42_fil4.h"

namespace gsynth {
namespace minimal {

/* what the pedalboard holds fixed */
namespace fixed {
    constexpr float kTrackHpFreq = 80.f, kTrackHpQ = 0.7f, kTrackLpFreq = 1000.f, kTrackLpQ = 1.0f;
    constexpr float kConfidencePct = 12.5f, kTolerancePct = 10.f;
    constexpr bool  kHoldPitch = true;
    constexpr InvadaCompressor::Settings kComp = { 0.5f, 0.005992f, 0.0010000000475f, -22.5f, 9.90625f, 3.84375f, true };
    constexpr float kAudioToCvLevel = 10.f;
    constexpr float kGateOpen = 0.01f, kGateClose = 0.f;
    constexpr float kEnvRiseMs = 1.f, kEnvFallMs = 31.34687500144355f;
    constexpr float kEnvBoost = 3.9130430221557617f;            // booster
    constexpr float kNoteRiseMs = 4.f, kNoteFallMs = 14.591303825378418f;
    constexpr float kFixVca = 1.4f;                             // booster
    constexpr float kDynVca = 1.f;                              // attenuverter
    constexpr float kVcoEdge = 1.f, kVcoPw = 0.5f;
    constexpr float kKeySlewMs = 15.723438262939453f;
    constexpr float kVcfExpFmGain = 2.34375f, kVcfLinFmGain = 1.f;
    constexpr float kSweepCv = 10.f;
    constexpr float kVcaOutputLevel = 0.24016599357128143f;
    constexpr float kToneFreq = 7000.f;
    constexpr float kPlateBandwidth = 1.f, kPlateDamping = 0.375f;
}

/* what it had on knobs and footswitches; defaults are the loaded pedalboard */
struct Params {
    float sensitivity = 50.04999923706055f;
    int   octave = -1;
    int   waveform = AmsVco3::Sawtooth;
    float filterFreq = 2.f;
    float filterReso = 0.871f;
    float filterDepth = 6.5f;               // DYN>VCF, booster
    bool  sweep = false;                    // FLTR >
    float sweepTimeMs = 710.173584f;        // FILTER TIME
    bool  reverb = true;
    float reverbMix = 0.19475446428571436f;
    float reverbDecay = 0.5f;
    float levelDb = 0.f;
};

class Engine {
public:
    bool init(float sampleRate)
    {
        sr = sampleRate;
        if (!tracker.init(sr) || !plate.init(sr))
            return false;
        reset();
        return true;
    }

    void destroy() { tracker.destroy(); plate.destroy(); }

    void reset()
    {
        trackHp.setup(sr, fixed::kTrackHpFreq, fixed::kTrackHpQ);
        trackLp.setup(sr, fixed::kTrackLpFreq, fixed::kTrackLpQ, false);
        tracker.reset();
        tracker.setTolerance(fixed::kTolerancePct);
        comp.setup(sr, fixed::kComp);
        gate = KeyedGate();
        envSlew = Slew(); envSlew.setTimes(fixed::kEnvRiseMs, fixed::kEnvFallMs, sr);
        noteSlew = Slew(); noteSlew.setTimes(fixed::kNoteRiseMs, fixed::kNoteFallMs, sr);
        keySlew = Slew(); keySlew.setTimes(fixed::kKeySlewMs, fixed::kKeySlewMs, sr);
        sweepSlew = Slew();
        vco = AmsVco3();
        vcf.setup(sr);
        tone.setup(sr, fixed::kToneFreq);
        plate.setTone(fixed::kPlateBandwidth, fixed::kPlateDamping);
        plate.reset();
        const Params p;
        sFreq.setup(sr, 5.f, p.filterFreq);
        sReso.setup(sr, 5.f, p.filterReso);
        sDepth.setup(sr, 5.f, boosterCoef(p.filterDepth));
        sMix.setup(sr, 10.f, p.reverb ? std::pow(p.reverbMix, 1.6f) : 0.f);
        sSend.setup(sr, 10.f, p.reverb ? 1.f : 0.f);
        sDecay.setup(sr, 10.f, p.reverbDecay);
        sLevel.setup(sr, 10.f, 1.f);
    }

    float detectedHz() const { return tracker.pitchHz; }

    void process(const float* in, float* outL, float* outR, uint32_t n, const Params& p)
    {
        const float envBoost = boosterCoef(fixed::kEnvBoost);
        const float noteBoost = boosterCoef(fixed::kFixVca);
        const float depthTarget = boosterCoef(p.filterDepth);
        const float mixTarget = p.reverb ? std::pow(p.reverbMix, 1.6f) : 0.f;
        const float sendTarget = p.reverb ? 1.f : 0.f;
        const float levelTarget = std::pow(10.f, 0.05f * std::fmax(-20.f, std::fmin(20.f, p.levelDb)));
        const float sweepTarget = p.sweep ? fixed::kSweepCv : 0.f;
        sweepSlew.setTimes(p.sweepTimeMs, p.sweepTimeMs, sr);

        for (uint32_t i = 0; i < n; ++i) {
            const float x = in[i];

            tracker.process(trackLp.process(trackHp.process(x)) * p.sensitivity, p.octave,
                            fixed::kConfidencePct, fixed::kHoldPitch);
            const float pitchCv = tracker.cvPitch;

            float env = std::fabs(comp.process(x) * fixed::kAudioToCvLevel);
            env = gate.process(env, pitchCv, fixed::kGateOpen, fixed::kGateClose);
            env = envSlew.process(env) * envBoost;
            const float note = noteSlew.process(tracker.cvGate) * noteBoost;

            float s = vco.process(p.waveform, AmsVco3::hz(pitchCv), sr, fixed::kVcoEdge, fixed::kVcoPw);
            const double hz = AmsVcf::cutoffHz(sFreq.process(p.filterFreq), keySlew.process(pitchCv),
                                               fixed::kVcfExpFmGain, sweepSlew.process(sweepTarget),
                                               fixed::kVcfLinFmGain, env * sDepth.process(depthTarget));
            s = vcf.process(AmsVcf::ResonantLowpass, s, hz, sReso.process(p.filterReso));
            s *= (env * fixed::kDynVca + note) * 0.2f * fixed::kVcaOutputLevel;
            s = tone.process(s);

            float wl, wr;
            plate.process(s * sSend.process(sendTarget), sDecay.process(p.reverbDecay), &wl, &wr);
            const float blend = sMix.process(mixTarget);
            const float level = sLevel.process(levelTarget);
            outL[i] = level * ((1.f - blend) * s + blend * wl);
            outR[i] = level * ((1.f - blend) * s + blend * wr);
        }

        trackHp.endBlock();
        trackLp.endBlock();
        comp.endBlock();
        tone.endBlock();
    }

private:
    float sr = 48000.f;
    Fil4HighPass trackHp;
    Fil4LowPass trackLp;
    PitchTracker tracker;
    InvadaCompressor comp;
    KeyedGate gate;
    Slew envSlew, noteSlew, keySlew, sweepSlew;
    AmsVco3 vco;
    AmsVcf vcf;
    ModLowPass3 tone;
    CapsPlate plate;
    Smooth sFreq, sReso, sDepth, sMix, sSend, sDecay, sLevel;
};

} // namespace minimal
} // namespace gsynth

#endif
