/*
 * MOD GSynth Creator - engine: GSYNTH:Flexible
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* The GSYNTH:Flexible pedalboard (pedalboards/factory/GSYNTH_Flexible.pedalboard),
 * the documented reference design of the family. Names in capitals are the ones
 * its author put on the hardware and in the notes on the pedalboard.
 *
 *  Keytracking chain
 *   in - fil4 HP 80 / LP 1k - tracker -+- pitch - range --+---------- VCO 1, 2, 3 pitch
 *                                      |      LFO 1 - LFO>VCO -+
 *                                      +- pitch - slew - KEY>VCF -+-- VCF cutoff (key)
 *                                      |          LFO 2 - LFO>VCF -+
 *  Fixed Velocity chain                +- gate - slew - FIX>VCA ---+
 *  Dynamic chain (envelope follower)                               +- VCA gain
 *   in - compressor (CMP DYN) - x10 - |x| - gate - slew - boost -+- DYN>VCA
 *                                                                +- DYN>VCF -- VCF cutoff (linear)
 *                      FLTR > (+10 V) and FLTR < (-20 V) - slew (FLT TIME) -- VCF cutoff (exponential)
 *
 *   VCO 1 + (VCO 2 + VCO 3, -5 dB: VCO2+3) - VCF - VCA - fil4 HP 30 / LP 8k - Modulay - Plate - level - out
 */

#ifndef GSYNTH_ENGINE_FLEXIBLE_H
#define GSYNTH_ENGINE_FLEXIBLE_H

#include "../blocks/ams.h"
#include "../blocks/caps_plate.h"
#include "../blocks/invada_compressor.h"
#include "../blocks/mod_cv.h"
#include "../blocks/pitch_tracker.h"
#include "../blocks/shiro_modulay.h"
#include "../blocks/x42_fil4.h"

namespace gsynth {
namespace flexible {

/* what the pedalboard holds fixed */
namespace fixed {
    constexpr float kTrackHpFreq = 80.f, kTrackHpQ = 0.7f, kTrackLpFreq = 1000.f, kTrackLpQ = 1.0f;
    constexpr bool  kHoldPitch = true;
    constexpr float kRangeMin = 0.414079f, kRangeMax = 9.02692f;   // mod-cv-range, reset to zero
    constexpr InvadaCompressor::Settings kComp = { 0.5f, 0.005992f, 0.0010000000475f, -22.5f, 9.90625f, 3.84375f, true };
    constexpr float kAudioToCvLevel = 10.f;
    constexpr float kGateOpen = 0.01f, kGateClose = 0.f;
    constexpr float kEnvRiseMs = 5.f, kEnvFallMs = 15.723438262939453f;
    constexpr float kEnvBoost = 3.9130430221557617f;                // booster
    constexpr float kNoteRiseMs = 4.f, kNoteFallMs = 14.591303825378418f;
    constexpr float kKeySlewMs = 15.723438262939453f;
    constexpr float kLfoVcoPreGain = 0.1f;                          // the attenuverter ahead of LFO>VCO
    constexpr float kVcoEdge = 1.f, kVcoPw = 0.5f;
    constexpr int   kVco23Wave = AmsVco3::Sawtooth;
    constexpr float kVco23TrimDb = -5.f;                            // MOD Gain, switched in with VCO 2+3
    constexpr float kVcfExpFmGain = 2.34375f, kVcfLinFmGain = 1.f;
    constexpr float kSweepUpCv = 10.f, kSweepDownCv = -20.f;        // Control to CV 10 V; and the same through x -2
    constexpr float kVcaOutputLevel = 0.240166f;
    constexpr float kPostHpFreq = 30.f, kPostHpQ = 0.7f, kPostLpFreq = 8000.f, kPostLpQ = 0.91875f;
    constexpr float kDelayRateHz = 2.f, kDelayDepthMs = 0.358031f, kDelayMorph = 50.f, kDelayToneHz = 3223.71f;
}

/* what it had on knobs and footswitches, in the pedalboard's own units;
 * defaults are the pedalboard as it loads (snapshot "Acid Saw Bass") */
struct Params {
    // page 1: VCO
    int   octave = -1;              // MSTR OCT
    int   vco1Wave = AmsVco3::Sawtooth;
    bool  vco23 = false;            // VCO2+3
    int   vco2Oct = 0, vco2Semi = 7;
    int   vco3Oct = 0, vco3Semi = 4;
    bool  lfoVco = false;           // LFO>VCO footswitch
    int   lfoVcoWave = AmsLfo2::Sine;
    float lfoVcoFreq = 6.5f;
    float lfoVcoDepth = 0.1f;       // LFO>VCO knob, attenuverter
    // page 2: VCF
    int   vcfType = AmsVcf::ResonantLowpass;
    float vcfFreq = 2.f;
    float vcfReso = 0.876f;
    float dynVcf = 6.45f;           // booster
    float keyVcf = 1.f;             // attenuverter
    int   lfoVcfWave = AmsLfo2::Sine;
    float lfoVcfFreq = 0.8f;
    float lfoVcfDepth = 0.075f;     // attenuverter
    bool  sweepUp = false;          // FLTR >
    bool  sweepDown = false;        // FLTR <
    float sweepTimeMs = 595.04f;    // FLT TIME
    // page 3: VCA / dynamics
    float fixVca = 1.455f;          // booster
    float dynVca = 1.f;             // attenuverter
    bool  compDyn = true;           // CMP DYN
    float sensitivity = 50.05f;
    float confidence = 12.5f;
    float tolerance = 10.f;
    float levelDb = 0.f;
    // page 4: FX
    bool  delay = false;
    float delayTimeMs = 485.5f, delayRepeats = 48.4f, delayMix = 30.f;
    bool  reverb = true;
    float reverbDecay = 0.655f, reverbMix = 0.195f, reverbDamping = 0.375f, reverbBandwidth = 1.f;
};

class Engine {
public:
    bool init(float sampleRate)
    {
        sr = sampleRate;
        if (!tracker.init(sr) || !plate.init(sr) || !delay.init(sr))
            return false;
        reset();
        return true;
    }

    void destroy() { tracker.destroy(); plate.destroy(); delay.destroy(); }

    void reset()
    {
        trackHp.setup(sr, fixed::kTrackHpFreq, fixed::kTrackHpQ);
        trackLp.setup(sr, fixed::kTrackLpFreq, fixed::kTrackLpQ, false);
        postHp.setup(sr, fixed::kPostHpFreq, fixed::kPostHpQ);
        postLp.setup(sr, fixed::kPostLpFreq, fixed::kPostLpQ, true);
        tracker.reset();
        range = CvRange();
        comp.setup(sr, fixed::kComp);
        gate = KeyedGate();
        envSlew = Slew(); envSlew.setTimes(fixed::kEnvRiseMs, fixed::kEnvFallMs, sr);
        noteSlew = Slew(); noteSlew.setTimes(fixed::kNoteRiseMs, fixed::kNoteFallMs, sr);
        keySlew = Slew(); keySlew.setTimes(fixed::kKeySlewMs, fixed::kKeySlewMs, sr);
        sweepSlew = Slew();
        vco1 = vco2 = vco3 = AmsVco3();
        lfoVco.setup(sr);
        lfoVcf.setup(sr);
        vcf.setup(sr);
        delay.reset();
        plate.reset();
        const Params p;
        sFreq.setup(sr, 5.f, p.vcfFreq);
        sReso.setup(sr, 5.f, p.vcfReso);
        sDynVcf.setup(sr, 5.f, boosterCoef(p.dynVcf));
        sKeyVcf.setup(sr, 5.f, p.keyVcf);
        sLfoVcf.setup(sr, 5.f, p.lfoVcfDepth);
        sLfoVco.setup(sr, 5.f, p.lfoVco ? p.lfoVcoDepth : 0.f);
        sFixVca.setup(sr, 5.f, boosterCoef(p.fixVca));
        sDynVca.setup(sr, 5.f, p.dynVca);
        sVco23.setup(sr, 10.f, p.vco23 ? 1.f : 0.f);
        sDelay.setup(sr, 10.f, p.delay ? 1.f : 0.f);
        sMix.setup(sr, 10.f, p.reverb ? std::pow(p.reverbMix, 1.6f) : 0.f);
        sSend.setup(sr, 10.f, p.reverb ? 1.f : 0.f);
        sDecay.setup(sr, 10.f, p.reverbDecay);
        sLevel.setup(sr, 10.f, 1.f);
    }

    float detectedHz() const { return tracker.pitchHz; }

    void process(const float* in, float* outL, float* outR, uint32_t n, const Params& p)
    {
        const float envBoost = boosterCoef(fixed::kEnvBoost);
        const float trim = std::pow(10.f, 0.05f * fixed::kVco23TrimDb);
        const float levelTarget = std::pow(10.f, 0.05f * std::fmax(-20.f, std::fmin(20.f, p.levelDb)));
        const float sweepTarget = (p.sweepUp ? fixed::kSweepUpCv : 0.f) + (p.sweepDown ? fixed::kSweepDownCv : 0.f);
        const float mixTarget = p.reverb ? std::pow(p.reverbMix, 1.6f) : 0.f;
        const float vco2Volts = p.vco2Oct + p.vco2Semi / 12.f, vco3Volts = p.vco3Oct + p.vco3Semi / 12.f;
        const ShiroModulay::Settings echo = { p.delayTimeMs, p.delayRepeats, p.delayMix, fixed::kDelayRateHz,
                                              fixed::kDelayDepthMs, fixed::kDelayMorph, fixed::kDelayToneHz };
        sweepSlew.setTimes(p.sweepTimeMs, p.sweepTimeMs, sr);
        tracker.setTolerance(p.tolerance);
        plate.setTone(p.reverbBandwidth, p.reverbDamping);

        for (uint32_t i = 0; i < n; ++i) {
            const float x = in[i];

            // keytracking chain
            tracker.process(trackLp.process(trackHp.process(x)) * p.sensitivity, p.octave, p.confidence, fixed::kHoldPitch);
            const float pitchCv = tracker.cvPitch, gateCv = tracker.cvGate;
            const float vibrato = lfoVco.process(p.lfoVcoWave, p.lfoVcoFreq, gateCv) * fixed::kLfoVcoPreGain
                                * sLfoVco.process(p.lfoVco ? p.lfoVcoDepth : 0.f);
            const float vcoCv = range.process(pitchCv, fixed::kRangeMin, fixed::kRangeMax, true) + vibrato;

            // dynamic chain and fixed velocity chain
            float env = std::fabs((p.compDyn ? comp.process(x) : x) * fixed::kAudioToCvLevel);
            env = gate.process(env, pitchCv, fixed::kGateOpen, fixed::kGateClose);
            env = envSlew.process(env) * envBoost;
            const float note = noteSlew.process(gateCv);

            // oscillators
            float s = vco1.process(p.vco1Wave, AmsVco3::hz(vcoCv), sr, fixed::kVcoEdge, fixed::kVcoPw);
            const float on23 = sVco23.process(p.vco23 ? 1.f : 0.f);
            if (on23 > 1e-4f) {
                s += on23 * (vco2.process(fixed::kVco23Wave, AmsVco3::hz(vcoCv + vco2Volts), sr, fixed::kVcoEdge, fixed::kVcoPw)
                           + vco3.process(fixed::kVco23Wave, AmsVco3::hz(vcoCv + vco3Volts), sr, fixed::kVcoEdge, fixed::kVcoPw));
                s *= 1.f + on23 * (trim - 1.f);
            } else {
                vco2.advance(AmsVco3::hz(vcoCv + vco2Volts), sr);
                vco3.advance(AmsVco3::hz(vcoCv + vco3Volts), sr);
            }

            // filter
            const float keyCv = keySlew.process(pitchCv) * sKeyVcf.process(p.keyVcf)
                              + lfoVcf.process(p.lfoVcfWave, p.lfoVcfFreq, gateCv) * sLfoVcf.process(p.lfoVcfDepth);
            const double hz = AmsVcf::cutoffHz(sFreq.process(p.vcfFreq), keyCv, fixed::kVcfExpFmGain,
                                               sweepSlew.process(sweepTarget), fixed::kVcfLinFmGain,
                                               env * sDynVcf.process(boosterCoef(p.dynVcf)));
            s = vcf.process(p.vcfType, s, hz, sReso.process(p.vcfReso));

            // amplifier
            s *= (env * sDynVca.process(p.dynVca) + note * sFixVca.process(boosterCoef(p.fixVca))) * 0.2f * fixed::kVcaOutputLevel;
            s = postLp.process(postHp.process(s));

            // delay: dry + echoes; switched off it passes the signal and runs on silence
            const float delayOn = sDelay.process(p.delay ? 1.f : 0.f);
            const float send = s * delayOn;
            s += (delay.process(send, echo) - send) * delayOn;

            // reverb and output
            float wl, wr;
            plate.process(s * sSend.process(p.reverb ? 1.f : 0.f), sDecay.process(p.reverbDecay), &wl, &wr);
            const float blend = sMix.process(mixTarget);
            const float level = sLevel.process(levelTarget);
            outL[i] = level * ((1.f - blend) * s + blend * wl);
            outR[i] = level * ((1.f - blend) * s + blend * wr);
        }

        trackHp.endBlock(); trackLp.endBlock();
        postHp.endBlock(); postLp.endBlock();
        comp.endBlock();
    }

private:
    float sr = 48000.f;
    Fil4HighPass trackHp, postHp;
    Fil4LowPass trackLp, postLp;
    PitchTracker tracker;
    CvRange range;
    InvadaCompressor comp;
    KeyedGate gate;
    Slew envSlew, noteSlew, keySlew, sweepSlew;
    AmsVco3 vco1, vco2, vco3;
    AmsLfo2 lfoVco, lfoVcf;
    AmsVcf vcf;
    ShiroModulay delay;
    CapsPlate plate;
    Smooth sFreq, sReso, sDynVcf, sKeyVcf, sLfoVcf, sLfoVco, sFixVca, sDynVca, sVco23, sDelay, sMix, sSend, sDecay, sLevel;
};

} // namespace flexible
} // namespace gsynth

#endif
