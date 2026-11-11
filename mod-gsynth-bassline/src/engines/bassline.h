/*
 * MOD GSynth Creator - engine: bass synth pedal
 * Copyright (C) 2026 MOD Audio
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version. See LICENSE.
 */

/* Not derived from a pedalboard: designed after the signal flow and control set
 * of a monophonic bass-synth stompbox (main oscillator with three shapes, extra
 * harmonically tuned voices, sub octave, noise, 4-pole resonant low-pass with a
 * triggered attack/decay envelope, glide, chorus and filter LFO, dry blend),
 * and assembled from the block library. The dynamics are the GSYNTH
 * pedalboards' two chains, which is what a "compression" control on such a
 * pedal amounts to: how much the level follows the playing.
 *
 *   in -+- fil4 HP/LP - decimate - tracker -+- pitch - octave - glide -+- main VCO --+
 *       |   (three tracker set-ups:         |                          +- sub VCO  --+- noise mix -+
 *       |    5-string, 4-string, guitar)    |                          +- voice VCOs +             |
 *       |                                   +- gate - amp attack ---------+                        |
 *       +- onset detector - attack/decay envelope - amount --+            |                        |
 *       +- compressor - x10 - |x| - gate - slew - boost -----|------------+- VCA gain              |
 *       |                     (dynamic chain; "fixed" chain is the gate)  |                        |
 *       +- dry ---(Dry Thru FX: into the filter instead)------------------|---+                    |
 *                                                   LFO - Mod, Mod Blend -+   |                    |
 *   ... 24 dB low-pass (cutoff + key + envelope + LFO) - VCA - chorus (Mod, Mod Blend) - dry mix - gain - out
 */

#ifndef GSYNTH_ENGINE_BASSLINE_H
#define GSYNTH_ENGINE_BASSLINE_H

#include "../blocks/ams.h"
#include "../blocks/extras.h"
#include "../blocks/invada_compressor.h"
#include "../blocks/mod_cv.h"
#include "../blocks/pitch_tracker.h"
#include "../blocks/x42_fil4.h"

namespace gsynth {
namespace bassline {

namespace fixed {
    /* tracker set-ups: input filter, rate the analysis runs at, window and hop in analysis samples.
     * yinfast reaches down to rate / (window / 2). */
    struct Tracking { float hp, lp, lpQ, analysisRate; uint32_t window, hop; };
    constexpr Tracking kTracking[3] = {
        { 25.f,  500.f, 0.7f, 12000.f, 1024, 128 },    // 5-string bass: 85 ms window, to 23 Hz
        { 35.f,  600.f, 0.7f, 12000.f,  768,  96 },    // 4-string bass: 64 ms window, to 31 Hz, quicker
        { 70.f, 1000.f, 1.0f, 24000.f,  704, 176 },    // guitar: 29 ms window, to 68 Hz (the pedalboards' span)
    };
    constexpr float kConfidencePct = 12.5f, kTolerancePct = 10.f;      // as the pedalboards
    constexpr bool  kHoldPitch = true;

    // dynamic chain, as GSYNTH:Flexible
    constexpr InvadaCompressor::Settings kComp = { 0.5f, 0.005992f, 0.0010000000475f, -22.5f, 9.90625f, 3.84375f, true };
    constexpr float kAudioToCvLevel = 10.f, kGateOpen = 0.01f, kGateClose = 0.f;
    constexpr float kEnvRiseMs = 5.f, kEnvFallMs = 15.723438262939453f, kEnvBoost = 3.9130430221557617f;
    constexpr float kFixedLevelVolts = 6.f;         // the fixed chain's level: about what the dynamic chain gives a firm note
    constexpr float kAmpFallMs = 40.f;
    constexpr float kVcaOutputLevel = 0.240166f;
    constexpr float kMakeUp = 1.5f;
    constexpr float kResoMakeUp = 0.3f * 4.1f;      // the ladder filter loses passband level as resonance rises (feedback 4.1 q); give most of it back
    constexpr float kEnvPendingMs = 300.f;          // how long after a pick the tracker's answer still belongs to that pick
    constexpr float kNewNoteVolts = 0.03f;          // a pitch change this large (36 cents) within that time is the new note landing

    constexpr float kVcoEdge = 1.f, kVcoPw = 0.5f;
    constexpr float kVoice1Volts = 1.f, kVoice2Volts = 19.f / 12.f;    // harmonics 2 and 3 of the main oscillator
    constexpr float kSubVolts = -1.f;
    constexpr float kKeySlewMs = 15.723438262939453f, kKeyTrack = 1.f;  // x0.2 inside the VCF: 20 % key tracking
    constexpr float kEnvOctaves = 5.f, kLfoOctaves = 1.5f, kLfoHz = 5.f;
    constexpr float kMaxReso = 0.95f;
    constexpr float kChorusHz = 0.6f, kChorusBaseMs = 12.f, kChorusDepthMs = 4.f;
}

struct Params {
    // the pedal's front panel
    int   shape = 1;                // 0 triangle, 1 sawtooth, 2 square
    bool  voice = false;
    float mix = 1.f;                // dry .. synth
    float subMix = 0.3f;            // main .. sub
    float cutoff = 4.f;             // octaves above 32.7 Hz
    float resonance = 0.4f;
    float envelope = 0.35f;         // 0 = envelope off; else its time, plucky .. sweeping
    float mod = 0.f;
    // the "voice" menu
    float voiceMix = 0.5f;
    float noiseMix = 0.f;
    float envAmount = 0.6f;         // -1 .. 1: closes .. opens the filter
    float glide = 0.f;
    float ampAttack = 0.f;
    float modBlend = 0.f;           // chorus .. filter LFO
    // the "setup" menu
    float compression = 0.7f;       // level follows the playing .. every note the same
    float gainDb = 0.f;
    float sensitivity = 50.f;
    int   tracking = 0;             // 0 five-string bass, 1 four-string bass, 2 guitar
    bool  dryThruFx = false;
    int   octave = 0;
};

class Engine {
public:
    bool init(float sampleRate)
    {
        sr = sampleRate;
        bool ok = chorus.init(sr);
        for (int m = 0; m < 3; ++m) {
            const fixed::Tracking& t = fixed::kTracking[m];
            uint32_t d = (uint32_t)std::lrint(sr / t.analysisRate);
            ok = tracker[m].init(sr, t.window, t.hop, d < 1 ? 1 : d) && ok;
        }
        if (!ok)
            return false;
        reset();
        return true;
    }

    void destroy() { for (auto& t : tracker) t.destroy(); chorus.destroy(); }

    void reset()
    {
        mode = -1;
        primed = false;
        envPending = 0;
        gateWasUp = false;
        envPitch = 0.f;
        comp.setup(sr, fixed::kComp);
        gate = KeyedGate();
        onset.setup(sr);
        filterEnv.setup(sr);
        envSlew = Slew(); envSlew.setTimes(fixed::kEnvRiseMs, fixed::kEnvFallMs, sr);
        keySlew = Slew(); keySlew.setTimes(fixed::kKeySlewMs, fixed::kKeySlewMs, sr);
        glideSlew = Slew();
        ampSlew = Slew();
        main = sub = voice1 = voice2 = AmsVco3();
        lfo.setup(sr);
        vcf.setup(sr);
        chorus.reset();
        const Params p;
        sMix.setup(sr, 10.f, p.mix);         sSub.setup(sr, 10.f, p.subMix);
        sVoice.setup(sr, 10.f, 0.f);         sNoise.setup(sr, 10.f, p.noiseMix);
        sCutoff.setup(sr, 5.f, p.cutoff);    sReso.setup(sr, 5.f, p.resonance);
        sAmount.setup(sr, 5.f, p.envAmount); sMod.setup(sr, 10.f, p.mod);
        sBlend.setup(sr, 10.f, p.modBlend);  sComp.setup(sr, 10.f, p.compression);
        sGain.setup(sr, 10.f, 1.f);          sThru.setup(sr, 10.f, 0.f);
    }

    float detectedHz() const { return mode >= 0 ? tracker[mode].pitchHz : 0.f; }

    void process(const float* in, float* outL, float* outR, uint32_t n, const Params& p)
    {
        const int wantMode = p.tracking < 0 ? 0 : (p.tracking > 2 ? 2 : p.tracking);
        if (wantMode != mode) {
            mode = wantMode;
            const fixed::Tracking& t = fixed::kTracking[mode];
            trackHp.setup(sr, t.hp, 0.7f);
            trackLp.setup(sr, t.lp, t.lpQ, false);
            tracker[mode].reset();
            tracker[mode].setTolerance(fixed::kTolerancePct);
        }
        PitchTracker& trk = tracker[mode];

        static const int kWaves[3] = { AmsVco3::Triangle, AmsVco3::Sawtooth, AmsVco3::Rectangle };
        const int wave = kWaves[p.shape < 0 ? 0 : (p.shape > 2 ? 2 : p.shape)];
        const float envBoost = boosterCoef(fixed::kEnvBoost);
        const float gainTarget = std::pow(10.f, 0.05f * std::fmax(-18.f, std::fmin(18.f, p.gainDb)));
        const float onsetFloor = 0.01f * 50.f / std::fmax(0.1f, p.sensitivity);

        // Envelope: one knob for both times, 25 ms (plucky) to 2.5 s (sweeping); fully down switches it off
        const bool envOn = p.envelope > 0.005f;
        const float envMs = 25.f * std::pow(100.f, p.envelope);
        // Glide 0 is no glide at all (a slew with time 0 would never move); else 5 ms .. 600 ms
        const bool glideOn = p.glide > 0.005f;
        if (glideOn) { const float ms = 5.f * std::pow(120.f, p.glide); glideSlew.setTimes(ms, ms, sr); }
        ampSlew.setTimes(1.f * std::pow(1000.f, p.ampAttack), fixed::kAmpFallMs, sr);

        // the first block after activation starts from the controls as they are, not from the defaults
        if (!primed) {
            primed = true;
            sMix.z = p.mix; sSub.z = p.subMix; sVoice.z = p.voice ? p.voiceMix : 0.f; sNoise.z = p.noiseMix;
            sCutoff.z = p.cutoff; sReso.z = p.resonance; sAmount.z = p.envAmount; sMod.z = p.mod;
            sBlend.z = p.modBlend; sComp.z = p.compression; sGain.z = gainTarget; sThru.z = p.dryThruFx ? 1.f : 0.f;
        }

        for (uint32_t i = 0; i < n; ++i) {
            const float x = in[i];

            // pitch
            trk.process(trackLp.process(trackHp.process(x)) * p.sensitivity, p.octave, fixed::kConfidencePct, fixed::kHoldPitch);
            float pitchCv = trk.cvPitch;
            if (glideOn) pitchCv = glideSlew.process(pitchCv);
            else glideSlew.out = pitchCv;

            // dynamics: the pedalboards' dynamic chain, and the gate as the fixed one
            float dyn = std::fabs(comp.process(x) * fixed::kAudioToCvLevel);
            dyn = gate.process(dyn, trk.cvPitch, fixed::kGateOpen, fixed::kGateClose);
            dyn = envSlew.process(dyn) * envBoost;
            const float noteOn = ampSlew.process(trk.cvGate > 0.f ? 1.f : 0.f);
            const float c = sComp.process(p.compression);
            const float level = noteOn * ((1.f - c) * dyn + c * fixed::kFixedLevelVolts);

            // Filter envelope, retriggered by every picked note. A pick is heard well before the tracker has
            // its pitch. If nothing is sounding, nothing will until then, so the envelope waits for the gate:
            // a plucky one would be over before the note begins. If a note is sounding, the envelope fires at
            // once, and again when the new pitch lands some tens of milliseconds later.
            const bool gateUp = trk.cvGate > 0.f;
            if (onset.process(x, onsetFloor)) {
                envPending = (uint32_t)(0.001f * fixed::kEnvPendingMs * sr);
                if (gateUp) { filterEnv.trigger(); envPitch = trk.cvPitch; }
            }
            if (envPending > 0) {
                --envPending;
                if (gateUp && (!gateWasUp || std::fabs(trk.cvPitch - envPitch) > fixed::kNewNoteVolts)) {
                    filterEnv.trigger();
                    envPitch = trk.cvPitch;
                    envPending = 0;
                }
            }
            gateWasUp = gateUp;
            const float fenv = envOn ? filterEnv.process(envMs * 0.12f, envMs) : 0.f;

            // oscillators
            const float subMix = sSub.process(p.subMix);
            float osc = (1.f - subMix) * main.process(wave, AmsVco3::hz(pitchCv), sr, fixed::kVcoEdge, fixed::kVcoPw)
                      + subMix * sub.process(wave, AmsVco3::hz(pitchCv + fixed::kSubVolts), sr, fixed::kVcoEdge, fixed::kVcoPw);
            const float voices = sVoice.process(p.voice ? p.voiceMix : 0.f);
            if (voices > 1e-4f)
                osc += voices * 0.5f * (voice1.process(wave, AmsVco3::hz(pitchCv + fixed::kVoice1Volts), sr, fixed::kVcoEdge, fixed::kVcoPw)
                                      + voice2.process(wave, AmsVco3::hz(pitchCv + fixed::kVoice2Volts), sr, fixed::kVcoEdge, fixed::kVcoPw));
            const float noiseMix = sNoise.process(p.noiseMix);
            osc = (1.f - noiseMix) * osc + noiseMix * noise.process();

            // Dry Thru FX: the dry signal goes through filter, VCA and chorus with the synth
            const float mix = sMix.process(p.mix);
            const float thru = sThru.process(p.dryThruFx ? 1.f : 0.f);
            // the VCA and the make-up gain multiply by about 0.7 at a firm note; scale the dry signal so Mix stays a fair blend
            float s = mix * osc + thru * (1.f - mix) * x * (3.f / fixed::kMakeUp);

            // filter: cutoff + key tracking + envelope + LFO, the last two in octaves
            const float modAmt = sMod.process(p.mod), blend = sBlend.process(p.modBlend);
            const float wobble = lfo.process(AmsLfo2::Sine, fixed::kLfoHz, 0.f) * 0.2f;      // +-1
            const float octaves = fenv * sAmount.process(p.envAmount) * fixed::kEnvOctaves
                                + wobble * modAmt * blend * fixed::kLfoOctaves;
            const double hz = AmsVcf::cutoffHz(sCutoff.process(p.cutoff), keySlew.process(trk.cvPitch) * fixed::kKeyTrack,
                                               5.f, octaves, 0.f, 0.f);
            const float q = std::fmin(fixed::kMaxReso, sReso.process(p.resonance));
            s = vcf.process(AmsVcf::Lowpass24I, s * (1.f + fixed::kResoMakeUp * q), hz, q);

            // amplifier
            s *= level * 0.2f * fixed::kVcaOutputLevel * fixed::kMakeUp;
            // an open filter on a hard-picked note runs hot; round it off instead of letting the host clip it
            s = InvadaCompressor::noClip(s);

            // chorus, then whatever dry signal did not go through the effects
            float wl, wr;
            chorus.process(s, fixed::kChorusHz, fixed::kChorusBaseMs, fixed::kChorusDepthMs, &wl, &wr);
            const float ch = 0.5f * modAmt * (1.f - blend);
            const float dry = (1.f - thru) * (1.f - mix) * x;
            const float g = sGain.process(gainTarget);
            outL[i] = g * (s * (1.f - ch) + wl * ch + dry);
            outR[i] = g * (s * (1.f - ch) + wr * ch + dry);
        }

        trackHp.endBlock(); trackLp.endBlock();
        comp.endBlock();
    }

private:
    float sr = 48000.f;
    int mode = -1;
    bool primed = false, gateWasUp = false;
    uint32_t envPending = 0;
    float envPitch = 0.f;
    Fil4HighPass trackHp;
    Fil4LowPass trackLp;
    PitchTracker tracker[3];
    InvadaCompressor comp;
    KeyedGate gate;
    OnsetDetector onset;
    AdEnvelope filterEnv;
    Slew envSlew, keySlew, glideSlew, ampSlew;
    AmsVco3 main, sub, voice1, voice2;
    WhiteNoise noise;
    AmsLfo2 lfo;
    AmsVcf vcf;
    StereoChorus chorus;
    Smooth sMix, sSub, sVoice, sNoise, sCutoff, sReso, sAmount, sMod, sBlend, sComp, sGain, sThru;
};

} // namespace bassline
} // namespace gsynth

#endif
