/*
 * MOD GSynth - offline test host
 * Copyright (C) 2026 MOD Audio
 * GPL-3.0-or-later, see LICENSE.
 *
 * Loads a built synth the way a host does, plays a run of synthetic plucked
 * notes into it in 128-frame blocks and checks what comes out. Works for any
 * generated synth: the controls come from the controls.txt written next to it.
 *
 *   test_host <synth.so> <controls.txt> [out.wav] [--chords] [--bass] [symbol=value ...]
 *
 * --chords: the synth plays more than one pitch per note, skip the tuning check.
 * --bass:   play a bass instead of a guitar: low B (30.9 Hz) to D3, with the weak
 *           fundamental a real bass string has.
 * symbol=value: set a control to something other than its default.
 *
 * Exit status is the number of failed checks.
 */

#include <lv2/core/lv2.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <string>
#include <vector>

namespace {

constexpr double kRate = 48000.0;
constexpr uint32_t kBlock = 128;

enum { kIn, kOutL, kOutR, kFirstControl };

struct Control { std::string symbol; float def, min, max; };

int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++failures;
}

/* a plucked-string stand-in: harmonic series with faster decay up the series */
double fundamentalLevel = 1.0;

void pluck(std::vector<float>& buf, size_t start, size_t len, double hz, double amp)
{
    for (size_t i = 0; i < len && start + i < buf.size(); ++i) {
        const double t = i / kRate;
        double s = 0;
        for (int h = 1; h <= 8; ++h)
            s += (h == 1 ? fundamentalLevel : 1.0) * std::sin(2 * M_PI * hz * h * t) * std::exp(-t * (1.5 + 1.2 * h)) / h;
        const double attack = std::fmin(1.0, t / 0.002);
        buf[start + i] += (float)(amp * attack * s);
    }
}

/* The synth's pitch, given the note that was played: normalised autocorrelation, searched only within
 * 3 % of the expected period and of its octaves (the synths transpose by whole octaves), the best of those
 * refined by a parabola. Searching the whole lag range instead is fragile: multiples of the period score
 * almost alike, and a difference 75 dB down (another compiler flag) was enough to flip the choice. */
double fundamental(const float* x, size_t n, double noteHz)
{
    auto corr = [&](size_t lag) {
        double acc = 0, e0 = 0, e1 = 0;
        for (size_t i = 0; i + lag < n; ++i) {
            acc += (double)x[i] * x[i + lag];
            e0 += (double)x[i] * x[i];
            e1 += (double)x[i + lag] * x[i + lag];
        }
        return acc / (std::sqrt(e0 * e1) + 1e-30);
    };
    double bestR = -2, bestHz = 0;
    for (int octave = 3; octave >= -3; --octave) {                 // highest first: equal scores go to the shorter period
        const double period = kRate / (noteHz * std::pow(2.0, octave));
        const size_t lo = (size_t)(period * 0.97), hi = (size_t)(period * 1.03) + 1;
        if (lo < 8 || hi + 1 >= n / 2) continue;
        size_t best = lo; double rBest = -2;
        for (size_t lag = lo; lag <= hi; ++lag) { const double r = corr(lag); if (r > rBest) { rBest = r; best = lag; } }
        if (rBest > bestR + 0.01) {
            const double a = corr(best - 1), c = corr(best + 1), denom = a - 2 * rBest + c;
            double shift = denom < -1e-12 ? 0.5 * (a - c) / denom : 0.0;
            if (shift > 0.5) shift = 0.5; else if (shift < -0.5) shift = -0.5;
            bestR = rBest;
            bestHz = kRate / ((double)best + shift);
        }
    }
    return bestHz;
}

void writeWav(const char* path, const std::vector<float>& l, const std::vector<float>& r)
{
    FILE* f = std::fopen(path, "wb");
    if (!f) return;
    const uint32_t frames = (uint32_t)l.size(), dataBytes = frames * 4, riff = 36 + dataBytes;
    const uint32_t fmtLen = 16, rate = (uint32_t)kRate, byteRate = rate * 4;
    const uint16_t pcm = 1, ch = 2, align = 4, bits = 16;
    std::fwrite("RIFF", 1, 4, f); std::fwrite(&riff, 4, 1, f); std::fwrite("WAVEfmt ", 1, 8, f);
    std::fwrite(&fmtLen, 4, 1, f); std::fwrite(&pcm, 2, 1, f); std::fwrite(&ch, 2, 1, f);
    std::fwrite(&rate, 4, 1, f); std::fwrite(&byteRate, 4, 1, f); std::fwrite(&align, 2, 1, f);
    std::fwrite(&bits, 2, 1, f); std::fwrite("data", 1, 4, f); std::fwrite(&dataBytes, 4, 1, f);
    for (uint32_t i = 0; i < frames; ++i) {
        const float s[2] = { l[i], r[i] };
        for (float v : s) {
            v = v > 1.f ? 1.f : (v < -1.f ? -1.f : v);
            const int16_t q = (int16_t)std::lrint(v * 32767.f);
            std::fwrite(&q, 2, 1, f);
        }
    }
    std::fclose(f);
}

} // namespace

int main(int argc, char** argv)
{
    bool chords = false, bass = false;
    std::vector<const char*> args;
    std::vector<std::string> overrides;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--chords") == 0) chords = true;
        else if (std::strcmp(argv[i], "--bass") == 0) bass = true;
        else if (std::strchr(argv[i], '=') != nullptr && i > 2) overrides.push_back(argv[i]);
        else args.push_back(argv[i]);
    }
    if (bass) fundamentalLevel = 0.35;
    if (args.size() < 2) {
        std::fprintf(stderr, "usage: %s <synth.so> <controls.txt> [out.wav] [--chords]\n", argv[0]);
        return 99;
    }

    std::vector<Control> controls;
    {
        std::ifstream f(args[1]);
        Control c;
        while (f >> c.symbol >> c.def >> c.min >> c.max) controls.push_back(c);
    }
    if (controls.empty()) { std::fprintf(stderr, "no controls in %s\n", args[1]); return 99; }
    for (const std::string& o : overrides) {
        const size_t eq = o.find('=');
        bool found = false;
        for (Control& c : controls)
            if (c.symbol == o.substr(0, eq)) { c.def = (float)std::atof(o.c_str() + eq + 1); found = true; }
        if (!found) { std::fprintf(stderr, "no control named %s\n", o.substr(0, eq).c_str()); return 99; }
    }
    const uint32_t nctl = (uint32_t)controls.size(), kDetected = kFirstControl + nctl;
    auto find = [&](const char* a, const char* b) { for (uint32_t i = 0; i < nctl; ++i) if (controls[i].symbol == a || controls[i].symbol == b) return (int)i; return -1; };
    const int sweepPort = find("sweep", "sweep_up"), reverbPort = find("reverb", "space");

    void* const lib = dlopen(args[0], RTLD_NOW | RTLD_LOCAL);
    if (!lib) { std::fprintf(stderr, "dlopen: %s\n", dlerror()); return 99; }
    const auto entry = (const LV2_Descriptor* (*)(uint32_t))dlsym(lib, "lv2_descriptor");
    const LV2_Descriptor* const d = entry ? entry(0) : nullptr;
    if (!d) { std::fprintf(stderr, "no descriptor\n"); return 99; }
    std::printf("plugin: %s, %u controls\n", d->URI, nctl);

    const LV2_Feature* const features[] = { nullptr };
    LV2_Handle h = d->instantiate(d, kRate, "", features);
    if (!h) { std::fprintf(stderr, "instantiate failed\n"); return 99; }

    std::vector<float> ctl(nctl + 1, 0.f);
    for (uint32_t i = 0; i < nctl; ++i) {
        ctl[i] = controls[i].def;
        d->connect_port(h, kFirstControl + i, &ctl[i]);
    }
    d->connect_port(h, kDetected, &ctl[nctl]);

    // guitar: E2 A2 D3 G3 B3 E4; bass: B0 E1 A1 D2 G2 D3. One second each, then a held note, then silence
    const double guitarNotes[] = { 82.41, 110.0, 146.83, 196.0, 246.94, 329.63 };
    const double bassNotes[] = { 30.87, 41.20, 55.0, 73.42, 98.0, 146.83 };
    const double* const notes = bass ? bassNotes : guitarNotes;
    const size_t nNotes = 6;
    const size_t sec = (size_t)kRate;
    const size_t total = ((nNotes + 3 + 5) * sec / kBlock) * kBlock;
    std::vector<float> in(total, 0.f), outL(total, 0.f), outR(total, 0.f);
    for (size_t i = 0; i < nNotes; ++i)
        pluck(in, i * sec, sec, notes[i], 0.25);
    pluck(in, nNotes * sec, 3 * sec, bass ? 55.0 : 110.0, 0.25);

    std::vector<float> detectedAt(nNotes, 0.f);
    std::vector<double> latencyMs(nNotes, -1.0);
    if (d->activate) d->activate(h);
    const auto t0 = std::chrono::steady_clock::now();
    double worstBlock = 0;
    std::vector<float> blockUs;
    blockUs.reserve(total / kBlock);
    for (size_t pos = 0; pos < total; pos += kBlock) {
        // in-place on the left channel, as hosts are allowed to do
        std::memcpy(&outL[pos], &in[pos], kBlock * sizeof(float));
        d->connect_port(h, kIn, &outL[pos]);
        d->connect_port(h, kOutL, &outL[pos]);
        d->connect_port(h, kOutR, &outR[pos]);
        const auto b0 = std::chrono::steady_clock::now();
        d->run(h, kBlock);
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - b0).count();
        if (us > worstBlock) worstBlock = us;
        blockUs.push_back((float)us);
        for (size_t i = 0; i < nNotes; ++i) {
            if (pos <= i * sec + sec / 2 && i * sec + sec / 2 < pos + kBlock)
                detectedAt[i] = ctl[nctl];
            // how long after the pick the tracker first reports this note (within 30 cents)
            if (latencyMs[i] < 0 && pos >= i * sec && pos < (i + 1) * sec && ctl[nctl] > 0
                && std::fabs(1200 * std::log2(ctl[nctl] / notes[i])) < 30)
                latencyMs[i] = 1000.0 * (pos + kBlock - i * sec) / kRate;
        }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    std::printf("\ntracking%s:\n", chords ? "" : " and tuning (the synth must land on the played note, some octaves away)");
    for (size_t i = 0; i < nNotes; ++i) {
        const double trackedCents = detectedAt[i] > 0 ? 1200 * std::log2(detectedAt[i] / notes[i]) : 9999;
        char msg[200];
        if (chords) {
            std::snprintf(msg, sizeof msg, "%7.2f Hz in: tracked %7.2f Hz (%+5.1f c) after %3.0f ms", notes[i], detectedAt[i], trackedCents, latencyMs[i]);
            check(std::fabs(trackedCents) < 15, msg);
            continue;
        }
        const double f0 = fundamental(&outL[i * sec + sec / 4], sec / 2, notes[i]);
        const double octaves = std::log2(f0 / notes[i]), cents = 1200 * (octaves - std::round(octaves));
        std::snprintf(msg, sizeof msg, "%7.2f Hz in: tracked %7.2f Hz (%+5.1f c) after %3.0f ms, synth %7.2f Hz = %+.0f oct %+5.1f c",
                      notes[i], detectedAt[i], trackedCents, latencyMs[i], f0, std::round(octaves), cents);
        check(std::fabs(trackedCents) < 15 && std::fabs(cents) < 25, msg);
    }

    std::printf("\nsignal:\n");
    bool finite = true;
    float peak = 0, diff = 0;
    for (size_t i = 0; i < total; ++i) {
        if (!std::isfinite(outL[i]) || !std::isfinite(outR[i])) finite = false;
        peak = std::fmax(peak, std::fmax(std::fabs(outL[i]), std::fabs(outR[i])));
        diff = std::fmax(diff, std::fabs(outL[i] - outR[i]));
    }
    char msg[200];
    check(finite, "no NaN or Inf");
    std::snprintf(msg, sizeof msg, "peak %.3f (%.1f dBFS), audible and below full scale", peak, 20 * std::log10(peak + 1e-30));
    check(peak > 0.01f && peak < 1.0f, msg);
    if (reverbPort >= 0) {
        std::snprintf(msg, sizeof msg, "reverb makes L and R differ (max |L-R| %.4f)", diff);
        check(diff > 1e-4f, msg);
    }
    {
        // information for A/B runs with symbol=value: stereo width, how close the output is to the input,
        // and how bright the first 60 ms of each note are against its next 300 ms (a filter envelope shows here)
        float dry = 0;
        for (size_t i = 0; i < total; ++i) dry = std::fmax(dry, std::fabs(outL[i] - in[i]));
        auto bright = [&](size_t a, size_t n) { double e = 0, de = 0; for (size_t i = a + 1; i < a + n; ++i) { e += (double)outL[i] * outL[i]; const double dx = outL[i] - outL[i - 1]; de += dx * dx; } return std::sqrt(de / (e + 1e-30)); };
        double attack = 0, body = 0;
        for (size_t i = 1; i < nNotes; ++i) { attack += bright(i * sec + sec / 10, sec / 16); body += bright(i * sec + sec / 2, sec / 3); }
        std::printf("  [info] max |L-R| %.4f; max |out-in| %.4f; brightness early in the note %.3f, later %.3f\n", diff, dry, attack / (nNotes - 1), body / (nNotes - 1));
    }

    auto rms = [&](size_t a, size_t n) { double e = 0; for (size_t i = a; i < a + n; ++i) e += (double)outL[i] * outL[i]; return std::sqrt(e / n); };

    if (sweepPort >= 0) {
        // A/B: the same held note with the switch off and on, compared over the same window late
        // in the note (the envelope alone opens the filter on the attack).
        // brightness = rms of the first difference over rms.
        auto lateBrightness = [&](bool sweepOn) {
            LV2_Handle v = d->instantiate(d, kRate, "", features);
            std::vector<float> c(ctl);
            c[sweepPort] = sweepOn ? 1.f : 0.f;
            std::vector<float> x(((3 * sec) / kBlock) * kBlock, 0.f), yl(x.size()), yr(x.size());
            pluck(x, 0, x.size(), 110.0, 0.25);
            for (uint32_t p = 0; p <= nctl; ++p) d->connect_port(v, kFirstControl + p, &c[p]);
            if (d->activate) d->activate(v);
            for (size_t pos = 0; pos < x.size(); pos += kBlock) {
                d->connect_port(v, kIn, &x[pos]); d->connect_port(v, kOutL, &yl[pos]); d->connect_port(v, kOutR, &yr[pos]);
                d->run(v, kBlock);
            }
            d->cleanup(v);
            double e = 0, de = 0;
            for (size_t i = 2 * sec; i < 2 * sec + sec / 2; ++i) { e += (double)yl[i] * yl[i]; const double dx = yl[i] - yl[i - 1]; de += dx * dx; }
            return std::sqrt(de / (e + 1e-30));
        };
        const double closed = lateBrightness(false), opened = lateBrightness(true);
        std::snprintf(msg, sizeof msg, "%s opens the filter: late-note brightness %.3f off, %.3f on", controls[sweepPort].symbol.c_str(), closed, opened);
        check(opened > closed * 2, msg);
    }

    const double tail = rms(total - sec / 2, sec / 2);
    std::snprintf(msg, sizeof msg, "output dies away after the input stops (last 0.5 s rms %.2e)", tail);
    check(tail < 1e-3, msg);

    std::printf("\ncost on this machine: %.2f s of audio in %.3f s (%.2f%% of realtime), worst 128-frame block %.0f us of %.0f us\n",
                total / kRate, wall, 100 * wall / (total / kRate), worstBlock, 1e6 * kBlock / kRate);

    // The pitch tracker analyses once per 1408 samples, so one block in eleven carries an FFT:
    // the median says what a plain block costs, the upper percentiles what a block with the FFT costs.
    {
        const double budget = 1e6 * kBlock / kRate;
        std::vector<float> sorted(blockUs);
        std::sort(sorted.begin(), sorted.end());
        auto pct = [&](double p) { return sorted[(size_t)(p * (sorted.size() - 1))]; };
        size_t over = 0;
        for (float us : blockUs) if (us > budget) ++over;
        std::printf("block times: median %.0f us, p95 %.0f, p99 %.0f, max %.0f; %zu of %zu blocks over the %.0f us budget\n",
                    pct(.5), pct(.95), pct(.99), sorted.back(), over, blockUs.size(), budget);
    }

    if (args.size() > 2) {
        writeWav(args[2], outL, outR);
        std::printf("wrote %s\n", args[2]);
    }

    d->cleanup(h);
    dlclose(lib);
    std::printf("\n%d check(s) failed\n", failures);
    return failures;
}
