// libfpv - procedural motor sound.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "audio/synth.h"

#include <algorithm>
#include <cmath>

namespace fpv {

namespace {
constexpr double kTwoPi = 6.283185307179586;
constexpr int kHarmonics = 6;
constexpr float kPolePairs = 7.0f;  // typical 12N14P FPV motor
}  // namespace

void AudioSynth::configure(uint32_t sampleRate, uint32_t channels) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rate = std::clamp<uint32_t>(sampleRate, 8000, 192000);
    m_channels = std::clamp<uint32_t>(channels, 1, 8);
}

void AudioSynth::setParams(const AudioParams &p) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_target = p;
}

void AudioSynth::setVolume(float v) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_volume = std::clamp(v, 0.0f, 2.0f);
}

uint32_t AudioSynth::read(float *out, uint32_t frames) {
    AudioParams target;
    uint32_t rate, channels;
    float volume;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        target = m_target;
        rate = m_rate;
        channels = m_channels;
        volume = m_volume;
    }
    const double dt = 1.0 / rate;
    // parameters glide over ~15 ms so host frame updates do not click
    const float glide = 1.0f - std::exp(-(float)frames / (rate * 0.015f));
    AudioParams from = m_cur;
    AudioParams &cur = m_cur;
    for (int m = 0; m < 4; m++) {
        cur.rpm[m] += (target.rpm[m] - cur.rpm[m]) * glide;
        cur.load[m] += (target.load[m] - cur.load[m]) * glide;
        cur.damage[m] = target.damage[m];
        cur.pan[m] = target.pan[m];
    }
    cur.maxRpm = target.maxRpm;
    cur.blades = target.blades;
    cur.beeper = target.beeper;
    float pausedGain = target.paused ? 0.0f : 1.0f;

    for (uint32_t f = 0; f < frames; f++) {
        float t = frames > 1 ? (float)f / (frames - 1) : 1.0f;
        float left = 0, right = 0;
        for (int m = 0; m < 4; m++) {
            float rpm = from.rpm[m] + (cur.rpm[m] - from.rpm[m]) * t;
            float load = from.load[m] + (cur.load[m] - from.load[m]) * t;
            float spin = std::clamp(std::fabs(rpm) / std::max(1000.0f, cur.maxRpm), 0.0f, 1.2f);
            if (spin < 0.002f) continue;
            double rev = std::fabs(rpm) / 60.0;
            double bpf = rev * cur.blades;  // blade-pass frequency
            m_phase[m] += bpf * dt;
            m_phase[m] -= std::floor(m_phase[m]);
            m_whine[m] += rev * kPolePairs * dt;
            m_whine[m] -= std::floor(m_whine[m]);
            m_wobble[m] += rev * dt;
            m_wobble[m] -= std::floor(m_wobble[m]);

            // blade-pass buzz: band-limited saw, harmonics above Nyquist dropped
            float buzz = 0;
            for (int k = 1; k <= kHarmonics; k++) {
                if (bpf * k >= rate * 0.45) break;
                buzz += (float)std::sin(kTwoPi * m_phase[m] * k) / (float)k;
            }
            float whine = (rev * kPolePairs < rate * 0.45) ? (float)std::sin(kTwoPi * m_whine[m]) : 0.0f;
            // air noise, low-passed with a cutoff that follows the rpm
            m_seed = m_seed * 1664525u + 1013904223u;
            float white = (float)(int32_t)m_seed / 2147483648.0f;
            float a = (float)std::min(0.9, kTwoPi * bpf * 2.0 * dt);
            m_noiseLp[m] += (white - m_noiseLp[m]) * a;

            float dmg = cur.damage[m];
            float wobble = 1.0f + dmg * 0.6f * (float)std::sin(kTwoPi * m_wobble[m]);
            float amp = spin * spin * (0.35f + 0.65f * load);
            float s = (buzz * 0.55f + whine * 0.08f * (0.3f + load) + m_noiseLp[m] * (0.6f + dmg * 1.5f)) * amp * wobble;
            float pan = std::clamp(cur.pan[m], -1.0f, 1.0f);
            left += s * (0.5f - 0.35f * pan);
            right += s * (0.5f + 0.35f * pan);
        }
        if (cur.beeper) {
            m_beepPhase += 2700.0 * dt;
            m_beepPhase -= std::floor(m_beepPhase);
            float b = m_beepPhase < 0.5 ? 0.12f : -0.12f;
            left += b;
            right += b;
        }
        left = std::tanh(left * volume) * pausedGain;
        right = std::tanh(right * volume) * pausedGain;
        float *o = out + (size_t)f * channels;
        if (channels == 1) {
            o[0] = 0.5f * (left + right);
        } else {
            o[0] = left;
            o[1] = right;
            for (uint32_t c = 2; c < channels; c++) o[c] = 0;
        }
    }
    return frames;
}

}  // namespace fpv
