#include "../include/reverb_filter.h"

#include <cmath>

namespace {

// Delay lengths in samples at 44.1 kHz -- Freeverb's full set. Mutually prime
// so the combs never line up into a ringing pitch, which is what makes a cheap
// reverb sound metallic. The SHORT ones matter most: they are what fills the
// gaps between the long taps and turns a row of echoes into a tail.
constexpr int CombLengths[8] = {
    1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617
};
constexpr int AllpassLengths[4] = { 556, 441, 341, 225 };

// Early reflections, in samples at 44.1 kHz: 8.5 / 13.9 / 21.3 / 31.7 ms.
// These are the individual slap-backs off nearby hard surfaces, and they are
// what makes a tunnel sound like a tunnel rather than like a large room --
// close walls put distinct echoes in front of the diffuse tail. Prime-ish and
// unevenly spaced so they never fuse into a pitch.
constexpr int EarlyTapLengths[4] = { 375, 613, 941, 1399 };
constexpr float EarlyTapGains[4] = { 0.85f, 0.62f, 0.44f, 0.30f };
// Longest tap plus headroom.
constexpr int EarlyBufferLength = 1600;

int scaleLength(int base, float sampleRate) {
    const int n = (int)std::lround(base * (double)sampleRate / 44100.0);
    return (n < 8) ? 8 : n;
}

} // namespace

ReverbFilter::ReverbFilter() {
    /* void */
}

ReverbFilter::~ReverbFilter() {
    destroy();
}

void ReverbFilter::initialize(float sampleRate) {
    destroy();

    for (int i = 0; i < CombCount; ++i) {
        m_combs[i].length = scaleLength(CombLengths[i], sampleRate);
        m_combs[i].buffer = new float[m_combs[i].length]();
        m_combs[i].offset = 0;
        m_combs[i].store = 0.0f;
    }

    for (int i = 0; i < AllpassCount; ++i) {
        m_allpasses[i].length = scaleLength(AllpassLengths[i], sampleRate);
        m_allpasses[i].buffer = new float[m_allpasses[i].length]();
        m_allpasses[i].offset = 0;
    }

    m_earlyLength = scaleLength(EarlyBufferLength, sampleRate);
    m_earlyBuffer = new float[m_earlyLength]();
    m_earlyOffset = 0;
    for (int i = 0; i < EarlyTapCount; ++i) {
        int tap = scaleLength(EarlyTapLengths[i], sampleRate);
        if (tap >= m_earlyLength) tap = m_earlyLength - 1;
        m_earlyTaps[i] = tap;
    }
}

void ReverbFilter::destroy() {
    for (int i = 0; i < CombCount; ++i) {
        delete[] m_combs[i].buffer;
        m_combs[i].buffer = nullptr;
    }

    for (int i = 0; i < AllpassCount; ++i) {
        delete[] m_allpasses[i].buffer;
        m_allpasses[i].buffer = nullptr;
    }

    delete[] m_earlyBuffer;
    m_earlyBuffer = nullptr;
    m_earlyLength = 0;
}

void ReverbFilter::reset() {
    for (int i = 0; i < CombCount; ++i) {
        if (m_combs[i].buffer == nullptr) continue;
        for (int s = 0; s < m_combs[i].length; ++s) m_combs[i].buffer[s] = 0.0f;
        m_combs[i].store = 0.0f;
    }

    for (int i = 0; i < AllpassCount; ++i) {
        if (m_allpasses[i].buffer == nullptr) continue;
        for (int s = 0; s < m_allpasses[i].length; ++s) {
            m_allpasses[i].buffer[s] = 0.0f;
        }
    }

    if (m_earlyBuffer != nullptr) {
        for (int s = 0; s < m_earlyLength; ++s) m_earlyBuffer[s] = 0.0f;
    }
}

void ReverbFilter::setRoomSize(float roomSize) {
    if (roomSize < 0.0f) roomSize = 0.0f;
    if (roomSize > 1.0f) roomSize = 1.0f;

    // 0.62 (a short outdoor slap, ~0.25 s) up to 0.94 (a long tunnel, ~2 s).
    // Each lap of the longest comb is ~37 ms, so this is the range that spans
    // "open ground" to "concrete tube" without ever reaching a cathedral.
    m_feedback = 0.62f + 0.32f * roomSize;

    // Keep the wet return near unity regardless. See m_inputGain in the header:
    // the comb loop's own 1/(1-f) gain otherwise makes a short room inaudible
    // and a long one overwhelming, with the same reverbMix setting.
    m_inputGain = (1.0f - m_feedback) / (float)CombCount;
}

void ReverbFilter::setEarlyLevel(float level) {
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;

    m_earlyLevel = level;
}

void ReverbFilter::setDamping(float damping) {
    if (damping < 0.0f) damping = 0.0f;
    if (damping > 1.0f) damping = 1.0f;

    m_damp1 = damping;
    m_damp2 = 1.0f - damping;
}

float ReverbFilter::f(float sample) {
    if (m_combs[0].buffer == nullptr) return 0.0f;

    // Early reflections: discrete slap-backs off the nearest hard surfaces,
    // arriving before the diffuse tail. In a tunnel these ARE the sound.
    float early = 0.0f;
    if (m_earlyLevel > 0.0f && m_earlyBuffer != nullptr) {
        for (int i = 0; i < EarlyTapCount; ++i) {
            int idx = m_earlyOffset - m_earlyTaps[i];
            if (idx < 0) idx += m_earlyLength;
            early += m_earlyBuffer[idx] * EarlyTapGains[i];
        }

        early *= m_earlyLevel;
    }

    if (m_earlyBuffer != nullptr) {
        m_earlyBuffer[m_earlyOffset] = sample;
        m_earlyOffset = (m_earlyOffset + 1) % m_earlyLength;
    }

    // See m_inputGain in the header: this tracks the feedback so the wet
    // return stays near unity at any room size.
    const float in = sample * m_inputGain;

    float out = 0.0f;

    // Parallel damped combs build the density of the tail.
    for (int i = 0; i < CombCount; ++i) {
        Comb &c = m_combs[i];

        const float delayed = c.buffer[c.offset];
        // One-pole low-pass inside the feedback path: every trip around the
        // loop loses more top end, which is the audible signature of distance
        // and soft surroundings rather than a hard room.
        c.store = delayed * m_damp2 + c.store * m_damp1;
        if (!std::isfinite(c.store)) c.store = 0.0f;

        c.buffer[c.offset] = in + c.store * m_feedback;
        c.offset = (c.offset + 1) % c.length;

        out += delayed;
    }

    // Series allpasses smear the echo pattern so it reads as a tail rather
    // than as a handful of distinct repeats.
    for (int i = 0; i < AllpassCount; ++i) {
        Allpass &a = m_allpasses[i];

        const float delayed = a.buffer[a.offset];
        const float input = out;

        out = -input + delayed;
        a.buffer[a.offset] = input + delayed * 0.5f;
        a.offset = (a.offset + 1) % a.length;
    }

    out += early;

    return std::isfinite(out) ? out : 0.0f;
}
