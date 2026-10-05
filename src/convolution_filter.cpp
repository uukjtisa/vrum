#include "../include/convolution_filter.h"

#include <assert.h>
#include <string.h>
#include <cmath>

ConvolutionFilter::ConvolutionFilter() {
    m_shiftRegister = nullptr;
    m_impulseResponse = nullptr;

    m_shiftOffset = 0;
    m_sampleCount = 0;
    m_dirty = false;

    m_headPos = 0;
    m_partitions = 0;
    m_inputSpectraHead = 0;
    m_blockPos = 0;
}

ConvolutionFilter::~ConvolutionFilter() {
    assert(m_shiftRegister == nullptr);
    assert(m_impulseResponse == nullptr);
}

void ConvolutionFilter::initialize(int samples) {
    m_sampleCount = samples;
    m_shiftOffset = 0;
    m_shiftRegister = new float[samples];
    m_impulseResponse = new float[samples];

    memset(m_shiftRegister, 0, sizeof(float) * samples);
    memset(m_impulseResponse, 0, sizeof(float) * samples);

    m_dirty = true;
}

void ConvolutionFilter::destroy() {
    delete[] m_shiftRegister;
    delete[] m_impulseResponse;

    m_shiftRegister = nullptr;
    m_impulseResponse = nullptr;

    m_tailSpectra.clear();
    m_inputSpectra.clear();
}

void ConvolutionFilter::fft(cfloat *data, int n, const cfloat *twiddles, bool inverse) {
    // Iterative radix-2. n is 2 * BlockSize, a power of two.
    for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }

    for (int len = 2; len <= n; len <<= 1) {
        const int half = len >> 1;
        const int stride = n / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; ++k) {
                cfloat w = twiddles[k * stride];
                if (inverse) w = std::conj(w);
                const cfloat u = data[i + k];
                const cfloat v = data[i + k + half] * w;
                data[i + k] = u + v;
                data[i + k + half] = u - v;
            }
        }
    }
}

void ConvolutionFilter::prepare() {
    m_dirty = false;

    const int B = BlockSize;
    const int N = 2 * B;

    m_headHistory.assign(B, 0.0f);
    m_headPos = 0;

    m_twiddles.resize(N / 2);
    for (int k = 0; k < N / 2; ++k) {
        const double a = -2.0 * 3.14159265358979323846 * k / N;
        m_twiddles[k] = cfloat((float)std::cos(a), (float)std::sin(a));
    }

    const int tailTaps = (m_sampleCount > B) ? (m_sampleCount - B) : 0;
    m_partitions = (tailTaps + B - 1) / B;

    m_tailSpectra.assign((size_t)m_partitions * N, cfloat(0, 0));
    for (int p = 0; p < m_partitions; ++p) {
        cfloat *H = &m_tailSpectra[(size_t)p * N];
        for (int m = 0; m < B; ++m) {
            const int tap = B + p * B + m;
            H[m] = cfloat((tap < m_sampleCount) ? m_impulseResponse[tap] : 0.0f, 0.0f);
        }
        fft(H, N, m_twiddles.data(), false);
    }

    m_inputSpectra.assign((size_t)m_partitions * N, cfloat(0, 0));
    m_inputSpectraHead = 0;
    m_prevBlock.assign(B, 0.0f);
    m_currBlock.assign(B, 0.0f);
    m_tailOut.assign(B, 0.0f);
    m_scratch.assign(N, cfloat(0, 0));
    m_blockPos = 0;
}

void ConvolutionFilter::processBlock() {
    const int B = BlockSize;
    const int N = 2 * B;

    // Spectrum of [previous block, current block] -> newest slot of the
    // frequency-domain delay line.
    m_inputSpectraHead = (m_inputSpectraHead - 1 + m_partitions) % m_partitions;
    cfloat *X = &m_inputSpectra[(size_t)m_inputSpectraHead * N];
    for (int i = 0; i < B; ++i) {
        X[i] = cfloat(m_prevBlock[i], 0.0f);
        X[i + B] = cfloat(m_currBlock[i], 0.0f);
    }
    fft(X, N, m_twiddles.data(), false);

    // Y = sum_p H_p * X_{k-p}
    std::fill(m_scratch.begin(), m_scratch.end(), cfloat(0, 0));
    for (int p = 0; p < m_partitions; ++p) {
        const cfloat *H = &m_tailSpectra[(size_t)p * N];
        const cfloat *Xp =
            &m_inputSpectra[(size_t)((m_inputSpectraHead + p) % m_partitions) * N];
        for (int i = 0; i < N; ++i) m_scratch[i] += H[i] * Xp[i];
    }

    fft(m_scratch.data(), N, m_twiddles.data(), true);

    // Overlap-save: the last B samples are the valid linear-convolution part.
    // Because the tail taps start B samples late, this block's result is the
    // tail contribution for the NEXT B output samples.
    const float scale = 1.0f / N;
    for (int i = 0; i < B; ++i) m_tailOut[i] = m_scratch[i + B].real() * scale;

    m_prevBlock.swap(m_currBlock);
}

float ConvolutionFilter::f(float sample) {
    if (m_dirty) prepare();
    if (m_sampleCount <= 0) return 0.0f;

    const int B = BlockSize;

    // Head: direct form over the first min(B, N) taps.
    m_headHistory[m_headPos] = sample;
    const int headTaps = (m_sampleCount < B) ? m_sampleCount : B;
    float result = 0.0f;
    int idx = m_headPos;
    for (int m = 0; m < headTaps; ++m) {
        result += m_impulseResponse[m] * m_headHistory[idx];
        idx = (idx == 0) ? B - 1 : idx - 1;
    }
    m_headPos = (m_headPos + 1) % B;

    if (m_partitions > 0) {
        result += m_tailOut[m_blockPos];
        m_currBlock[m_blockPos] = sample;
        if (++m_blockPos == B) {
            m_blockPos = 0;
            processBlock();
        }
    }

    return result;
}

float ConvolutionFilter::directF(float sample) {
    m_shiftRegister[m_shiftOffset] = sample;

    float result = 0;
    for (int i = 0; i < m_sampleCount - m_shiftOffset; ++i) {
        result += m_impulseResponse[i] * m_shiftRegister[i + m_shiftOffset];
    }

    for (int i = m_sampleCount - m_shiftOffset; i < m_sampleCount; ++i) {
        result += m_impulseResponse[i] * m_shiftRegister[i - (m_sampleCount - m_shiftOffset)];
    }

    m_shiftOffset = (m_shiftOffset - 1 + m_sampleCount) % m_sampleCount;

    return result;
}
