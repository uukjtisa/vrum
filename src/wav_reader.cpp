#include "../include/wav_reader.h"

#include <cstring>
#include <fstream>

namespace {
    bool readExact(std::ifstream &f, void *dst, size_t bytes) {
        f.read(reinterpret_cast<char *>(dst), static_cast<std::streamsize>(bytes));
        return f.gcount() == static_cast<std::streamsize>(bytes);
    }
}

bool WavReader::open(const std::string &path) {
    m_samples.clear();
    m_error.clear();
    m_sampleRate = 0;
    m_channelCount = 0;

    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) {
        m_error = "could not open " + path;
        return false;
    }

    char riff[4], wave[4];
    uint32_t riffSize = 0;
    if (!readExact(f, riff, 4) || !readExact(f, &riffSize, 4) || !readExact(f, wave, 4)) {
        m_error = "truncated header";
        return false;
    }

    if (std::memcmp(riff, "RIFF", 4) != 0 || std::memcmp(wave, "WAVE", 4) != 0) {
        m_error = "not a RIFF/WAVE file";
        return false;
    }

    uint16_t bitsPerSample = 0;
    bool haveFmt = false;

    // Walk chunks until "data" is found.
    while (f.good()) {
        char id[4];
        uint32_t size = 0;
        if (!readExact(f, id, 4) || !readExact(f, &size, 4)) break;

        if (std::memcmp(id, "fmt ", 4) == 0) {
            uint16_t format = 0, channels = 0, blockAlign = 0;
            uint32_t sampleRate = 0, byteRate = 0;
            if (!readExact(f, &format, 2) || !readExact(f, &channels, 2)
                || !readExact(f, &sampleRate, 4) || !readExact(f, &byteRate, 4)
                || !readExact(f, &blockAlign, 2) || !readExact(f, &bitsPerSample, 2)) {
                m_error = "truncated fmt chunk";
                return false;
            }

            if (format != 1) {
                m_error = "not uncompressed PCM (format " + std::to_string(format) + ")";
                return false;
            }

            m_sampleRate = static_cast<int>(sampleRate);
            m_channelCount = static_cast<int>(channels);
            haveFmt = true;

            // Skip any extension bytes beyond the 16 read above.
            if (size > 16) f.seekg(size - 16, std::ios::cur);
        }
        else if (std::memcmp(id, "data", 4) == 0) {
            if (!haveFmt) {
                m_error = "data chunk before fmt chunk";
                return false;
            }
            if (bitsPerSample != 16) {
                m_error = "only 16-bit PCM supported (got "
                    + std::to_string(bitsPerSample) + ")";
                return false;
            }

            m_samples.resize(size / sizeof(int16_t));
            if (!m_samples.empty()
                && !readExact(f, m_samples.data(), m_samples.size() * sizeof(int16_t))) {
                // A capture killed mid-write can claim more data than exists;
                // keep whatever actually made it to disk rather than failing.
                m_samples.resize(static_cast<size_t>(f.gcount()) / sizeof(int16_t));
            }

            return true;
        }
        else {
            f.seekg(size + (size & 1), std::ios::cur);   // chunks are word-aligned
        }
    }

    m_error = "no data chunk found";
    return false;
}
