#ifndef ATG_ENGINE_SIM_WAV_READER_H
#define ATG_ENGINE_SIM_WAV_READER_H

#include <cinttypes>
#include <string>
#include <vector>

// Minimal 16-bit PCM RIFF/WAVE reader.
//
// The GUI app loads impulse responses through delta-studio's audio layer, which
// drags in the whole graphics stack. The headless bench needs the same WAV data
// with no windowing system, so it reads them itself. Walks the chunk list rather
// than assuming a fixed 44-byte header, since real-world WAVs carry LIST/fact
// chunks between "fmt " and "data".
class WavReader {
    public:
        bool open(const std::string &path);

        const std::vector<int16_t> &getSamples() const { return m_samples; }
        int getSampleRate() const { return m_sampleRate; }
        int getChannelCount() const { return m_channelCount; }
        const std::string &getError() const { return m_error; }

    protected:
        std::vector<int16_t> m_samples;
        std::string m_error;
        int m_sampleRate = 0;
        int m_channelCount = 0;
};

#endif /* ATG_ENGINE_SIM_WAV_READER_H */
