/**
 * @file: REWhistleDetector.h
 *
 * @description This is the RoboEireann whistle detector source ported the
 * BH2025 framework.
 *
 * @author: Shauna Recto
 */

#pragma once

#include "Framework/Module.h"
#include "Math/RingBuffer.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Whistle.h"

#include <CompiledNN2ONNX/CompiledNN.h>

#include <fftw3.h> // Library used for obtaining FFTs
#include <string>

#define MAX_SIZE 1600

/* Module Declaration */
MODULE(REWhistleDetector,
    {
        ,
        REQUIRES(AudioData),
        REQUIRES(FrameInfo),
        PROVIDES(Whistle),

        LOADS_PARAMETERS(
            {
                ,
                (std::string)whistleDetName,
                (int)detectionDelayMs,
                (int)fftSize,
                (int)fftStep,
                (unsigned)bufferSize,
                (int) numChannels,
                (float)whistleConfidenceThresh,
                (float)sampleGain,
            }),
    });

class REWhistleDetector : public REWhistleDetectorBase {
public:
    REWhistleDetector();
    ~REWhistleDetector();

private:
    std::vector<RingBuffer<AudioData::Sample>> channelBuffers;  // Sample buffers for all channels.
    RingBuffer<AudioData::Sample, MAX_SIZE> monoBuffer;         // Mono mixdown buffer.
    RingBuffer<AudioData::Sample> leftOverBuffer;               // Unused samples for next cycle.

    /* FFTW related members */
    float* samples;             // The samples that will be used with the FFT
    fftwf_complex* spectrum;     // the spectrum of the samples
    fftwf_plan fft;              // The plan to compute the FFT.

    /* magSpectrums Contains numFFTs FFTs of size spectrumSize */
    RingBuffer<std::vector<float>> magSpectrums;
    int spectrumSize;

    /* Other FFT Related members */
    std::vector<float> hannWindow;

    /* CompiledNN members */
    NeuralNetworkONNX::CompiledNN whistleDetector;
    std::unique_ptr<NeuralNetworkONNX::Model> model;

    void update(Whistle&);              /* Runs update cycle for WD */
    void processAudio(Whistle&, bool);  /* Processes Audio for inferrence */
    float inferenceCheck();             /* Checks audio contents against NN for whistles */
};
