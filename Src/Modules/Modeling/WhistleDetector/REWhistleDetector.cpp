/**
 * @file: REWhistleDetector.cpp
 *
 * @description This is the RoboEireann whistle detector source ported the
 * BH2025 framework.
 *
 * @author: Shauna Recto
 */

#include "REWhistleDetector.h"

#include "Debugging/Annotation.h"
#include "Debugging/DebugImages.h"
#include "Debugging/Plot.h"

#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

#include <algorithm>

MAKE_MODULE(REWhistleDetector);

static DECLARE_SYNC;

REWhistleDetector::REWhistleDetector()
{
    OUTPUT_TEXT("Loading whistle detector model...");
    /* Obtain location for Whistle Detector model */
    const std::string baseDir = std::string(File::getBHDir()) + "/Config/NeuralNets/WhistleDetector/";
    model = std::make_unique<NeuralNetworkONNX::Model>(baseDir + whistleDetName);

    /* Memory Allocation for FFT */
    spectrumSize = static_cast<int>((fftSize / 2) + 1);
    samples = fftwf_alloc_real(fftSize);
    spectrum = fftwf_alloc_complex(spectrumSize);

    /* Compile NN */
    whistleDetector.compile(*model);

    ASSERT(whistleDetector.numOfInputs() == 1);
    ASSERT(whistleDetector.numOfOutputs() == 1);

    /* This should fit the input dimension of 18x257x1 */
    ASSERT(whistleDetector.input(0).dims(0) == 18);
    ASSERT(whistleDetector.input(0).dims(1) == 257);
    ASSERT(whistleDetector.input(0).dims(2) == 1);

    /* We only want 1 output */
    ASSERT(whistleDetector.output(0).dims(0) == 1);

    SYNC;

    /* Implementing FFTW plans */
    fft = fftwf_plan_dft_r2c_1d(fftSize, samples, spectrum, FFTW_MEASURE);

    /* Initialize samples buffer are all set to 0 */
    std::memset(samples, 0, sizeof(float) * fftSize);

    /* Implement Hanning Window */
    hannWindow.resize(fftSize);
    for (int i = 0; i < fftSize; i++)
        hannWindow[i] = static_cast<float>(std::pow(std::sin(pi * i / fftSize), 2));

    /* Resize magSpectrums according specifications */
    magSpectrums.reserve(whistleDetector.input(0).dims(0));

    /* Make initial empty spectrums which will be dropped later on */
    for (std::vector<float>& magSpect : magSpectrums) {
        magSpect.resize(spectrumSize);
        std::fill(magSpect.begin(), magSpect.end(), 0);
    }

    /* Set our leftOverBuffer */
    leftOverBuffer.reserve(1);
    leftOverBuffer.push_front(0);
}

REWhistleDetector::~REWhistleDetector()
{
    SYNC;
    fftwf_destroy_plan(fft);
    fftwf_free(spectrum);
    fftwf_free(samples);
}

void REWhistleDetector::update(Whistle& theWhistle) {
    DECLARE_PLOT("module:REWhistleDetector:audioBuffer");
    DECLARE_PLOT("module:REWhistleDetector:whistleConfidence");
    DECLARE_PLOT("module:REWhistleDetector:whistleConfidenceThresh");

    bool validAudio = (!theAudioData.samples.empty() && (theAudioData.channels >= numChannels));

    /* If not receiving audio input */
    if (!validAudio) {
        /* Do nothing, leave Whistle representation with details for prev whistle detected */
        theWhistle.channelsUsedForWhistleDetection = 0; // no detection this time

    } /* go for whistle detection */ else {
        /* Adapt number of channels to audio data; we assume this only changes at startup */
        channelBuffers.resize(theAudioData.channels);
        for (auto &buffer : channelBuffers)
            buffer.reserve(bufferSize);

        processAudio(theWhistle, true);
    }
}

void REWhistleDetector::processAudio(Whistle& theWhistle, bool detectWhistle) {
    /* Couple of things to consider
     * 1. Samples from AudioProvider are interleaved meaning that each sample per channel
     *    increments only after the previous channel gets their audio sample.
     *    [Example] -> [C1N0][C2N0][C3N0][C4N0][C1N1][C2N1][C3N1]...
     * 2. stepSize helps get only samples for a specific channel, skipping other channels' samples.
     * 3. Since we may have previously recieved but unused samples, we can now push them first to
     *    the monoBuffer (remember to read ring buffer correctly).
     */
    const std::size_t stepSize = theAudioData.channels;
    monoBuffer.reserve((theAudioData.samples.size() / stepSize) + leftOverBuffer.size());

    /* Place samples from leftOverBuffer to monoBuffer */
    for (std::size_t sample = 0; sample < leftOverBuffer.size(); sample++)
        monoBuffer.push_front(leftOverBuffer.from_back(sample));

    /* Copy from interleaved audioData to separate ring buffers, one per channel
     * (only the most recent bufferSize samples will be available in the ring buffer
     * after this)
     */
    for (std::size_t i = 0; i < theAudioData.samples.size(); i += stepSize) {
        /* Total sum of samples */
        float monoSum = 0;

        /* For each channel */
        for (std::size_t channel = 0; channel < numChannels; ++channel) {
            /* Obtain desired audio sample and perform clamping checks before
             * pushing to buffers.
             */
            const float audioSample = std::clamp(theAudioData.samples[i + channel], -1.f , 1.f);
            channelBuffers[channel].push_front(audioSample);

            /* add samples from all channels into one mono sample */
            monoSum += audioSample;
        }

        /* normalize the mono sample level and add to the mono mix buffer */
        monoBuffer.push_front(std::clamp(monoSum / numChannels * sampleGain, -1.f , 1.f));

        /* Plot monoBuffer Data
         * TODO: Check whether this is the right way for indexing through monoBuffer.
         */
        PLOT("module:REWhistleDetector:audioBuffer", monoBuffer[i]);
    }

    /* We process our buffer as multiple FFT windows.
     * We determine the whistleConfidence from all FFT windows passed
     * into our NN which produces a single float value */
    float whistleConfidence = inferenceCheck();

    /* Set the whistleDetection to false at start of the block */
    theWhistle.whistleDetected = false;

    /* Set the detection info */
    if (detectWhistle) { 
        /* Checks whether the confidence value from the NN */
        if (whistleConfidence > whistleConfidenceThresh) {
            theWhistle.whistleDetected = true;
            theWhistle.lastTimeWhistleDetected = theFrameInfo.time;

            /* Part of the whistle module. TODO if these are needed still or used
            * elsewhere apart from whistleDetector or whistleHandler.
            */
            theWhistle.confidenceOfLastWhistleDetection = whistleConfidence;
            theWhistle.channelsUsedForWhistleDetection = static_cast<unsigned char>(channelBuffers.size());
        }
    } else {
        theWhistle.channelsUsedForWhistleDetection = 0; // no detection enabled at this time
    }

}

float REWhistleDetector::inferenceCheck() {
    /* Process the window of data starting at oldest.
     * NOTE: Ringbuffers are indexed from newest to oldest so we need
     * to reverse this when populating the fft samples.
     * Done with .from_back()
     */

    /* Gets the number of FFTs to compute.
     * fftOffset sets the starting point of the monoBuffer to read from.
     * numFFTs is capped to 15, with the excess left for next cycle.
     */
    int numFFTs = std::min(18, static_cast<int>((monoBuffer.size() / fftStep)));
    int fftOffset = 0;

    /* For all the available FFT from buffer */
    for (int i = 0; i < numFFTs; i++) {

        /* Push new FFT vector into RingBuffer */
        magSpectrums.push_front(std::vector<float>());
        magSpectrums.front().resize(spectrumSize);

        /* Get our buffer samples for the FFT, with gain scaling */
        for (int sampleIndex = 0; sampleIndex < fftSize; sampleIndex++)
            samples[sampleIndex] = monoBuffer.from_back(fftOffset + sampleIndex) * hannWindow[sampleIndex];

        /* Perform the FFT on current set of samples */
        fftwf_execute(fft);
        for (int j = 0; j < spectrumSize; j++)
        {
            /* calculate the magnitude spectrum with usual FFT normalization */
            magSpectrums.front()[j] =
                std::clamp((spectrum[j][0] * spectrum[j][0] + spectrum[j][1] * spectrum[j][1]) / fftSize , -1.f, 1.f);
        }

        /* Use the next set of samples */
        fftOffset += fftStep;
    }

    /* Store excess samples into leftOverBuffer.
     * Make assertions to determine if there are samples to hold for next cycle.
     * Then determine the number of samples left over.
     */
    std::size_t leftOverSize = monoBuffer.size() - (numFFTs * fftStep);
    ASSERT(leftOverSize > 0);

    leftOverBuffer.reserve(leftOverSize);

    /* Position the pointer in RingBuffer of last sample to be used.
    * As the previous loop ends with the last offset + an unused step,
    * revert said change.
    */
    std::size_t leftOverOffset = fftOffset - fftStep;

    /* Iterate over the remaining samples to be put into the leftOverBuffer. */
    for (std::size_t sample = 0; sample < leftOverSize; sample++)
        leftOverBuffer.push_front(monoBuffer.from_back(leftOverOffset + sample));

    /* Passing FFTs into NN */
    float *fftDest = whistleDetector.input(0).data();

    /* Adapted for loop to gurantee taking from the back of RingBuffer */
    for (std::size_t magSpectInd = 0; magSpectInd < magSpectrums.size(); magSpectInd++) {
        memcpy(fftDest, magSpectrums.from_back(magSpectInd).data(), spectrumSize * sizeof(float));
        fftDest += spectrumSize;
    }

    /* Comparison of FFT and NN */
    whistleDetector.apply();
    const float whistleConfidence = whistleDetector.output(0).data()[0];

    /* Prints values of the whistleConfidence. Debugging. */
    PLOT("module:REWhistleDetector:whistleConfidence", whistleConfidence);
    PLOT("module:REWhistleDetector:whistleConfidenceThresh", whistleConfidenceThresh);

    return whistleConfidence;

}
