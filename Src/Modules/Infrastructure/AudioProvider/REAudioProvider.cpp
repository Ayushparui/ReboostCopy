/**
 * @file REAudioProvider.cpp
 *
 * @description: RoboEireann Audio Provider Module Source
 *
 * @author Shauna Recto
 */

#include "REAudioProvider.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"

MAKE_MODULE(REAudioProvider);

#ifdef TARGET_ROBOT

REAudioProvider::REAudioProvider()
{
    OUTPUT_TEXT("Initializing audio capture device...");
    /* Check for whether we can open the input audio device */
    for (uint retryNumber = 0; retryNumber < retries; retryNumber++) {
      int retVal = snd_pcm_open(&inputDevice, deviceName.c_str(), SND_PCM_STREAM_CAPTURE, 0);
      if (retVal == 0) {
          break;
      }
      FAIL("Failed to open microphone...");
      Thread::sleep(retryDelay);
    }

    currentCaptureVolume = 100.f;

    /* Static Allocation to reduce malloc issues */
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(inputDevice, params);

    snd_pcm_hw_params_set_access(inputDevice, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(inputDevice, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(inputDevice, params, numChannels);
    snd_pcm_hw_params_set_rate_near(inputDevice, params, &sampleRate, nullptr);

    VERIFY(!snd_pcm_hw_params(inputDevice, params));

    OUTPUT_TEXT("Starting audio device...");
    snd_pcm_prepare(inputDevice);
    VERIFY(!snd_pcm_start(inputDevice));

    OUTPUT_TEXT("Initializing WakeupAPI...");
    VERIFY(!Duerwen_wakeup_init(&wakeupHandle, 0));

    bufferInit = false;
    setVolume();
}

REAudioProvider::~REAudioProvider()
{
  Duerwen_wakeup_unit(wakeupHandle);
  snd_mixer_close(inputMixer);
}

bool REAudioProvider::setVolume()
{
    if (mixerName == "") {
        OUTPUT_WARNING("There is no mixer. volume not set. Expect 100%...");
        return false;
    }

    VERIFY(!snd_mixer_open(&inputMixer, 0));
    VERIFY(!snd_mixer_attach(inputMixer, deviceName.c_str()));

    VERIFY(!snd_mixer_selem_register(inputMixer, nullptr, nullptr));
    VERIFY(!snd_mixer_load(inputMixer));

    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_index(sid, 0);
    snd_mixer_selem_id_set_name(sid, mixerName.c_str());

    elem = snd_mixer_find_selem(inputMixer, sid);
    VERIFY(elem != 0);

    long min = -1, max = -1;
    int returnVal = snd_mixer_selem_get_capture_volume_range(elem, &min, &max);
    VERIFY(!returnVal);
    VERIFY(min < max);

    currentCaptureVolume = std::clamp(captureVolume, 0.f, 100.f);
    long volume = static_cast<long>(currentCaptureVolume * max / 100.f);

    snd_mixer_selem_set_capture_volume_all(elem, volume);
    return true;
}

void REAudioProvider::update(AudioData& audioData)
{
    /* Create our audio buffers and clear values to 0 */
    if (!bufferInit) {
      OUTPUT_TEXT("Initializing audio buffers...");
      audioFramesInt.resize(maxFrames * numChannels);
      audioData.samples.resize(maxFrames * outChannels);
      std::fill(audioFramesInt.begin(), audioFramesInt.end(), 0);
      std::fill(audioData.samples.begin(), audioData.samples.end(), 0);

      audioData.channels = outChannels;
      audioData.sampleRate = sampleRate;

      inputAudioBuffers.resize(usedChannels);
      outputAudioBuffers.resize(usedChannels);

      for (size_t bufferInd = 0; bufferInd < inputAudioBuffers.size(); bufferInd++) {
        inputAudioBuffers[bufferInd].resize(wakeupAPIBufferSize);
        outputAudioBuffers[bufferInd].resize(wakeupAPIBufferSize);
      }

      bufferInit = true;
    }

    const unsigned cycleMs = 1000 / tickRate;
    const unsigned timeMs = Time::getCurrentSystemTime();
    snd_pcm_sframes_t availableFrames = snd_pcm_avail(inputDevice);

    if (availableFrames < 0) {
        OUTPUT_WARNING("Error fetch available frames, recovering...");
        snd_pcm_recover(inputDevice, static_cast<int>(availableFrames), 0);
        return;
    }

    uint numReadableFrames = std::min(static_cast<uint>(availableFrames), maxFrames);
    int numAudioFrames = static_cast<int>(snd_pcm_readi(inputDevice, audioFramesInt.data(), numReadableFrames));

    /* Try to recover the audio device */
    if (numAudioFrames < 0) {
        OUTPUT_WARNING("Audio record error, recovering...");
        numAudioFrames = snd_pcm_recover(inputDevice, numAudioFrames, 0);
    } else {

      /* Deinterleave the frames for the wakeupAPI */
      waitBuffer.insert(waitBuffer.end(), audioFramesInt.begin(), audioFramesInt.begin() + (numAudioFrames * numChannels));

      const size_t maxWaitSize = wakeupAPIBufferSize * numChannels * 2;
      if (waitBuffer.size() > maxWaitSize) {
        waitBuffer.erase(waitBuffer.begin(), waitBuffer.end() - maxWaitSize);
      }

      audioData.samplesCaptured = 0;

      while (waitBuffer.size() > (wakeupAPIBufferSize * numChannels)) {
        for (size_t bufferInd = 0; bufferInd < wakeupAPIBufferSize; bufferInd++) {
          inputAudioBuffers[0][bufferInd] = waitBuffer[bufferInd * numChannels + 0];
          inputAudioBuffers[1][bufferInd] = waitBuffer[bufferInd * numChannels + 1];
          inputAudioBuffers[2][bufferInd] = waitBuffer[bufferInd * numChannels + 2];
          inputAudioBuffers[3][bufferInd] = waitBuffer[bufferInd * numChannels + 4];
        }

        Duerwen_wakeup_three_write_data(
          wakeupHandle,
          inputAudioBuffers[0].data(),
          inputAudioBuffers[1].data(),
          inputAudioBuffers[2].data(),
          inputAudioBuffers[3].data(),
          outputAudioBuffers[0].data(),
          outputAudioBuffers[1].data(),
          outputAudioBuffers[2].data(),
          outputAudioBuffers[3].data()
        );

        for (size_t bufferInd = 0; bufferInd < wakeupAPIBufferSize; bufferInd++) {
          audioFramesInt[bufferInd * 3 + 0] = outputAudioBuffers[0][bufferInd];
          audioFramesInt[bufferInd * 3 + 1] = outputAudioBuffers[1][bufferInd];
          audioFramesInt[bufferInd * 3 + 2] = outputAudioBuffers[2][bufferInd];
        }

        waitBuffer.erase(waitBuffer.begin(), waitBuffer.begin() + (wakeupAPIBufferSize * numChannels));
        audioData.samplesCaptured = wakeupAPIBufferSize;
        signedToFloat(audioData);
      }
    }

  const unsigned workTimeMs = Time::getCurrentSystemTime() - timeMs;
  tickWatchDog = Time::getCurrentSystemTime();
  if (workTimeMs < cycleMs) {
    Thread::sleep(cycleMs - workTimeMs);
  }
}

void REAudioProvider::signedToFloat(AudioData& audioData)
{
    for (uint sampleIndex = 0; sampleIndex < audioData.samplesCaptured * audioData.channels; sampleIndex++) {
        audioData.samples[sampleIndex] = static_cast<float>(audioFramesInt[sampleIndex]) / INT16_MAX;
    }
}

#else /* NOT TARGET ROBOT */

REAudioProvider::REAudioProvider() = default;
REAudioProvider::~REAudioProvider() = default;

void REAudioProvider::update(AudioData&)
{
    Thread::yield();
    tickWatchDog = Time::getCurrentSystemTime();
}

#endif
