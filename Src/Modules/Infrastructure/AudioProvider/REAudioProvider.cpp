/**
 * @file REAudioProvider.cpp
 *
 * RoboEireann Audio Provider Module Source
 *
 * @author Shauna Recto
 */

#include "REAudioProvider.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"

MAKE_MODULE(REAudioProvider);


REAudioProvider::REAudioProvider()
{
#ifdef TARGET_ROBOT

  if (!enabled)
    return;

  OUTPUT_TEXT("Initializing audio capture device...");
  /* Check for whether we can open the input audio device */
  for (uint retryNumber = 0; retryNumber < retries; retryNumber++)
  {
    if (snd_pcm_open(&inputDevice, deviceName.c_str(), SND_PCM_STREAM_CAPTURE, 0) == 0)
      break;

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

  bufferInit = false;
  setVolume();
#endif
}

REAudioProvider::~REAudioProvider()
{
#ifdef TARGET_ROBOT
  if (!enabled)
    return;

  Duerwen_wakeup_unit(wakeupHandle);
  if (inputMixer)
    snd_mixer_close(inputMixer);
#endif
}

#ifdef TARGET_ROBOT
bool REAudioProvider::setVolume()
{
  if (mixerName == "")
  {
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

  long min = -1;
  long max = -1;
  int returnVal = snd_mixer_selem_get_capture_volume_range(elem, &min, &max);
  VERIFY(!returnVal);
  VERIFY(min < max);

  currentCaptureVolume = std::clamp(captureVolume, 0.f, 100.f);
  long volume = static_cast<long>(currentCaptureVolume * max / 100.f);

  snd_mixer_selem_set_capture_volume_all(elem, volume);
  return true;
}

// void REAudioProvider::writeFloatSamples(AudioData& audioData)
// {
//   for (uint sampleIndex = 0; sampleIndex < audioData.samplesCaptured * audioData.channels; sampleIndex++)
//     audioData.samples[sampleIndex] = static_cast<float>(audioFramesInt[sampleIndex]) / INT16_MAX;
// }


void REAudioProvider::initBuffers(AudioData& audioData)
{
  int retDuerwen = -1;
  for (int initRetries = 0; initRetries < 10; initRetries++)
  {
    OUTPUT_TEXT("Initializing WakeupAPI...");
    retDuerwen = Duerwen_wakeup_init(&wakeupHandle, 0);
    if (retDuerwen == 0)
      break;
    else
    {
      if (wakeupHandle != nullptr)
      {
        Duerwen_wakeup_unit(wakeupHandle);
        wakeupHandle = nullptr;
      }
      Thread::sleep(100);
    }
  }

  VERIFY(retDuerwen == 0);

  OUTPUT_TEXT("Initializing audio buffers...");

  audioFramesInt.resize(maxFrames * numChannels);
  audioData.samples.reserve(maxFrames * outChannels);
  std::fill(audioFramesInt.begin(), audioFramesInt.end(), 0);
  // std::fill(audioData.samples.begin(), audioData.samples.end(), 0);

  audioData.channels = outChannels;
  audioData.sampleRate = sampleRate;

  inputAudioBuffers.resize(usedChannels);
  outputAudioBuffers.resize(usedChannels);

  for (size_t bufferInd = 0; bufferInd < inputAudioBuffers.size(); bufferInd++)
  {
    inputAudioBuffers[bufferInd].resize(wakeupAPIBufferSize);
    outputAudioBuffers[bufferInd].resize(wakeupAPIBufferSize);
  }
}
#endif

void REAudioProvider::update([[maybe_unused]] AudioData& audioData)
{
#ifdef TARGET_ROBOT
  if (!enabled)
  {
    Thread::sleep(30);
    return;
  }

  /* Create our audio buffers and clear values to 0 */
  if (!bufferInit)
  {
    initBuffers(audioData); // note: might block for a period of time retrying this
    bufferInit = true;
  }

  audioData.samples.clear(); // size is now zero

  // use the following loop to ensure that the update function generally doesn't return
  // until audioData contains usable samples

  unsigned startMs = Time::getCurrentSystemTime();
  for (; Time::getTimeSince(startMs) < 256; ) // 256 ms is 4 times as long as the expected time to receive 1024 samples at 16000Hz
  {
    snd_pcm_sframes_t availableFrames = snd_pcm_avail(inputDevice);

    if (availableFrames < 0)
    {
      OUTPUT_WARNING("Error fetch available frames, recovering...");
      snd_pcm_recover(inputDevice, static_cast<int>(availableFrames), 0);
      Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
      break;
    }
    else // available frames OK
    {
      unsigned numReadableFrames = std::min(static_cast<unsigned>(availableFrames), maxFrames);
      int numAudioFrames = static_cast<int>(snd_pcm_readi(inputDevice, audioFramesInt.data(), numReadableFrames));

      // Try to recover the audio device
      if (numAudioFrames < 0)
      {
        OUTPUT_WARNING("Audio record error, recovering...");
        numAudioFrames = snd_pcm_recover(inputDevice, numAudioFrames, 0);
        Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
        break;
      }
      else if (numAudioFrames == 0)
      {
        Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
        // falls through to repeating from the top of for(;;) and polls again without returning an empty AudioData - this is intended
      }
      else // we have some audio frame
      {
        // Deinterleave the frames for the wakeupAPI
        waitBuffer.insert(waitBuffer.end(), audioFramesInt.begin(),
                          audioFramesInt.begin() + (numAudioFrames * numChannels));

        const size_t maxWaitSize = wakeupAPIBufferSize * numChannels * 2;
        if (waitBuffer.size() > maxWaitSize)
          waitBuffer.erase(waitBuffer.begin(), waitBuffer.end() - maxWaitSize);

        if (waitBuffer.size() > (wakeupAPIBufferSize * numChannels))
        {
          // deinterleave from waitBuffer
          for (size_t bufferInd = 0; bufferInd < wakeupAPIBufferSize; bufferInd++)
          {
            inputAudioBuffers[0][bufferInd] = waitBuffer[bufferInd * numChannels + 0];
            inputAudioBuffers[1][bufferInd] = waitBuffer[bufferInd * numChannels + 1];
            inputAudioBuffers[2][bufferInd] = waitBuffer[bufferInd * numChannels + 2];
            inputAudioBuffers[3][bufferInd] = waitBuffer[bufferInd * numChannels + 4];
          }

          Duerwen_wakeup_three_write_data(wakeupHandle, inputAudioBuffers[0].data(), inputAudioBuffers[1].data(),
                                          inputAudioBuffers[2].data(), inputAudioBuffers[3].data(),
                                          outputAudioBuffers[0].data(), outputAudioBuffers[1].data(),
                                          outputAudioBuffers[2].data(), outputAudioBuffers[3].data());

          // interleave the output back into audioFrameInt
          for (size_t bufferInd = 0; bufferInd < wakeupAPIBufferSize; bufferInd++)
          {
            audioFramesInt[bufferInd * 3 + 0] = outputAudioBuffers[0][bufferInd];
            audioFramesInt[bufferInd * 3 + 1] = outputAudioBuffers[1][bufferInd];
            audioFramesInt[bufferInd * 3 + 2] = outputAudioBuffers[2][bufferInd];
          }

          // erase the waitBuffer data we just processed
          waitBuffer.erase(waitBuffer.begin(), waitBuffer.begin() + (wakeupAPIBufferSize * numChannels));

          // finally convert the signed audioFramesInt samples to float samples
          unsigned firstSample = 0; // audioData.samples.size();
          unsigned samplesSize = wakeupAPIBufferSize * audioData.channels;
          audioData.samples.resize(firstSample + samplesSize);
          for (unsigned sampleIndex = firstSample; sampleIndex < (firstSample + samplesSize); sampleIndex++)
            audioData.samples[sampleIndex] = static_cast<float>(audioFramesInt[sampleIndex]) / INT16_MAX;

          break;
        }
        else
        {
          // If we read nonzero frames but they weren't enough to complete a
          // wakeupAPI chunk (the `if` above didn't run), we'd otherwise loop
          // straight back to snd_pcm_avail with no sleep at all. Add the same
          // short throttle here so partial-chunk reads can't spin tightly.
          Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
        }
      }
    }
  }

  audioTimestamp = Time::getCurrentSystemTime();
#else // not TARGET_ROBOT
  audioData.samples.clear();
  Thread::sleep(30);
  audioTimestamp = Time::getCurrentSystemTime();
#endif
}
