/**
 * @file REAudioProvider.h
 *
 * @description: RoboEireann Audio Provider Module Header
 *
 * @author Shauna Recto
 */

#pragma once

#ifdef TARGET_ROBOT
#include <alsa/asoundlib.h>
#endif

#include "Framework/Module.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include <string>

MODULE(REAudioProvider,
{,
  REQUIRES(GameState),
  REQUIRES(DamageConfigurationHead),
  REQUIRES(AudioData),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  PROVIDES(FrameInfo),
  LOADS_PARAMETERS(
  {,
    (unsigned) retries,              /* Number of tries to open device. */
    (unsigned) retryDelay,           /* Delay before a retry to open device. */
    (unsigned) sampleRate,           /* Sample rate to capture on microphone. */
    (unsigned) maxFrames,            /* Maximum number of frames read in one tick. */
    (std::string) deviceName,        /* The ALSA sound card to be used for audio capture. */
    (std::string) mixerName,         /* The ALSA sound card to be used for audio capture. */
    (float) captureVolume,           /* The microphone volume in percent. */
  }),
});

class REAudioProvider : public REAudioProviderBase
{
private:

#ifdef TARGET_ROBOT
  snd_pcm_t* inputDevice;
  int channels;
  float currentCaptureVolume = -1.f;

  bool setVolume(const std::string& element, float volume);
#endif

  unsigned tickWatchDog = 0;

  void update(AudioData& audioData) override;
  void update(FrameInfo& frameInfo) override {frameInfo.time = tickWatchDog;};

public:

  REAudioProvider();
  ~REAudioProvider();
};
