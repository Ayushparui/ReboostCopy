#include "SkillBehaviorControl.h"

DECL_OPTION_TLOGGER(HandlePlayerState, TextLogging::INFO);


option((SkillBehaviorControl) HandlePlayerState)
{
  common_transition
  {
    switch(theGameState.playerState)
    {
      case GameState::unstiff:
        goto unstiff;
      case GameState::prepare:
        goto prepare;
      case GameState::calibration:
        goto calibration;
      case GameState::active:
        if(theExtendedGameState.wasPenalized())
        {
          ANNOTATION("Behavior", "Unpenalized");
          SystemCall::say("Not penalized");
        }
        goto playing;
      default:
        if(!theExtendedGameState.wasPenalized())
        {
          ANNOTATION("Behavior", "Penalized " << TypeRegistry::getEnumName(theGameState.playerState));
          SystemCall::say("Penalized");
        }
        goto penalized;
    }
    FAIL("Unknown player state.");
  }

  initial_state(playing)
  {
    TLOGD_STATE_ENTRY(playing);
    if (stateChanged.valueChanged())
      TLOGD(tlogger, "PlayerState {}, GameState {}", TypeRegistry::getEnumName(theGameState.playerState),
            TypeRegistry::getEnumName(theGameState.state));
  }

  state(unstiff)
  {
    action
    {
      TLOGD_STATE_ENTRY(unstiff);
      LookAtAngles({.pan = JointAngles::off, .tilt = JointAngles::off});
      PlayDead();
    }
  }

  state(prepare)
  {
    action 
    { 
      TLOGD_STATE_ENTRY(prepare);
      LookAtAngles({.pan = theJointAngles.angles[Joints::headYaw], .tilt = theJointAngles.angles[Joints::headPitch]}); 
      Prepare();
    }
  }

  state(calibration)
  {
    action
    {
      CalibrationControl();
    }
  }

  state(penalized)
  {
    action
    {
      TLOGD_STATE_ENTRY(penalized);

      LookForward();
      Stand({.high = true});
    }
  }
}
