/**
 * @file GamePadControlK1.cpp
 *
 * A module to control the K1 robot using its GamePad (by passing behaviour).
 * It provides the motion request and head motion request directly.
 *
 * Keymap:
 *
 * LB + back (-)      | disable control
 * LB + start (+)     | enable control
 * LB + X             | unstiff
 * LB + B             | prepare
 * LB + A             | stand (active) - required before any walking/kicking
 *
 * L stick            | walk fwd/back/sideways/diagonal
 * R stick            | walk rotation
 * Hat                | pan and tilt head
 *
 * RT + L/R stick     | dribble TODO
 * 
 * RT + A             | Kick left foot
 * RT + B             | Kick right foot
 *
 * @author Rudi Villing
 */

#include "GamePadControlK1.h"
#include "TextLogging/TextLogging.h"



MAKE_MODULE(GamePadControlK1);

DECL_TLOGGER(tlogger, "GamePadControlK1", TextLogging::INFO);


void GamePadControlK1::update(MotionRequest& theMotionRequest)
{
  frameNumber++;

  theMotionRequest.odometryData = theOdometryData;

  if (theRobotSafetyState.state != RobotSafetyState::operational)
  {
    lastGamePad = GamePad();

    if (theMotionRequest.motion != MotionRequest::playDead)
    {
      theMotionRequest.motion = MotionRequest::playDead;
      theMotionRequest.standHigh = true;

      TLOGI(tlogger, "safety state not operational, so playDead");
    }
  }
  else if (theGamePad.connected)
  {
    if (theGamePad.justPressed(GamePad::maskLB | GamePad::maskBack, lastGamePad) && controlEnabled)
    {
      controlEnabled = false;
      TLOGI(tlogger, "GamePad control disabled");
    }
    else if (theGamePad.justPressed(GamePad::maskLB | GamePad::maskStart, lastGamePad) && !controlEnabled)
    {
      controlEnabled = true;
      TLOGI(tlogger, "GamePad control enabled");
    }
    else if (theGamePad.pressed(GamePad::maskLB | GamePad::maskX) && (theMotionRequest.motion != MotionRequest::playDead))
    {
      theMotionRequest.motion = MotionRequest::playDead;
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh = true;
      TLOGI(tlogger, "requesting MotionRequest::playDead");
    }
    else if (theGamePad.pressed(GamePad::maskLB | GamePad::maskB) && (theMotionRequest.motion != MotionRequest::prepare))
    {
      theMotionRequest.motion = MotionRequest::prepare;
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh = true;
      TLOGI(tlogger, "requesting MotionRequest::prepare");
    }
    else if (theGamePad.justPressed(GamePad::maskLB | GamePad::maskA, lastGamePad))
    {
      theMotionRequest.motion = MotionRequest::stand;
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh ^= true; // toggle stand high
      TLOGI(tlogger, "requesting MotionRequest::stand, high={}", theMotionRequest.standHigh);
    }
    else if (!updateKickMotion(theMotionRequest))
    {
      updateWalkMotion(theMotionRequest);
    }
  }
  else // GamePad not connected
  {
    stopGracefully(theMotionRequest);
  }

  lastGamePad = theGamePad;
}


void GamePadControlK1::setHeadZero(bool keepJointsEnabled)
{
  headMotionRequest.mode = HeadMotionRequest::panTiltMode;
  headMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
  headMotionRequest.pan = keepJointsEnabled ? 0_deg : JointAngles::off;
  headMotionRequest.tilt = keepJointsEnabled ? 0_deg : JointAngles::off;
  headMotionRequest.speed = panTiltSpeed;
  headMotionRequest.stopAndGoMode = false;
}


bool GamePadControlK1::updateKickMotion(MotionRequest& theMotionRequest)
{
  if (!theGamePad.pressed(GamePad::maskRT)) // are we doing any sort of kicking?
    return false;
  if (!theGamePad.anyPressed(GamePad::maskA | GamePad::maskB))
    return false;

  KickInfo::KickType kickType = KickInfo::walkForwardsLeftLong;
  if (theGamePad.pressed(GamePad::maskB))
    kickType = KickInfo::walkForwardsRightLong;


  auto applyDeadZone = [this](float value)
  { 
    return (std::abs(value) <= deadZone ? 0.f : (value - deadZone * sgn(value)) / (1.f - deadZone)); 
  };

  // for kicks, the motion engine decides where to walk based on the ball position
  // so the GamePad x,y input only say how fast to move, not whether to move in x
  // or in y. Hence we just extract the max magnitude of these inputs.

  const float speed = std::max(std::abs(applyDeadZone(theGamePad.lStickX)),
                               std::abs(applyDeadZone(theGamePad.lStickY)));

  theMotionRequest.motion = MotionRequest::walkToBallAndKick;
  theMotionRequest.kickType = kickType;
  theMotionRequest.walkSpeed = {speed, speed, speed};
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;
  theMotionRequest.directionPrecision = {-5_deg, 5_deg};
  theMotionRequest.standHigh = true;
  theMotionRequest.energySavingWalk = false;

  if (intervalLogKick.triggered(5000u))
    TLOGI(tlogger, "requesting MotionRequest::walkToBallAndKick {}",
          TypeRegistry::getEnumName(theMotionRequest.kickType));

  // TODO in future add possibility of dribble either here or in a new method

  return true;
}



void GamePadControlK1::updateWalkMotion(MotionRequest& theMotionRequest)
{
  auto applyDeadZone = [this](float value)
  { 
    return (std::abs(value) <= deadZone ? 0.f : (value - deadZone * sgn(value)) / (1.f - deadZone)); 
  };

  theMotionRequest.energySavingWalk = false;
  theMotionRequest.standHigh = false;

  theMotionRequest.walkSpeed.translation.x() = applyDeadZone(theGamePad.lStickX);
  theMotionRequest.walkSpeed.translation.y() = applyDeadZone(theGamePad.lStickY);
  theMotionRequest.walkSpeed.rotation = applyDeadZone(theGamePad.rStickX);

  if (theMotionRequest.walkSpeed != Pose2f())
    theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
  else
    theMotionRequest.motion = MotionRequest::stand;

  if (intervalLogWalk.triggered(5000u))
    TLOGI(tlogger, "requesting MotionRequest::{} speed={{ rot={:.2f}, x={:.2f}, y={:.2f} }}",
          TypeRegistry::getEnumName(theMotionRequest.motion), (float)theMotionRequest.walkSpeed.rotation,
          theMotionRequest.walkSpeed.translation.x(), theMotionRequest.walkSpeed.translation.y());
}



void GamePadControlK1::stopGracefully(MotionRequest& theMotionRequest)
{
  using State = StopGracefullyState;
  using FsmType = Fsm<StopGracefullyState>;
  FsmType& fsm = stopGracefullyFsm;

  auto transitionTo = [&](State newState)
  {
    fsm.state = newState;
    fsm.firstFrame = true;
  };


  // handle auto-reset
  fsm.autoResetIfNeeded(frameNumber);
  
  if (fsm.state == State::start)
  {
    switch (theMotionRequest.motion)
    {
    case MotionRequest::playDead: transitionTo(State::playDead);
    case MotionRequest::prepare: transitionTo(State::prepare);

    case MotionRequest::walkAtAbsoluteSpeed:
    case MotionRequest::walkAtRelativeSpeed:
    case MotionRequest::walkToPose:
    case MotionRequest::walkToBallAndKick:
    case MotionRequest::dribble: transitionTo(State::stopWalking);

    default: transitionTo(State::stand);
    }
  }

  // read firstFrame after initial_state so any transition it made is seen correctly
  bool firstFrame = fsm.firstFrame;
  fsm.firstFrame = false;  

  // WARNING WARNING WARNING
  // The following FSM works without some additional tricks like goto labels
  // because all transitions go forward to states written after. If this ever
  // changes you will either need to add a loop mechanism (care needed) or
  // add something like the if (false) { label stateName: .... } trick of CABSL

  if (fsm.state == State::stopWalking)
  {
    if (!firstFrame)
    {
      if (theMotionInfo.speed.translation.norm() == 0.f && theMotionInfo.speed.rotation == 0.f)
        transitionTo(State::prepare);
    }
    {
      if (firstFrame)
        TLOGI(tlogger, "stopGracefully: stopWalking");

      theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
      theMotionRequest.walkSpeed = {0.f, 0.f, 0.f};
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh = true;

      setHeadZero();
    }
  }

  if (fsm.state == State::stand)
  {
    if (!firstFrame)
    {
      if (theMotionInfo.executedPhase == MotionPhase::stand)
        transitionTo(State::prepare);
    }
    {
      if (firstFrame)
        TLOGI(tlogger, "stopGracefully: stand");
        
      theMotionRequest.motion = MotionRequest::stand;
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh = true;

      setHeadZero();
    }
  }

  if (fsm.state == State::prepare)
  {
    fsm.done = true;

    {
      if (firstFrame)
        TLOGI(tlogger, "stopGracefully: prepare");
        
      theMotionRequest.motion = MotionRequest::prepare;
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh = true;

      setHeadZero();
    }
  }

  if (fsm.state == State::playDead)
  {
    fsm.done = true;

    {
      if (firstFrame)
        TLOGI(tlogger, "stopGracefully: playDead");
        
      theMotionRequest.motion = MotionRequest::playDead;
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh = true;

      setHeadZero(false);
    }
  }
}

