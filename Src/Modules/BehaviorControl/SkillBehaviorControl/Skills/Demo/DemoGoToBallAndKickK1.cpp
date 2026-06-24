/**
 * @file DemoGoToBallAndKickK1.cpp
 *
 * This file implements the DemoGoToBallAndKickK1 skill.
 *
 * It is a variant of DemoGoToBallAndKick that uses the K1 specific kicks.
 * Currently only "newKick" is supported. This should be expanded as we are
 * forming a library of K1 compatible kicks
 * 
 * @authos James Florin Petri
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoGoToBallAndKickK1,
       defs((Vector2f)({-200.f, 0.f}) waitPosition, /**< We can not trust the detected ball, therefor wait behind the known ball location. */
            (int)(1500) ballSeenDuration, /**< The ball was seen in this time frame, therefore we can trust the detection. */
            (int)(1000) lastKickDuration, /**< Ignore ball detections when we recently kicked. */
            (float)(350.f) ballDistanceClose, /**< Distance to consider the ball being close. */
            (int)(1500) timeForBallBeingCloseToKick), /**< Ball must be close for this time, to start kick. */
       vars((unsigned)(theFrameInfo.time) lastTimeBallWasClose)) /**< Timestamp ball was last time close. */
{
  if(theBallModel.estimate.position.squaredNorm() > sqr(ballDistanceClose))
    lastTimeBallWasClose = theFrameInfo.time;

  const bool shouldKick = (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < ballSeenDuration  // We recently saw the ball
                           || theFrameInfo.getTimeSince(theMotionInfo.lastKickTimestamp) > lastKickDuration) // The last kick was long ago
                          && theBallModel.timeWhenLastSeen > theMotionInfo.lastKickTimestamp + lastKickDuration // The detected ball timestamp must be AFTER the
                          && theFrameInfo.getTimeSince(lastTimeBallWasClose) > timeForBallBeingCloseToKick; // Walk close to the ball and stop before kicking

  initial_state(goToLastKnownPosition)
  {
    transition
    {
      if(shouldKick && state_time > 2000)
        goto kickBall;
    }
    action
    {
      LookAtBall();
      WalkToPoint({.target = {theBallModel.estimate.position + waitPosition + Vector2f(0.f, theRobotDimensions.yHipOffset)},
                   .reduceWalkSpeedType = ReduceWalkSpeedType::slow,
                   .rough = true,
                   .disableObstacleAvoidance = true,
                   .disableStanding = true});
    }
  }

  state(kickBall)
  {
    transition
    {
      if(!shouldKick)
        goto goToLastKnownPosition;
    }
    action
    {
      LookAtBall();
      WalkToBallAndKick({.targetDirection = -theKickInfo[KickInfo::newKick].rotationOffset,
                         .kickType = KickInfo::newKick});
    }
  }
}
