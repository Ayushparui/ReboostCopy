/**
 * @file BHLoggingController.cpp
 *
 * This file implements the class \c BHLoggingController.
 *
 * @author Arne Hasselbring
 */

#include "BHLoggingController.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/RobotSafetyState.h"
#include "Framework/Blackboard.h"
#include "Platform/Time.h"
#include "Streaming/InStreams.h"

BHLoggingController::BHLoggingController() :
  gameState(Blackboard::getInstance().alloc<GameState>("GameState")),
  robotSafetyState(Blackboard::getInstance().alloc<RobotSafetyState>("RobotSafetyState"))
{
  InMapFile stream("teamList.cfg");
  if(stream.exists())
  {
    TeamList teamList;
    stream >> teamList;
    for(const Team& team : teamList.teams)
      teams[team.number] = team.name;
  }

  InMapFile paramsStream("BHLoggingController.cfg");
  if (paramsStream.exists())
  {
    paramsStream >> params;
  }
}

BHLoggingController::~BHLoggingController()
{
  Blackboard::getInstance().free("GameState");
}

bool BHLoggingController::shouldLog(bool wasLogging) const
{
  // We want to log in most states, but there are exceptions:
  // - The INITIAL state is excluded.
  // - The FINISHED state is excluded, except for the first 10s if we have been logging before.
  // return !(gameState.isInitial(/* orStandby: */ false)
  //          || (gameState.isFinished() && !(wasLogging && Time::getTimeSince(gameState.timeWhenStateStarted) < 10000)));
  // return !((gameState.isFinished() && !(wasLogging && Time::getTimeSince(gameState.timeWhenStateStarted) < 10000)));

  bool isSoftwareStartupNow = isSoftwareStartup;
  isSoftwareStartup = false;

  if (!wasLogging) // should we start logging
  {
    // Start if ANY start condition is met
    return std::any_of(params.startConditions.begin(), params.startConditions.end(),
                       [this,isSoftwareStartupNow](LoggingCondition cond) -> bool
                       {
                         switch (cond)
                         {
                         case softwareStart: return isSoftwareStartupNow; // recommend prepare as an alternative if you need logging to restart after finishing
                         case safetyStateOperational: return robotSafetyState.state == RobotSafetyState::operational;
                         case safetyStateSafetyTripped: return robotSafetyState.state == RobotSafetyState::safetyTripped;
                         case playerStatePrepare: return gameState.playerState == GameState::prepare;
                         case playerStateActive: return gameState.playerState == GameState::active;
                         case stateInitial: return gameState.isInitial(false);
                         case stateReady: return gameState.isReady();
                         case stateSet: return gameState.isSet();
                         case statePlaying: return gameState.isPlaying();
                         case penaltyShootoutInitial: return gameState.state == GameState::beforePenaltyShootout;
                         case penaltyShootoutShotSet:
                           return (gameState.state == GameState::waitForOwnPenaltyShot) ||
                                  (gameState.state == GameState::waitForOpponentPenaltyShot);
                         case penaltyShootoutShotPlaying:
                           return (gameState.state == GameState::ownPenaltyShot) ||
                                  (gameState.state == GameState::opponentPenaltyShot);
                         case penaltyShootoutShotFinished:
                           return (gameState.state == GameState::afterOwnPenaltyShot) ||
                                  (gameState.state == GameState::afterOpponentPenaltyShot);
                         default: // unstiff, finished, etc don't make sense for start conditions
                           return gameState.isReady() || gameState.isSet() || gameState.isPlaying();
                         }
                       });
  }
  else // should we continue logging (true) or stop logging (false)?
  {
    const bool safetyStateLongEnough =
        Time::getTimeSince(robotSafetyState.timeWhenStateStarted) > params.stopConditionTimeout;
    const bool playerStateLongEnough =
        Time::getTimeSince(gameState.timeWhenPlayerStateStarted) >= params.stopConditionTimeout;
    const bool gameStateLongEnough = Time::getTimeSince(gameState.timeWhenStateStarted) >= params.stopConditionTimeout;

    // Stop logging (return false) if ANY stop condition triggers
    const bool anyStopTriggered = std::any_of(
        params.stopConditions.begin(), params.stopConditions.end(),
        [this, playerStateLongEnough, gameStateLongEnough, safetyStateLongEnough](LoggingCondition cond) -> bool
        {
          switch (cond)
          {
          case safetyStateOperational:
            return (robotSafetyState.state == RobotSafetyState::operational) && safetyStateLongEnough;
          case safetyStateSafetyTripped:
            return (robotSafetyState.state == RobotSafetyState::safetyTripped) && safetyStateLongEnough;
          case playerStateUnstiff: return (gameState.playerState == GameState::unstiff && playerStateLongEnough);
          case playerStatePrepare: return (gameState.playerState == GameState::prepare && playerStateLongEnough);
          case stateReady: return (gameState.isReady() && gameStateLongEnough);
          case stateSet: return (gameState.isSet() && gameStateLongEnough);
          case statePlaying: return (gameState.isPlaying() && gameStateLongEnough);
          case stateFinished: return (gameState.isFinished() && gameStateLongEnough);
          case penaltyShootoutShotSet:
            return ((gameState.state == GameState::waitForOwnPenaltyShot) ||
                    (gameState.state == GameState::waitForOpponentPenaltyShot)) &&
                   gameStateLongEnough;
          case penaltyShootoutShotPlaying:
            return ((gameState.state == GameState::ownPenaltyShot) ||
                    (gameState.state == GameState::opponentPenaltyShot)) &&
                   gameStateLongEnough;
          case penaltyShootoutShotFinished:
            return ((gameState.state == GameState::afterOwnPenaltyShot) ||
                    (gameState.state == GameState::afterOpponentPenaltyShot)) &&
                   gameStateLongEnough;
          default:       // most states don't make sense to stop logging
            return true; // continue logging
          }
        });

    return anyStopTriggered ? false : true; // should we continue logging?
  }
}

std::string BHLoggingController::getDescription() const
{
  if(gameState.gameControllerActive)
  {
    auto team = teams.find(static_cast<std::uint8_t>(gameState.opponentTeam.number));
    if(team != teams.end())
      return team->second + "_"
             + (gameState.phase == GameState::penaltyShootout ? "shootout"
                : gameState.phase == GameState::firstHalf ? "half1" : "half2");
  }
  return "NoGC";
}
