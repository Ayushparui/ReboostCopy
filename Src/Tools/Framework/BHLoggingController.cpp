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

  if (!wasLogging) // should we start logging
  {
    switch (params.startCondition)
    {
    case softwareStart: params.startCondition = playerStatePrepare; return true;
    case safetyStateOperational: return robotSafetyState.state == RobotSafetyState::operational;
    case playerStatePrepare: return gameState.playerState == GameState::prepare;
    case playerStateActive: return gameState.playerState == GameState::active;
    case gcStateInitial: return gameState.isInitial(false);
    case gcStateReady: return gameState.isReady();
    case gcStateSet: return gameState.isSet();
    case gcStatePlaying: return gameState.isPlaying();
    default: // unstiff, finished, etc don't make sense for start conditions
      return gameState.isReady() || gameState.isSet() || gameState.isPlaying();
    }
  }
  else // should we continue logging (true) or stop logging (false)?
  {
    const bool playerStateLongEnough = Time::getTimeSince(gameState.timeWhenPlayerStateStarted) >= params.stopConditionTimeout;
    const bool gameStateLongEnough = Time::getTimeSince(gameState.timeWhenStateStarted) >= params.stopConditionTimeout;

    switch (params.stopCondition)
    {
    case playerStateUnstiff: return !(gameState.playerState == GameState::unstiff && playerStateLongEnough);
    case playerStatePrepare: return !(gameState.playerState == GameState::prepare && playerStateLongEnough);
    case gcStateReady: return !(gameState.isReady() && gameStateLongEnough);
    case gcStateSet: return !(gameState.isSet() && gameStateLongEnough);
    case gcStatePlaying: return !(gameState.isPlaying() && gameStateLongEnough);
    case gcStateFinished: return !(gameState.isFinished() && gameStateLongEnough);
    default: // most states don't make sense to stop logging
      return true; // continue logging
    }
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
