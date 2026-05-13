/**
 * @file KeyStates.h
 *
 * Declaration of struct KeyStates
 *
 * @author unknown
 * @author Jesse Richter-Klug
 */

#pragma once

#include "Streaming/EnumIndexedArray.h"
#include "Streaming/Function.h"

/**
 * The struct represents the states of the keys.
 */
STREAMABLE(KeyStates,
{
  ENUM(Key,
  {,
    // Nao touch sensors:
    nao_firstKey,
    headFront = nao_firstKey,
    headMiddle,
    headRear,
    lHandBack,
    lHandLeft,
    lHandRight,
    rHandBack,
    rHandLeft,
    rHandRight,

    // Nao bumpers:
    lFootLeft,
    lFootRight,
    rFootLeft,
    rFootRight,
    chest,

    nao_lastKey = chest,

    // BoosterK1
    k1_firstKey,
    k1_f1 = k1_firstKey,
    k1_stand,
    k1_walk,
    k1_lastKey = k1_walk
  });

  KeyStates();

  /**** separate header from streamable fields, note comma after comment ****/,

  (ENUM_INDEXED_ARRAY(bool, Key)) pressed,
});

inline KeyStates::KeyStates()
{
  pressed.fill(false);
}

STREAMABLE_WITH_BASE(EnhancedKeyStates, KeyStates,
{
  /*
   * calculates the 'current' hit streak for a given key state sensor
   *
   * @param key the sensor
   * @param timeOut is if the sensor is triggered longer than this, the trigger state gets rejected
   * @param allowedTimeBetweenHitStreak allowed time between to hits to be considered as streak
   *
   * @return the size of the current finished hit streak recently
   */
  FUNCTION(unsigned(KeyStates::Key key, unsigned timeOut, unsigned allowedTimeBetweenHitStreak)) getHitStreakFor;

  /*
   * checks whether a button press just reached a certain duration in this frame
   *
   * @param key the sensor
   * @param duration the duration for which to check
   *
   * @return whether a button press just reached a certain duration in this frame
   */
  FUNCTION(bool(KeyStates::Key key, unsigned duration)) isPressedFor,

  (ENUM_INDEXED_ARRAY(unsigned, Key)) hitStreak,       ///< hit streak per key calculated with default parameters of the provider (finished recently)
  (ENUM_INDEXED_ARRAY(unsigned, Key)) pressedDuration, ///< duration per key since this one is pressed
                                                       ///<     (0 == pressed[key] for the first time this frame || !pressed[key]) -> True
});
