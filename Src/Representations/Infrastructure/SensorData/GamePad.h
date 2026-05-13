/**
 * @file GamePad.h
 *
 * A struct that represents the current snapshot of an Xbox style game pad/controller
 * that may be used to remotely control a robot
 *
 * @author Rudi Villing
 */

#pragma once

#include "Streaming/EnumIndexedArray.h"
#include "Streaming/Function.h"

STREAMABLE(GamePad,
{
  // intentionally using an enum bit mask rather than ENUM with consecutive values
  HIDE_COMMAS(enum Button : unsigned
  {
    a     = 0x0001,
    b     = 0x0002,
    x     = 0x0004,
    y     = 0x0008,
    lb    = 0x0010,
    rb    = 0x0020,
    lt    = 0x0040,  // digital threshold; analog is lTrigger
    rt    = 0x0080,
    ls    = 0x0100,  // left stick click
    rs    = 0x0200,
    back  = 0x0400,
    start = 0x0800,
    guide = 0x1000 // unused for now
  });

  ENUM(Hat,
  {,
    centered,
    up,
    upRight,
    right,
    downRight,
    down,
    downLeft,
    left,
    upLeft,
  });

  bool hatChanged(GamePad& prev) const { return hat != prev.hat; }
  bool hatActive() const { return hat != Hat::centered; }

  bool buttonsChanged(GamePad& prev) const { return buttons != prev.buttons; }
  unsigned buttonsNew(GamePad& prev) const { return ~prev.buttons & buttons; } // just pressed in this snapshot
  unsigned buttonsReleased(GamePad& prev) const { return prev.buttons & ~buttons; } // just released in this snapshot

  /**** separate header from streamable fields - comma at end of comment ****/,

  (bool)(false) connected,

  (unsigned)(0) buttons,         // currently held - e.g. if (buttons & (Button::lt | Button::a)) doSomething...

  (float)(0.f) lStickX,   // –1.0 (left) … +1.0 (right)
  (float)(0.f) lStickY,   // –1.0 (fwd)  … +1.0 (back)
  (float)(0.f) rStickX,
  (float)(0.f) rStickY,

  // analogue triggers - not used currently
  // (float)(0.f) lTrigger,  // 0.0 (off) … 1.0 (full)
  // (float)(0.f) rTrigger,

  (Hat)(centered) hat,
});