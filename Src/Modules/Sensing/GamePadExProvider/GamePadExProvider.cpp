/**
 * GamePadExProvider.cpp
 *
 * @author Rudi Villing
 */

#include "GamePadExProvider.h"
#include "TextLogging/TextLogging.h"

MAKE_MODULE(GamePadExProvider);

DECL_TLOGGER(tlogger, "GamePadExProvider", TextLogging::INFO);

void GamePadExProvider::update(GamePadEx& gamePadEx)
{
  gamePadEx.prevButtons = gamePadEx.buttons;
  gamePadEx.prevHat = gamePadEx.hat;
  static_cast<GamePad &>(gamePadEx) = theGamePad;
}