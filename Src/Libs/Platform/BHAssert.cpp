/**
 * @file Platform/BHAssert.cpp
 * Some helper functions for low level debugging
 * @author Colin Graf
 */

#include "Platform/BHAssert.h"

#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <iostream>
#ifdef WINDOWS
#include <Windows.h>
#endif

void Assert::print(const char* file, int line, const char* format, ...)
{
  char data[320];
  int length = std::snprintf(data, sizeof(data) - 2, "%s:%d: ", file, line);
  if(length < 0)
    length = sizeof(data) - 2;
  va_list ap;
  va_start(ap, format);
  int i = vsnprintf(data + length, sizeof(data) - length - 2, format, ap);
  if(i < 0)
    length = sizeof(data) - 2;
  else
    length += i;
  va_end(ap);
  data[length++] = '\n';
  data[length] = '\0';
  fputs(data, stderr);
  fflush(stderr);
#ifdef WINDOWS
  OutputDebugString(data);
#endif
}

void Assert::print(const char* file, int line, const std::string& message)
{
#ifdef WINDOWS
  const std::string expandedMessage = std::string(file) + ":" + std::to_string(line) + ": " + message + "\n";
  OutputDebugStringA(expandedMessage.c_str());
#else
  std::cerr << file << ":" << line << ": " << message << std::endl;
#endif
}

void Assert::abort()
{
#ifdef WINDOWS
  __debugbreak();
#elif defined MACOS && defined __x86_64__
  __builtin_trap();
#else
  ::abort();
#endif
}

#if defined TARGET_ROBOT || defined LINUX

#include <signal.h>

const char* Assert::getSignalName(int signum)
{
  const char* signalNames[] =
  {
    "",
    "sigHUP",
    "sigINT",
    "sigQUIT",
    "sigILL",
    "sigTRAP",
    "sigABRT",
    "sigBUS",
    "sigFPE",
    "sigKILL",
    "sigUSR1",
    "sigSEGV",
    "sigUSR2",
    "sigPIPE",
    "sigALRM",
    "sigTERM"
  };

  if (0 <= signum && signum < int(sizeof(signalNames) / sizeof(*signalNames)))
    return signalNames[signum];
  else
  {
    switch (signum)
    {
    case SIGUSR1:
      return "sigUSR1";

    default:
      static char buf[20];
      sprintf(buf, "signal %d", signum);
      return buf;
    }
  }
}

#endif // defined TARGET_ROBOT || defined LINUX

