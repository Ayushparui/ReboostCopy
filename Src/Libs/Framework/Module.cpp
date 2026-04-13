/**
 * @file Module.cpp
 * The class attributes of the module handling schema.
 * @author Thomas Röfer
 * @author Rudi Villing (modifications)
 */

#include "Module.h"
#include "Streaming/InStreams.h"

#include "TextLogging/TextLogging.h"

DECL_TLOGGER(tlogger, "ModuleBase", TextLogging::DEBUG);

ModuleBase* ModuleBase::first = nullptr;

void loadModuleParameters(Streamable& parameters, const char* moduleName, const char* fileName, const char* prefix)
{
  // we will allow config files with name identical to module name in addition to the
  // B-Human convention of forcing initial uppercase to lowercase

  const std::string pfx = prefix ? prefix : "";

  // Build primary name: if no fileName, apply B-Human lowercase convention
  std::string name;
  if (!fileName)
  {
    name = moduleName;
    name[0] = static_cast<char>(tolower(name[0]));
    if (name.size() > 1 && isupper(name[1]))
      for (int i = 1; i + 1 < static_cast<int>(name.size()) && isupper(name[i + 1]); ++i)
        name[i] = static_cast<char>(tolower(name[i]));
    name = pfx + name + ".cfg";
  }
  else
    name = pfx + fileName;

  // Try primary name (scope block makes RAII lifetime explicit)
  {
    InMapFile stream(name);
    if (stream.exists())
    {
      TLOGD(tlogger, "Loading config '{}'", name);
      stream >> parameters;
      return;
    }
  }

  // If no explicit fileName was given, also try the module name without lowercase mod
  if (!fileName)
  {
    const std::string fallback = pfx + moduleName + ".cfg";
    InMapFile stream(fallback);
    if (stream.exists())
    {
      TLOGD(tlogger, "Loading config '{}'", fallback);
      stream >> parameters;
      return;
    }
    TLOG_ABORT(tlogger, "File '{}' (alt: '{}') does not exist in search path", name, fallback);
  }
  else
  {
    TLOG_ABORT(tlogger, "File '{}' does not exist in search path", name);
  }
}
