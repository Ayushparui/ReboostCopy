/**
 * @file Annotation.h
 * @author Andreas Stolpmann
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Debugging/AnnotationManager.h"
#include "Debugging/Debugging.h"

#include "fmt/core.h"

/**
 * A macro for sending annotation messages.
 *
 * @param name The part of the system where this annotation originates.
 * @param message A message streamable as text.
 *
 * Examples:
 * <pre>
 * ANNOTATION("Behavior", "Kickin'!");
 * ANNOTATION("TacticProvider", "Changed Tactic from " << oldTactic << " to " << newTactic << ".");
 * </pre>
 */
#define ANNOTATION(name, message)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    char _buf[1024];                                                                                                   \
    OutTextMemory _text(sizeof(_buf), _buf);                                                                           \
    _text << name << message;                                                                                          \
    Global::getAnnotationManager().add().write(_text.data(), _text.size());                                            \
  } while (false)


/**
 * A macro for sending annotation messages with fmtlib style formatting
 * 
 * usage:
 * 
 *   ANNOTATON_FMT("someName", "someFormatString {}", argsToSubstituteIntoFormatString...);
 */
#define ANNOTATION_FMT(name, ...)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    fmt::memory_buffer _msgBuf;                                                                                        \
    fmt::format_to(std::back_inserter(_msgBuf), __VA_ARGS__);                                                          \
    _msgBuf.push_back('\0');                                                                                           \
    ANNOTATION(name, _msgBuf.data());                                                                                  \
  } while (false)
