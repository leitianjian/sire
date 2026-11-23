#ifndef SIRE_PROFILER_HPP_
#define SIRE_PROFILER_HPP_

// Project-level profiling API.
//
// Goals:
// - One stable set of macros used across the codebase.
// - Backend is selectable at build time (default: no-op).
// - When disabled, overhead should be effectively zero.
//
// Backends:
// - Tracy: define SIRE_PROFILE_TRACY (and link Tracy::TracyClient).

#include <cstdint>

#if defined(SIRE_PROFILE_TRACY)
  #include <tracy/Tracy.hpp>

  #define SIRE_PROFILE_FUNCTION() ZoneScoped
  #define SIRE_PROFILE_SCOPE(name_literal) ZoneScopedN(name_literal)
  #define SIRE_PROFILE_FRAME() FrameMark
  #define SIRE_PROFILE_FRAME_NAMED(name_literal) FrameMarkNamed(name_literal)
  #define SIRE_PROFILE_PLOT(name_literal, value) TracyPlot(name_literal, value)
  #define SIRE_PROFILE_TEXT(msg_literal) TracyMessageLiteral(msg_literal)
#else
  #define SIRE_PROFILE_FUNCTION() ((void)0)
  #define SIRE_PROFILE_SCOPE(name_literal) ((void)0)
  #define SIRE_PROFILE_FRAME() ((void)0)
  #define SIRE_PROFILE_FRAME_NAMED(name_literal) ((void)0)
  #define SIRE_PROFILE_PLOT(name_literal, value) ((void)0)
  #define SIRE_PROFILE_TEXT(msg_literal) ((void)0)
#endif

#endif  // SIRE_PROFILER_HPP_
