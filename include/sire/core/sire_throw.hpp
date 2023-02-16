#ifndef SIRE_THROW_HPP_
#define SIRE_THROW_HPP_

#include <type_traits>

#include "sire/core/sire_assert.hpp"

/// @file
/// Provides a convenient wrapper to throw an exception when a condition is
/// unmet.  This is similar to an assertion, but uses exceptions instead of
/// ::abort(), and cannot be disabled.

namespace sire {
namespace core {
// Throw an error message.
[[noreturn]] void Throw(const char* condition, const char* func,
                        const char* file, int line);
}  // namespace core
}  // namespace sire

/// Evaluates @p condition and iff. the value is false will throw an exception
/// with a message showing at least the condition text, function name, file,
/// and line.
///
/// The condition must not be a pointer, where we'd implicitly rely on its
/// nullness. Instead, always write out "!= nullptr" to be precise.
///
/// Correct: `SIRE_THROW_UNLESS(foo != nullptr);`
/// Incorrect: `SIRE_THROW_UNLESS(foo);`
///
/// Because this macro is intended to provide a useful exception message to
/// users, we should err on the side of extra detail about the failure. The
/// meaning of "foo" isolated within error message text does not make it
/// clear that a null pointer is the proximate cause of the problem.
#define SIRE_THROW_UNLESS(condition)                                         \
  do {                                                                       \
    typedef ::sire::assert::ConditionTraits<                                 \
        typename std::remove_cv_t<decltype(condition)>>                      \
        Trait;                                                               \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    static_assert(                                                           \
        !std::is_pointer_v<decltype(condition)>,                             \
        "When using SIRE_THROW_UNLESS on a raw pointer, always write out "   \
        "SIRE_THROW_UNLESS(foo != nullptr), do not write SIRE_THROW_UNLESS"  \
        "(foo) and rely on implicit pointer-to-bool conversion.");           \
    if (!Trait::Evaluate(condition)) {                                       \
      ::sire::core::Throw(#condition, __func__, __FILE__, __LINE__);         \
    }                                                                        \
  } while (0)

#endif