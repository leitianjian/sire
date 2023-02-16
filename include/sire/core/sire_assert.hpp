#ifndef SIRE_ASSERT_HPP_
#define SIRE_ASSERT_HPP_

#include <type_traits>

/// \file
/// Provides Sire's assertion implementation, which is copied from Sire
/// project.  This is intended to be used both within Sire and by other
/// software. Sire's asserts can be armed and disarmed independently from the
/// system-wide asserts.

#ifdef SIRE_DOXYGEN_CXX
/// @p SIRE_ASSERT(condition) is similar to the built-in @p assert(condition)
/// from the C++ system header @p <cassert>.  Unless Sire's assertions are
/// disarmed by the pre-processor definitions listed below, @p SIRE_ASSERT
/// will evaluate @p condition and iff the value is false will trigger an
/// assertion failure with a message showing at least the condition text,
/// function name, file, and line.
///
/// Assertions are enabled or disabled using the following pre-processor macros:
///
/// - If @p SIRE_ENABLE_ASSERTS is defined, then @p SIRE_ASSERT is armed.
/// - If @p SIRE_DISABLE_ASSERTS is defined, then @p SIRE_ASSERT is disarmed.
/// - If both macros are defined, then it is a compile-time error.
/// - If neither are defined, then NDEBUG governs assertions as usual.
///
/// This header will define exactly one of either @p SIRE_ASSERT_IS_ARMED or
/// @p SIRE_ASSERT_IS_DISARMED to indicate whether @p SIRE_ASSERT is armed.
///
/// This header will define both `constexpr bool sire::kSireAssertIsArmed`
/// and `constexpr bool sire::kSireAssertIsDisarmed` globals.
///
/// One difference versus the standard @p assert(condition) is that the
/// @p condition within @p SIRE_ASSERT is always syntax-checked, even if
/// Sire's assertions are disarmed.
///
/// Treat @p SIRE_ASSERT like a statement -- it must always be used
/// in block scope, and must always be followed by a semicolon.
#define SIRE_ASSERT(condition)
/// Like @p SIRE_ASSERT, except that the expression must be void-valued; this
/// allows for guarding expensive assertion-checking subroutines using the same
/// macros as stand-alone assertions.
#define SIRE_ASSERT_VOID(expression)
/// Evaluates @p condition and iff the value is false will trigger an assertion
/// failure with a message showing at least the condition text, function name,
/// file, and line.
#define SIRE_DEMAND(condition)
/// Silences a "no return value" compiler warning by calling a function that
/// always raises an exception or aborts (i.e., a function marked noreturn).
/// Only use this macro at a point where (1) a point in the code is truly
/// unreachable, (2) the fact that it's unreachable is knowable from only
/// reading the function itself (and not, e.g., some larger design invariant),
/// and (3) there is a compiler warning if this macro were removed.  The most
/// common valid use is with a switch-case-return block where all cases are
/// accounted for but the enclosing function is supposed to return a value.  Do
/// *not* use this macro as a "logic error" assertion; it should *only* be used
/// to silence false positive warnings.  When in doubt, throw an exception
/// manually instead of using this macro.
#define SIRE_UNREACHABLE()
#else  //  SIRE_DOXYGEN_CXX

// Users should NOT set these; only this header should set them.
#ifdef SIRE_ASSERT_IS_ARMED
#error Unexpected SIRE_ASSERT_IS_ARMED defined.
#endif
#ifdef SIRE_ASSERT_IS_DISARMED
#error Unexpected SIRE_ASSERT_IS_DISARMED defined.
#endif

// Decide whether Sire assertions are enabled.
#if defined(SIRE_ENABLE_ASSERTS) && defined(SIRE_DISABLE_ASSERTS)
#error Conflicting assertion toggles.
#elif defined(SIRE_ENABLE_ASSERTS)
#define SIRE_ASSERT_IS_ARMED
#elif defined(SIRE_DISABLE_ASSERTS) || defined(NDEBUG)
#define SIRE_ASSERT_IS_DISARMED
#else
#define SIRE_ASSERT_IS_ARMED
#endif

namespace sire::core {
// Abort the program with an error message.
[[noreturn]] void Abort(const char* condition, const char* func,
                        const char* file, int line);
// Report an assertion failure; will either Abort(...) or throw.
[[noreturn]] void AssertionFailed(const char* condition, const char* func,
                                  const char* file, int line);
namespace assert {
// Allows for specialization of how to bool-convert Conditions used in
// assertions, in case they are not intrinsically convertible.  See
// common/symbolic/expression/formula.h for an example use.  This is a public
// interface to extend; it is intended to be specialized by unusual Scalar
// types that require special handling.
template <typename Condition>
struct ConditionTraits {
  static constexpr bool is_valid = std::is_convertible_v<Condition, bool>;
  static bool Evaluate(const Condition& value) { return value; }
};
}  // namespace assert
}  // namespace sire::core

#define SIRE_UNREACHABLE()                                                  \
  ::sire::core::Abort("Unreachable code was reached?!", __func__, __FILE__, \
                      __LINE__)

#define SIRE_DEMAND(condition)                                                 \
  do {                                                                         \
    typedef ::sire::assert::ConditionTraits<                                   \
        typename std::remove_cv_t<decltype(condition)>>                        \
        Trait;                                                                 \
    static_assert(Trait::is_valid, "Condition should be bool-convertible.");   \
    static_assert(                                                             \
        !std::is_pointer_v<decltype(condition)>,                               \
        "When using SIRE_DEMAND on a raw pointer, always write out "           \
        "SIRE_DEMAND(foo != nullptr), do not write SIRE_DEMAND(foo) "          \
        "and rely on implicit pointer-to-bool conversion.");                   \
    if (!Trait::Evaluate(condition)) {                                         \
      ::sire::core::AssertionFailed(#condition, __func__, __FILE__, __LINE__); \
    }                                                                          \
  } while (0)
#ifdef SIRE_ASSERT_IS_ARMED
// Assertions are enabled.
namespace sire {
constexpr bool kSireAssertIsArmed = true;
constexpr bool kSireAssertIsDisarmed = false;
}  // namespace sire
#define SIRE_ASSERT(condition) SIRE_DEMAND(condition)
#define SIRE_ASSERT_VOID(expression)                                \
  do {                                                               \
    static_assert(std::is_convertible_v<decltype(expression), void>, \
                  "Expression should be void.");                     \
    expression;                                                      \
  } while (0)
#else
// Assertions are disabled, so just typecheck the expression.
namespace sire {
constexpr bool kSireAssertIsArmed = false;
constexpr bool kSireAssertIsDisarmed = true;
}  // namespace sire
#define SIRE_ASSERT(condition)                                              \
  do {                                                                       \
    typedef ::sire::assert::ConditionTraits<                                \
        typename std::remove_cv_t<decltype(condition)>>                      \
        Trait;                                                               \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    static_assert(                                                           \
        !std::is_pointer_v<decltype(condition)>,                             \
        "When using SIRE_ASSERT on a raw pointer, always write out "        \
        "SIRE_ASSERT(foo != nullptr), do not write SIRE_ASSERT(foo) "      \
        "and rely on implicit pointer-to-bool conversion.");                 \
  } while (0)
#define SIRE_ASSERT_VOID(expression)                              \
  static_assert(std::is_convertible_v<decltype(expression), void>, \
                "Expression should be void.")
#endif

#endif // SIRE_DOXYGEN_CXX
#endif
