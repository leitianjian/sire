// This file contains the implementation of both sire_assert and sire_throw.
/* clang-format off to disable clang-format-includes */
#include "sire/core/sire_assert.hpp"
#include "sire/core/sire_throw.hpp"
/* clang-format on */

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "sire/core/sire_assertion_error.hpp"

namespace sire {
namespace core {
namespace {

// pybinding!
// Singleton to manage assertion configuration.
// struct AssertionConfig {
//   static AssertionConfig& singleton() {
//     static never_destroyed<AssertionConfig> global;
//     return global.access();
//   }
//
//   std::atomic<bool> assertion_failures_are_exceptions;
// };

// Stream into @p out the given failure details; only @p condition may be null.
void PrintFailureDetailTo(std::ostream& out, const char* condition,
                          const char* func, const char* file, int line) {
  out << "Failure at " << file << ":" << line << " in " << func << "()";
  if (condition) {
    out << ": condition '" << condition << "' failed.";
  } else {
    out << ".";
  }
}
}  // namespace

// Declared in sire_assert.h.
void Abort(const char* condition, const char* func, const char* file,
           int line) {
  std::cerr << "abort: ";
  PrintFailureDetailTo(std::cerr, condition, func, file, line);
  std::cerr << std::endl;
  std::abort();
}

// Declared in sire_throw.h.
void Throw(const char* condition, const char* func, const char* file,
           int line) {
  std::ostringstream what;
  PrintFailureDetailTo(what, condition, func, file, line);
  throw assertion_error(what.str().c_str());
}

// Declared in sire_assert.h.
void AssertionFailed(const char* condition, const char* func, const char* file,
                     int line) {
  // if (AssertionConfig::singleton().assertion_failures_are_exceptions) {
  Throw(condition, func, file, line);
  // } else {
  //  Abort(condition, func, file, line);
  // }
}

}  // namespace core
}  // namespace sire

// pybinding!
// Configures the DRAKE_ASSERT and DRAKE_DEMAND assertion failure handling
// behavior.
//
// By default, assertion failures will result in an ::abort().  If this method
// has ever been called, failures will result in a thrown exception instead.
//
// Assertion configuration has process-wide scope.  Changes here will affect
// all assertions within the current process.
//
// This method is intended ONLY for use by pydrake bindings, and thus is not
// declared in any header file, to discourage anyone from using it.
// extern "C" void drake_set_assertion_failure_to_throw_exception() {
//   drake::internal::AssertionConfig::singleton()
//       .assertion_failures_are_exceptions = true;
// }
