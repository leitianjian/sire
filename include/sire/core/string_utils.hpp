#ifndef SIRE_STRING_UTILS_HPP_
#define SIRE_STRING_UTILS_HPP_
#include <algorithm>
#include <cctype>
#include <locale>

namespace sire::core {
// trim from start
static inline std::string_view ltrim(std::string_view s) {
  s.remove_prefix(std::min(s.find_first_not_of(" \f\n\r\t\v"), s.size()));
  return s;
}

// trim from end
static inline std::string_view rtrim(std::string_view s) {
  if (auto trim_pos = s.find_last_not_of(" \f\n\r\t\v"); trim_pos != s.npos) {
    s.remove_suffix(s.size() - 1 - trim_pos);
  } else {
    s.remove_suffix(s.size());
  }
  return s;
}

// trim from both ends
static inline std::string_view trim(std::string_view s) {
  s = rtrim(s);
  s = ltrim(s);
  return s;
}
}  // namespace sire::core
#endif