#ifndef SIRE_STRING_UTILS_HPP_
#define SIRE_STRING_UTILS_HPP_
#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <locale>
#include <string>
#include <vector>

namespace sire::core {
// trim from start
inline std::string_view ltrim(std::string_view s) {
  s.remove_prefix(std::min(s.find_first_not_of(" \f\n\r\t\v"), s.size()));
  return s;
}

// trim from end
inline std::string_view rtrim(std::string_view s) {
  if (auto trim_pos = s.find_last_not_of(" \f\n\r\t\v"); trim_pos != s.npos) {
    s.remove_suffix(s.size() - 1 - trim_pos);
  } else {
    s.remove_suffix(s.size());
  }
  return s;
}

// trim from both ends
inline std::string_view trim(std::string_view s) {
  s = rtrim(s);
  s = ltrim(s);
  return s;
}

// Code from stackoverflow answers C++11 above
// https://stackoverflow.com/a/49812018/8462770
// requires at least C++11
const inline std::string string_format(const char* const zcFormat, ...) {
  // initialize use of the variable argument array
  va_list vaArgs;
  va_start(vaArgs, zcFormat);

  // reliably acquire the size
  // from a copy of the variable argument array
  // and a functionally reliable call to mock the formatting
  va_list vaArgsCopy;
  va_copy(vaArgsCopy, vaArgs);
  const int iLen = std::vsnprintf(NULL, 0, zcFormat, vaArgsCopy);
  va_end(vaArgsCopy);

  // return a formatted string without risking memory mismanagement
  // and without assuming any compiler or platform specific behavior
  std::vector<char> zc(iLen + 1);
  std::vsnprintf(zc.data(), zc.size(), zcFormat, vaArgs);
  va_end(vaArgs);
  return std::string(zc.data(), iLen);
}
}  // namespace sire::core
#endif