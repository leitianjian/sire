#ifndef SIRE_BASE64_HPP_
#define SIRE_BASE64_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace sire::server::base64_helper {
std::vector<uint8_t> Decode(const std::string& encoded);
std::string Encode(const std::vector<uint8_t>& binary);
}  // namespace sire::server
#endif