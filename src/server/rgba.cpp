#include "sire/server/rgba.hpp"

#include <stdexcept>

#include "sire/core/string_utils.hpp"

namespace sire {
namespace server {

void Rgba::set(const double* rgba) {
  // Check the size (and pad it to 4).
  // Eigen::Vector4d new_value;
  // if (rgba.size() == 3) {
  //   new_value.head(3) = rgba;
  //   new_value[3] = 1.0;
  // } else if (rgba.size() == 4) {
  //   new_value = rgba;
  // } else {
  //   throw std::runtime_error(fmt::format(
  //       "Rgba must contain either 3 or 4 elements (given [{}])",
  //       rgba.size()));
  // }

  // Check the domain.
  for (int i = 0; i < 4; ++i) {
    if (!(rgba[i] >= 0 && rgba[i] <= 1.0)) {
      throw std::runtime_error(sire::core::string_format(
          "Rgba values must be within the range [0, 1]. Values provided: "
          "(r=%g, g=%g, b=%g, a=%g)",
          rgba[0], rgba[1], rgba[2], rgba[3]));
    }
  }
  for (int i = 0; i < 4; ++i) {
    value_[i] = rgba[i];
  }
}

}  // namespace server
}  // namespace sire
