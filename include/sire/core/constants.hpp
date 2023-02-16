#ifndef SIRE_CONSTANTS_HPP_
#define SIRE_CONSTANTS_HPP_

#include <cstddef>

namespace sire {

using Size = std::size_t;

constexpr double PI = 3.141592653589793;

constexpr int kQuaternionSize = 4;

constexpr int kSpaceDimension = 3;

constexpr int kRpySize = 3;

/// https://en.wikipedia.org/wiki/Screw_theory#Twist
constexpr int kTwistSize = 6;

/// http://www.euclideanspace.com/maths/geometry/affine/matrix4x4/
constexpr int kHomogeneousTransformSize = 16;

const int kRotmatSize = kSpaceDimension * kSpaceDimension;

enum class ToleranceType { kAbsolute, kRelative };

}  // namespace sire
#endif