#pragma once

#include <cmath>

#ifdef __CUDACC__
#define SIRE_ASSEMBLY_HD __host__ __device__
#else
#define SIRE_ASSEMBLY_HD
#endif

namespace sire::physics::contact::detail {

struct ContactInverseInertiaBlock {
  double aa, ab, ba, bb;
};

// Sire uses the negative of the usual J_rel M^-1 J_rel^T convention.
SIRE_ASSEMBLY_HD inline auto signedRelativeInertia(ContactInverseInertiaBlock block)
    -> double {
  return block.ab + block.ba - block.aa - block.bb;
}

SIRE_ASSEMBLY_HD inline auto penetrationScale(double minimum_stiffness) -> double {
  return ::pow(10.0, -::round(::log10(minimum_stiffness) / 2.0));
}

// Multiply each body's row by the paired spring/damper force, then subtract
// the B acceleration from the A acceleration. Keep this ordering through
// cancellation, as in the SOR and ADMM normal coefficient calculation.
SIRE_ASSEMBLY_HD inline auto normalContactCoefficient(ContactInverseInertiaBlock block,
                                                      double coefficient) -> double {
  const double on_a = -block.aa * coefficient + block.ab * coefficient;
  const double on_b = -block.ba * coefficient + block.bb * coefficient;
  return on_a - on_b;
}

}  // namespace sire::physics::contact::detail

#undef SIRE_ASSEMBLY_HD
