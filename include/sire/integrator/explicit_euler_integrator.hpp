#ifndef SIRE_EXPLICIT_EULER_INTEGRATOR_HPP_
#define SIRE_EXPLICIT_EULER_INTEGRATOR_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/integrator/integrator_base.hpp"

namespace sire::simulator {
class SIRE_API ExplicitEulerIntegrator final : public IntegratorBase {
 public:
  auto doStep(double dt) -> bool override;
  auto integrate(double** diff_data_in, double* old_result, double* result_out)
      -> bool override;
  ~ExplicitEulerIntegrator() = default;
  ExplicitEulerIntegrator();
  ARIS_DECLARE_BIG_FOUR(ExplicitEulerIntegrator);
};
}  // namespace sire::simulator
#endif