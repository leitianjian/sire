#ifndef SIRE_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP_
#define SIRE_SEMI_IMPLICIT_EULER_INTEGRATOR_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/integrator/integrator_base.hpp"

namespace sire::simulator {
class SIRE_API SemiImplicitEulerIntegrator final : public IntegratorBase {
 public:
  auto doStep(double dt) -> bool override;
  auto integrate(double** diff_data_in, double* old_result, double* result_out)
      -> bool override;
  ~SemiImplicitEulerIntegrator() = default;
  SemiImplicitEulerIntegrator();
  ARIS_DECLARE_BIG_FOUR(SemiImplicitEulerIntegrator);
};
}  // namespace sire::simulator
#endif