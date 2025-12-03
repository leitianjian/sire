#include "sire/integrator/runge_kutta4_integrator.hpp"

#include <aris/core/reflection.hpp>

#include "sire/core/constants.hpp"

namespace sire::simulator {
RK4Integrator::RK4Integrator() : IntegratorBase(){};
auto RK4Integrator::integrate(double** diff_data_in, double* old_result,
                              double* result_out) -> bool {
  for (int i = 0; i < dataLength(); ++i) {
    result_out[i] =
        old_result[i] +
        stepSize() *
            (diff_data_in[0][i] + 4 * diff_data_in[1][i] + diff_data_in[2][i]) /
            6.0;
  }
  return true;
}
ARIS_DEFINE_BIG_FOUR_CPP(RK4Integrator);

ARIS_REGISTRATION {
  aris::core::class_<RK4Integrator>("RK4Integrator").inherit<IntegratorBase>();
}
}  // namespace sire::simulator