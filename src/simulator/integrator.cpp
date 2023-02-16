#include "sire/simulator/integrator.hpp"

#include <aris/core/reflection.hpp>

#include "sire/core/constants.hpp"

namespace sire::simulator {
auto Integrator::stepSize() const -> double { return step_size_; };
auto Integrator::setStepSize(double step_size) -> void {
  step_size_ = step_size;
};
auto Integrator::dataLength() const -> sire::Size { return data_length_; };
auto Integrator::setDataLength(sire::Size data_length) -> void {
  data_length_ = data_length;
};
Integrator::Integrator(sire::Size data_length, double step_size)
    : step_size_(step_size), data_length_(data_length){};
ARIS_DEFINE_BIG_FOUR_CPP(Integrator);
RK4Integrator::RK4Integrator(sire::Size data_length, double step_size)
    : Integrator(data_length, step_size){};
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
  aris::core::class_<Integrator>("Integrator")
      .prop("step_size", &Integrator::setStepSize, &Integrator::stepSize)
      .prop("data_length", &Integrator::setDataLength, &Integrator::dataLength);
  aris::core::class_<RK4Integrator>("RK4Integrator").inherit<Integrator>();
}
}  // namespace sire::simulator