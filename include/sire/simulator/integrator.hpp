#ifndef SIRE_INTEGRATOR_HPP_
#define SIRE_INTEGRATOR_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"

namespace sire::simulator {
class SIRE_API Integrator {
 public:
  auto virtual integrate(double** diff_data_in, double* old_result,
                         double* result_out) -> bool = 0;
  virtual ~Integrator() = default;
  auto stepSize() const -> double;
  auto setStepSize(double step_size) -> void;
  auto dataLength() const -> sire::Size;
  auto setDataLength(sire::Size data_length) -> void;
  Integrator(sire::Size data_length = 6, double step_size = 0.001);
  ARIS_DECLARE_BIG_FOUR(Integrator);

 private:
  double step_size_;
  sire::Size data_length_;
};

class SIRE_API RK4Integrator final : public Integrator {
 public:
  auto integrate(double** diff_data_in, double* old_result, double* result_out)
      -> bool override;
  ~RK4Integrator() = default;
  RK4Integrator(sire::Size data_length = 6, double step_size = 0.001);
  ARIS_DECLARE_BIG_FOUR(RK4Integrator);
};

}  // namespace sire::simulator
#endif
