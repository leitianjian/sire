#ifndef SIRE_INTEGRATOR_HPP_
#define SIRE_INTEGRATOR_HPP_

#include <sire_lib_export.h>
#include <aris.hpp>

namespace sire::simulator {
class SIRE_API Integrator {
 public:
  auto virtual integrate(double** diff_data_in, double* old_result,
                         double* result_out) -> bool = 0;
  virtual ~Integrator() = default;
  auto stepSize() const -> double;
  auto setStepSize(double step_size) -> void;
  auto dataLength() const -> aris::Size;
  auto setDataLength(aris::Size data_length) -> void;
  Integrator(aris::Size data_length = 6, double step_size = 0.001);
  ARIS_DECLARE_BIG_FOUR(Integrator);

 private:
  double step_size_;
  aris::Size data_length_;
};

class SIRE_API RK4Integrator final : public Integrator {
 public:
  auto integrate(double** diff_data_in, double* old_result, double* result_out)
      -> bool override;
  ~RK4Integrator() = default;
  RK4Integrator(aris::Size data_length = 6, double step_size = 0.001);
  ARIS_DECLARE_BIG_FOUR(RK4Integrator);
};

}  // namespace sire::simulator
#endif
