#ifndef FORCE_CONTROL_SIMULATION_TRANSFER_HPP_
#define FORCE_CONTROL_SIMULATION_TRANSFER_HPP_

#include <sire_lib_export.h>

#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/integrator/integrator_base.hpp"

namespace sire::transfer {
class SIRE_API ForceControlSimulationTransfer
    : public aris::server::TransferModelController {
 public:
  auto updateDataController2Model(const std::vector<std::uint64_t>& options,
                                  const aris::control::Controller* controller,
                                  aris::dynamic::ModelBase* model)
      -> void override;
  auto updateDataModel2Controller(const std::vector<std::uint64_t>& options,
                                  const aris::dynamic::ModelBase* model,
                                  aris::control::Controller* controller)
      -> void override;
  // Semi-implicit RK4 method      ¡Ì
  // Explicit(Forward) RK4 method  ¡Á
  // implicit(Backward) RK4 method ¡Á
  // 
  // Using acceleration screw at current time step, integrate 
  // acceleration to get velocity screw of next time step. Integrate
  // time velocity screw by RK4.
  auto integrateAs2Ps(double* vs_in[3], double* as_in[3], double* old_ps,
                      double* ps_out) -> void;
  auto resetIntegrator(simulator::IntegratorBase* integrator) -> void;
  auto integrator() -> simulator::IntegratorBase&;
  auto integrator() const -> const simulator::IntegratorBase&;
  ForceControlSimulationTransfer();

 private:
  std::unique_ptr<simulator::IntegratorBase> integrator_;
  sire::Size part_pool_length_;
  sire::Size motion_pool_length_;
  sire::Size general_motion_pool_length_;
  double dt_seconds_;
  std::vector<double> motion_force_;
  std::array<std::vector<double>, 2> parts_ps_array_;
  double* old_parts_ps_;
  double* now_parts_ps_;
  std::array<std::vector<double>, 3> parts_vs_array_;
  std::array<double*, 3> parts_vs_ptrs_;
  std::array<std::vector<double>, 3> parts_as_array_;
  std::array<double*, 3> parts_as_ptrs_;
};
}  // namespace sire::transfer

#endif