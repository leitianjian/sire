#ifndef SIRE_RK4_INTEGRATOR_HPP_
#define SIRE_RK4_INTEGRATOR_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/integrator/integrator_base.hpp"
namespace sire::simulator {
/// <summary>
///  Integrate acceleration to distance using Runge-kutta4 method
///  1. position based on current estimated velocity
///  2. position based on previous estimated velocity
/// Reference :
/// https//www.physicsforums.com/threads/using-runge-kutta-method-for-position-calc.553663/
///  Formula:
///   suppose v, p, a = f(v, p) is state at current time
///   starting from
///   v1 = v, p1 = p, a1 = a = f(v1, p1)
///
///   v2 = v + 1/2 * dt * a1
///   p2 = p + 1/2 * dt * v2
///   a2 = f(v2, p2)
///
///   v3 = v + 1/2 * dt * a2
///   p3 = p + 1/2 * dt * v3
///   a3 = f(v3, p3)
///
///   v4 = v + dt * a3
///   p4 = p + dt * v4
///   a4 = f(v4, p4)
///
///   v[i + 1] = v + 1/6 * dt * (a1 + 2 * a2 + 2 * a3 + a4)
///   p[i + 1] = p + 1/6 * dt * (v1 + 2 * v2 + 2 * v3 + v4)
///   a[i + 1] = f(v[i + 1], p[i + 1])
///  
///  Formula:
///   p1 = p v1 = v a1 = f(v1, p1) 
///   
///   p2 = p + 1/2 * ¦¤t * v1 
///   v2 = v + 1/2 * ¦¤t * a1 
///   a2 = f(v2, p2) 
/// 
///   p3 = p + 1/2 * ¦¤t * v2 
///   v3 = v + 1/2 * ¦¤t * a2 
///   a3 = f(v3, p3) 
/// 
///   p4 = p + ¦¤t * v3 
///   v4 = v + ¦¤t * a3 
///   a4 = f(v4, p4)
/// 
///   p[i+1] = p + 1/6 * ¦¤t * (v1 + 2 v2 + 2 v3 + v4) 
///   v[i+1] = v + 1/6 * ¦¤t * (a1 + 2 a2 + 2 a3 + a4) 
///   a[i+1] = f(v[i+1], p[i+1])
/// </summary>

class SIRE_API RK4Integrator final : public IntegratorBase {
 public:
  auto integrate(double** diff_data_in, double* old_result, double* result_out)
      -> bool override;
  ~RK4Integrator() = default;
  RK4Integrator();
  ARIS_DECLARE_BIG_FOUR(RK4Integrator);
};

}  // namespace sire::simulator
#endif