#ifndef SIRE_INTEGRATOR_BASE_HPP_
#define SIRE_INTEGRATOR_BASE_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/constants.hpp"

namespace sire {
namespace physics {
class PhysicsEngine;
}
namespace simulator {
class SIRE_API IntegratorBase {
 public:
  auto virtual integrate(double** diff_data_in, double* old_result,
                         double* result_out) -> bool = 0;
  auto init(physics::PhysicsEngine* engine) -> void;
  auto step(double dt) -> bool;

  /**
   * 派生类可以重写这个方法来实现想要的初始化，这个方法会在init()方法中被调用
   * 默认的doInit()默认没有做任何处理
   */
  auto virtual doInit() -> void{};

  /**
  * TODO(leitianjian):
  *   可以传入Model或PhysicsEngine的指针，而不是在初始化时确定指针，
  *   有比较强的可扩展性。
  * 派生类必须实现这个方法，实现这个方法需要做到两件事情
  * 1. 积分系统连续状态（p, v, a），状态步进dt时间
  * 2. 设置误差估计相关的参数，如果有的话
  * @param dt 积分步进时间
  * @returns 如果成功，返回`true`，如果因为各种原因失败，没有办法让状态步进
             一个时间dt，就返回`false`，如因为误差太大而步进失败
  * @post 如果当前时刻是t，如果返回`true`，则时间会被前进到t+dt，如果返回
  *       `false`时间会重设回t
  */
  auto virtual doStep(double dt) -> bool = 0;
  auto stepSize() const -> double;
  auto setStepSize(double step_size) -> void;
  auto dataLength() const -> sire::Size;
  auto setDataLength(sire::Size data_length) -> void;
  IntegratorBase();
  virtual ~IntegratorBase();
  ARIS_DECLARE_BIG_FOUR(IntegratorBase);

 protected:
  // Model基础信息
  aris::dynamic::Model* model_ptr_{nullptr};
  sire::Size part_pool_length_;
  sire::Size motion_pool_length_;
  sire::Size general_motion_pool_length_;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace simulator
}  // namespace sire
#endif