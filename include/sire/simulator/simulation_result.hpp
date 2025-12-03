#ifndef SIRE_SIMULATION_REUSLT_HPP_
#define SIRE_SIMULATION_REUSLT_HPP_

#include <sire_lib_export.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>
#include <aris/dynamic/model_interaction.hpp>
#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/physics/common/motion_force.hpp"

namespace sire::simulator {
// inner usage class with std::vector as function return type.
// 
class SIRE_API SimResult : public aris::dynamic::Element {
 public:
  class TimeResult : public aris::dynamic::Element {
   public:
    auto record() -> void;
    auto restore(sire::Size pos) -> void;

    virtual ~TimeResult();
    explicit TimeResult(const std::string& name = "time_result");
    ARIS_DECLARE_BIG_FOUR_NOEXCEPT(TimeResult);

   private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;

    friend class SimResult;
  };
  class PartResult : public aris::dynamic::Element {
   public:
    auto part() -> aris::dynamic::Part& {
      return const_cast<aris::dynamic::Part&>(
          static_cast<const PartResult*>(this)->part());
    }
    auto part() const -> const aris::dynamic::Part&;
    auto record() -> void;
    auto restore(sire::Size pos) -> void;

    virtual ~PartResult();
    explicit PartResult(const std::string& name = "part_result",
                        aris::dynamic::Part* part = nullptr);
    ARIS_DECLARE_BIG_FOUR_NOEXCEPT(PartResult);

   private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;

    friend class SimResult;
  };
  class ConstraintResult : public aris::dynamic::Element {
   public:
    auto constraint() -> aris::dynamic::Constraint& {
      return const_cast<aris::dynamic::Constraint&>(
          static_cast<const ConstraintResult*>(this)->constraint());
    }
    auto constraint() const -> const aris::dynamic::Constraint&;
    auto record() -> void;
    auto restore(sire::Size pos) -> void;

    virtual ~ConstraintResult();
    explicit ConstraintResult(const std::string& name = "constraint_result",
                              aris::dynamic::Constraint* constraint = nullptr);
    ARIS_DECLARE_BIG_FOUR_NOEXCEPT(ConstraintResult);

   private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;

    friend class SimResult;
  };
  class ForceResult : public aris::dynamic::Element {
   public:
    auto force() -> sire::physics::Force& {
      return const_cast<sire::physics::Force&>(
          static_cast<const ForceResult*>(this)->force());
    }
    auto force() const -> const sire::physics::Force&;
    auto record() -> void;
    auto restore(sire::Size pos) -> void;

    virtual ~ForceResult();
    // explicit ForceResult(const std::string& name = "force_result",
    //                      sire::physics::Force* force = nullptr);
    // ARIS_DECLARE_BIG_FOUR_NOEXCEPT(ForceResult);

   private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;

    friend class SimResult;
  };

  auto timeResult() -> TimeResult& {
    return const_cast<TimeResult&>(
        static_cast<const SimResult*>(this)->timeResult());
  }
  auto timeResult() const -> const TimeResult&;
  // auto partResultPool() const -> const std::vector<PartResult>&;
  // auto constraintResultPool() -> std::vector<ConstraintResult>& {
  //   return const_cast<std::vector<ConstraintResult>&>(
  //       static_cast<const SimResult*>(this)->constraintResultPool());
  // }
  // auto constraintResultPool() const -> const std::vector<ConstraintResult>&;
  // auto forceResultPool() -> std::vector<ForceResult>& {
  //   return const_cast<std::vector<ForceResult>&>(
  //       static_cast<const SimResult*>(this)->forceResultPool());
  // }
  // auto forceResultPool() const -> const std::vector<ForceResult>&;
  auto allocateMemory() -> void;
  auto record() -> void;
  auto restore(sire::Size pos) -> void;
  auto size() const -> sire::Size;
  auto clear() -> void;

  virtual ~SimResult();
  explicit SimResult(const std::string& name = "sim_result");
  ARIS_DECLARE_BIG_FOUR_NOEXCEPT(SimResult);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::simulator
#endif