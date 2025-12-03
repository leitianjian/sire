#include "sire/physics/common/motion_force.hpp"

#include <aris/core/reflection.hpp>
namespace sire::physics {
auto SingleComponentForce::cptGlbFs(double* fsI, double* fsJ) const noexcept
    -> void {
  aris::dynamic::s_tf(*makI()->prtPm(), fs(), fsJ);
  aris::dynamic::s_tf(*makI()->fatherPart().pm(), fsJ, fsI);
  aris::dynamic::s_vi(6, fsI, fsJ);
}
SingleComponentForce::SingleComponentForce(const std::string& name,
                                           aris::dynamic::Marker* makI,
                                           aris::dynamic::Marker* makJ,
                                           sire::Size componentID)
    : Force(name, makI, makJ), component_axis_(componentID) {}

ARIS_REGISTRATION {
  aris::core::class_<Force>("SireForce").inherit<aris::dynamic::Interaction>();

  aris::core::class_<GeneralForce>("SireGeneralForce")
      .inherit<sire::physics::Force>();

  aris::core::class_<SingleComponentForce>("SireSingleComponentForce")
      .inherit<sire::physics::Force>()
      .prop("component", &SingleComponentForce::setComponentAxis,
            &SingleComponentForce::componentAxis);
}
}  // namespace sire::physics
