// Auto-split from sire_bindings.cpp
#include <codecvt>
#include <fstream>
#include <iostream>
#include <locale>
#include <vector>

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <aris.hpp>

#include "sire/actuator/actuator.hpp"
#include "sire/core/constants.hpp"
#include "sire/core/event_base.hpp"
#include "sire/core/geometry/box_geometry.hpp"
#include "sire/core/geometry/capsule_geometry.hpp"
#include "sire/core/geometry/cylinder_geometry.hpp"
#include "sire/core/geometry/mesh_geometry.hpp"
#include "sire/core/geometry/shape_calculator.hpp"
#include "sire/core/geometry/sphere_geometry.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/simulator/event_manager.hpp"
#include "sire/simulator/simulator.hpp"
// #include "sire/physics/contact/avg_force_contact_solver.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/core/sire_fixed_joint.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/contact/analytical_implicit_friction_solver.hpp"
#include "sire/physics/contact/contact_position_force_solver.hpp"
#include "sire/physics/contact/ps_vs_solver.hpp"
#include "sire/physics/contact/ps_vs_solver2.hpp"
#include "sire/physics/geometry/box_collision_geometry.hpp"
#include "sire/physics/geometry/capsule_collision_geometry.hpp"
#include "sire/physics/geometry/collidable.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/geometry/cylinder_collision_geometry.hpp"
#include "sire/physics/geometry/mesh_collision_geometry.hpp"
#include "sire/physics/geometry/sphere_collision_geometry.hpp"

#include "pybind11_json.hpp"
namespace py = pybind11;
using namespace pybind11::literals;

// Forward declarations from split binding files

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>

namespace py = pybind11;
using namespace pybind11::literals;

void init_core(py::module& m) {
  // PointerArray<T> bindings moved to bindings_pointer_arrays.cpp

  py::class_<aris::dynamic::Solver, aris::dynamic::Element>(m, "Solver");
  py::class_<aris::dynamic::UniversalSolver, aris::dynamic::Solver>(
      m, "UniversalSolver")
      .def(py::init<>());
  py::class_<aris::dynamic::InverseDynamicSolver,
             aris::dynamic::UniversalSolver>(m, "InverseDynamicSolver")
      .def(py::init<>())
      .def("dynAccAndFce", &aris::dynamic::InverseDynamicSolver::dynAccAndFce);
  py::class_<aris::dynamic::InverseKinematicSolver,
             aris::dynamic::UniversalSolver>(m, "InverseKinematicSolver")
      .def(py::init<>())
      .def("kinPos",
           [](aris::dynamic::InverseKinematicSolver& self) {
             return self.kinPos();
           })
      .def("kinVel", &aris::dynamic::InverseKinematicSolver::kinVel)
      .def("ci", &aris::dynamic::InverseKinematicSolver::ci);
  py::class_<aris::dynamic::AdamsSimulator>(m, "AdamsSimulator")
      .def(py::init<>());
  py::class_<aris::dynamic::Motion, aris::dynamic::MotionBase>(m, "Motion")
      .def_property("name", &aris::dynamic::Motion::name,
                    &aris::dynamic::Motion::setName)
      .def_property("mp", &aris::dynamic::Motion::mp,
                    &aris::dynamic::Motion::setMp)
      .def_property("mv", &aris::dynamic::Motion::mv,
                    &aris::dynamic::Motion::setMv)
      .def_property("ma", &aris::dynamic::Motion::ma,
                    &aris::dynamic::Motion::setMa)
      .def_property("mf", &aris::dynamic::Motion::mf,
                    &aris::dynamic::Motion::setMf)
      .def("updA", &aris::dynamic::Motion::updA)
      .def("updV", &aris::dynamic::Motion::updV)
      .def("updP", &aris::dynamic::Motion::updP);
  py::class_<aris::dynamic::Simulator, aris::dynamic::Element>(m, "ArisSimulator");

  py::enum_<sire::actuator::ControlTarget>(m, "ControlTarget")
      .value("Position", sire::actuator::ControlTarget::Position)
      .value("Velocity", sire::actuator::ControlTarget::Velocity)
      .value("Acceleration", sire::actuator::ControlTarget::Acceleration)
      .export_values();

  py::class_<sire::actuator::ActuatorSISO, aris::dynamic::Motion>(
      m, "ActuatorSISO")
      .def(py::init<const std::string&, aris::dynamic::Marker*,
                    aris::dynamic::Marker*, aris::Size, const double*, double,
                    double, bool>(),
           py::arg("name") = "actuator_siso", py::arg("makI") = nullptr,
           py::arg("makJ") = nullptr, py::arg("component_axis") = 2,
           py::arg("frc_coe") = nullptr, py::arg("mp_offset") = 0.0,
           py::arg("mp_factor") = 1.0, py::arg("active") = true)
      .def_property("name", &sire::actuator::ActuatorSISO::name,
                    &sire::actuator::ActuatorSISO::setName)
      .def_property("kp", &sire::actuator::ActuatorSISO::kp,
                    &sire::actuator::ActuatorSISO::setKp)
      .def_property("kd", &sire::actuator::ActuatorSISO::kd,
                    &sire::actuator::ActuatorSISO::setKd)
      .def_property("desiredValue", &sire::actuator::ActuatorSISO::desiredValue,
                    &sire::actuator::ActuatorSISO::setDesiredValue)
      .def_property("minForce", &sire::actuator::ActuatorSISO::minForce,
                    &sire::actuator::ActuatorSISO::setMinForce)
      .def_property("maxForce", &sire::actuator::ActuatorSISO::maxForce,
                    &sire::actuator::ActuatorSISO::setMaxForce)
      .def_property_readonly("limitedDesiredValue",
                             &sire::actuator::ActuatorSISO::limitedDesiredValue)
      .def_property_readonly("appliedValue",
                             &sire::actuator::ActuatorSISO::appliedValue)
      .def_property("minPosition", &sire::actuator::ActuatorSISO::minPosition,
                    &sire::actuator::ActuatorSISO::setMinPosition)
      .def_property("maxPosition", &sire::actuator::ActuatorSISO::maxPosition,
                    &sire::actuator::ActuatorSISO::setMaxPosition)
      .def("enforcePositionLimits",
           &sire::actuator::ActuatorSISO::enforcePositionLimits,
           py::arg("tolerance") = 0.0)
      .def("forward", &sire::actuator::ActuatorSISO::forward)
      .def_static("add2Model", &sire::actuator::ActuatorSISO::add2Model,
                  py::return_value_policy::reference);
}
