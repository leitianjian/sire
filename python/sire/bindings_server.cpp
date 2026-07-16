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
#include "sire/physics/contact/analytical_tangent_force_solver.hpp"
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

void init_server(py::module& m) {
  py::class_<aris::server::ControlServer,
             std::unique_ptr<aris::server::ControlServer, py::nodelete>>(
      m, "ControlServer")
      .def_static("instance", &aris::server::ControlServer::instance,
                  py::return_value_policy::reference)
      .def("init", &aris::server::ControlServer::init)
      .def("clear",
           [](aris::server::ControlServer& self) {
             self.makeModel<aris::dynamic::Model>();
             self.makeMaster<aris::control::Master>();
             self.makeController<aris::control::Controller>("controller");
             self.makePlanRoot<aris::plan::PlanRoot>("plan_root");
             self.resetMiddleWare(new sire::middleware::SireMiddleware);
           })
      .def("start", &aris::server::ControlServer::start)
      .def("stop", &aris::server::ControlServer::stop)
      .def("open", &aris::server::ControlServer::open)
      .def("close", &aris::server::ControlServer::close)
      .def("model", py::overload_cast<>(&aris::server::ControlServer::model),
           py::return_value_policy::reference_internal)
      .def(
          "model",
          [](aris::server::ControlServer& self) -> aris::dynamic::Model& {
            return dynamic_cast<aris::dynamic::Model&>(self.model());
          },
          py::return_value_policy::reference_internal)
      .def("runCmdLine", &aris::server::ControlServer::runCmdLine)
      .def(
          "addSireMiddleware",
          [](aris::server::ControlServer& self)
              -> sire::middleware::SireMiddleware& {
            self.resetMiddleWare(new sire::middleware::SireMiddleware);
            return dynamic_cast<sire::middleware::SireMiddleware&>(
                self.middleWare());
          },
          py::return_value_policy::reference_internal)
      .def(
          "simulator",
          [](aris::server::ControlServer& self)
              -> sire::simulator::SimulationLoop& {
            auto& middleware = dynamic_cast<sire::middleware::SireMiddleware&>(
                self.middleWare());
            return middleware.simulationLoop();
          },
          py::return_value_policy::reference_internal)
      .def(
          "physicsEngine",
          [](aris::server::ControlServer& self)
              -> sire::physics::PhysicsEngine& {
            auto& middleware = dynamic_cast<sire::middleware::SireMiddleware&>(
                self.middleWare());
            return middleware.physicsEngine();
          },
          py::return_value_policy::reference_internal);

  py::class_<aris::server::MiddleWare>(m, "Middleware").def(py::init<>());

  py::class_<sire::middleware::SireMiddleware, aris::server::MiddleWare>(
      m, "SireMiddleware")
      .def(py::init<>())                                     // 默认构造函数
      .def("init", &sire::middleware::SireMiddleware::init)  // 初始化模型
      .def("simulationLoop",
           py::overload_cast<>(
               &sire::middleware::SireMiddleware::simulationLoop),
           py::return_value_policy::reference_internal)
      .def(
          "physicsEngine",
          py::overload_cast<>(&sire::middleware::SireMiddleware::physicsEngine),
          py::return_value_policy::reference_internal)
      .def("simulatorModules",
           py::overload_cast<>(
               &sire::middleware::SireMiddleware::simulatorModules),
           py::return_value_policy::reference_internal);
}
