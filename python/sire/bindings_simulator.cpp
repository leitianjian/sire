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

void init_simulator(py::module& m) {
  py::class_<sire::simulator::Simulator>(m, "Simulator")
      .def(py::init<>())                               // 默认构造函数
      .def("init", &sire::simulator::Simulator::init)  // 初始化模型
      .def("model", py::overload_cast<>(&sire::simulator::Simulator::model),
           py::return_value_policy::reference_internal)
      .def("simulationLoop",
           py::overload_cast<>(&sire::simulator::Simulator::simulationLoop),
           py::return_value_policy::reference_internal)
      .def("physicsEngine",
           py::overload_cast<>(&sire::simulator::Simulator::physicsEngine),
           py::return_value_policy::reference_internal)
      .def("simulatorModules",
           py::overload_cast<>(&sire::simulator::Simulator::simulatorModules),
           py::return_value_policy::reference_internal)
      .def("reset", &sire::simulator::Simulator::simReset)
      .def("displayInitJson",
           [](sire::simulator::Simulator& self) -> nlohmann::json {
             // get control server config of geometry in part pool
             nlohmann::json geo_pool;
             // 取出 Part下面的每一个geometry
             nlohmann::json displayInitJson = nlohmann::json::object();
             auto& model = self.model();
             for (sire::Size i = 0; i < model.partPool().size(); ++i) {
               nlohmann::json json;
               aris::dynamic::Part& part = model.partPool().at(i);
               std::array<double, 16> buffer;
               for (sire::Size j = 0; j < part.geometryPool().size(); ++j) {
                 dynamic_cast<sire::geometry::GeometryBase&>(
                     part.geometryPool().at(j))
                     .to_json(json);
                 aris::dynamic::s_vc(
                     16,
                     const_cast<double*>(
                         *dynamic_cast<sire::geometry::GeometryBase&>(
                              part.geometryPool().at(j))
                              .pm()),
                     buffer.data());
                 json["init_pm"] = buffer;
                 geo_pool.push_back(json);
               }
             }
             auto& gp = self.physicsEngine().geometryPool();
             for (sire::Size i = 0; i < gp.size(); ++i) {
               if (gp[i].visible()) {
                 std::array<double, 16> buffer;
                 nlohmann::json json;
                 gp.at(i).to_json(json);
                 aris::dynamic::s_vc(16, const_cast<double*>(*gp.at(i).pm()),
                                     buffer.data());
                 json["init_pm"] = buffer;
                 geo_pool.push_back(json);
               }
             }
             // 设置part相关初始化的信息
             nlohmann::json part_init_config;
             part_init_config.push_back(
                 std::array<double, sire::kPosQuatSize>({0, 0, 0, 0, 0, 0, 1}));
             for (sire::Size i = 1; i < model.partPool().size(); ++i) {
               aris::dynamic::Part& part = model.partPool().at(i);
               std::array<double, sire::kPosQuatSize> part_pq_buffer;
               part.getPq(part_pq_buffer.data());
               part_init_config.push_back(part_pq_buffer);
             }
             displayInitJson["geometry_pool"] = geo_pool;
             displayInitJson["part_init_config"] = part_init_config;
             return displayInitJson;
           });

  // TODO: 没有添加global variable
  // pool的支持，涉及propmap，目前不需要，后续再说。
  py::class_<sire::simulator::SimulationLoop>(m, "SimulationLoop")
      .def(py::init<>())
      .def_property("deltaT", &sire::simulator::SimulationLoop::deltaT,
                    &sire::simulator::SimulationLoop::setDeltaT)
      .def_property("ctrlT", &sire::simulator::SimulationLoop::ctrlT,
                    &sire::simulator::SimulationLoop::setCtrlT)
      .def_property("realtimeRate",
                    &sire::simulator::SimulationLoop::realtimeRate,
                    &sire::simulator::SimulationLoop::setRealtimeRate)
      .def_property("simDuration",
                    &sire::simulator::SimulationLoop::simDuration,
                    &sire::simulator::SimulationLoop::setSimDuration)
      .def("start", &sire::simulator::SimulationLoop::start)
      .def("step", &sire::simulator::SimulationLoop::step, "frame_skip"_a = 1,
           "pause_if_fast"_a = false)
      .def("integrate", &sire::simulator::SimulationLoop::integrate)
      .def("headerIsCtrl", &sire::simulator::SimulationLoop::headerIsCtrl)
      .def("handleContact", &sire::simulator::SimulationLoop::handleContact)
      // ---- MuJoCo-style control-timed API ----
      .def("applyActuators", &sire::simulator::SimulationLoop::applyActuators)
      .def("stepPhysics", &sire::simulator::SimulationLoop::stepPhysics)
      .def("stepSimple", &sire::simulator::SimulationLoop::stepSimple)
      .def("advanceToSimTime",
           &sire::simulator::SimulationLoop::advanceToSimTime)
      .def("stepPhysicsSimple",
           &sire::simulator::SimulationLoop::stepPhysicsSimple)
      // -----------------------------------------
      .def("simTime", &sire::simulator::SimulationLoop::simTime)
      .def("reset", &sire::simulator::SimulationLoop::reset)
      .def("resetRL", &sire::simulator::SimulationLoop::resetRL)
      .def("resetRecorder", &sire::simulator::SimulationLoop::resetRecorder)
      .def("stop", &sire::simulator::SimulationLoop::stop)
      .def("pause", &sire::simulator::SimulationLoop::pause)
      .def("isTimeout", &sire::simulator::SimulationLoop::isTimeout)
      .def("isRunning", &sire::simulator::SimulationLoop::isRunning)
      .def("isEventListEmpty",
           &sire::simulator::SimulationLoop::isEventListEmpty)
      .def("recordsToJson", &sire::simulator::SimulationLoop::recordsToJson)
      .def("recordsContactCptInfo",
           &sire::simulator::SimulationLoop::recordsContactCptInfo)
      // Build the initial-display json (geometry pool + part init poses) for
      // the meshcat visualizer.  The runtime pipeline drives a SimulationLoop
      // (via SireMiddleware), never the standalone Simulator, so the same
      // helper has to live here — otherwise `simulator.displayInitJson()`
      // raises AttributeError and visualization is silently skipped.
      .def("displayInitJson",
           [](sire::simulator::SimulationLoop& self) -> nlohmann::json {
             nlohmann::json geo_pool;
             nlohmann::json displayInitJson = nlohmann::json::object();
             auto* model = self.model();
             if (model == nullptr) {
               throw std::runtime_error(
                   "SimulationLoop has no model; call init() first");
             }
             // 取出 Part 下面的每一个 geometry
             for (sire::Size i = 0; i < model->partPool().size(); ++i) {
               nlohmann::json json;
               aris::dynamic::Part& part = model->partPool().at(i);
               std::array<double, 16> buffer;
               for (sire::Size j = 0; j < part.geometryPool().size(); ++j) {
                 dynamic_cast<sire::geometry::GeometryBase&>(
                     part.geometryPool().at(j))
                     .to_json(json);
                 aris::dynamic::s_vc(
                     16,
                     const_cast<double*>(
                         *dynamic_cast<sire::geometry::GeometryBase&>(
                              part.geometryPool().at(j))
                              .pm()),
                     buffer.data());
                 json["init_pm"] = buffer;
                 geo_pool.push_back(json);
               }
             }
             // 引擎侧（例如 height field 等）可见几何
             auto* engine = self.physicsEnginePtr();
             if (engine != nullptr) {
               auto& gp = engine->geometryPool();
               for (sire::Size i = 0; i < gp.size(); ++i) {
                 if (gp[i].visible()) {
                   std::array<double, 16> buffer;
                   nlohmann::json json;
                   gp.at(i).to_json(json);
                   aris::dynamic::s_vc(16, const_cast<double*>(*gp.at(i).pm()),
                                       buffer.data());
                   json["init_pm"] = buffer;
                   geo_pool.push_back(json);
                 }
               }
             }
             // 设置 part 相关初始化的信息
             nlohmann::json part_init_config;
             part_init_config.push_back(
                 std::array<double, sire::kPosQuatSize>({0, 0, 0, 0, 0, 0, 1}));
             for (sire::Size i = 1; i < model->partPool().size(); ++i) {
               aris::dynamic::Part& part = model->partPool().at(i);
               std::array<double, sire::kPosQuatSize> part_pq_buffer;
               part.getPq(part_pq_buffer.data());
               part_init_config.push_back(part_pq_buffer);
             }
             displayInitJson["geometry_pool"] = geo_pool;
             displayInitJson["part_init_config"] = part_init_config;
             return displayInitJson;
           })
      .def("lastContactPairResults",
           [](sire::simulator::SimulationLoop& sl) -> py::list {
             py::list lst;
             const auto& latest = sl.recorder().latestContactPairResults();
             for (const auto& r : latest) {
               lst.append(py::make_tuple(
                   r.geomIdA, r.geomIdB,
                   r.force_W[0], r.force_W[1], r.force_W[2],
                   r.point_W[0], r.point_W[1], r.point_W[2]));
             }
             return lst;
           })
      // ---- MuJoCo-style: contact pair results with part IDs instead of geom IDs ----
      .def("lastContactPairResultsWithPartIds",
           [](sire::simulator::SimulationLoop& sl) -> py::list {
             py::list lst;
             const auto& latest = sl.recorder().latestContactPairResults();
             auto* engine = sl.physicsEnginePtr();
             for (const auto& r : latest) {
               sire::Size pa = 0, pb = 0;
               if (engine != nullptr) {
                 auto* geomA = engine->queryGeometryPoolById(r.geomIdA);
                 auto* geomB = engine->queryGeometryPoolById(r.geomIdB);
                 pa = (geomA != nullptr) ? geomA->partId() : sire::Size(0);
                 pb = (geomB != nullptr) ? geomB->partId() : sire::Size(0);
               }
               lst.append(py::make_tuple(
                   pa, pb,
                   r.force_W[0], r.force_W[1], r.force_W[2],
                   r.point_W[0], r.point_W[1], r.point_W[2]));
             }
             return lst;
           })
      // TODO: 没有办法动态添加事件处理规则，后续可以添加，现在不管
      .def("addEventHandlerRule",
           [](sire::simulator::SimulationLoop& self, sire::core::EventId name1,
              sire::core::HandlerId name2) {
             self.eventManager().addEventHandlerRule(name1, name2);
           })
      .def("setEventHandlerMap",
           [](sire::simulator::SimulationLoop& self,
              std::map<sire::Size, sire::Size>& map) {
             self.eventManager().eventHandlerMap().swap(map);
           })
      .def("resetEventHandlerRule", [](sire::simulator::SimulationLoop& self) {
        self.eventManager().eventHandlerMap().clear();
        self.eventManager().eventHandlerPairPool().clear();
      });
}
