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
// #include "sire/physics/contact/avg_force_contact_solver.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/core/sire_fixed_joint.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/contact/contact_position_force_solver.hpp"
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
PYBIND11_MODULE(sire, m) {
  // 常量绑定
  m.attr("kPosQuatSize") = sire::kPosQuatSize;
  // 配置文件读取
  m.def("toXmlString", [](const aris::dynamic::Model& self) {
    return aris::core::toXmlString(self);
  });
  m.def("toXmlString", [](const aris::server::ControlServer& self) {
    return aris::core::toXmlString(self);
  });
  m.def("fromXmlFile",
        [](aris::server::ControlServer& self, const std::string& xml) {
          std::cout << "fromXmlFile: " << xml << std::endl;
          try {
            aris::core::fromXmlFile(self, xml);
          } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
          }
        });
  m.def("fromXmlFile", [](aris::dynamic::Model& self, const std::string& xml) {
    std::cout << "fromXmlFile: " << xml << std::endl;
    try {
      aris::core::fromXmlFile(self, xml);
    } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl;
    }
  });
  m.def("fromXmlString",
        [](aris::server::ControlServer& self, const std::string& xml) {
          aris::core::fromXmlString(self, xml);
        });
  m.def("fromXmlString",
        [](aris::dynamic::Model& self, const std::string& xml) {
          aris::core::fromXmlString(self, xml);
        });
  // 其他常用构建函数
  m.def("createModelDelta", [](const aris::dynamic::DeltaParam& param) {
    return aris::dynamic::createModelDelta(param);
  });
  m.def("fpm2fs", [](std::vector<double>& f, std::vector<double>& pm) {
    std::vector<double> fs(6);
    sire::core::screw::s_fpm2fs(f.data(), pm.data(), fs.data());
    return fs;
  });
  m.def("vs2va", [](std::vector<double>& vs, std::vector<double>& p) {
    std::vector<double> va(6);
    aris::dynamic::s_vs2va(vs.data(), p.data(), va.data());
    return va;
  });
  m.def("vs2va", [](std::vector<double>& pq, std::vector<double>& vs,
                    std::vector<double>& p) {
    std::vector<double> va(6);
    std::array<double, 3> pWorld{0};
    aris::dynamic::s_pq_dot_v3(pq.data(), p.data(), pWorld.data());
    aris::dynamic::s_vs2va(vs.data(), pWorld.data(), va.data());
    return va;
  });
  m.def("vs2vp", [](std::vector<double>& vs, std::vector<double>& p) {
    std::vector<double> vp(3);
    aris::dynamic::s_vs2vp(vs.data(), p.data(), vp.data());
    return vp;
  });
  m.def("vs2vp", [](std::vector<double>& pq, std::vector<double>& vs,
                    std::vector<double>& p) {
    std::vector<double> vp(3);
    std::array<double, 3> pWorld{0};
    aris::dynamic::s_pq_dot_v3(pq.data(), p.data(), pWorld.data());
    aris::dynamic::s_vs2vp(vs.data(), pWorld.data(), vp.data());
    return vp;
  });
  m.def("vp2vs", [](std::vector<double>& pp, std::vector<double>& vp) {
    std::vector<double> vs(6);
    aris::dynamic::s_vp2vs(pp.data(), vp.data(), vs.data());
    return vs;
  });
  m.def("as2ap", [](std::vector<double>& pq, std::vector<double>& vs,
                    std::vector<double>& as, std::vector<double>& p) {
    std::vector<double> ap(3);
    std::array<double, 3> pWorld{0};
    aris::dynamic::s_pq_dot_v3(pq.data(), p.data(), pWorld.data());
    aris::dynamic::s_as2ap(vs.data(), as.data(), pWorld.data(), ap.data());
    return ap;
  });
  m.def("as2ap", [](aris::dynamic::Part& part, std::vector<double>& p,
                    bool is_relative) {
    std::vector<double> ap(3);
    std::array<double, 3> pWorld{0};
    aris::dynamic::s_pm_dot_v3(*part.pm(), p.data(), pWorld.data());
    aris::dynamic::s_as2ap(part.vs(), part.as(),
                           is_relative ? pWorld.data() : p.data(), ap.data());
    return ap;
  });
  m.def(
      "simulator",
      [](aris::server::ControlServer& cs) -> sire::simulator::SimulationLoop& {
        auto* middleware_ptr =
            dynamic_cast<sire::middleware::SireMiddleware*>(&cs.middleWare());
        if (!middleware_ptr) {
          throw std::runtime_error(
              "middleWare() is not of type SireMiddleware");
        }
        auto& middleware = *middleware_ptr;
        return middleware.simulationLoop();
      },
      py::return_value_policy::reference_internal);
  m.def("iv2iv",
        [](const std::vector<double>& pm, const std::vector<double>& iv) {
          std::vector<double> iv_out(10);
          aris::dynamic::s_iv2iv(pm.data(), iv.data(), iv_out.data());
          return iv_out;
        });
  m.def(
      "model",
      [](aris::server::ControlServer& cs) -> aris::dynamic::Model& {
        return dynamic_cast<aris::dynamic::Model&>(cs.model());
      },
      py::return_value_policy::reference_internal);

  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const std::exception& e) {
      PyErr_SetString(PyExc_RuntimeError, e.what());
    } catch (...) {
      PyErr_SetString(PyExc_RuntimeError, "Unknown C++ exception");
    }
  });

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
          py::return_value_policy::reference)
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
      .def("handleContact", &sire::simulator::SimulationLoop::handleContact)
      .def("simTime", &sire::simulator::SimulationLoop::simTime)
      .def("stop", &sire::simulator::SimulationLoop::stop)
      .def("pause", &sire::simulator::SimulationLoop::pause)
      .def("isTimeout", &sire::simulator::SimulationLoop::isTimeout)
      .def("isRunning", &sire::simulator::SimulationLoop::isRunning)
      .def("isEventListEmpty",
           &sire::simulator::SimulationLoop::isEventListEmpty)
      .def("recordsToJson", &sire::simulator::SimulationLoop::recordsToJson)
      .def("recordsContactCptInfo",
           &sire::simulator::SimulationLoop::recordsContactCptInfo)
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

  py::class_<sire::physics::collision::CollisionFilter>(m, "CollisionFilter")
      .def(py::init<>())
      .def("saveMatConfig",
           &sire::physics::collision::CollisionFilter::saveMatConfig)
      .def("loadMatConfig",
           &sire::physics::collision::CollisionFilter::loadMatConfig)
      .def("enableCollisionPair",
           &sire::physics::collision::CollisionFilter::enableCollisionPair);

  py::class_<sire::physics::PhysicsEngine>(m, "PhysicsEngine")
      .def(py::init<>())
      .def_property("collisionDetectionFlag",
                    &sire::physics::PhysicsEngine::collisionDetectionFlag,
                    &sire::physics::PhysicsEngine::setCollisionDetectionFlag)
      .def_property("contactSolverFlag",
                    &sire::physics::PhysicsEngine::contactSolverFlag,
                    &sire::physics::PhysicsEngine::setContactSolverFlag)
      .def_property("geometryPool", &sire::physics::PhysicsEngine::geometryPool,
                    &sire::physics::PhysicsEngine::resetGeometryPool)
      .def(
          "addBoxGeometry",
          [](sire::physics::PhysicsEngine& self, double x, double y, double z,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             const std::string& material, const std::string& propStr) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            self.geometryPool()
                .add<sire::physics::geometry::BoxCollisionGeometry>(
                    x, y, z, part_id, is_dynamic, pm.data(), material, propStr);
          },
          py::arg("x"), py::arg("y"), py::arg("z"), py::arg("part_id"),
          py::arg("is_dynamic") = true, py::arg("prt_pm") = py::list(),
          py::arg("material") = "m1", py::arg("propStr") = "{}")
      .def(
          "addSphereGeometry",
          [](sire::physics::PhysicsEngine& self, double radius,
             sire::PartId part_id, bool is_dynamic) {
            self.geometryPool()
                .add<sire::physics::geometry::SphereCollisionGeometry>(
                    radius, part_id, is_dynamic);
          },
          py::arg("radius"), py::arg("part_id"), py::arg("is_dynamic") = true)
      .def(
          "addSphereGeometry",
          [](sire::physics::PhysicsEngine& self, double radius,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             const std::string& material, const std::string& propStr) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            // 创建球体几何体
            self.geometryPool()
                .add<sire::physics::geometry::SphereCollisionGeometry>(
                    radius, part_id, is_dynamic, prt_pm);
          },
          py::arg("radius"), py::arg("part_id"), py::arg("is_dynamic") = true,
          py::arg("prt_pm") = py::none(), py::arg("material") = "m1",
          py::arg("propStr") = "{}")
      .def(
          "addMeshGeometry",
          [](sire::physics::PhysicsEngine& self, const std::string& resPath,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             const std::string& material, const std::string& propStr) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            // 创建网格几何体
            self.geometryPool()
                .add<sire::physics::geometry::MeshCollisionGeometry>(
                    resPath, part_id, is_dynamic, prt_pm);
          },
          py::arg("resPath"), py::arg("part_id"), py::arg("is_dynamic") = true,
          py::arg("prt_pm") = py::none(), py::arg("material") = "m1",
          py::arg("propStr") = "{}")
      .def(
          "addCapsuleGeometry",
          [](sire::physics::PhysicsEngine& self, double radius, double length,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             const std::string& material, const std::string& propStr) {
            // 创建胶囊几何体
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            self.geometryPool()
                .add<sire::physics::geometry::CapsuleCollisionGeometry>(
                    radius, length, part_id, is_dynamic, prt_pm);
          },
          py::arg("radius"), py::arg("length"), py::arg("part_id"),
          py::arg("is_dynamic") = true, py::arg("prt_pm") = py::none(),
          py::arg("material") = "m1", py::arg("propStr") = "{}")
      .def(
          "addCylinderGeometry",
          [](sire::physics::PhysicsEngine& self, double radius, double length,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             const std::string& material, const std::string& propStr) {
            // 创建胶囊几何体
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            self.geometryPool()
                .add<sire::physics::geometry::CylinderCollisionGeometry>(
                    radius, length, part_id, is_dynamic, prt_pm);
          },
          py::arg("radius"), py::arg("length"), py::arg("part_id"),
          py::arg("is_dynamic") = true, py::arg("prt_pm") = py::none(),
          py::arg("material") = "m1", py::arg("propStr") = "{}")
      .def("contactSolver",
           py::overload_cast<>(&sire::physics::PhysicsEngine::contactSolver),
           py::return_value_policy::reference_internal)
      .def(
          "contactPositionForceSolver",
          [](sire::physics::PhysicsEngine& self)
              -> sire::physics::contact::contact_force::
                  ContactPositionForceSolver& {
                    return dynamic_cast<sire::physics::contact::contact_force::
                                            ContactPositionForceSolver&>(
                        self.contactSolver());
                  },
          py::return_value_policy::reference_internal)
      .def(
          "addContactPositionForceSolver",
          [](sire::physics::PhysicsEngine& self)
              -> sire::physics::contact::contact_force::
                  ContactPositionForceSolver& {
                    self.resetContactSolver(
                        new sire::physics::contact::contact_force::
                            ContactPositionForceSolver);
                    return dynamic_cast<sire::physics::contact::contact_force::
                                            ContactPositionForceSolver&>(
                        self.contactSolver());
                  },
          py::return_value_policy::reference_internal)
      .def("collisionFilter", &sire::physics::PhysicsEngine::collisionFilter,
           py::return_value_policy::reference_internal)
      .def(
          "addCollisionFilter",
          [](sire::physics::PhysicsEngine& self, const std::string& filterStr) {
            static aris::core::Calculator c;
            static int i = 0;
            if (i == 0) {
              c.addVariable("PI", "Number", double(aris::PI));
              i = 1;
            }

            auto mat = c.calculateExpression(std::string("Matrix({") +
                                             std::string(filterStr) + "})")
                           .second;
            self.collisionFilter().setStateMat(
                std::any_cast<const aris::core::Matrix&>(mat));
          })
      .def("addCollisionFilter", [](sire::physics::PhysicsEngine& self,
                                    std::vector<double>& filterVec) {
        self.collisionFilter().setStateMat(
            aris::core::Matrix(1, filterVec.size(), filterVec.data()));
      });

  py::class_<sire::physics::contact::contact_force::ContactPositionForceSolver>(
      m, "ContactPositionForceSolver")
      .def(py::init<>())
      .def("addMaterialPair",
           [](sire::physics::contact::contact_force::ContactPositionForceSolver&
                  self,
              const std::string& name1, const std::string& name2,
              const std::string& prop) {
             self.materialManager().addProp(
                 sire::core::SortedPair<std::string>(name1, name2),
                 sire::core::PropMap(prop));
           })
      .def("setDefaultProp",
           [](sire::physics::contact::contact_force::ContactPositionForceSolver&
                  self,
              const std::string& prop) {
             self.materialManager().setDefaultProp(sire::core::PropMap(prop));
           });  // 默认构造函数

  py::class_<aris::dynamic::DeltaParam>(m, "DeltaParam")
      .def(py::init<>())  // 默认构造函数
      .def_readwrite("a", &aris::dynamic::DeltaParam::a)
      .def_readwrite("b", &aris::dynamic::DeltaParam::b)
      .def_readwrite("c", &aris::dynamic::DeltaParam::c)
      .def_readwrite("d", &aris::dynamic::DeltaParam::d)
      .def_readwrite("e", &aris::dynamic::DeltaParam::e);

  py::class_<aris::dynamic::Model>(m, "Model")
      .def(py::init<>())                         // 默认构造函数
      .def("init", &aris::dynamic::Model::init)  // 初始化模型
      .def("setGravity",
           [](aris::dynamic::Model& self, const std::vector<double>& g) {
             if (g.size() != 6) {
               throw std::runtime_error(
                   "Input array 'g' size must be 6, representing gravity "
                   "vector in ground frame!");
             }
             self.environment().setGravity(g.data());
           })
      .def("addFixedJointAbs",
           [](aris::dynamic::Model& self, aris::dynamic::Part& part1,
              aris::dynamic::Part& part2, const std::vector<double>& position1,
              const std::vector<double>& position2) -> void {
             sire::core::FixedJoint::add2ModelAbs(
                 &self, part1, part2, position1.data(), position2.data());
           })
      .def("addSolvers",
           [](aris::dynamic::Model& self) {
             self.solverPool().add<aris::dynamic::InverseKinematicSolver>();
             self.solverPool().add<aris::dynamic::ForwardKinematicSolver>();
             self.solverPool().add<aris::dynamic::InverseDynamicSolver>();
             self.solverPool().add<aris::dynamic::ForwardDynamicSolver>();
           })
      .def("displayInitJson",
           [](aris::dynamic::Model& self) -> nlohmann::json {
             // get control server config of geometry in part pool
             nlohmann::json geo_pool;
             // 取出 Part下面的每一个geometry
             nlohmann::json displayInitJson = nlohmann::json::object();
             for (sire::Size i = 0; i < self.partPool().size(); ++i) {
               nlohmann::json json;
               aris::dynamic::Part& part = self.partPool().at(i);
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
             // 设置part相关初始化的信息
             nlohmann::json part_init_config;
             part_init_config.push_back(
                 std::array<double, sire::kPosQuatSize>({0, 0, 0, 0, 0, 0, 1}));
             for (sire::Size i = 1; i < self.partPool().size(); ++i) {
               aris::dynamic::Part& part = self.partPool().at(i);
               std::array<double, sire::kPosQuatSize> part_pq_buffer;
               part.getPq(part_pq_buffer.data());
               part_init_config.push_back(part_pq_buffer);
             }
             displayInitJson["geometry_pool"] = geo_pool;
             displayInitJson["part_init_config"] = part_init_config;
             return displayInitJson;
           })
      .def("numLinks",
           [](const aris::dynamic::Model& self) -> int {
             return (int)self.partPool().size();
           })  // 获取连杆数
      .def(
          "link",
          [](const aris::dynamic::Model& self, int i) -> aris::dynamic::Part& {
            if (i < 0 || i >= self.partPool().size()) {
              throw std::out_of_range("Index out of range");
            }
            return const_cast<aris::dynamic::Part&>(self.partPool().at(i));
          },
          py::return_value_policy::reference_internal)  // 获取连杆
      .def("numMotions",
           [](const aris::dynamic::Model& self) -> int {
             return (int)self.motionPool().size();
           })
      .def(
          "motion",
          [](const aris::dynamic::Model& self,
             int i) -> aris::dynamic::Motion& {
            if (i < 0 || i >= self.motionPool().size()) {
              throw std::out_of_range("Index out of range");
            }
            return const_cast<aris::dynamic::Motion&>(self.motionPool().at(i));
          },
          py::return_value_policy::reference_internal)
      .def("numJoints",
           [](const aris::dynamic::Model& self) -> int {
             return (int)self.jointPool().size();
           })
      .def(
          "joint",
          [](const aris::dynamic::Model& self, int i) -> aris::dynamic::Joint& {
            if (i < 0 || i >= self.jointPool().size()) {
              throw std::out_of_range("Index out of range");
            }
            return const_cast<aris::dynamic::Joint&>(self.jointPool().at(i));
          },
          py::return_value_policy::reference_internal)
      .def("numForces",
           [](const aris::dynamic::Model& self) -> int {
             return (int)self.forcePool().size();
           })
      .def(
          "force",
          [](const aris::dynamic::Model& self, int i) -> aris::dynamic::Force& {
            if (i < 0 || i >= self.forcePool().size()) {
              throw std::out_of_range("Index out of range");
            }
            return const_cast<aris::dynamic::Force&>(self.forcePool().at(i));
          },
          py::return_value_policy::reference_internal)
      .def(
          "addPartByPe",
          [](aris::dynamic::Model& self, const std::vector<double>& pe,
             const char* eul_type,
             const std::vector<double>& prt_im) -> aris::dynamic::Part& {
            if (pe.empty()) {
              throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            auto& part = self.addPartByPe(pe.data(), eul_type, prt_im.data());
            return part;
          },
          py::return_value_policy::reference_internal)
      .def(
          "addPartByPe",
          [](aris::dynamic::Model& self, const std::vector<double>& pe,
             const char* eul_type) -> aris::dynamic::Part& {
            if (pe.empty()) {
              throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            return self.addPartByPe(pe.data(), eul_type);
          },
          py::return_value_policy::reference_internal)
      .def(
          "addPartByPe",
          [](aris::dynamic::Model& self,
             const std::vector<double>& pe) -> aris::dynamic::Part& {
            if (pe.empty()) {
              throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            return self.addPartByPe(pe.data(), "313");
          },
          py::return_value_policy::reference_internal)
      .def(
          "addPartByPe",
          [](aris::dynamic::Model& self, const std::vector<double>& pe,
             double mass) -> aris::dynamic::Part& {
            if (pe.empty()) {
              throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            std::vector<double> iv(sire::default_iv, sire::default_iv + 10);
            iv[0] = mass;
            return self.addPartByPe(pe.data(), "313", iv.data());
          },
          py::return_value_policy::reference_internal)
      .def(
          "addRevoluteJoint",
          [](aris::dynamic::Model& self, aris::dynamic::Part& part1,
             aris::dynamic::Part& part2, const std::vector<double>& position,
             const std::vector<double>& axis) -> aris::dynamic::RevoluteJoint& {
            if (position.size() != 3 || axis.size() != 3) {
              throw std::runtime_error(
                  "Position and axis must be 3-element arrays!");
            }
            return self.addRevoluteJoint(part1, part2, position.data(),
                                         axis.data());
          },
          py::return_value_policy::reference_internal)
      .def(
          "addPrismaticJoint",
          [](aris::dynamic::Model& self, aris::dynamic::Part& part1,
             aris::dynamic::Part& part2, const std::vector<double>& position,
             const std::vector<double>& axis)
              -> aris::dynamic::PrismaticJoint& {
            if (position.size() != 3 || axis.size() != 3) {
              throw std::runtime_error(
                  "Position and axis must be 3-element arrays!");
            }
            return self.addPrismaticJoint(part1, part2, position.data(),
                                          axis.data());
          },
          py::return_value_policy::reference_internal)
      .def(
          "addSphericalJoint",
          [](aris::dynamic::Model& self, aris::dynamic::Part& part1,
             aris::dynamic::Part& part2, const std::vector<double>& position)
              -> aris::dynamic::SphericalJoint& {
            if (position.size() != 3) {
              throw std::runtime_error(
                  "Position and axis must be 3-element arrays!");
            }
            return self.addSphericalJoint(part1, part2, position.data());
          },
          py::return_value_policy::reference_internal)
      .def("addMotion", py::overload_cast<>(&aris::dynamic::Model::addMotion),
           py::return_value_policy::reference_internal)  // 添加驱动
      .def("addMotion",
           py::overload_cast<aris::dynamic::Joint&>(
               &aris::dynamic::Model::addMotion),
           py::return_value_policy::reference_internal)  // 添加驱动
      // .def("addGeneralMotionByPe",
      // &aris::dynamic::Model::addGeneralMotionByPe) // 添加末端
      .def(
          "addGeneralMotionByPe",
          [](aris::dynamic::Model& self, aris::dynamic::Part& part,
             aris::dynamic::Part& ground, const std::vector<double>& pe,
             const char* eul_type) -> aris::dynamic::GeneralMotion& {
            if (pe.empty()) {
              throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            return self.addGeneralMotionByPe(part, ground, pe.data(), eul_type);
          },
          py::return_value_policy::reference_internal)
      .def("init", &aris::dynamic::Model::init)        // 初始化模型
      .def("settime", &aris::dynamic::Model::setTime)  // 设置时间
      .def("setInputPos",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setInputPos(input.data());
           })
      .def("setOutputPos",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setOutputPos(input.data());
           })
      .def("getInputPos",
           [](const aris::dynamic::Model& self) {
             std::vector<double> pos(self.inputPosSize());
             self.getInputPos(pos.data());
             return pos;
           })
      .def("getOutputPos",
           [](const aris::dynamic::Model& self) {
             std::vector<double> pos(self.outputPosSize());
             self.getOutputPos(pos.data());
             return pos;
           })
      .def("setInputVel",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setInputVel(input.data());
           })
      .def("setOutputVel",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setOutputVel(input.data());
           })
      .def("getInputVel",
           [](const aris::dynamic::Model& self) {
             std::vector<double> vel(self.inputVelSize());
             self.getInputVel(vel.data());
             return vel;
           })
      .def("getOutputVel",
           [](const aris::dynamic::Model& self) {
             std::vector<double> vel(self.outputVelSize());
             self.getOutputVel(vel.data());
             return vel;
           })
      .def("setInputAcc",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setInputAcc(input.data());
           })
      .def("setOutputAcc",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setOutputAcc(input.data());
           })
      .def("getInputAcc",
           [](const aris::dynamic::Model& self) {
             std::vector<double> acc(self.inputAccSize());
             self.getInputAcc(acc.data());
             return acc;
           })
      .def("getOutputAcc",
           [](const aris::dynamic::Model& self) {
             std::vector<double> acc(self.outputAccSize());
             self.getOutputAcc(acc.data());
             return acc;
           })
      .def("setInputFce",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setInputFce(input.data());
           })
      .def("setOutputFce",
           [](aris::dynamic::Model& self, const std::vector<double>& input) {
             if (input.empty()) {
               throw std::runtime_error("Input array 'input' cannot be empty!");
             }
             self.setOutputFce(input.data());
           })
      .def("getInputFce",
           [](const aris::dynamic::Model& self) {
             std::vector<double> fce(self.inputFceSize());
             self.getInputFce(fce.data());
             return fce;
           })
      .def("getOutputFce",
           [](const aris::dynamic::Model& self) {
             std::vector<double> fce(self.outputFceSize());
             self.getOutputFce(fce.data());
             return fce;
           })
      .def("forwardKinematics",
           [](aris::dynamic::Model& self) { return self.forwardKinematics(); })
      .def("inverseKinematics",
           [](aris::dynamic::Model& self) { return self.inverseKinematics(); })
      .def("forwardDynamics",
           [](aris::dynamic::Model& self) { return self.forwardDynamics(); })
      .def("inverseDynamics",
           [](aris::dynamic::Model& self) { return self.inverseDynamics(); })
      .def("solverPool",
           static_cast<aris::core::PointerArray<aris::dynamic::Solver,
                                                aris::dynamic::Element>& (
               aris::dynamic::Model::*)()>(&aris::dynamic::Model::solverPool),
           py::return_value_policy::reference_internal)
      .def("solverPool",
           static_cast<const aris::core::PointerArray<aris::dynamic::Solver,
                                                      aris::dynamic::Element>& (
               aris::dynamic::Model::*)() const>(
               &aris::dynamic::Model::solverPool),
           py::return_value_policy::reference_internal)
      .def("motionPool",
           static_cast<aris::core::PointerArray<aris::dynamic::Motion,
                                                aris::dynamic::Element>& (
               aris::dynamic::Model::*)()>(&aris::dynamic::Model::motionPool),
           py::return_value_policy::reference_internal)
      .def("motionPool",
           static_cast<const aris::core::PointerArray<aris::dynamic::Motion,
                                                      aris::dynamic::Element>& (
               aris::dynamic::Model::*)() const>(
               &aris::dynamic::Model::motionPool),
           py::return_value_policy::reference_internal)
      .def("generalMotionPool",
           static_cast<aris::core::PointerArray<aris::dynamic::MotionBase,
                                                aris::dynamic::Element>& (
               aris::dynamic::Model::*)()>(
               &aris::dynamic::Model::generalMotionPool),
           py::return_value_policy::reference_internal)
      .def("generalMotionPool",
           static_cast<const aris::core::PointerArray<aris::dynamic::MotionBase,
                                                      aris::dynamic::Element>& (
               aris::dynamic::Model::*)() const>(
               &aris::dynamic::Model::generalMotionPool),
           py::return_value_policy::reference_internal)
      .def("partPool",
           static_cast<aris::core::PointerArray<aris::dynamic::Part,
                                                aris::dynamic::Element>& (
               aris::dynamic::Model::*)()>(&aris::dynamic::Model::partPool),
           py::return_value_policy::reference_internal)
      .def("partPool",
           static_cast<const aris::core::PointerArray<aris::dynamic::Part,
                                                      aris::dynamic::Element>& (
               aris::dynamic::Model::*)() const>(
               &aris::dynamic::Model::partPool),
           py::return_value_policy::reference_internal)
      .def("jointPool",
           static_cast<aris::core::PointerArray<aris::dynamic::Joint,
                                                aris::dynamic::Element>& (
               aris::dynamic::Model::*)()>(&aris::dynamic::Model::jointPool),
           py::return_value_policy::reference_internal)
      .def("jointPool",
           static_cast<const aris::core::PointerArray<aris::dynamic::Joint,
                                                      aris::dynamic::Element>& (
               aris::dynamic::Model::*)() const>(
               &aris::dynamic::Model::jointPool),
           py::return_value_policy::reference_internal)
      .def("forcePool",
           static_cast<aris::core::PointerArray<aris::dynamic::Force,
                                                aris::dynamic::Element>& (
               aris::dynamic::Model::*)()>(&aris::dynamic::Model::forcePool),
           py::return_value_policy::reference_internal)
      .def("forcePool",
           static_cast<const aris::core::PointerArray<aris::dynamic::Force,
                                                      aris::dynamic::Element>& (
               aris::dynamic::Model::*)() const>(
               &aris::dynamic::Model::forcePool),
           py::return_value_policy::reference_internal)
      .def("ground",
           py::overload_cast<>(&aris::dynamic::Model::ground, py::const_),
           py::return_value_policy::reference_internal)
      .def("ground", py::overload_cast<>(&aris::dynamic::Model::ground),
           py::return_value_policy::reference_internal)
      .def(
          "simulatorPool",
          static_cast<aris::core::PointerArray<aris::dynamic::Simulator,
                                               aris::dynamic::Element>& (
              aris::dynamic::Model::*)()>(&aris::dynamic::Model::simulatorPool),
          py::return_value_policy::reference_internal);  // 未重载还需修改
  py::class_<aris::dynamic::Element>(m, "Element")
      .def(py::init<>())
      .def_property("id", &aris::dynamic::Element::id,
                    &aris::dynamic::Element::setId);
  py::class_<aris::dynamic::Coordinate, aris::dynamic::Element>(m,
                                                                "Coordinate");
  py::class_<aris::dynamic::Part, aris::dynamic::Coordinate>(m, "Part")
      .def(py::init<>())
      .def_property("id", &aris::dynamic::Part::id, &aris::dynamic::Part::setId)
      .def_property("name", &aris::dynamic::Part::name,
                    &aris::dynamic::Part::setName)
      .def_property("geometryPool",
                    py::overload_cast<>(&aris::dynamic::Part::geometryPool),
                    &aris::dynamic::Part::resetGeometryPool)
      .def_property(
          "vs",
          [](const aris::dynamic::Part& self) {
            std::vector<double> vs(6, 0);
            self.getVs(vs.data());
            return vs;
          },
          [](aris::dynamic::Part& self, const std::vector<double>& vs) {
            if (vs.size() != 6) {
              throw std::runtime_error("Input array 'vs' size must be 6!");
            }
            self.setVs(vs.data());
          })
      .def("getAs",
           [](const aris::dynamic::Part& self) {
             std::vector<double> as(6, 0);
             self.getAs(as.data());
             return as;
           })
      .def("getVs",
           [](const aris::dynamic::Part& self) {
             std::vector<double> vs(6, 0);
             self.getVs(vs.data());
             return vs;
           })
      .def("getPq",
           [](const aris::dynamic::Part& self) {
             std::vector<double> pq(7, 0);
             self.getPq(pq.data());
             return pq;
           })
      .def("getPm",
           [](const aris::dynamic::Part& self) {
             std::vector<double> pm(16, 0);
             self.getPm(pm.data());
             return pm;
           })
      .def("addMarker", [](aris::dynamic::Part& self,
                           const std::string& name) { self.addMarker(name); })
      .def(
          "addMeshGeometry",
          [](aris::dynamic::Part& self, sire::PartId prtId,
             const std::string& resPath, std::vector<double>& pm) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            return self.geometryPool().add<sire::geometry::MeshGeometry>(
                resPath, prtId, true, prt_pm);
          },
          py::arg("part_id"), py::arg("resPath"),
          py::arg("prt_pm") = py::none(),
          py::return_value_policy::reference_internal)
      .def(
          "addBoxGeometry",
          [](aris::dynamic::Part& self, sire::PartId prtId, double x, double y,
             double z, std::vector<double>& pm) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            return self.geometryPool().add<sire::geometry::BoxGeometry>(
                x, y, z, prtId, true, prt_pm);
          },
          py::arg("part_id"), py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("prt_pm") = py::none(),
          py::return_value_policy::reference_internal)
      .def(
          "addSphereGeometry",
          [](aris::dynamic::Part& self, sire::PartId prtId, double radius,
             std::vector<double>& pm) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            return self.geometryPool().add<sire::geometry::SphereGeometry>(
                radius, prtId, true, prt_pm);
          },
          py::arg("part_id"), py::arg("radius"), py::arg("prt_pm") = py::none(),
          py::return_value_policy::reference_internal)
      .def(
          "addCylinderGeometry",
          [](aris::dynamic::Part& self, sire::PartId prtId, double radius,
             double length, std::vector<double>& pm) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            return self.geometryPool().add<sire::geometry::CylinderGeometry>(
                radius, length, prtId, true, prt_pm);
          },
          py::arg("part_id"), py::arg("radius"), py::arg("length"),
          py::arg("prt_pm") = py::none(),
          py::return_value_policy::reference_internal)
      .def(
          "addCapsuleGeometry",
          [](aris::dynamic::Part& self, sire::PartId prtId, double radius,
             double length, std::vector<double>& pm) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            return self.geometryPool().add<sire::geometry::CapsuleGeometry>(
                radius, length, prtId, true, prt_pm);
          },
          py::arg("part_id"), py::arg("radius"), py::arg("length"),
          py::arg("prt_pm") = py::none(),
          py::return_value_policy::reference_internal)
      .def("cptGeometryInertial2Part",
           [](aris::dynamic::Part& self, double mass) {
             sire::geometry::ShapeToInertia cal_;
             std::vector<double> part_iv(10, 0);
             for (sire::Size i = 0; i < self.geometryPool().size(); ++i) {
               std::vector<double> temp(10, 0), iv(10, 0);
               temp[0] = mass;
               auto& geometry = dynamic_cast<sire::geometry::GeometryBase&>(
                   self.geometryPool().at(i));
               geometry.shape()->Reify(&cal_, temp.data());
               //  std::cout << "part name: " << self.name()
               //            << ", geometry id: " << geometry.geometryId()
               //            << ", mass: " << temp[0] << std::endl;
               //  aris::dynamic::dsp(1, 10, temp.data());
               aris::dynamic::s_iv2iv(*geometry.pm(), temp.data(), iv.data());
               aris::dynamic::s_va(10, iv.data(), part_iv.data());
             }
             self.setPrtIv(part_iv.data());
           })
      .def("markerPm", [](aris::dynamic::Part& self, int i) {
        auto pm = *self.markerPool()[i].pm();
        return std::vector<double>(pm, pm + 16);
      });

  py::class_<aris::dynamic::Joint>(m, "Joint")
      .def_property("name", &aris::dynamic::Joint::name,
                    &aris::dynamic::Joint::setName)
      .def("dim", &aris::dynamic::Joint::dim)
      .def("cf",
           [](const aris::dynamic::Joint& self) {
             return std::vector<double>(self.cf(), self.cf() + self.dim());
           })
      .def("posAndZDirection", [](const aris::dynamic::Joint& self) {
        auto pm = *self.makI()->pm();
        return std::vector<double>{pm[3], pm[7], pm[11], pm[2], pm[6], pm[10]};
      });
  py::class_<aris::dynamic::RevoluteJoint, aris::dynamic::Joint>(
      m, "RevoluteJoint");
  py::class_<sire::core::FixedJoint, aris::dynamic::Joint>(m, "FixedJoint");
  py::class_<aris::dynamic::PrismaticJoint, aris::dynamic::Joint>(
      m, "PrismaticJoint");
  py::class_<aris::dynamic::SphericalJoint, aris::dynamic::Joint>(
      m, "SphericalJoint");
  py::class_<aris::dynamic::Force>(m, "Force");
  py::class_<aris::dynamic::SingleComponentForce, aris::dynamic::Force>(
      m, "SingleComponentForce")
      .def(py::init<>())
      .def_property("fce", &aris::dynamic::SingleComponentForce::fce,
                    py::overload_cast<double>(
                        &aris::dynamic::SingleComponentForce::setFce));
  py::class_<aris::dynamic::GeneralForce, aris::dynamic::Force>(m,
                                                                "GeneralForce")
      .def(py::init<>())
      .def_property(
          "fce",
          [](aris::dynamic::GeneralForce& self) {
            return std::vector<double>(self.fce(), self.fce() + 6);
          },
          [](aris::dynamic::GeneralForce& self, std::vector<double>& fce) {
            self.setFce(fce.data());
          });
  py::class_<aris::dynamic::Constraint>(m, "Constraint");
  py::class_<aris::dynamic::MotionBase, aris::dynamic::Constraint>(
      m, "MotionBase");
  py::class_<aris::dynamic::GeneralMotion, aris::dynamic::MotionBase>(
      m, "GeneralMotion")
      .def(py::init<>())  // 默认构造函数
      .def("setMpe",
           [](aris::dynamic::GeneralMotion& self, const std::vector<double>& pe,
              const char* eul_type) {
             if (pe.empty()) {
               throw std::runtime_error("Input array 'pe' cannot be empty!");
             }
             self.setMpe(pe.data(), eul_type);
           })
      .def("getMpe",
           [](aris::dynamic::GeneralMotion& self,
              const char* eul_type) -> std::vector<double> {
             double pe[6];
             self.getMpe(pe, eul_type);
             return std::vector<double>(pe, pe + 6);
           })
      .def("setMva",
           [](aris::dynamic::GeneralMotion& self,
              const std::vector<double>& mva) {
             if (mva.size() != 6) {
               throw std::runtime_error("Input array 'mva' must be of size 6!");
             }
             self.setMva(mva.data());
           })
      .def("getMaa",
           [](aris::dynamic::GeneralMotion& self) -> std::vector<double> {
             double maa[6];
             self.getMaa(maa);
             return std::vector<double>(maa, maa + 6);
           });

  py::class_<aris::dynamic::Geometry, aris::dynamic::Element>(m, "Geometry");

  py::class_<sire::geometry::GeometryBase, aris::dynamic::Geometry>(
      m, "GeometryBase")
      .def_property("id", &sire::geometry::GeometryBase::geometryId,
                    &sire::geometry::GeometryBase::setGeometryId)
      .def("setPm",
           [](sire::geometry::GeometryBase& self,
              const std::vector<double>& pm) {
             if (pm.size() != 16) {
               throw std::runtime_error("Input array 'pm' must be of size 16!");
             }
             self.setPm(pm.data());
           })
      .def("getPm",
           [](sire::geometry::GeometryBase& self) -> std::vector<double> {
             return std::vector<double>(*self.pm(), *self.pm() + 16);
           });

  py::class_<sire::geometry::GeometryOnPart, sire::geometry::GeometryBase>(
      m, "GeometryOnPart")
      .def(py::init<>())
      .def_property("id", &sire::geometry::GeometryOnPart::geometryId,
                    &sire::geometry::GeometryOnPart::setGeometryId)
      .def_property(
          "prtId", py::overload_cast<>(&sire::geometry::GeometryOnPart::partId),
          &sire::geometry::GeometryOnPart::setPartId)
      .def_property("isDynamic", &sire::geometry::GeometryOnPart::isDynamic,
                    &sire::geometry::GeometryOnPart::setDynamic);

  py::class_<sire::physics::geometry::CollidableGeometry,
             sire::geometry::GeometryOnPart>(m, "CollidableGeometry")
      .def(py::init<>())
      .def_property("material",
                    &sire::physics::geometry::CollidableGeometry::material,
                    &sire::physics::geometry::CollidableGeometry::setMaterial)
      .def_property(
          "contactProp",
          &sire::physics::geometry::CollidableGeometry::contactPropStr,
          py::overload_cast<std::string&>(
              &sire::physics::geometry::CollidableGeometry::setContactProp));

  py::class_<sire::geometry::MeshShape>(m, "MeshShape")
      .def(py::init<const std::string&>())
      .def("resourcePath",
           py::overload_cast<>(&sire::geometry::MeshShape::resourcePath))
      .def("setResourcePath", &sire::geometry::MeshShape::setResourcePath);
  py::class_<sire::geometry::MeshGeometry, sire::geometry::GeometryOnPart>(
      m, "MeshGeometry")
      .def(py::init<>())
      .def_readwrite("meshShape", &sire::geometry::MeshGeometry::meshShape);
  py::class_<sire::physics::geometry::MeshCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "MeshCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "meshShape",
          &sire::physics::geometry::MeshCollisionGeometry::meshShape);

  py::class_<sire::geometry::BoxShape>(m, "BoxShape")
      .def(py::init<double, double, double>())
      .def(py::init<>([](std::vector<double>& size) {
        return new sire::geometry::BoxShape(size[0], size[1], size[2]);
      }))
      .def("side", [](sire::geometry::BoxShape& self) {
        return std::vector<double>(self.side(), self.side() + 3);
      });
  py::class_<sire::geometry::BoxGeometry, sire::geometry::GeometryOnPart>(
      m, "BoxGeometry")
      .def(py::init<>())
      .def_readwrite("boxShape", &sire::geometry::BoxGeometry::boxShape);
  py::class_<sire::physics::geometry::BoxCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "BoxCollisionGeometry")
      .def(py::init<>())
      .def_readwrite("boxShape",
                     &sire::physics::geometry::BoxCollisionGeometry::boxShape);

  py::class_<sire::geometry::SphereShape>(m, "SphereShape")
      .def(py::init<double>())
      // .def(py::init<>(
      //     [](double r) { return new sire::geometry::SphereShape(r); }))
      .def("radius",
           [](sire::geometry::SphereShape& self) { return self.radius(); });
  py::class_<sire::geometry::SphereGeometry, sire::geometry::GeometryOnPart>(
      m, "SphereGeometry")
      .def(py::init<>())
      .def_readwrite("sphereShape",
                     &sire::geometry::SphereGeometry::sphereShape);
  py::class_<sire::physics::geometry::SphereCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "SphereCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "sphereShape",
          &sire::physics::geometry::SphereCollisionGeometry::sphereShape);

  py::class_<sire::geometry::CapsuleShape>(m, "CapsuleShape")
      .def(py::init<double, double>())
      .def_property("radius", &sire::geometry::CapsuleShape::radius,
                    &sire::geometry::CapsuleShape::setRadius)
      .def_property("length", &sire::geometry::CapsuleShape::length,
                    &sire::geometry::CapsuleShape::setLength);
  py::class_<sire::geometry::CapsuleGeometry, sire::geometry::GeometryOnPart>(
      m, "CapsuleGeometry")
      .def(py::init<>())
      .def_readwrite("capsuleShape",
                     &sire::geometry::CapsuleGeometry::capsuleShape);
  py::class_<sire::physics::geometry::CapsuleCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "CapsuleCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "capsuleShape",
          &sire::physics::geometry::CapsuleCollisionGeometry::capsuleShape);

  py::class_<sire::geometry::CylinderShape>(m, "CylinderShape")
      .def(py::init<double, double>())
      .def_property("radius", &sire::geometry::CylinderShape::radius,
                    &sire::geometry::CylinderShape::setRadius)
      .def_property("length", &sire::geometry::CylinderShape::length,
                    &sire::geometry::CylinderShape::setLength);
  py::class_<sire::geometry::CylinderGeometry, sire::geometry::GeometryOnPart>(
      m, "CylinderGeometry")
      .def(py::init<>())
      .def_readwrite("cylinderShape",
                     &sire::geometry::CylinderGeometry::cylinderShape);
  py::class_<sire::physics::geometry::CylinderCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "CylinderCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "cylinderShape",
          &sire::physics::geometry::CylinderCollisionGeometry::cylinderShape);

  py::class_<aris::core::PointerArray<
      sire::physics::geometry::CollidableGeometry, aris::dynamic::Geometry>>(
      m, "PointerArrayCollidableGeometry")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<
                 sire::physics::geometry::CollidableGeometry,
                 aris::dynamic::Geometry>& self,
             size_t index) -> sire::physics::geometry::CollidableGeometry& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal);

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
      // .def("mp", &aris::dynamic::Motion::mp)
      // .def("mv", &aris::dynamic::Motion::mv)
      // .def("ma", &aris::dynamic::Motion::ma)
      // .def("mf", &aris::dynamic::Motion::mf)
      // .def("setMa", &aris::dynamic::Motion::setMa)
      .def("updA", &aris::dynamic::Motion::updA)
      .def("updV", &aris::dynamic::Motion::updV)
      .def("updP", &aris::dynamic::Motion::updP);
  py::class_<aris::dynamic::Simulator, aris::dynamic::Element>(m, "Simulator");
  py::class_<aris::core::PointerArray<aris::dynamic::Geometry,
                                      aris::dynamic::Element>>(
      m, "PointerArrayGeometry")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Geometry,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Geometry& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal);
  py::class_<aris::core::PointerArray<aris::dynamic::Simulator,
                                      aris::dynamic::Element>>(
      m, "PointerArraySimulator")
      .def(py::init<>())
      .def(
          "add_adams",
          [](aris::core::PointerArray<aris::dynamic::Simulator,
                                      aris::dynamic::Element>& self)
              -> aris::dynamic::AdamsSimulator& {
            return self.add<aris::dynamic::AdamsSimulator>();
          },
          py::return_value_policy::reference_internal)
      .def(
          "add_solver_simulator",
          [](aris::core::PointerArray<aris::dynamic::Simulator,
                                      aris::dynamic::Element>& self)
              -> aris::dynamic::SolverSimulator& {
            return self.add<aris::dynamic::SolverSimulator>();
          },
          py::return_value_policy::reference_internal);
  py::class_<
      aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element>>(
      m, "PointerArraySolver")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Solver,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Solver& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal)
      .def(
          "add_ik",
          [](aris::core::PointerArray<aris::dynamic::Solver,
                                      aris::dynamic::Element>& self)
              -> aris::dynamic::InverseKinematicSolver& {
            // 把 Python 对象转换为 C++ 指针（Solver* 或派生类*）
            return self.add<aris::dynamic::InverseKinematicSolver>();
          },
          py::return_value_policy::reference_internal)
      .def(
          "add_fd",
          [](aris::core::PointerArray<aris::dynamic::Solver,
                                      aris::dynamic::Element>& self)
              -> aris::dynamic::ForwardDynamicSolver& {
            return self.add<aris::dynamic::ForwardDynamicSolver>();
          },
          py::return_value_policy::reference_internal)
      .def(
          "add_id",
          [](aris::core::PointerArray<aris::dynamic::Solver,
                                      aris::dynamic::Element>& self)
              -> aris::dynamic::InverseDynamicSolver& {
            return self.add<aris::dynamic::InverseDynamicSolver>();
          },
          py::return_value_policy::reference_internal)
      .def(
          "add_fk",
          [](aris::core::PointerArray<aris::dynamic::Solver,
                                      aris::dynamic::Element>& self)
              -> aris::dynamic::ForwardKinematicSolver& {
            return self.add<aris::dynamic::ForwardKinematicSolver>();
          },
          py::return_value_policy::reference_internal);
  py::class_<
      aris::core::PointerArray<aris::dynamic::Motion, aris::dynamic::Element>>(
      m, "PointerArrayMotion")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Motion,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Motion& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal);
  py::class_<
      aris::core::PointerArray<aris::dynamic::Force, aris::dynamic::Element>>(
      m, "PointerArrayForce")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Force,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Force& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal);

  py::class_<aris::core::PointerArray<aris::dynamic::GeneralMotion,
                                      aris::dynamic::Element>>(
      m, "PointerArrayGeneralMotion")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::GeneralMotion,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::GeneralMotion& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal);
  py::class_<
      aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>>(
      m, "PointerArrayPart")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Part,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Part& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal);
  py::class_<
      aris::core::PointerArray<aris::dynamic::Joint, aris::dynamic::Element>>(
      m, "PointerArrayJoint")
      .def(py::init<>())
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Joint,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Joint& {
            return self[index];  // 假设 self 支持 operator[]
          },
          py::return_value_policy::reference_internal);

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
      .def("forward", &sire::actuator::ActuatorSISO::forward)
      .def("cptOutput", &sire::actuator::ActuatorSISO::cptOutput)
      .def_static("add2Model", &sire::actuator::ActuatorSISO::add2Model,
                  py::return_value_policy::reference);
}