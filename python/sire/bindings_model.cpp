// Auto-split from sire_bindings.cpp
#include <array>
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

void init_model(py::module& m) {
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
      .def("cptProjectedMassMatrix",
           [](const aris::dynamic::Model& self) {
             auto& fd =
                 dynamic_cast<const aris::dynamic::ForwardDynamicSolver&>(
                     self.solverPool()[3]);
             const_cast<aris::dynamic::ForwardDynamicSolver&>(fd)
                 .cptProjectedMassMatrix();
           })
      .def("cptContactInverseInertiaMatrix",
           [](const aris::dynamic::Model& self, int nContact,
              std::vector<int> partid, std::vector<double> T_vec,
              std::vector<double> contactPoint) {
             std::vector<double> A_out, b_out;
             auto& fd =
                 dynamic_cast<const aris::dynamic::ForwardDynamicSolver&>(
                     self.solverPool()[3]);
             const_cast<aris::dynamic::ForwardDynamicSolver&>(fd)
                 .cptContactInverseInertiaMatrix(
                     nContact, partid.data(), T_vec.data(), contactPoint.data(),
                     A_out, b_out);
             return A_out;
           })
      .def_property_readonly("nbody",
                             [](const aris::dynamic::Model& self) -> int {
                               return (int)self.partPool().size();
                             })  // 获取刚体数
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
      .def("getPartAs",
           [](const aris::dynamic::Model& self) {
             std::vector<std::vector<double>> as(self.partPool().size());
             for (size_t i = 0; i < self.partPool().size(); ++i) {
               as[i].resize(7);
               self.partPool().at(i).getAs(as[i].data());
             }
             return as;
           })
      .def("setPartPq",
           [](aris::dynamic::Model& self,
              const std::vector<std::vector<double>>& pqs) {
             if (pqs.size() != self.partPool().size()) {
               throw std::runtime_error(
                   "Input array 'pqs' size must match the number of parts!");
             }

             for (size_t i = 0; i < pqs.size(); ++i) {
               if (pqs[i].size() != 7) {
                 throw std::runtime_error(
                     "Each part's 'pq' array must be 7-element!");
               }
               self.partPool().at(i).setPq(pqs[i].data());
             }
           })
      .def("setPartVs",
           [](aris::dynamic::Model& self,
              const std::vector<std::vector<double>>& vss) {
             if (vss.size() != self.partPool().size()) {
               throw std::runtime_error(
                   "Input array 'vss' size must match the number of parts!");
             }

             for (size_t i = 0; i < vss.size(); ++i) {
               if (vss[i].size() != 6) {
                 throw std::runtime_error(
                     "Each part's 'vs' array must be 6-element!");
               }
               self.partPool().at(i).setVs(vss[i].data());
             }
           })
      .def(
          "setMotorForce",
          [](aris::dynamic::Model& self, const std::vector<double>& forces) {
            if (forces.size() != self.motionPool().size()) {
              throw std::runtime_error(
                  "Input array 'forces' size must match the number of joints!");
            }
            sire::Size startIdx = self.forcePool().size() -
                                  self.motionPool().size() -
                                  self.partPool().size();
            for (size_t i = 0; i < forces.size(); ++i) {
              dynamic_cast<aris::dynamic::SingleComponentForce&>(
                  self.forcePool().at(startIdx + i))
                  .setFce(forces[i]);
            }
          })
      .def(
          "setGeneralForce",
          [](aris::dynamic::Model& self,
             const std::vector<std::vector<double>>& gf) {
            if (gf.size() != self.partPool().size()) {
              throw std::runtime_error(
                  "Input array 'forces' size must match the number of forces!");
            }
            sire::Size startIdx =
                self.forcePool().size() - self.partPool().size();
            for (size_t i = 0; i < gf.size(); ++i) {
              dynamic_cast<aris::dynamic::GeneralForce&>(
                  self.forcePool().at(startIdx + i))
                  .setFce(gf[i].data());
            }
          })
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
      .def("forwardKinematicsVel",
           [](aris::dynamic::Model& self) {
             return self.forwardKinematicsVel();
           })
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
      .def_property(
          "pq",
          [](const aris::dynamic::Part& self) {
            std::vector<double> pq(7, 0);
            self.getPq(pq.data());
            return pq;
          },
          [](aris::dynamic::Part& self, const std::vector<double>& pq) {
            if (pq.size() != 7) {
              throw std::runtime_error("Input array 'pq' size must be 7!");
            }
            self.setPq(pq.data());
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
            std::array<double, 3> default_scale{1.0, 1.0, 1.0};
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            return self.geometryPool().add<sire::geometry::MeshGeometry>(
                resPath, default_scale, prtId, true, prt_pm);
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
           })
      .def(
          "cptInertial",
          [](sire::geometry::GeometryBase& self, aris::dynamic::Part& part,
             double mass) {
            if (mass <= 0.0) {
              throw std::runtime_error("Geometry mass must be positive!");
            }

            auto* geometry_on_part =
                dynamic_cast<sire::geometry::GeometryOnPart*>(&self);
            if (geometry_on_part == nullptr) {
              throw std::runtime_error(
                  "Inertia can only be assigned from a geometry on a part!");
            }
            if (geometry_on_part->partId() != part.id()) {
              throw std::runtime_error(
                  "Geometry and target part have different part ids!");
            }

            auto* shape = self.shape();
            if (shape == nullptr) {
              throw std::runtime_error(
                  "This geometry does not expose a shape for inertia "
                  "calculation!");
            }

            sire::geometry::ShapeToInertia calculator;
            std::array<double, 10> geometry_iv{};
            std::array<double, 10> part_iv{};
            geometry_iv[0] = mass;
            shape->Reify(&calculator, geometry_iv.data());
            aris::dynamic::s_iv2iv(*self.pm(), geometry_iv.data(),
                                   part_iv.data());
            part.setPrtIv(part_iv.data());
          },
          py::arg("part"), py::arg("mass"),
          "Compute this geometry's inertia, express it in the part frame, "
          "and assign it to the target part.");

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
      .def_readwrite("meshShape", &sire::geometry::MeshGeometry::typedShape);
  py::class_<sire::physics::geometry::MeshCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "MeshCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "meshShape",
          &sire::physics::geometry::MeshCollisionGeometry::typedShape);

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
      .def_readwrite("boxShape", &sire::geometry::BoxGeometry::typedShape);
  py::class_<sire::physics::geometry::BoxCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "BoxCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "boxShape",
          &sire::physics::geometry::BoxCollisionGeometry::typedShape);

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
                     &sire::geometry::SphereGeometry::typedShape);
  py::class_<sire::physics::geometry::SphereCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "SphereCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "sphereShape",
          &sire::physics::geometry::SphereCollisionGeometry::typedShape);

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
                     &sire::geometry::CapsuleGeometry::typedShape);
  py::class_<sire::physics::geometry::CapsuleCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "CapsuleCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "capsuleShape",
          &sire::physics::geometry::CapsuleCollisionGeometry::typedShape);

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
                     &sire::geometry::CylinderGeometry::typedShape);
  py::class_<sire::physics::geometry::CylinderCollisionGeometry,
             sire::physics::geometry::CollidableGeometry>(
      m, "CylinderCollisionGeometry")
      .def(py::init<>())
      .def_readwrite(
          "cylinderShape",
          &sire::physics::geometry::CylinderCollisionGeometry::typedShape);
}
