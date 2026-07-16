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

void init_physics(py::module& m) {
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
      .def_property_readonly("contactForceIdx",
                             &sire::physics::PhysicsEngine::contactForceIdx)
      .def_property("collisionDetectionFlag",
                    &sire::physics::PhysicsEngine::collisionDetectionFlag,
                    &sire::physics::PhysicsEngine::setCollisionDetectionFlag)
      .def_property("contactSolverFlag",
                    &sire::physics::PhysicsEngine::contactSolverFlag,
                    &sire::physics::PhysicsEngine::setContactSolverFlag)
      .def_property("geometryPool", &sire::physics::PhysicsEngine::geometryPool,
                    &sire::physics::PhysicsEngine::resetGeometryPool)
      .def("numGeometries", &sire::physics::PhysicsEngine::numGeometries)
      .def(
          "addBoxGeometry",
          [](sire::physics::PhysicsEngine& self, double x, double y, double z,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             bool visible, const std::string& material,
             const std::string& propStr) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            self.geometryPool()
                .add<sire::physics::geometry::BoxCollisionGeometry>(
                    x, y, z, part_id, is_dynamic, pm.data(), visible, material,
                    propStr);
          },
          py::arg("x"), py::arg("y"), py::arg("z"), py::arg("part_id"),
          py::arg("is_dynamic") = true, py::arg("prt_pm") = py::list(),
          py::arg("visible") = true, py::arg("material") = "m1",
          py::arg("propStr") = "{}")
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
             bool visible, const std::string& material,
             const std::string& propStr) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            // 创建球体几何体
            self.geometryPool()
                .add<sire::physics::geometry::SphereCollisionGeometry>(
                    radius, part_id, is_dynamic, prt_pm, visible, material,
                    propStr);
          },
          py::arg("radius"), py::arg("part_id"), py::arg("is_dynamic") = true,
          py::arg("prt_pm") = py::none(), py::arg("visible") = true,
          py::arg("material") = "m1", py::arg("propStr") = "{}")
      .def(
          "addMeshGeometry",
          [](sire::physics::PhysicsEngine& self, const std::string& resPath,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             bool visible, const std::string& material,
             const std::string& propStr) {
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            std::array<double, 3> default_scale{1.0, 1.0, 1.0};
            // 创建网格几何体
            self.geometryPool()
                .add<sire::physics::geometry::MeshCollisionGeometry>(
                    resPath, default_scale, part_id, is_dynamic, prt_pm,
                    visible, material, propStr);
          },
          py::arg("resPath"), py::arg("part_id"), py::arg("is_dynamic") = true,
          py::arg("prt_pm") = py::none(), py::arg("visible") = true,
          py::arg("material") = "m1", py::arg("propStr") = "{}")
      .def(
          "addCapsuleGeometry",
          [](sire::physics::PhysicsEngine& self, double radius, double length,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             bool visible, const std::string& material,
             const std::string& propStr) {
            // 创建胶囊几何体
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            self.geometryPool()
                .add<sire::physics::geometry::CapsuleCollisionGeometry>(
                    radius, length, part_id, is_dynamic, prt_pm, visible,
                    material, propStr);
          },
          py::arg("radius"), py::arg("length"), py::arg("part_id"),
          py::arg("is_dynamic") = true, py::arg("prt_pm") = py::none(),
          py::arg("visible") = true, py::arg("material") = "m1",
          py::arg("propStr") = "{}")
      .def(
          "addCylinderGeometry",
          [](sire::physics::PhysicsEngine& self, double radius, double length,
             sire::PartId part_id, bool is_dynamic, std::vector<double>& pm,
             bool visible, const std::string& material,
             const std::string& propStr) {
            // 创建胶囊几何体
            const double* prt_pm =
                pm.size() != 16 ? sire::default_pm : pm.data();
            self.geometryPool()
                .add<sire::physics::geometry::CylinderCollisionGeometry>(
                    radius, length, part_id, is_dynamic, prt_pm, visible,
                    material, propStr);
          },
          py::arg("radius"), py::arg("length"), py::arg("part_id"),
          py::arg("is_dynamic") = true, py::arg("prt_pm") = py::none(),
          py::arg("visible") = true, py::arg("material") = "m1",
          py::arg("propStr") = "{}")
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
          "psVsSolver",
          [](sire::physics::PhysicsEngine& self)
              -> sire::physics::contact::ps_vs_solver::PsVsSolver& {
            return dynamic_cast<
                sire::physics::contact::ps_vs_solver::PsVsSolver&>(
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
      .def(
          "addPsVsSolver2",
          [](sire::physics::PhysicsEngine& self)
              -> sire::physics::contact::ps_vs_solver2::PsVsSolver2& {
            self.resetContactSolver(
                new sire::physics::contact::ps_vs_solver2::PsVsSolver2);
            return dynamic_cast<
                sire::physics::contact::ps_vs_solver2::PsVsSolver2&>(
                self.contactSolver());
          },
          py::return_value_policy::reference_internal)
      .def(
          "addAnalyticalTangentForceSolver",
          [](sire::physics::PhysicsEngine& self)
              -> sire::physics::contact::analytical_tangent_force::
                  AnalyticalTangentForceSolver& {
                    self.resetContactSolver(
                        new sire::physics::contact::analytical_tangent_force::
                            AnalyticalTangentForceSolver);
                    return dynamic_cast<
                        sire::physics::contact::analytical_tangent_force::
                            AnalyticalTangentForceSolver&>(
                        self.contactSolver());
                  },
          py::return_value_policy::reference_internal)
      .def(
          "addAnalyticalImplicitFrictionSolver",
          [](sire::physics::PhysicsEngine& self)
              -> sire::physics::contact::analytical_implicit_friction::
                  AnalyticalImplicitFrictionSolver& {
                    self.resetContactSolver(
                        new sire::physics::contact::
                            analytical_implicit_friction::
                                AnalyticalImplicitFrictionSolver);
                    return dynamic_cast<
                        sire::physics::contact::analytical_implicit_friction::
                            AnalyticalImplicitFrictionSolver&>(
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
      })
      // ---- MuJoCo-style mj_geom2body: geometry ID → part ID ----
      .def("geomIdToPartId",
           [](sire::physics::PhysicsEngine& self,
              sire::Size geomId) -> sire::Size {
             auto* geom = self.queryGeometryPoolById(geomId);
             if (geom == nullptr) {
               throw std::runtime_error(
                   "Geometry ID " + std::to_string(geomId) +
                   " not found in physics engine geometry pool");
             }
             return geom->partId();
           },
           py::arg("geom_id"),
           "Map a geometry ID to the part (link/body) ID that owns it.  "
           "Analogous to MuJoCo's mj_geom2body.");

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

  py::class_<sire::physics::contact::ps_vs_solver::PsVsSolver>(m, "PsVsSolver")
      .def(py::init<>())
      .def("addMaterialPair",
           [](sire::physics::contact::ps_vs_solver::PsVsSolver& self,
              const std::string& name1, const std::string& name2,
              const std::string& prop) {
             self.materialManager().addProp(
                 sire::core::SortedPair<std::string>(name1, name2),
                 sire::core::PropMap(prop));
           })
      .def("setDefaultProp",
           [](sire::physics::contact::ps_vs_solver::PsVsSolver& self,
              const std::string& prop) {
             self.materialManager().setDefaultProp(sire::core::PropMap(prop));
           });  // 默认构造函数
  py::class_<sire::physics::contact::ps_vs_solver2::PsVsSolver2>(m,
                                                                 "PsVsSolver2")
      .def(py::init<>())
      .def("addMaterialPair",
           [](sire::physics::contact::ps_vs_solver2::PsVsSolver2& self,
              const std::string& name1, const std::string& name2,
              const std::string& prop) {
             self.materialManager().addProp(
                 sire::core::SortedPair<std::string>(name1, name2),
                 sire::core::PropMap(prop));
           })
      .def("setDefaultProp",
           [](sire::physics::contact::ps_vs_solver2::PsVsSolver2& self,
              const std::string& prop) {
             self.materialManager().setDefaultProp(sire::core::PropMap(prop));
           });  // 默认构造函数
  py::class_<sire::physics::contact::analytical_tangent_force::
                 AnalyticalTangentForceSolver>(m,
                                               "AnalyticalTangentForceSolver")
      .def(py::init<>())
      .def("addMaterialPair",
           [](sire::physics::contact::analytical_tangent_force::
                  AnalyticalTangentForceSolver& self,
              const std::string& name1, const std::string& name2,
              const std::string& prop) {
             self.materialManager().addProp(
                 sire::core::SortedPair<std::string>(name1, name2),
                 sire::core::PropMap(prop));
           })
      .def("setDefaultProp",
           [](sire::physics::contact::analytical_tangent_force::
                  AnalyticalTangentForceSolver& self,
              const std::string& prop) {
             self.materialManager().setDefaultProp(sire::core::PropMap(prop));
           });  // 默认构造函数

  py::class_<sire::physics::contact::analytical_implicit_friction::
                 AnalyticalImplicitFrictionSolver>(
      m, "AnalyticalImplicitFrictionSolver")
      .def(py::init<>())
      .def("addMaterialPair",
           [](sire::physics::contact::analytical_implicit_friction::
                  AnalyticalImplicitFrictionSolver& self,
              const std::string& name1, const std::string& name2,
              const std::string& prop) {
             self.materialManager().addProp(
                 sire::core::SortedPair<std::string>(name1, name2),
                 sire::core::PropMap(prop));
           })
      .def("addMaterialPair",
           [](sire::physics::contact::analytical_implicit_friction::
                  AnalyticalImplicitFrictionSolver& self,
              const std::string& name1, const std::string& name2,
              const std::string& prop) {
             self.materialManager().addProp(
                 sire::core::SortedPair<std::string>(name1, name2),
                 sire::core::PropMap(prop));
           })
      .def("setDefaultProp",
           [](sire::physics::contact::analytical_implicit_friction::
                  AnalyticalImplicitFrictionSolver& self,
              const std::string& prop) {
             self.materialManager().setDefaultProp(sire::core::PropMap(prop));
           });  // 默认构造函数
}
