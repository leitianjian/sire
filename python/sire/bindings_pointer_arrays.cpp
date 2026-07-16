// ═══════════════════════════════════════════════════════════════════
//  PointerArray<T>  bindings  (shared by all other modules)
//  - __len__  → self.size()
//  - __getitem__ with bounds check → IndexError instead of crash
// ═══════════════════════════════════════════════════════════════════
#include <pybind11/pybind11.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/physics/geometry/collidable_geometry.hpp"

namespace py = pybind11;

void init_pointer_arrays(py::module& m) {
  // ── PointerArray<CollidableGeometry> ──
  py::class_<aris::core::PointerArray<
      sire::physics::geometry::CollidableGeometry, aris::dynamic::Geometry>>(
      m, "PointerArrayCollidableGeometry")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<
               sire::physics::geometry::CollidableGeometry,
               aris::dynamic::Geometry>& self) { return self.size(); })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<
                 sire::physics::geometry::CollidableGeometry,
                 aris::dynamic::Geometry>& self,
             size_t index) -> sire::physics::geometry::CollidableGeometry* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal);

  // ── PointerArray<Geometry> ──
  py::class_<aris::core::PointerArray<aris::dynamic::Geometry,
                                      aris::dynamic::Element>>(
      m, "PointerArrayGeometry")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::Geometry,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Geometry,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Geometry* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal);

  // ── PointerArray<Simulator> ──
  py::class_<aris::core::PointerArray<aris::dynamic::Simulator,
                                      aris::dynamic::Element>>(
      m, "PointerArraySimulator")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::Simulator,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Simulator,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Simulator* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal)
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

  // ── PointerArray<Solver> ──
  py::class_<
      aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element>>(
      m, "PointerArraySolver")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::Solver,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Solver,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Solver* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal)
      .def(
          "add_ik",
          [](aris::core::PointerArray<aris::dynamic::Solver,
                                      aris::dynamic::Element>& self)
              -> aris::dynamic::InverseKinematicSolver& {
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

  // ── PointerArray<Motion> ──
  py::class_<
      aris::core::PointerArray<aris::dynamic::Motion, aris::dynamic::Element>>(
      m, "PointerArrayMotion")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::Motion,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Motion,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Motion* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal);

  // ── PointerArray<Force> ──
  py::class_<
      aris::core::PointerArray<aris::dynamic::Force, aris::dynamic::Element>>(
      m, "PointerArrayForce")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::Force,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Force,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Force* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal);

  // ── PointerArray<GeneralMotion> ──
  py::class_<aris::core::PointerArray<aris::dynamic::GeneralMotion,
                                      aris::dynamic::Element>>(
      m, "PointerArrayGeneralMotion")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::GeneralMotion,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::GeneralMotion,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::GeneralMotion* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal);

  // ── PointerArray<Part> ──
  py::class_<
      aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>>(
      m, "PointerArrayPart")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::Part,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Part,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Part* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal);

  // ── PointerArray<Joint> ──
  py::class_<
      aris::core::PointerArray<aris::dynamic::Joint, aris::dynamic::Element>>(
      m, "PointerArrayJoint")
      .def(py::init<>())
      .def("__len__",
           [](const aris::core::PointerArray<aris::dynamic::Joint,
                                             aris::dynamic::Element>& self) {
             return self.size();
           })
      .def(
          "__getitem__",
          [](aris::core::PointerArray<aris::dynamic::Joint,
                                      aris::dynamic::Element>& self,
             size_t index) -> aris::dynamic::Joint* {
            if (index >= self.size()) {
              PyErr_SetString(PyExc_IndexError, "index out of range");
              throw py::error_already_set();
            }
            return &self[index];
          },
          py::return_value_policy::reference_internal);
}
