// ═══════════════════════════════════════════════════════════════════
//  sire Python bindings — main entry point
//  Domain-specific bindings are in bindings_*.cpp
// ═══════════════════════════════════════════════════════════════════
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
#include "sire/core/force_screw.hpp"
#include "sire/core/geometry/box_geometry.hpp"
#include "sire/core/geometry/capsule_geometry.hpp"
#include "sire/core/geometry/cylinder_geometry.hpp"
#include "sire/core/geometry/mesh_geometry.hpp"
#include "sire/core/geometry/shape_calculator.hpp"
#include "sire/core/geometry/sphere_geometry.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/core/sire_fixed_joint.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/contact/analytical_implicit_friction_solver.hpp"
#include "sire/physics/contact/contact_position_force_solver.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/physics/contact/ps_vs_solver.hpp"
#include "sire/physics/contact/ps_vs_solver2.hpp"
#include "sire/physics/geometry/box_collision_geometry.hpp"
#include "sire/physics/geometry/capsule_collision_geometry.hpp"
#include "sire/physics/geometry/collidable.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/geometry/cylinder_collision_geometry.hpp"
#include "sire/physics/geometry/mesh_collision_geometry.hpp"
#include "sire/physics/geometry/sphere_collision_geometry.hpp"
#include "sire/simulator/event_manager.hpp"
#include "sire/simulator/simulator.hpp"

#include "pybind11_json.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

// Forward declarations
void init_pointer_arrays(py::module& m);
void init_utils(py::module& m);
void init_server(py::module& m);
void init_simulator(py::module& m);
void init_physics(py::module& m);
void init_model(py::module& m);
void init_core(py::module& m);

PYBIND11_MODULE(sire, m) {
  m.attr("kPosQuatSize") = sire::kPosQuatSize;

  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const std::exception& e) {
      PyErr_SetString(PyExc_RuntimeError, e.what());
    } catch (...) {
      PyErr_SetString(PyExc_RuntimeError, "Unknown C++ exception");
    }
  });

  init_utils(m);
  init_pointer_arrays(m);
  init_server(m);
  init_simulator(m);
  init_physics(m);
  init_model(m);
  init_core(m);
}
