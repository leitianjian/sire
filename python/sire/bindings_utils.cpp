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

#define BIND_FROM_XML_FILE(m, T)                             \
  m.def("fromXmlFile", [](T& self, const std::string& xml) { \
    aris::core::Instance ins(self);                          \
    aris::core::fromXmlFile(ins, xml);                       \
  })

#define BIND_TO_XML_STRING(m, T)         \
  m.def("toXmlString", [](T& self) {     \
    aris::core::Instance ins(self);      \
    return aris::core::toXmlString(ins); \
  })

void init_utils(py::module& m) {
  // 常量绑定
  m.attr("kPosQuatSize") = sire::kPosQuatSize;
  BIND_TO_XML_STRING(m, aris::server::ControlServer);
  BIND_TO_XML_STRING(m, aris::dynamic::Model);
  BIND_TO_XML_STRING(m, sire::simulator::Simulator);
  // 使用
  BIND_FROM_XML_FILE(m, aris::server::ControlServer);
  BIND_FROM_XML_FILE(m, aris::dynamic::Model);
  BIND_FROM_XML_FILE(m, sire::simulator::Simulator);
  m.def("fromXmlString",
        [](aris::server::ControlServer& self, const std::string& xml) {
          aris::core::fromXmlString(self, xml);
        });
  m.def("fromXmlString",
        [](aris::dynamic::Model& self, const std::string& xml) {
          aris::core::fromXmlString(self, xml);
        });
  m.def("s_mm", [](int m, int n, int k, std::vector<double>& pm1,
                   std::vector<double>& pm2) {
    std::vector<double> pm_out(m * n);
    aris::dynamic::s_mm(m, n, k, pm1.data(), pm2.data(), pm_out.data());
    return pm_out;
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
  m.def("vp2vs", [](std::vector<double>& pp, std::vector<double>& vp,
                    std::vector<double>& w) {
    std::vector<double> vs(6);
    std::copy(w.begin(), w.end(), vs.begin() + 3);   // vs[3:6] = ω
    aris::dynamic::s_vp2vs(pp.data(), vp.data(), vs.data());
    return vs;
  });
  // backward-compatible 2-arg overload (w defaults to zero)
  m.def("vp2vs", [](std::vector<double>& pp, std::vector<double>& vp) {
    std::vector<double> w_zero{0.0, 0.0, 0.0};
    std::vector<double> vs(6);
    aris::dynamic::s_vp2vs(pp.data(), vp.data(), vs.data());
    return vs;
  });
  m.def("as2ap", [](std::vector<double>& vs, std::vector<double>& as,
                    std::vector<double>& pWorld) {
    std::vector<double> ap(3);
    aris::dynamic::s_as2ap(vs.data(), as.data(), pWorld.data(), ap.data());
    return ap;
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
      "simulationLoop",
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

  // ── Batch motion state read/write (avoids per-joint pybind11 wrappers) ──
  m.def(
      "getMotionMps",
      [](aris::dynamic::Model& model) {
        auto& pool = model.motionPool();
        std::vector<double> mps(pool.size());
        for (size_t i = 0; i < pool.size(); ++i) {
          mps[i] = pool[i].mp();
        }
        return mps;
      },
      "Read all motion positions (rad) into a list in motionPool order.");
  m.def(
      "getMotionMvs",
      [](aris::dynamic::Model& model) {
        auto& pool = model.motionPool();
        std::vector<double> mvs(pool.size());
        for (size_t i = 0; i < pool.size(); ++i) {
          mvs[i] = pool[i].mv();
        }
        return mvs;
      },
      "Read all motion velocities (rad/s) into a list in motionPool order.");
  m.def(
      "setMotionDesiredValues",
      [](aris::dynamic::Model& model, const std::vector<double>& values) {
        auto& pool = model.motionPool();
        size_t n = std::min(values.size(), pool.size());
        for (size_t i = 0; i < n; ++i) {
          auto* actuator =
              dynamic_cast<sire::actuator::ActuatorSISO*>(&pool[i]);
          if (actuator != nullptr) {
            actuator->setDesiredValue(values[i]);
          }
        }
      },
      "Write desired torque/position values to all actuators in motionPool order.",
      py::arg("model"), py::arg("values"));
  m.def(
      "setMotionMps",
      [](aris::dynamic::Model& model, const std::vector<double>& mps) {
        auto& pool = model.motionPool();
        size_t n = std::min(mps.size(), pool.size());
        for (size_t i = 0; i < n; ++i) {
          pool[i].setMp(mps[i]);
        }
      },
      "Write all motion positions (rad) in motionPool order.",
      py::arg("model"), py::arg("mps"));
  m.def(
      "setMotionMvs",
      [](aris::dynamic::Model& model, const std::vector<double>& mvs) {
        auto& pool = model.motionPool();
        size_t n = std::min(mvs.size(), pool.size());
        for (size_t i = 0; i < n; ++i) {
          pool[i].setMv(mvs[i]);
        }
      },
      "Write all motion velocities (rad/s) in motionPool order.",
      py::arg("model"), py::arg("mvs"));
  m.def(
      "getBasePqVs",
      [](aris::dynamic::Model& model, int part_idx) {
        auto& part = model.partPool().at(part_idx);
        std::vector<double> pq(7), vs(6);
        part.getPq(pq.data());
        part.getVs(vs.data());
        return std::make_pair(pq, vs);
      },
      "Read base link pq (7) and vs (6) in one call. Returns (pq, vs) tuple.",
      py::arg("model"), py::arg("part_idx") = 1);

  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const std::exception& e) {
      PyErr_SetString(PyExc_RuntimeError, e.what());
    } catch (...) {
      PyErr_SetString(PyExc_RuntimeError, "Unknown C++ exception");
    }
  });

  // ── PointerArray<T> bindings (must register before classes that expose them) ──
}
