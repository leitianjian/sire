// Auto-split from sire_bindings.cpp
#include <algorithm>
#include <codecvt>
#include <fstream>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>
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
#include "sire/physics/contact/ps_vs_solver3.hpp"
#include "sire/physics/contact/ps_vs_solver_v5.hpp"
#include "sire/physics/contact/simple_admm_contact_solver.hpp"
#include "sire/physics/contact/newton_pipg_contact_solver.hpp"
#include "sire/physics/contact/exact_coulomb_contact_solver.hpp"
#include "sire/physics/geometry/box_collision_geometry.hpp"

// Forward-declare solver functions (v3/v4 defined in ps_vs_solver3.cpp, not in header):
namespace sire::physics::contact::ps_vs_solver3 {
auto cptContactForceWithTargetState3(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double;
auto cptContactForceWithTargetState4(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double;
}  // namespace sire::physics::contact::ps_vs_solver3
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
  m.def("pe2pq", [](std::vector<double>& pe, std::string& order) {
    std::vector<double> pq(7);
    aris::dynamic::s_pe2pq(pe.data(), pq.data(), order.c_str());
    return pq;
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

  // ── Contact force solver functions (v2 Clarabel, v3 fixed-point, v4 SOR, v5 ADMM) ──
  m.def(
      "cptContactForceWithTargetState2",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::ps_vs_solver3::
            cptContactForceWithTargetState2(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
                result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 30, py::arg("max_err") = 1e-8,
      "v2: Clarabel SOCP solver");

  m.def(
      "cptContactForceWithTargetState3",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::ps_vs_solver3::
            cptContactForceWithTargetState3(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
                result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 30, py::arg("max_err") = 1e-8,
      "v3: fixed-point iteration on friction direction");

  m.def(
      "cptContactForceWithTargetState4",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::ps_vs_solver3::
            cptContactForceWithTargetState4(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
                result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 30, py::arg("max_err") = 1e-8,
      "v4: per-contact SOR + stick/slip bisection (same interface as Raisim)");

  m.def(
      "cptContactForceWithTargetState5",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::ps_vs_solver_v5::
            cptContactForceWithTargetState5(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
                result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 30, py::arg("max_err") = 1e-8,
      "v5: ADMM-based DAE-NCP contact solver");

  m.def(
      "cptContactForceWithTargetState6",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::ps_vs_solver_v5::
            cptContactForceWithTargetState6(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
                result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 30, py::arg("max_err") = 1e-8,
      "v6: nested ADMM NCP solver with a frozen De Saxce shift");

  m.def(
      "cptContactForceSpectralAdmm",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::simple_admm::
            cptContactForceSpectralAdmm(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Spectral-ADMM: Carpentier et al. Algorithm 1 in impulse coordinates");

  // Backward-compatible Python name for existing scripts.
  m.def(
      "cptContactForceSimpleAdmm",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::simple_admm::
            cptContactForceSpectralAdmm(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Deprecated alias of cptContactForceSpectralAdmm");

  m.def(
      "cptContactForceWithTargetState7",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::ps_vs_solver_v5::
            cptContactForceWithTargetState7(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
                result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 30, py::arg("max_err") = 1e-8,
      "v7: Davis-Yin three-operator splitting");

  m.def("cptContactForceWithTargetState8",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err)
          -> std::pair<std::vector<double>, double> {
        std::vector<double> result(3 * n);
        double error = sire::physics::contact::ps_vs_solver_v5::
            cptContactForceWithTargetState8(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
                result, static_cast<sire::Size>(max_iters), max_err);
        return {result, error};
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 30, py::arg("max_err") = 1e-8,
      "v8: PDDY — DYS with exact F prox (LDLT)");

  m.def(
      "cptContactForceNewtonPipg",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err) {
        std::vector<double> result(3 * n);
        const double error = sire::physics::contact::newton_pipg::
            cptContactForceNewtonPipg(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return std::make_pair(result, error);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Hybrid Newton-PIPG solver with SOC face detection and rank compression");

  m.def(
      "cptContactForceExactCoulomb",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err) {
        std::vector<double> result(3 * n);
        const double error = sire::physics::contact::exact_coulomb::
            cptContactForceExactCoulomb(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return std::make_pair(result, error);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Target-shifted exact Coulomb solver: De Saxce FBF plus mixed SSN");

  m.def(
      "cptContactForceSingleLoopFbfSsn",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err) {
        std::vector<double> result(3 * n);
        const double error = sire::physics::contact::exact_coulomb::
            cptContactForceSingleLoopFbfSsn(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return std::make_pair(result, error);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Single-loop tracked-prox FBF with safeguarded mixed SSN");

  m.def(
      "cptContactForceExplicitFbfGatedSsn",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err) {
        std::vector<double> result(3 * n);
        const double error = sire::physics::contact::exact_coulomb::
            cptContactForceExplicitFbfGatedSsn(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return std::make_pair(result, error);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Explicit Tseng FBF with residual-gated mixed SSN");

  m.def(
      "cptContactForceModePredictorSsn",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err) {
        std::vector<double> result(3 * n);
        const double error = sire::physics::contact::exact_coulomb::
            cptContactForceModePredictorSsn(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return std::make_pair(result, error);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Block projected mode predictor, mode-gated mixed SSN and rare FBF rescue");

  m.def(
      "cptContactForceModePredictorSsnDetailed",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err,
         int maximum_predictor_sweeps, int stable_mode_sweeps,
         double newton_gate_residual, double forced_newton_residual,
         int repair_sweeps, int maximum_fbf_rescues) {
        using namespace sire::physics::contact::exact_coulomb;
        ModePredictorSsnOptions options;
        options.maximum_predictor_sweeps = static_cast<sire::Size>(
            std::max(1, maximum_predictor_sweeps));
        options.stable_mode_sweeps =
            static_cast<sire::Size>(std::max(1, stable_mode_sweeps));
        options.newton_gate_residual = newton_gate_residual;
        options.forced_newton_residual = forced_newton_residual;
        options.repair_sweeps =
            static_cast<sire::Size>(std::max(1, repair_sweeps));
        options.maximum_fbf_rescues =
            static_cast<sire::Size>(std::max(1, maximum_fbf_rescues));

        std::vector<double> result(3 * n);
        ModePredictorSsnStatistics statistics;
        const double error = cptContactForceModePredictorSsnWithOptions(
            static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
            result, static_cast<sire::Size>(max_iters), max_err, options,
            &statistics);

        py::dict stats;
        stats["nonlinear_iterations"] = statistics.nonlinear_iterations;
        stats["predictor_sweeps"] = statistics.predictor_sweeps;
        stats["predictor_backtracks"] = statistics.predictor_backtracks;
        stats["mode_changes"] = statistics.mode_changes;
        stats["uncertain_mode_sweeps"] =
            statistics.uncertain_mode_sweeps;
        stats["newton_attempts"] = statistics.newton_attempts;
        stats["newton_accepted"] = statistics.newton_accepted;
        stats["newton_rejected"] = statistics.newton_rejected;
        stats["fbf_rescues"] = statistics.fbf_rescues;
        stats["fbf_backtracks"] = statistics.fbf_backtracks;
        stats["full_hybrid_fallbacks"] =
            statistics.full_hybrid_fallbacks;
        stats["initial_residual"] = statistics.initial_residual;
        stats["residual_before_first_newton"] =
            statistics.residual_before_first_newton;
        stats["final_residual"] = statistics.final_residual;
        stats["first_newton_accepted"] =
            statistics.first_newton_accepted;
        stats["converged"] = statistics.converged;
        return py::make_tuple(result, error, stats);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      py::arg("maximum_predictor_sweeps") = 8,
      py::arg("stable_mode_sweeps") = 2,
      py::arg("newton_gate_residual") = 5e-2,
      py::arg("forced_newton_residual") = 1e-3,
      py::arg("repair_sweeps") = 2,
      py::arg("maximum_fbf_rescues") = 3,
      "Detailed block mode-predictor/SSN/FBF-rescue experiment");

  m.def(
      "cptContactForceExactCoulombFbf",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err) {
        std::vector<double> result(3 * n);
        const double error = sire::physics::contact::exact_coulomb::
            cptContactForceExactCoulombFbf(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return std::make_pair(result, error);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Target-shifted exact Coulomb ablation: FBF updates only");

  m.def(
      "cptContactForceExactCoulombSemismoothNewton",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, int max_iters, double max_err) {
        std::vector<double> result(3 * n);
        const double error = sire::physics::contact::exact_coulomb::
            cptContactForceExactCoulombSemismoothNewton(
                static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b,
                h, result, static_cast<sire::Size>(max_iters), max_err);
        return std::make_pair(result, error);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      "Target-shifted exact Coulomb ablation: damped mixed SSN only");

  m.def(
      "cptContactForceExactCoulombDetailed",
      [](int n, std::vector<double> fri_coef, std::vector<double> invM,
         std::vector<double> v0, std::vector<double> v_target,
         std::vector<double> b, double h, const std::string& mode,
         int max_iters, double max_err, int newton_start_iteration,
         double newton_switch_residual, bool collect_history) {
        using namespace sire::physics::contact::exact_coulomb;
        ExactCoulombOptions options;
        if (mode == "fbf") {
          options.mode = ExactCoulombMode::kFbfOnly;
        } else if (mode == "newton") {
          options.mode = ExactCoulombMode::kSemismoothNewtonOnly;
        } else if (mode == "hybrid") {
          options.mode = ExactCoulombMode::kHybrid;
        } else {
          throw std::invalid_argument(
              "mode must be one of: fbf, newton, hybrid");
        }
        options.newton_start_iteration = static_cast<sire::Size>(
            std::max(0, newton_start_iteration));
        options.newton_switch_residual = newton_switch_residual;
        options.collect_history = collect_history;

        std::vector<double> result(3 * n);
        ExactCoulombStatistics statistics;
        const double error = cptContactForceExactCoulombWithOptions(
            static_cast<sire::Size>(n), fri_coef, invM, v0, v_target, b, h,
            result, static_cast<sire::Size>(max_iters), max_err, options,
            &statistics);

        std::vector<int> step_history;
        step_history.reserve(statistics.step_history.size());
        for (const auto step : statistics.step_history)
          step_history.push_back(static_cast<int>(step));

        py::dict stats;
        stats["nonlinear_iterations"] = statistics.nonlinear_iterations;
        stats["newton_attempts"] = statistics.newton_attempts;
        stats["newton_accepted"] = statistics.newton_accepted;
        stats["newton_rejected"] = statistics.newton_rejected;
        stats["fbf_steps"] = statistics.fbf_steps;
        stats["fbf_backtracks"] = statistics.fbf_backtracks;
        stats["initial_residual"] = statistics.initial_residual;
        stats["final_residual"] = statistics.final_residual;
        stats["converged"] = statistics.converged;
        stats["terminated_on_newton_rejection"] =
            statistics.terminated_on_newton_rejection;
        stats["residual_history"] = statistics.residual_history;
        stats["step_history"] = step_history;
        return py::make_tuple(result, error, stats);
      },
      py::arg("n"), py::arg("fri_coef"), py::arg("invM"), py::arg("v0"),
      py::arg("v_target"), py::arg("b"), py::arg("h"), py::arg("mode"),
      py::arg("max_iters") = 200, py::arg("max_err") = 1e-8,
      py::arg("newton_start_iteration") = 2,
      py::arg("newton_switch_residual") =
          std::numeric_limits<double>::infinity(),
      py::arg("collect_history") = false,
      "Detailed FBF/Newton/hybrid exact-Coulomb experiment entry point");

  // ── PointerArray<T> bindings (must register before classes that expose them) ──
}
