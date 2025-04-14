#include <pybind11/pybind11.h>
#include <sire/core/constants.hpp>
#include <aris.hpp>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <codecvt>
#include <locale>

namespace py = pybind11;

PYBIND11_MODULE(sire_python, m) {
    // 常量绑定
    m.attr("kPosQuatSize") = sire::kPosQuatSize;

    m.def("toXmlString", [](const aris::dynamic::Model& self) {
        return aris::core::toXmlString(self);
    });
    m.def("fromXmlFile", [](aris::server::ControlServer& self, const std::string& xml) {
        std::cout << "fromXmlFile: " << xml << std::endl;
        try {
            aris::core::fromXmlFile(self, xml);
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        std::cout << "fromXmlFile end" << std::endl;
    });
    m.def("fromXmlString", [](aris::server::ControlServer& self, const std::string& xml) {
        aris::core::fromXmlString(self, xml);
    });


    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) std::rethrow_exception(p);
        } catch (const std::exception &e) {
            PyErr_SetString(PyExc_RuntimeError, e.what());
        } catch (...) {
            PyErr_SetString(PyExc_RuntimeError, "Unknown C++ exception");
        }
    });

    py::class_<aris::server::ControlServer, std::unique_ptr<aris::server::ControlServer, py::nodelete>>(m, "ControlServer")
        .def_static("instance", &aris::server::ControlServer::instance, py::return_value_policy::reference)
        .def("init", &aris::server::ControlServer::init)
        .def("start", &aris::server::ControlServer::start)
        .def("stop", &aris::server::ControlServer::stop)
        .def("open", &aris::server::ControlServer::open)
        .def("close", &aris::server::ControlServer::close)
        .def("runCmdLine", &aris::server::ControlServer::runCmdLine);


    
    py::class_<aris::dynamic::Model>(m, "Model")
        .def(py::init<>()) // 默认构造函数
        .def("init", &aris::dynamic::Model::init) // 初始化模型
        .def("addPartByPe", [](aris::dynamic::Model &self, const std::vector<double> &pe, const char *eul_type, const std::vector<double> &prt_im) -> aris::dynamic::Part& {
            if (pe.empty()) {
                throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            return self.addPartByPe(pe.data(), eul_type, prt_im.data());
        }, py::return_value_policy::reference_internal)
        .def("addRevoluteJoint", [](aris::dynamic::Model &self, aris::dynamic::Part &part1, aris::dynamic::Part &part2, const std::vector<double> &position, const std::vector<double> &axis) -> aris::dynamic::RevoluteJoint& {
            if (position.size() != 3 || axis.size() != 3) {
                throw std::runtime_error("Position and axis must be 3-element arrays!");
            }
            return self.addRevoluteJoint(part1, part2, position.data(), axis.data());
        }, py::return_value_policy::reference_internal)
        
        .def("addPrismaticJoint", [](aris::dynamic::Model &self, aris::dynamic::Part &part1, aris::dynamic::Part &part2, const std::vector<double> &position, const std::vector<double> &axis) -> aris::dynamic::PrismaticJoint& {
            if (position.size() != 3 || axis.size() != 3) {
                throw std::runtime_error("Position and axis must be 3-element arrays!");
            }
            return self.addPrismaticJoint(part1, part2, position.data(), axis.data());
        }, py::return_value_policy::reference_internal)
        .def("addMotion", py::overload_cast<>(&aris::dynamic::Model::addMotion)) // 添加驱动
        .def("addMotion", py::overload_cast<aris::dynamic::Joint &>(&aris::dynamic::Model::addMotion)) // 添加驱动
        // .def("addGeneralMotionByPe", &aris::dynamic::Model::addGeneralMotionByPe) // 添加末端
        .def("addGeneralMotionByPe", [](aris::dynamic::Model &self, aris::dynamic::Part &part, aris::dynamic::Part &ground, const std::vector<double> &pe, const char *eul_type) -> aris::dynamic::GeneralMotion& {
            if (pe.empty()) {
                throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            return self.addGeneralMotionByPe(part, ground, pe.data(), eul_type);
        }, py::return_value_policy::reference_internal)
        .def("init", &aris::dynamic::Model::init) // 初始化模型
        .def("settime", &aris::dynamic::Model::setTime) // 设置时间
        .def("solverPool", static_cast<aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element>& (aris::dynamic::Model::*)()>(&aris::dynamic::Model::solverPool), py::return_value_policy::reference_internal)        
        .def("solverPool", static_cast<const aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element>& (aris::dynamic::Model::*)() const>(&aris::dynamic::Model::solverPool), py::return_value_policy::reference_internal)
        .def("ground", py::overload_cast<>(&aris::dynamic::Model::ground, py::const_), py::return_value_policy::reference_internal)
        .def("ground", py::overload_cast<>(&aris::dynamic::Model::ground), py::return_value_policy::reference_internal)
        .def("simulatorPool", static_cast<aris::core::PointerArray<aris::dynamic::Simulator, aris::dynamic::Element>& (aris::dynamic::Model::*)()>(&aris::dynamic::Model::simulatorPool), py::return_value_policy::reference_internal); // 未重载还需修改

    py::class_<aris::dynamic::Part>(m, "Part");
    py::class_<aris::dynamic::Joint>(m, "Joint");
    py::class_<aris::dynamic::Coordinate>(m, "Coordinate");
    py::class_<aris::dynamic::Constraint>(m, "Constraint");
    py::class_<aris::dynamic::MotionBase, aris::dynamic::Constraint>(m, "MotionBase");
    py::class_<aris::dynamic::GeneralMotion, aris::dynamic::MotionBase>(m, "GeneralMotion")
        .def(py::init<>()) // 默认构造函数
        .def("setMpe", [](aris::dynamic::GeneralMotion &self, const std::vector<double> &pe, const char *eul_type) {
            if (pe.empty()) {
                throw std::runtime_error("Input array 'pe' cannot be empty!");
            }
            self.setMpe(pe.data(), eul_type);
        })
        .def("getMpe", [](aris::dynamic::GeneralMotion &self, const char *eul_type) -> std::vector<double> {
            double pe[6];
            self.getMpe(pe, eul_type);
            return std::vector<double>(pe, pe + 6);
        })
        .def("setMva", [](aris::dynamic::GeneralMotion &self, const std::vector<double> &mva) {
            if (mva.size() != 6) {
                throw std::runtime_error("Input array 'mva' must be of size 6!");
            }
            self.setMva(mva.data());
        })
        .def("getMaa", [](aris::dynamic::GeneralMotion &self) -> std::vector<double> {
            double maa[6];
            self.getMaa(maa);
            return std::vector<double>(maa, maa + 6);
        });
    py::class_<aris::dynamic::Element>(m, "Element");
    py::class_<aris::dynamic::Solver, aris::dynamic::Element>(m, "Solver");
    py::class_<aris::dynamic::UniversalSolver, aris::dynamic::Solver>(m, "UniversalSolver")
        .def(py::init<>());
    py::class_<aris::dynamic::InverseDynamicSolver, aris::dynamic::UniversalSolver>(m, "InverseDynamicSolver")
        .def(py::init<>())
        .def("dynAccAndFce", &aris::dynamic::InverseDynamicSolver::dynAccAndFce);
    py::class_<aris::dynamic::InverseKinematicSolver, aris::dynamic::UniversalSolver>(m, "InverseKinematicSolver")
        .def(py::init<>())
        .def("kinPos", [](aris::dynamic::InverseKinematicSolver& self) {          
            auto result = self.kinPos();
            return result;
        })
        .def("kinVel", &aris::dynamic::InverseKinematicSolver::kinVel)      
        .def("ci", &aris::dynamic::InverseKinematicSolver::ci);
    py::class_<aris::dynamic::AdamsSimulator>(m, "AdamsSimulator")
        .def(py::init<>());
    py::class_<aris::dynamic::Motion, aris::dynamic::MotionBase>(m, "Motion")
        .def("mp", &aris::dynamic::Motion::mp)
        .def("mf", &aris::dynamic::Motion::mf)
        .def("mv", &aris::dynamic::Motion::mv)
        .def("setMa", &aris::dynamic::Motion::setMa);
    py::class_<aris::dynamic::Simulator, aris::dynamic::Element>(m, "Simulator");
    py::class_<aris::dynamic::RevoluteJoint, aris::dynamic::Joint>(m, "RevoluteJoint");
    py::class_<aris::dynamic::PrismaticJoint, aris::dynamic::Joint>(m, "PrismaticJoint");
    py::class_<aris::core::PointerArray<aris::dynamic::Geometry, aris::dynamic::Element>>(m, "PointerArrayGeometry");
    py::class_<aris::core::PointerArray<aris::dynamic::Simulator, aris::dynamic::Element>>(m, "PointerArraySimulator")
        .def(py::init<>())
        .def("add_adams", [](aris::core::PointerArray<aris::dynamic::Simulator, aris::dynamic::Element> &self) -> aris::dynamic::AdamsSimulator& {
            return self.add<aris::dynamic::AdamsSimulator>();
        }, py::return_value_policy::reference_internal)
        .def("add_solver_simulator", [](aris::core::PointerArray<aris::dynamic::Simulator, aris::dynamic::Element> &self) -> aris::dynamic::SolverSimulator& {
            return self.add<aris::dynamic::SolverSimulator>();
        }, py::return_value_policy::reference_internal)
        ;
    py::class_<aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element>>(m, "PointerArraySolver")
        .def(py::init<>())
        .def("__getitem__", [](aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element> &self, size_t index) -> aris::dynamic::Solver& {
            return self[index];  // 假设 self 支持 operator[]
        }, py::return_value_policy::reference_internal)
        // .def("add_ik_solver", [](aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element> &self) -> aris::dynamic::InverseKinematicSolver& {
        //     return self.add<aris::dynamic::InverseKinematicSolver>();
        // }, py::return_value_policy::reference_internal)
        .def("add_ik", [](aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element> &self) -> aris::dynamic::InverseKinematicSolver& {
            // 把 Python 对象转换为 C++ 指针（Solver* 或派生类*）
            return self.add<aris::dynamic::InverseKinematicSolver>();         
        }, py::return_value_policy::reference_internal)
        .def("add_fd", [](aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element> &self) -> aris::dynamic::ForwardDynamicSolver& {
            return self.add<aris::dynamic::ForwardDynamicSolver>();
        }, py::return_value_policy::reference_internal)
        .def("add_id", [](aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element> &self) -> aris::dynamic::InverseDynamicSolver& {
            return self.add<aris::dynamic::InverseDynamicSolver>();
        }, py::return_value_policy::reference_internal)
        .def("add_fk", [](aris::core::PointerArray<aris::dynamic::Solver, aris::dynamic::Element> &self) -> aris::dynamic::ForwardKinematicSolver& {
            return self.add<aris::dynamic::ForwardKinematicSolver>();
        }, py::return_value_policy::reference_internal)
        ;
}