#ifndef SIRE_PHYSICS_UTILS_HPP_
#define SIRE_PHYSICS_UTILS_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/physics/common/penetration_as_point_pair.hpp"
namespace sire::physics {
using aris::dynamic::Model;
using PartPool =
    aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>;
using aris::dynamic::Part;
using MarkerPool =
    aris::core::PointerArray<aris::dynamic::Marker, aris::dynamic::Element>;
using aris::dynamic::Marker;
using GeometryPool =
    aris::core::PointerArray<aris::dynamic::Geometry, aris::dynamic::Element>;
using aris::dynamic::Geometry;

auto cptContactFrame(std::vector<common::PenetrationAsPointPair>& pairs,
                     std::vector<std::array<double, 16>>& frames) -> void;

auto compareAndCopy(const Geometry* src, Geometry* dest) -> void;
auto compareAndCopy(const GeometryPool* src, const GeometryPool* dest) -> void;
auto compareAndCopy(const Marker* src, Marker* dest) -> void;
auto compareAndCopy(const MarkerPool* src, const MarkerPool* dest) -> void;
auto compareAndCopy(const Part* src, Part* dest) -> void;
auto compareAndCopy(const PartPool* src, const PartPool* dest) -> void;
auto compareAndCopy(const Model* src, Model* dest) -> void;
};  // namespace sire::physics
#endif