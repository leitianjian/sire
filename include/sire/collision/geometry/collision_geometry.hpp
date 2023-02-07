#ifndef COLLISION_GEOMETRY_H_
#define COLLISION_GEOMETRY_H_

#include <sire_lib_export.h>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>
#include <hpp/fcl/collision_object.h>
#include <atomic>
#include <string>

namespace sire::collision {
namespace geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;
using GeometryId = int64_t;
static std::atomic<GeometryId> geometry_id_flag = 0;
static auto generate_new_id() -> GeometryId { return ++geometry_id_flag; }

static const double default_pm[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 1};

class SIRE_API CollisionGeometry : public aris::dynamic::Geometry {
 public:
  auto geometryId() -> GeometryId;
  auto setGeometryId(GeometryId id) -> void;
  auto prtPm() const -> const aris::dynamic::double4x4&;
  auto isDynamic() -> bool;
  auto setDynamic(bool is_dynamic) -> void;
  auto partId() -> int;
  auto part() -> aris::dynamic::Part*;
  auto setPart(aris::dynamic::Part* part, int part_id) -> void;
  auto getCollisionObject() -> fcl::CollisionObject*;
  auto resetCollisionObject(fcl::CollisionObject* object) -> void;
  auto virtual updateLocation(const double* prt_pm) -> void;
  auto virtual init() -> void;
  explicit CollisionGeometry(const double* prt_pm = nullptr);
  virtual ~CollisionGeometry();
  CollisionGeometry(const CollisionGeometry& other) = delete;
  CollisionGeometry(CollisionGeometry&& other) =
      delete;  // 不明白为啥这里不能是default，明明下面的都没啥问题
  CollisionGeometry& operator=(const CollisionGeometry& other) = delete;
  CollisionGeometry& operator=(CollisionGeometry&& other) = delete;

 private:
  struct Imp;
  unique_ptr<Imp> imp_;
};
class SIRE_API SphereGeometry : public CollisionGeometry {
 public:
  auto radius() -> double;
  auto setRadius(double radius) -> void;
  auto init() -> void override;
  explicit SphereGeometry(double radius = 0, const double* prt_pm = nullptr);
  virtual ~SphereGeometry();
  SphereGeometry(const SphereGeometry& other) = delete;
  SphereGeometry(SphereGeometry&& other) = delete;
  SphereGeometry& operator=(const SphereGeometry& other) = delete;
  SphereGeometry& operator=(SphereGeometry&& other) = delete;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

class SIRE_API MeshGeometry : public CollisionGeometry {
 public:
  auto resourcePath() const -> const string&;
  auto setResourcePath(string_view resource_path) -> void;
  auto scale() const -> const double*;
  auto setScale(const double* scale) -> void;
  auto init() -> void override;
  explicit MeshGeometry(const string& resource_path = "",
                        const double* prt_pm = nullptr);
  virtual ~MeshGeometry();
  MeshGeometry(const MeshGeometry& other) = delete;
  MeshGeometry(MeshGeometry&& other) = delete;
  MeshGeometry& operator=(const MeshGeometry& other) = delete;
  MeshGeometry& operator=(MeshGeometry&& other) = delete;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace geometry
}  // namespace sire::collision
#endif