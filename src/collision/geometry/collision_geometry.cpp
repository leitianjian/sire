#include "sire/collision/geometry/collision_geometry.hpp"

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include <hpp/fcl/bvh/BVH_model.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <array>
#include <memory>
#include <string>
#include <string_view>

namespace sire::collision::geometry {
struct CollisionGeometry::Imp {
  GeometryId id_{generate_new_id()};
  double prt_pm_[4][4]{{0}};
  bool is_dynamic_{false};
  int part_id_{0};
  aris::dynamic::Part* part_ptr_{nullptr};
  unique_ptr<fcl::CollisionObject> fcl_object_ptr_{nullptr};
};
auto CollisionGeometry::geometryId() -> GeometryId { return imp_->id_; }
auto CollisionGeometry::setGeometryId(GeometryId id) -> void {
  if (id > geometry::geometry_id_flag) {
    geometry_id_flag = id;
  }
  imp_->id_ = id;
}
auto CollisionGeometry::prtPm() const -> const aris::dynamic::double4x4& {
  return imp_->prt_pm_;
}
auto CollisionGeometry::isDynamic() -> bool { return imp_->is_dynamic_; }
auto CollisionGeometry::setDynamic(bool is_dynamic) -> void {
  imp_->is_dynamic_ = is_dynamic;
}
auto CollisionGeometry::partId() -> int { return imp_->part_id_; }
auto CollisionGeometry::part() -> aris::dynamic::Part* {
  return imp_->part_ptr_;
}
auto CollisionGeometry::setPart(aris::dynamic::Part* part, int part_id)
    -> void {
  imp_->part_id_ = part_id;
  imp_->part_ptr_ = part;
}
auto CollisionGeometry::getCollisionObject() -> fcl::CollisionObject* {
  return imp_->fcl_object_ptr_.get();
}
auto CollisionGeometry::resetCollisionObject(fcl::CollisionObject* object)
    -> void {
  imp_->fcl_object_ptr_.reset(object);
}
auto CollisionGeometry::updateLocation(const double* prt_pm) -> void {
  prt_pm = prt_pm ? prt_pm : default_pm;
  double res[16];
  aris::dynamic::s_pm_dot_pm(prt_pm, *imp_->prt_pm_, res);
  imp_->fcl_object_ptr_->setTransform(
      fcl::Transform3f(fcl::Matrix3f{{res[0], res[1], res[2]},
                                     {res[4], res[5], res[6]},
                                     {res[8], res[9], res[10]}},
                       fcl::Vec3f{res[3], res[7], res[11]}));
  imp_->fcl_object_ptr_->computeAABB();
}
auto CollisionGeometry::init() -> void {}
CollisionGeometry::CollisionGeometry(const double* prt_pm)
    : aris::dynamic::Geometry(), imp_(new Imp) {
  prt_pm = prt_pm ? prt_pm : default_pm;
  aris::dynamic::s_vc(16, prt_pm, *imp_->prt_pm_);
}
CollisionGeometry::~CollisionGeometry() = default;

struct SphereGeometry::Imp {
  double radius_{0};
};
auto SphereGeometry::radius() -> double { return imp_->radius_; }
auto SphereGeometry::setRadius(double radius) -> void {
  if (radius < 0) {
    throw std::logic_error(string("Sphere radius should be >= 0 (was ") +
                           to_string(radius) + string(")."));
  }
  imp_->radius_ = radius;
}
auto SphereGeometry::init() -> void {
  if (imp_->radius_ < 0) {
    throw std::logic_error(string("Sphere radius should be >= 0 (was ") +
                           to_string(imp_->radius_) + string(")."));
  }
  if (imp_->radius_ != 0) {
    fcl::Transform3f trans(
        fcl::Matrix3f{{prtPm()[0][0], prtPm()[0][1], prtPm()[0][2]},
                      {prtPm()[1][0], prtPm()[1][1], prtPm()[1][2]},
                      {prtPm()[2][0], prtPm()[2][1], prtPm()[2][2]}},
        fcl::Vec3f{prtPm()[0][3], prtPm()[1][3], prtPm()[2][3]});
    resetCollisionObject(new fcl::CollisionObject(
        make_shared<fcl::Sphere>(imp_->radius_), trans));
  }
}
SphereGeometry::SphereGeometry(double radius, const double* prt_pm)
    : CollisionGeometry(prt_pm), imp_(new Imp) {
  if (radius < 0) {
    throw std::logic_error(string("Sphere radius should be >= 0 (was ") +
                           to_string(radius) + string(")."));
  } else {
    imp_->radius_ = radius;
  }
}
SphereGeometry::~SphereGeometry() = default;

struct MeshGeometry::Imp {
  aris::dynamic::double3 scale_{1, 1, 1};
  string resource_path_;
};
auto MeshGeometry::resourcePath() const -> const std::string& {
  return imp_->resource_path_;
}
auto MeshGeometry::setResourcePath(std::string_view resource_path) -> void {
  if (!resource_path.empty()) {
    imp_->resource_path_ = resource_path;
  }
}
auto MeshGeometry::scale() const -> const double* { return imp_->scale_; }
auto MeshGeometry::setScale(const double* scale) -> void {
  if (scale) std::copy_n(scale, 3, imp_->scale_);
}
auto MeshGeometry::init() -> void {
  shared_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_model =
      make_shared<fcl::BVHModel<fcl::OBBRSS>>();
  fcl::loadPolyhedronFromResource(imp_->resource_path_,
                                  fcl::Vec3f(imp_->scale_), bvh_model);
  fcl::Transform3f trans(
      fcl::Matrix3f{{prtPm()[0][0], prtPm()[0][1], prtPm()[0][2]},
                    {prtPm()[1][0], prtPm()[1][1], prtPm()[1][2]},
                    {prtPm()[2][0], prtPm()[2][1], prtPm()[2][2]}},
      fcl::Vec3f{prtPm()[0][3], prtPm()[1][3], prtPm()[2][3]});
  resetCollisionObject(new fcl::CollisionObject(bvh_model, trans));
}
MeshGeometry::MeshGeometry(const string& resource_path, const double* prt_pm)
    : CollisionGeometry(prt_pm), imp_(new Imp) {
  if (!resource_path.empty()) {
    imp_->resource_path_ = resource_path;
  }
}
MeshGeometry::~MeshGeometry() = default;

ARIS_REGISTRATION {
  auto getCollisionGeometryPm = [](CollisionGeometry* g) -> aris::core::Matrix {
    double pm[16];
    aris::dynamic::s_vc(16, *g->prtPm(), pm);
    return aris::core::Matrix(4, 4, pm);
  };
  auto setCollisionGeometryPm = [](CollisionGeometry* g,
                                   aris::core::Matrix pm) -> void {
    std::copy_n(pm.data(), 16, const_cast<double*>(*g->prtPm()));
  };
  auto setPart = [](CollisionGeometry* geometry, int part_id) {
    geometry->setPart(&dynamic_cast<aris::dynamic::Model&>(
                           aris::server::ControlServer::instance().model())
                           .partPool()
                           .at(part_id),
                      part_id);
  };
  auto getPart = [](CollisionGeometry* geometry) { return geometry->partId(); };
  aris::core::class_<CollisionGeometry>("CollisionGeometry")
      .inherit<aris::dynamic::Geometry>()
      .prop("id", &CollisionGeometry::setGeometryId,
            &CollisionGeometry::geometryId)
      .prop("pm", &setCollisionGeometryPm, &getCollisionGeometryPm)
      .prop("part_id", &setPart, &getPart)
      .prop("is_dynamic", &CollisionGeometry::setDynamic,
            &CollisionGeometry::isDynamic);

  aris::core::class_<SphereGeometry>("SphereGeometry")
      .inherit<CollisionGeometry>()
      .prop("radius", &SphereGeometry::setRadius, &SphereGeometry::radius);

  auto getMeshGeometryScale = [](MeshGeometry* geo) {
    return aris::core::Matrix(1, 3, geo->scale());
  };
  auto setMeshGeometryScale = [](MeshGeometry* geo, aris::core::Matrix scale) {
    geo->setScale(scale.data());
  };
  aris::core::class_<MeshGeometry>("MeshGeometry")
      .inherit<CollisionGeometry>()
      .prop("resource_path", &MeshGeometry::setResourcePath,
            &MeshGeometry::resourcePath)
      .prop("scale", &setMeshGeometryScale, &getMeshGeometryScale);
}
}  // namespace sire::collision::geometry