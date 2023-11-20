#include "sire/physics/physics_engine.hpp"

#include <aris/core/reflection.hpp>
#include <aris/core/serialization.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_interaction.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/force_screw.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"
#include "sire/physics/contact/stiffness_damping_contact_solver.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/geometry/sphere_collision_geometry.hpp"

namespace sire::physics {
using namespace std;
struct PhysicsEngine::Imp {
  // Config options which should determined before init() called
  // Collision detection method enabled options
  bool collision_detection_flag_{false};
  // Contact solver enabled options
  bool contact_solver_flag_{false};

  // Self management data.
  unique_ptr<aris::core::PointerArray<geometry::CollidableGeometry,
                                      aris::dynamic::Geometry>>
      geometry_pool_;
  unordered_map<GeometryId, geometry::CollidableGeometry*>
      dynamic_objects_map_{};
  unordered_map<GeometryId, geometry::CollidableGeometry*>
      anchored_objects_map_{};
  unique_ptr<collision::CollisionDetection> collision_detection_;
  unique_ptr<contact::ContactSolver> contact_solver_;
  unique_ptr<collision::CollisionFilter> collision_filter_;

  std::vector<common::PenetrationAsPointPair> penetration_pairs_;
  std::vector<common::PointPairContactInfo> contact_info_;

  // Useful pointer and information from outer module
  aris::dynamic::Model* model_ptr_{nullptr};
  aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>*
      part_pool_ptr_{nullptr};
  sire::Size part_size_{0};

  Imp()
      : collision_detection_flag_(false),
        contact_solver_flag_(false),
        geometry_pool_(
            std::make_unique<aris::core::PointerArray<
                geometry::CollidableGeometry, aris::dynamic::Geometry>>()),
        collision_detection_(std::make_unique<collision::CollisionDetection>()),
        contact_solver_(
            std::make_unique<contact::StiffnessDampingContactSolver>()),
        collision_filter_(std::make_unique<collision::CollisionFilter>()),
        model_ptr_(nullptr),
        part_pool_ptr_(nullptr),
        part_size_(0) {}
};
PhysicsEngine::PhysicsEngine() : imp_(new Imp) {}
PhysicsEngine::~PhysicsEngine() { sire::geometry::reset_geometry_id(); };
// TODO(leitianjian): 精简PhysicsEngine的资源管理
//   PhysicsEngine中管理的资源是两个引擎都需要的资源，如果只是自己需要的没必要放在外面
auto PhysicsEngine::doInit() -> void {
  imp_->part_pool_ptr_ = &imp_->model_ptr_->partPool();
  imp_->part_size_ = imp_->part_pool_ptr_->size();
  if (collisionDetectionFlag()) {
    // 根据当前的PhysicalEngine的GeometryPool初始化碰撞检测引擎，添加到里面的Tree
    imp_->collision_detection_->init(this);
    // Filter如果可以只放在 detectionEngine 中就放过去，可以不放这的
    // 根据GeometryPool()添加进CollisionFilter中
    for (auto& geometry : *imp_->geometry_pool_) {
      if (geometry.isDynamic()) {
        addDynamicGeometry(geometry);
        imp_->collision_filter_->addGeometry(geometry);
      } else {
        addAnchoredGeometry(geometry);
        imp_->collision_filter_->addGeometry(geometry);
      }
    }
    // 加载CollisionFilter的配置
    imp_->collision_filter_->loadMatConfig();
  }

  if (contactSolverFlag()) {
    imp_->contact_solver_->init(this);
  }
}
auto PhysicsEngine::init() -> void {
  // 初始化Model与ControlServer相关的指针
  imp_->model_ptr_ = dynamic_cast<aris::dynamic::Model*>(
      &aris::server::ControlServer::instance().model());
  doInit();
}
auto PhysicsEngine::initByModel(aris::dynamic::Model* m) -> void {
  SIRE_DEMAND(m != nullptr);
  imp_->model_ptr_ = m;
  doInit();
}
auto PhysicsEngine::collisionDetectionFlag() const -> bool {
  return imp_->collision_detection_flag_;
}
auto PhysicsEngine::setCollisionDetectionFlag(bool flag) -> void {
  imp_->collision_detection_flag_ = flag;
}
auto PhysicsEngine::contactSolverFlag() const -> bool {
  return imp_->contact_solver_flag_;
}
auto PhysicsEngine::setContactSolverFlag(bool flag) -> void {
  imp_->contact_solver_flag_ = flag;
}
auto PhysicsEngine::resetCollisionFilter(collision::CollisionFilter* filter)
    -> void {
  imp_->collision_filter_.reset(filter);
}
auto PhysicsEngine::collisionFilter() -> collision::CollisionFilter& {
  return *imp_->collision_filter_;
}
inline auto PhysicsEngine::resetGeometryPool(
    aris::core::PointerArray<geometry::CollidableGeometry,
                             aris::dynamic::Geometry>* pool) -> void {
  imp_->geometry_pool_.reset(pool);
}
inline auto PhysicsEngine::geometryPool() noexcept
    -> aris::core::PointerArray<geometry::CollidableGeometry,
                                aris::dynamic::Geometry>& {
  return *imp_->geometry_pool_;
}
auto PhysicsEngine::queryGeometryPoolById(const GeometryId& id) const
    -> geometry::CollidableGeometry* {
  if (auto& it = imp_->dynamic_objects_map_.find(id);
      it != imp_->dynamic_objects_map_.end()) {
    return it->second;
  }
  if (auto& it = imp_->anchored_objects_map_.find(id);
      it != imp_->anchored_objects_map_.end()) {
    return it->second;
  }
  return nullptr;
}

auto PhysicsEngine::dynamicObjectsMap()
    -> std::unordered_map<GeometryId, geometry::CollidableGeometry*>& {
  return imp_->dynamic_objects_map_;
}
auto PhysicsEngine::anchoredObjectsMap()
    -> std::unordered_map<GeometryId, geometry::CollidableGeometry*>& {
  return imp_->anchored_objects_map_;
}
auto PhysicsEngine::addSphereGeometry(double radius, int part_id,
                                      const double* prt_pm, bool is_dynamic)
    -> bool {
  imp_->geometry_pool_->add<geometry::SphereCollisionGeometry>(
      radius, part_id, prt_pm, is_dynamic);
  return true;
}
auto PhysicsEngine::addDynamicGeometry(
    geometry::CollidableGeometry& dynamic_geometry) -> bool {
  imp_->collision_detection_->addDynamicGeometry2FCL(dynamic_geometry);
  imp_->dynamic_objects_map_[dynamic_geometry.geometryId()] = &dynamic_geometry;
  return true;
}
auto PhysicsEngine::addAnchoredGeometry(
    geometry::CollidableGeometry& anchored_geometry) -> bool {
  imp_->collision_detection_->addAnchoredGeometry2FCL(anchored_geometry);
  imp_->anchored_objects_map_[anchored_geometry.geometryId()] =
      &anchored_geometry;
  return true;
}
// TODO(leitianjian): 需要实现，但是优先级较低
auto PhysicsEngine::removeGeometry() -> bool { return false; }
auto PhysicsEngine::clearDynamicGeometries() -> bool {
  imp_->collision_detection_->clearDynamicGeometries();
  imp_->dynamic_objects_map_.clear();
  for (auto iter = imp_->geometry_pool_->begin();
       iter != imp_->geometry_pool_->end();) {
    if (iter->isDynamic()) {
      iter = imp_->geometry_pool_->erase(iter);
    } else {
      ++iter;
    }
  }
  return true;
}
auto PhysicsEngine::clearAnchoredGeometries() -> bool {
  imp_->collision_detection_->clearAnchoredGeometries();
  imp_->anchored_objects_map_.clear();
  for (auto iter = imp_->geometry_pool_->begin();
       iter != imp_->geometry_pool_->end();) {
    if (!iter->isDynamic()) {
      iter = imp_->geometry_pool_->erase(iter);
    } else {
      ++iter;
    }
  }
  return true;
}
auto PhysicsEngine::clearGeometries() -> bool {
  return clearAnchoredGeometries() && clearDynamicGeometries();
}
auto PhysicsEngine::resetCollisionDetection(
    collision::CollisionDetection* collision_detection_in) -> void {
  imp_->collision_detection_.reset(collision_detection_in);
}
auto PhysicsEngine::collisionDetection() const
    -> const collision::CollisionDetection& {
  return *imp_->collision_detection_;
}
auto PhysicsEngine::cptContactTime(
    const common::PenetrationAsPointPair& penetration) -> double {
  return penetration.depth / cptProximityVelocity(penetration);
}
// vn 为V_b - V_a在接触法线上的投影
// - vn > 0: 两个物体正在靠近，
// vn < 0: 两个物体正在远离
auto PhysicsEngine::cptProximityVelocity(
    const common::PenetrationAsPointPair& penetration) -> double {
  double vs_A[6], vs_B[6], vel_A[3], vel_B[3];
  auto* geometry_A = queryGeometryPoolById(penetration.id_A);
  auto* geometry_B = queryGeometryPoolById(penetration.id_B);
  SIRE_ASSERT(geometry_A != nullptr && geometry_B != nullptr);
  imp_->model_ptr_->partPool().at(geometry_A->partId()).getVs(vs_A);
  imp_->model_ptr_->partPool().at(geometry_B->partId()).getVs(vs_B);
  aris::dynamic::s_vs2vp(vs_A, penetration.p_WC.data(), vel_A);
  aris::dynamic::s_vs2vp(vs_B, penetration.p_WC.data(), vel_B);
  aris::dynamic::s_vs(3, vel_A, vel_B);
  return -aris::dynamic::s_vv(3, vel_B, penetration.nhat_AB_W.data());
}
auto PhysicsEngine::cptPointPairPenetration(
    std::vector<common::PenetrationAsPointPair>& pairs) -> void {
  imp_->collision_detection_->computePointPairPenetration(pairs);
}

auto PhysicsEngine::resetContactSolver(
    contact::ContactSolver* contact_solver_in) -> void {
  imp_->contact_solver_.reset(contact_solver_in);
}
auto PhysicsEngine::contactSolver() const -> const contact::ContactSolver& {
  return *imp_->contact_solver_;
}
inline auto PhysicsEngine::numDynamicGeometries() -> sire::Size {
  return imp_->collision_detection_->numDynamicGeometries();
}
auto PhysicsEngine::handleContact() -> void {
  std::vector<common::PenetrationAsPointPair> pairs;
  this->cptPointPairPenetration(pairs);
  this->resetPartContactForce();
  std::vector<common::PointPairContactInfo> contact_info;
  this->cptContactInfo(pairs, contact_info);
  this->cptGlbForceByContactInfo(contact_info);
}
auto PhysicsEngine::updateGeometryLocationFromModel() -> void {
  if (imp_->collision_detection_flag_) {
    imp_->collision_detection_->updateLocation(imp_->model_ptr_);
  }
}
auto PhysicsEngine::hasCollision() -> bool {
  if (imp_->collision_detection_flag_) {
    return imp_->collision_detection_->hasCollisions();
  }
}
auto PhysicsEngine::computePointPairPenetration()
    -> std::vector<common::PenetrationAsPointPair> {
  std::vector<common::PenetrationAsPointPair> pairs;
  if (imp_->collision_detection_flag_) {
    imp_->collision_detection_->computePointPairPenetration(pairs);
  }
  return pairs;
}
auto PhysicsEngine::cptContactInfo(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<common::PointPairContactInfo>& contact_info) -> bool {
  const int num_contacts = penetration_pairs.size();
  // 使用engine_ptr和当前Model的状态结合Penetration_pair，计算接触信息
  contact::ContactSolverResult solver_result;
  // 每个碰撞点构建的坐标系保存的位置，使用pm保存
  std::vector<std::array<double, 16>> T_C_vec;
  T_C_vec.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    const auto& pair = penetration_pairs[i];
    // 使用 nhat_AB_w 构建当前碰撞点的 T 矩阵
    aris::dynamic::s_sov_axes2pm(pair.p_WC.data(), pair.nhat_AB_W.data(),
                                 pair.nhat_AB_W.data(), T_C_vec.at(i).data(),
                                 "zx");
  }
  solver_result.resize(imp_->part_size_ * 6, penetration_pairs.size());
  imp_->contact_solver_->cptContactSolverResult(
      imp_->model_ptr_, penetration_pairs, T_C_vec, solver_result);
  // 需要计算接触点的运动学，即接触点的坐标系求解的f v，到世界坐标系
  std::vector<double>& fn = solver_result.fn;
  std::vector<double>& ft = solver_result.ft;
  std::vector<double>& vn = solver_result.vn;
  std::vector<double>& vt = solver_result.vt;

  SIRE_DEMAND(fn.size() >= num_contacts);
  SIRE_DEMAND(ft.size() >= 2 * num_contacts);
  SIRE_DEMAND(vn.size() >= num_contacts);
  SIRE_DEMAND(vt.size() >= 2 * num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    // std::cout << "fn=" << fn[i] << " ";
    const auto& pair = penetration_pairs[i];
    const geometry::CollidableGeometry* geometry_A_ptr =
        this->queryGeometryPoolById(pair.id_A);
    const geometry::CollidableGeometry* geometry_B_ptr =
        this->queryGeometryPoolById(pair.id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    const auto partId_A = geometry_A_ptr->partId();
    const auto partId_B = geometry_B_ptr->partId();
    // f of contact based on contact frame;
    double f_Bc_C[3]{ft[2 * i], ft[2 * i + 1], fn[i]};
    // 将接触坐标系下的力转换到世界坐标系
    double fs[6];
    core::screw::s_fpm2fs(f_Bc_C, T_C_vec.at(i).data(), fs);
    double pe_C[6];
    aris::dynamic::s_pm2pe(T_C_vec.at(i).data(), pe_C);
    double slip_speed = aris::dynamic::s_norm(2, vt.data() + 2 * i);
    double separation_speed = vn[i];

    contact_info.push_back(
        {partId_A, partId_B, fs, pe_C, separation_speed, slip_speed, pair});
  }
  return true;
}
auto PhysicsEngine::initPartContactForce2Model() -> void {
  // 初始化并使Model的ForcePool符合条件
  // 0. 设置电机力的Force，主要使用在动力学求解时
  // 1. 设置关节接触力产生的力旋量的GeneralForce，都当作ground产生的力，
  //    即使是两个part项目接触的力，要不然forcePool不好搞
  using aris::dynamic::GeneralForce;
  using aris::dynamic::SingleComponentForce;
  const sire::Size motion_size = imp_->model_ptr_->motionPool().size();
  const sire::Size part_size = imp_->part_size_;
  const sire::Size force_size = motion_size + part_size - 1;  // 减去ground的力
  auto& force_pool = imp_->model_ptr_->forcePool();
  auto& motion_pool = imp_->model_ptr_->motionPool();
  auto& part_pool = imp_->model_ptr_->partPool();
  force_pool.clear();
  for (int i = 0; i < motion_size; ++i) {
    auto& force = force_pool.add<SingleComponentForce>(
        std::string("mf_" + std::to_string(i)), motion_pool.at(i).makI(),
        motion_pool.at(i).makJ(), 5);
    force.setFce(0);
  }
  for (int i = 0; i < part_size; ++i) {
    auto& force = force_pool.add<GeneralForce>(
        std::string("cf_" + std::to_string(i)),
        &part_pool.at(i).markerPool().at(0),
        &part_pool.at(imp_->model_ptr_->ground().id()).markerPool().at(1));
    force.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
  }
}
auto PhysicsEngine::resetPartContactForce() -> void {
  using aris::dynamic::GeneralForce;
  using aris::dynamic::SingleComponentForce;
  const sire::Size motion_size = imp_->model_ptr_->motionPool().size();
  const sire::Size part_size = imp_->part_size_;
  auto& force_pool = imp_->model_ptr_->forcePool();
  auto& motion_pool = imp_->model_ptr_->motionPool();
  auto& part_pool = imp_->model_ptr_->partPool();
  for (int i = 0; i < motion_size; ++i) {
    dynamic_cast<SingleComponentForce&>(force_pool.at(i)).setFce(0);
  }
  for (int i = motion_size; i < part_size + motion_size; ++i) {
    dynamic_cast<GeneralForce&>(force_pool.at(i))
        .setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
  }
}
auto PhysicsEngine::cptGlbForceByContactInfo(
    const std::vector<common::PointPairContactInfo>& contact_info) -> bool {
  const int num_contacts = contact_info.size();
  const int contact_force_offset = imp_->model_ptr_->motionPool().size();
  auto& force_pool = imp_->model_ptr_->forcePool();
  for (int i = 0; i < num_contacts; ++i) {
    const common::PointPairContactInfo& info = contact_info.at(i);
    // 计算每个杆件的力旋量的和并设置给model
    // 对于partId_A，设置contact_force，直接用id + offset就是对应的force
    aris::dynamic::GeneralForce& force_A =
        dynamic_cast<aris::dynamic::GeneralForce&>(
            force_pool.at(info.partId_A() + contact_force_offset));
    aris::dynamic::GeneralForce& force_B =
        dynamic_cast<aris::dynamic::GeneralForce&>(
            force_pool.at(info.partId_B() + contact_force_offset));
    double fs_A[6]{0};
    aris::dynamic::s_vc(6, force_A.fce(), fs_A);
    aris::dynamic::s_vs(6, info.contact_force(), fs_A);
    force_A.setFce(fs_A);
    double fs_B[6]{0};
    aris::dynamic::s_vc(6, force_B.fce(), fs_B);
    aris::dynamic::s_va(6, info.contact_force(), fs_B);
    force_B.setFce(fs_B);
  }
  return true;
}
ARIS_REGISTRATION {
  aris::core::class_<aris::core::PointerArray<geometry::CollidableGeometry,
                                              aris::dynamic::Geometry>>(
      "GeometryPoolObject")
      .asRefArray();
  typedef sire::physics::collision::CollisionDetection& (
      PhysicsEngine::*CollisionDetectionFunc)();
  typedef sire::physics::contact::ContactSolver& (
      PhysicsEngine::*ContactSolverFunc)();
  typedef aris::core::PointerArray<physics::geometry::CollidableGeometry,
                                   aris::dynamic::Geometry>& (
      PhysicsEngine::*GeometryPoolFunc)();
  typedef sire::physics::collision::CollisionFilter& (
      PhysicsEngine::*CollisionFilterPoolFunc)();
  aris::core::class_<PhysicsEngine>("PhysicsEngine")
      .prop("enable_collision_detection",
            &PhysicsEngine::setCollisionDetectionFlag,
            &PhysicsEngine::collisionDetectionFlag)
      .prop("enable_contact_solver", &PhysicsEngine::setContactSolverFlag,
            &PhysicsEngine::contactSolverFlag)
      .prop("collision_detection", &PhysicsEngine::resetCollisionDetection,
            CollisionDetectionFunc(&PhysicsEngine::collisionDetection))
      .prop("contact_solver", &PhysicsEngine::resetContactSolver,
            ContactSolverFunc(&PhysicsEngine::contactSolver))
      .prop("geometry_pool", &PhysicsEngine::resetGeometryPool,
            GeometryPoolFunc(&PhysicsEngine::geometryPool))
      .prop("collision_filter", &PhysicsEngine::resetCollisionFilter,
            CollisionFilterPoolFunc(&PhysicsEngine::collisionFilter));
}
}  // namespace sire::physics