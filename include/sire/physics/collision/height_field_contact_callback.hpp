#ifndef SIRE_HEIGHT_FIELD_CONTACT_CALLBACK_HPP_
#define SIRE_HEIGHT_FIELD_CONTACT_CALLBACK_HPP_

#include <string>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/hfield.h>

#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::physics::collision {

/// @brief 针对 HeightField 的碰撞回调，一次检测全部接触点后去重。
///
/// 与 PenetrationAsPointPairCallback 不同：
///   - 对 HeightField 相关的碰撞对使用 num_max_contacts = 8，
///     避免 BVH 遍历顺序导致漏掉最深穿透。
///   - 对非 HeightField 的碰撞对直接复用原始行为。
///   - 对 HeightField 结果做空间去重，过滤掉距离过近的重复接触点。
class SIRE_API HeightFieldContactCallback
    : public coal::CollisionCallBackBase {
 public:
  coal::CollisionRequest request;

  bool collide(coal::CollisionObject* fcl_object_A_ptr,
               coal::CollisionObject* fcl_object_B_ptr) override;

  HeightFieldContactCallback(
      CollisionFilter* filter_in,
      std::vector<common::PenetrationAsPointPair>* point_pairs_in);
  virtual ~HeightFieldContactCallback() = default;

 private:
  CollisionFilter* filter_;
  std::vector<common::PenetrationAsPointPair>* point_pairs_;

  // TODO: 筛选接触点 — 空间去重阈值：两个接触点距离小于此值视为重复
  static constexpr double kSpatialTolerance = 1e-4;

  // TODO: 筛选接触点 — HeightField 碰撞检测时 COAL 的最大接触点数
  static constexpr int kMaxContactsPerPair = 8;

  /// 将一个 COAL Contact 转换为 PenetrationAsPointPair
  auto contactToPointPair(const coal::Contact& contact,
                          sire::geometry::GeometryId id_A,
                          sire::geometry::GeometryId id_B)
      -> common::PenetrationAsPointPair;

  /// 判断两个接触点在空间上是否过近
  static bool areTooClose(const common::PenetrationAsPointPair& a,
                          const common::PenetrationAsPointPair& b);

  /// 对 HeightField 碰撞结果进行 cell 级去重：同一 grid cell 最多保留 1 个
  /// 接触点（最深的），不同 cell 的接触点全部保留，实现与 MuJoCo 类似的稀疏多点效果。
  /// @param raw  原始接触点列表
  /// @param hfield_ptr  HeightField 对象指针（可为 nullptr，退化为原距离去重）
  /// @param hfield_tf    HeightField 的世界位姿（用于将接触点转到局部系）
  auto filterHeightFieldContacts(
      const std::vector<common::PenetrationAsPointPair>& raw,
      const coal::CollisionGeometry* hfield_geom = nullptr,
      const coal::Transform3s* hfield_tf = nullptr)
      -> std::vector<common::PenetrationAsPointPair>;
};

}  // namespace sire::physics::collision
#endif
