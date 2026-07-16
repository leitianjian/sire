#include "sire/physics/collision/height_field_contact_callback.hpp"

#include <unordered_map>

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/hfield.h>
#include <coal/math/transform.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics::collision {
using namespace coal;

auto HeightFieldContactCallback::contactToPointPair(
    const Contact& contact, sire::geometry::GeometryId id_A,
    sire::geometry::GeometryId id_B) -> common::PenetrationAsPointPair {
  common::PenetrationAsPointPair pair;
  pair.id_A = id_A;
  pair.id_B = id_B;
  pair.p_WCa = contact.nearest_points[0];
  pair.p_WCb = contact.nearest_points[1];
  pair.p_WC = contact.pos;
  pair.nhat_AB_W = contact.normal;
  pair.depth = std::abs(contact.penetration_depth);
  pair.modifiedDepth = pair.depth;
  return pair;
}

// TODO: 筛选接触点 — areTooClose: 距离 < kSpatialTolerance(0.1mm) 视为重复
auto HeightFieldContactCallback::areTooClose(
    const common::PenetrationAsPointPair& a,
    const common::PenetrationAsPointPair& b) -> bool {
  return (a.p_WC - b.p_WC).norm() < kSpatialTolerance;
}

auto HeightFieldContactCallback::filterHeightFieldContacts(
    const std::vector<common::PenetrationAsPointPair>& raw,
    const coal::CollisionGeometry* hfield_geom,
    const coal::Transform3s* hfield_tf)
    -> std::vector<common::PenetrationAsPointPair> {
  // TODO: 筛选接触点 — 1. 筛掉穿透深度过小（<= epsilon）的接触点
  std::vector<common::PenetrationAsPointPair> valid;
  for (auto& pair : raw) {
    if (pair.depth > std::numeric_limits<double>::epsilon()) {
      valid.push_back(pair);
    }
  }
  if (valid.size() <= 1) return valid;

  // 按深度排序（统一用于后续步骤）
  std::sort(valid.begin(), valid.end(),
            [](const auto& a, const auto& b) { return a.depth > b.depth; });

  std::vector<common::PenetrationAsPointPair> deduped;

  // TODO: 筛选接触点 — 2. Cell 级去重（仅当 HeightField 网格数据可用时）
  bool used_cell_dedup = false;
  if (hfield_geom != nullptr && hfield_tf != nullptr &&
      hfield_geom->getObjectType() == OT_HFIELD) {
    auto* hf = static_cast<const coal::HeightField<coal::AABB>*>(hfield_geom);
    const auto& x_grid = hf->getXGrid();
    const auto& y_grid = hf->getYGrid();
    int ncol = static_cast<int>(x_grid.size());
    int nrow = static_cast<int>(y_grid.size());
    if (ncol >= 2 && nrow >= 2) {
      double dx = x_grid[1] - x_grid[0];
      double dy = y_grid[1] - y_grid[0];
      if (dx > 0 && dy > 0) {
        std::unordered_map<uint64_t, bool> cell_occupied;
        for (auto& pair : valid) {
          coal::Vec3s local =
              hfield_tf->inverseTransform(coal::Vec3s(
                  pair.p_WC[0], pair.p_WC[1], pair.p_WC[2]));
          int cx = static_cast<int>(std::floor((local[0] - x_grid[0]) / dx));
          int cy = static_cast<int>(std::floor((local[1] - y_grid[0]) / dy));
          cx = std::max(0, std::min(ncol - 1, cx));
          cy = std::max(0, std::min(nrow - 1, cy));
          uint64_t key = (static_cast<uint64_t>(cy) << 32) | static_cast<uint32_t>(cx);
          if (!cell_occupied[key]) {
            cell_occupied[key] = true;
            deduped.push_back(pair);
          }
        }
        used_cell_dedup = true;
      }
    }
  }

  // TODO: 筛选接触点 — 3. 回退：距离去重（0.1mm 阈值，当 cell 去重不可用时）
  if (!used_cell_dedup) {
    for (auto& pair : valid) {
      bool too_close = false;
      for (auto& selected : deduped) {
        if (areTooClose(pair, selected)) {
          too_close = true;
          break;
        }
      }
      if (!too_close) {
        deduped.push_back(pair);
      }
    }
  }

  // TODO: 筛选接触点 — 4. 空间合并：所有路径最终都执行，阈值 1cm。
  //    同一 body pair 的接触点若间距 < 1cm 只保留最深的一个。
  //    这是 MuJoCo max_contacts=1 的等效实现。
  if (deduped.size() <= 1) return deduped;
  {
    std::vector<common::PenetrationAsPointPair> merged;
    for (auto& pair : deduped) {
      bool too_close = false;
      for (auto& selected : merged) {
        if (pair.id_A == selected.id_A && pair.id_B == selected.id_B &&
            (pair.p_WC - selected.p_WC).norm() < 0.01) {
          too_close = true;
          break;
        }
      }
      if (!too_close) {
        merged.push_back(pair);
      }
    }
    return merged;
  }
}

auto HeightFieldContactCallback::collide(
    CollisionObject* fcl_object_A_ptr,
    CollisionObject* fcl_object_B_ptr) -> bool {
  SIRE_DEMAND(point_pairs_ != nullptr);

  // 统一 id 顺序：id_A < id_B
  sire::geometry::GeometryId id_A = filter_->queryGeometryIdByPtr(
      fcl_object_A_ptr->collisionGeometry().get());
  sire::geometry::GeometryId id_B = filter_->queryGeometryIdByPtr(
      fcl_object_B_ptr->collisionGeometry().get());

  if (id_A > id_B) {
    std::swap(id_A, id_B);
    std::swap(fcl_object_A_ptr, fcl_object_B_ptr);
  }

  if (!filter_->canCollideWith(fcl_object_A_ptr, fcl_object_B_ptr))
    return false;

  // 判断是否涉及 HeightField
  const bool is_hfield =
      fcl_object_A_ptr->collisionGeometry()->getObjectType() == OT_HFIELD ||
      fcl_object_B_ptr->collisionGeometry()->getObjectType() == OT_HFIELD;

  if (is_hfield) {
    // ── HeightField 路径：多接触点 + 去重 ──────────────────────────
    CollisionRequest hf_req;
    // TODO: 筛选接触点 — COAL 碰撞检测最多返回 kMaxContactsPerPair 个接触点
    hf_req.num_max_contacts = kMaxContactsPerPair;
    hf_req.enable_contact = true;
    hf_req.gjk_tolerance = 2e-8;

    CollisionResult result;
    coal::collide(fcl_object_A_ptr, fcl_object_B_ptr, hf_req, result);

    if (!result.isCollision()) return false;

    // 收集所有原始接触点
    std::vector<common::PenetrationAsPointPair> raw_pairs;
    for (const auto& contact : result.getContacts()) {
      if (std::abs(contact.penetration_depth) <=
          std::numeric_limits<double>::epsilon())
        continue;
      raw_pairs.push_back(contactToPointPair(contact, id_A, id_B));
    }

    // 提取 HeightField 网格参数用于 cell 级去重
    // const coal::CollisionGeometry* hfield_geom = nullptr;
    // const coal::Transform3s* hfield_tf = nullptr;
    // coal::Transform3s tf_tmp;
    // if (fcl_object_A_ptr->collisionGeometry()->getObjectType() == OT_HFIELD) {
    //   hfield_geom = fcl_object_A_ptr->collisionGeometry().get();
    //   tf_tmp = coal::Transform3s(
    //       coal::Matrix3s(fcl_object_A_ptr->getRotation()),
    //       fcl_object_A_ptr->getTranslation());
    //   hfield_tf = &tf_tmp;
    // } else if (fcl_object_B_ptr->collisionGeometry()->getObjectType() ==
    //            OT_HFIELD) {
    //   hfield_geom = fcl_object_B_ptr->collisionGeometry().get();
    //   tf_tmp = coal::Transform3s(
    //       coal::Matrix3s(fcl_object_B_ptr->getRotation()),
    //       fcl_object_B_ptr->getTranslation());
    //   hfield_tf = &tf_tmp;
    // }

    // // cell 级去重后写入全局列表
    // auto filtered =
    //     filterHeightFieldContacts(raw_pairs, hfield_geom, hfield_tf);
    for (auto& pair : raw_pairs) {
      point_pairs_->push_back(std::move(pair));
    }

  } else {
    // ── 非 HeightField 路径：与原始 PenetrationAsPointPairCallback 一致 ─
    CollisionResult result;
    coal::collide(fcl_object_A_ptr, fcl_object_B_ptr, request, result);

    if (!result.isCollision()) return false;

    const Contact& contact = result.getContact(0);
    const double depth = std::abs(contact.penetration_depth);

    // FCL issue #375: osculation case, normal may be degenerate
    if (depth <= std::numeric_limits<double>::epsilon()) return false;

    common::PenetrationAsPointPair pair;
    pair.depth = depth;
    pair.modifiedDepth = depth;
    pair.nhat_AB_W = contact.normal;
    pair.p_WCa = contact.nearest_points[0];
    pair.p_WCb = contact.nearest_points[1];
    pair.p_WC = contact.pos;
    pair.id_A = id_A;
    pair.id_B = id_B;
    point_pairs_->push_back(std::move(pair));
  }

  // false = 不终止 broadphase 遍历
  return false;
}

HeightFieldContactCallback::HeightFieldContactCallback(
    CollisionFilter* filter_in,
    std::vector<common::PenetrationAsPointPair>* point_pairs_in)
    : CollisionCallBackBase(),
      filter_(filter_in),
      point_pairs_(point_pairs_in) {
  request.num_max_contacts = 1;
  request.enable_contact = true;
  request.gjk_tolerance = 2e-8;
}

}  // namespace sire::physics::collision
