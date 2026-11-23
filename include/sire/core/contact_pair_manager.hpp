#ifndef SIRE_CONTACT_PAIR_MANAGER_HPP_
#define SIRE_CONTACT_PAIR_MANAGER_HPP_
#include <unordered_map>
#include <unordered_set>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sorted_pair.hpp"
namespace sire::core {
enum class ContactPairState {
  START,
  CONTACTING,
  END,
};
enum class ContactPairType { SPLIT, STICK };
struct ContactPairValue {
  ContactPairState state;
  ContactPairType type;
  double init_penetration_depth_;
  bool is_depth_smaller_than_init_depth_;
  ContactPairValue(double init_penetration_depth = 0,
                   bool is_depth_smaller_than_init_depth = false)
      : state(ContactPairState::START),
        type(ContactPairType::SPLIT),
        init_penetration_depth_(init_penetration_depth),
        is_depth_smaller_than_init_depth_(is_depth_smaller_than_init_depth) {}
};

/// @brief Compound key for multi-contact scenarios (e.g., HeightField) where
/// the same geometry pair can have multiple contact points. Uses quantized
/// world-space contact position to distinguish contacts.
/// Precision is determined by the caller's quantization factor (e.g., *1e4
/// for 0.1mm, matching HeightFieldCallback::kSpatialTolerance).
struct SIRE_API ContactPointKey {
  core::SortedPair<sire::PartId> geom_pair;
  /// Quantized contact position (value = round(p_WC[m] * scale))
  long long qx, qy, qz;
};

/// @brief Hash for ContactPointKey
struct SIRE_API ContactPointKeyHash {
  std::size_t operator()(const ContactPointKey& k) const {
    auto h1 = std::hash<std::size_t>{}(k.geom_pair.first());
    auto h2 = std::hash<std::size_t>{}(k.geom_pair.second());
    auto h3 = std::hash<long long>{}(k.qx);
    auto h4 = std::hash<long long>{}(k.qy);
    auto h5 = std::hash<long long>{}(k.qz);
    return (((h1 ^ (h2 << 1)) ^ (h3 << 2)) ^ (h4 << 3)) ^ (h5 << 4);
  }
};

/// @brief Equality for ContactPointKey (exact match on quantized integers)
inline bool operator==(const ContactPointKey& a, const ContactPointKey& b) {
  return a.geom_pair == b.geom_pair && a.qx == b.qx && a.qy == b.qy &&
         a.qz == b.qz;
}

/// @brief Map type keyed by contact point (geometry pair + position)
using ContactPointMap =
    std::unordered_map<ContactPointKey, ContactPairValue, ContactPointKeyHash>;

// 需要三个表
// 1. 用来记录需要缩小时间步长的情况，只需要保存partId
// 2.
// 用来记录碰撞和碰撞初始穿深（用以去掉一开始的过大穿深导致的积分不收敛和获得能量的问题）
//    注意：在两个物体真正分离之前，都不应该丢弃这条记录，因为会导致不收敛。
//          如果 penetration - init_penetration < 0时，就当作接触力不存在就好了
// 3.
// 用来记录碰撞点导致的过大冲击，如果检测不到碰撞或prt没有被inspect，就可以从表中
//    删除相应的记录
//
// 表一：用来保存因碰撞受到较大冲击的Part
// 插入条件：
// 在dt内，有一个很大的冲击 > impact_threshold，存在以下两种情况
// 1. 如果检测到未在表三中记录的新的碰撞，则冲击来源可以判定为碰撞导致
// 2.
// 如果没检测到需要加入表三的新的碰撞，则可以判定是由外力导致的冲击，应该不加入表
//   （即不缩小步长）
// 综上：检测到新的碰撞，没有在表三的记录中 && 在下一个dt内，冲击 > threshold
//
// 删除条件：
// 记录中的prt，在dt内，impact小于 impact_threshold2
//
// 表二：用来记录碰撞点的初始穿深
// 插入条件：
// 检测到新的碰撞，没有在记录中
// 记录碰撞pair和初始的穿深，之后检测到这两个碰撞的时候，penetration都需要减掉这个初始穿深
//
// 删除条件：
// 检测不到该碰撞，就可以删除
//
// 好像不需要，直接用每一轮检测的结果就好了
// 表三：用来记录产生了较大冲击的碰撞点
// 插入条件：
// 碰撞点的任意prt在inspection set中 and 碰撞点没有被加入
// 删除条件：
// 检测不到碰撞 || 碰撞点的prt都不在inspection set中
class SIRE_API ContactPairManager {
 public:
  // auto getContactingSet() const
  //     -> const std::unordered_set<core::SortedPair<sire::PartId>>&;
  auto insert(const sire::PartId ground, const sire::PartId id_B,
              double init_penetration_depth) -> void;
  auto insert(const core::SortedPair<sire::PartId>& pair,
              double init_penetration_depth) -> void;
  auto insert(const core::SortedPair<sire::PartId>& pair,
              const ContactPairValue& value) -> void;
  auto containsContactPair(const core::SortedPair<sire::PartId>& pair) const
      -> bool;
  auto hasImpactedPrt(sire::PartId prt_id) const -> bool;
  auto isImpactedSetEmpty() const -> bool;
  auto impactedContactSet() -> std::unordered_set<SortedPair<sire::PartId>>&;
  auto contactPairMap()
      -> std::unordered_map<SortedPair<sire::PartId>, ContactPairValue>&;
  auto contactPointMap() -> ContactPointMap&;
  auto impactedPrtSet() -> std::unordered_set<sire::PartId>&;
  auto getValue(const core::SortedPair<sire::PartId>& pair) const
      -> const ContactPairValue&;
  auto getValue(const core::SortedPair<sire::PartId>& pair)
      -> ContactPairValue& {
    return const_cast<ContactPairValue&>(
        static_cast<const ContactPairManager*>(this)->getValue(pair));
  }
  auto setValue(const core::SortedPair<sire::PartId>& pair,
                const ContactPairValue& value) -> void;
  // auto setState(const core::SortedPair<sire::PartId>& pair, ContactPairState
  // state)
  //     -> void;
  // auto setType(const core::SortedPair<sire::PartId>& pair, ContactPairType
  // type)
  //     -> void;
  auto clear() -> void;
  auto init() -> void;
  ContactPairManager();
  ~ContactPairManager();
  ARIS_DECLARE_BIG_FOUR(ContactPairManager);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::core
#endif