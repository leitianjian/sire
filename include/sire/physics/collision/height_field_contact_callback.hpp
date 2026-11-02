#ifndef SIRE_HEIGHT_FIELD_CONTACT_CALLBACK_HPP_
#define SIRE_HEIGHT_FIELD_CONTACT_CALLBACK_HPP_

#include <string>
#include <cstdint>
#include <unordered_map>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/hfield.h>

#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::physics::collision {

struct HeightFieldTrackedContact {
  coal::Contact contact;
  std::uint64_t contact_id{0};
  std::uint64_t patch_id{0};
};

/// Persistent IDs and local anchors for contacts selected within one geometry
/// pair. The narrow-phase cell ID is deliberately excluded from the key.
class SIRE_API HeightFieldContactManifold {
 public:
  void clear();
  void beginQuery();
  void endQuery();
  auto select(const coal::CollisionObject& terrain,
              const coal::CollisionObject& shape,
              const std::vector<coal::Contact>& candidates,
              std::size_t maximum) -> std::vector<HeightFieldTrackedContact>;

 private:
  struct Key {
    const coal::CollisionGeometry* terrain;
    const coal::CollisionGeometry* shape;
    bool operator==(const Key& other) const {
      return terrain == other.terrain && shape == other.shape;
    }
  };
  struct KeyHash {
    std::size_t operator()(const Key& key) const;
  };
  struct Anchor {
    coal::Vec3s terrain_local;
    coal::Vec3s shape_local;
    coal::Vec3s normal_local;
    coal::Scalar penetration_depth;
    std::uint64_t contact_id;
    std::uint64_t patch_id;
  };
  struct Entry {
    std::vector<Anchor> anchors;
    std::uint64_t last_query{0};
  };
  std::unordered_map<Key, Entry, KeyHash> entries_;
  std::uint64_t query_{0};
  std::uint64_t next_contact_id_{1};
  std::uint64_t next_patch_id_{1};
};

/// @brief Selects HeightField contacts; patch mode requests leaf-level patch
/// candidates from Coal and reduces them through the persistent Sire manifold.
class SIRE_API HeightFieldContactCallback
    : public coal::CollisionCallBackBase {
 public:
  coal::CollisionRequest request;

  bool collide(coal::CollisionObject* fcl_object_A_ptr,
               coal::CollisionObject* fcl_object_B_ptr) override;

  HeightFieldContactCallback(
      CollisionFilter* filter_in,
      std::vector<common::PenetrationAsPointPair>* point_pairs_in,
      HeightFieldContactManifold* manifold = nullptr,
      bool use_generic_patch = false, bool use_prism_contacts = false,
      std::size_t prism_max_contacts = 50);
  virtual ~HeightFieldContactCallback() = default;

 private:
  CollisionFilter* filter_;
  std::vector<common::PenetrationAsPointPair>* point_pairs_;
  HeightFieldContactManifold* manifold_;
  bool use_generic_patch_;
  bool use_prism_contacts_;
  std::size_t prism_max_contacts_;

  // Maximum selected contacts for one HeightField/shape pair.
  static constexpr int kMaxContactsPerPair = 4;

  /// 将一个 COAL Contact 转换为 PenetrationAsPointPair
  auto contactToPointPair(const coal::Contact& contact,
                          sire::geometry::GeometryId id_A,
                          sire::geometry::GeometryId id_B)
      -> common::PenetrationAsPointPair;

};

}  // namespace sire::physics::collision
#endif
