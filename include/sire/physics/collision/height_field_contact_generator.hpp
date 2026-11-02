#ifndef SIRE_HEIGHT_FIELD_CONTACT_GENERATOR_HPP_
#define SIRE_HEIGHT_FIELD_CONTACT_GENERATOR_HPP_

#include <cstddef>
#include <vector>

#include <coal/collision_data.h>
#include <coal/collision_object.h>

namespace sire::physics::collision {

struct HeightFieldPrismQueryStats {
  std::size_t cells_visited{0};
  std::size_t prisms_tested{0};
  std::size_t prisms_height_rejected{0};
  bool limit_reached{false};
};

// MuJoCo-style candidate generation: bound the shape in terrain coordinates,
// index the overlapping sub-grid directly, and request one signed-distance
// witness per triangular prism. No per-leaf contact-patch expansion or reduction.
// The limit applies to raw candidates, not the final manifold. A large limit
// and stats must be used when measuring uncapped geometry overhead.
auto generateHeightFieldPrismContacts(
    const coal::CollisionObject& height_field,
    const coal::CollisionObject& shape, std::size_t max_contacts = 50,
    HeightFieldPrismQueryStats* stats = nullptr) -> std::vector<coal::Contact>;

// Generates HeightField-first contacts from the public COAL grid and shape
// data.  Sphere and box contacts are computed here so neither the number of
// returned points nor their witnesses depend on COAL's HeightField traversal.
// Other shapes remain on the normal COAL collision path in the callback.
auto generateHeightFieldContacts(const coal::CollisionObject& height_field,
                                 const coal::CollisionObject& shape,
                                 std::size_t max_contacts)
    -> std::vector<coal::Contact>;

// Uses COAL's shape-independent HeightField leaf contacts and contact patches.
// Expanded patch points are accepted only when both witnesses can be projected
// onto the actual geometry surfaces with a consistent contact normal.
auto generateHeightFieldPatchContacts(const coal::CollisionObject& height_field,
                                      const coal::CollisionObject& shape,
                                      std::size_t max_contacts)
    -> std::vector<coal::Contact>;

}  // namespace sire::physics::collision

#endif
