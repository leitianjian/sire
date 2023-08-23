#ifndef SIRE_SORTED_PAIR_HPP_
#define SIRE_SORTED_PAIR_HPP_
#include <functional>
#include <ostream>

#include <sire_lib_export.h>

#include "geometry/geometry_base.hpp"

namespace sire::core {
using namespace sire::geometry;
class SortedPair {
 public:
  SortedPair() : first_(), second_() {}

  /// Rvalue reference constructor,
  /// permits constructing with std::unique_ptr types
  SortedPair(GeometryId&& a, GeometryId&& b) {
    if (b < a) {
      first_ = std::move(b);
      second_ = std::move(a);
    } else {
      first_ = std::move(a);
      second_ = std::move(b);
    }
  }

  /// Constructs a SortedPair from two GeometryId objects.
  SortedPair(const GeometryId& a, const GeometryId& b) : first_(a), second_(b) {
    if (first_ > second_) {
      /// compatible with GeometryId self-defined swap function
      using std::swap;
      swap(first_, second_);
    }
  }

  /// Resets the stored GeometryId.
  auto set(GeometryId&& a, GeometryId&& b) -> void {
    first_ = std::forward<GeometryId>(a);
    second_ = std::forward<GeometryId>(b);
    if (first_ > second_) {
      using std::swap;
      swap(first_, second_);
    }
  }

  auto swap(SortedPair& p) -> void {
    using std::swap;
    swap(first_, p.first_);
    swap(second_, p.second_);
  }

  auto first() const -> const GeometryId& { return first_; }
  auto second() const -> const GeometryId& { return second_; }

 private:
  GeometryId first_;
  GeometryId second_;
};

inline std::ostream& operator<<(std::ostream& out, const SortedPair& pair) {
  out << "(" << pair.first() << ", " << pair.second() << ")";
  return out;
}

inline bool operator<(const SortedPair& a, const SortedPair& b) {
  return std::tie(a.first(), a.second()) < std::tie(b.first(), b.second());
}

inline bool operator>(const SortedPair& a, const SortedPair& b) {
  return b < a;
}

inline bool operator<=(const SortedPair& a, const SortedPair& b) {
  return !(b < a);
}

inline bool operator>=(const SortedPair& a, const SortedPair& b) {
  return !(a < b);
}

inline bool operator==(const SortedPair& a, const SortedPair& b) {
  return !(b < a) && !(a < b);
}

}  // namespace sire::core
namespace std {
template <>
inline auto swap<sire::core::SortedPair>(sire::core::SortedPair& a,
                                         sire::core::SortedPair& b) noexcept
    -> void {
  a.swap(b);
}

template <>
struct hash<sire::core::SortedPair> {
  std::size_t operator()(const sire::core::SortedPair& k) const {
    using std::hash, sire::core::GeometryId;
    // Compute individual hash values for first,
    // second and combine them using XOR and bit shifting:
    return hash<GeometryId>()(k.first()) ^
           (hash<GeometryId>()(k.second()) << 1);
  }
};
}  // namespace std
#endif