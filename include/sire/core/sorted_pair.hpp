#ifndef SIRE_SORTED_PAIR_HPP_
#define SIRE_SORTED_PAIR_HPP_
#include <algorithm>
#include <functional>
#include <ostream>
#include <utility>

#include <sire_lib_export.h>

#include "sire/core/is_less_than_comparable.hpp"

namespace sire::core {
template <class T>
class SortedPair {
  static_assert(is_less_than_comparable<T>::value,
                "SortedPair can only be used"
                " with types that can be compared using the less-than operator"
                " (operator<).");

 public:
  SortedPair() : first_(), second_() {}

  /// Rvalue reference constructor,
  /// permits constructing with std::unique_ptr types
  SortedPair(T&& a, T&& b) {
    if (b < a) {
      first_ = std::move(b);
      second_ = std::move(a);
    } else {
      first_ = std::move(a);
      second_ = std::move(b); 
    }
  }

  /// Constructs a SortedPair from two T objects.
  SortedPair(const T& a, const T& b) : first_(a), second_(b) {
    if (second_ < first_) {
      /// compatible with T self-defined swap function
      using std::swap;
      swap(first_, second_);
    }
  }

  /// Resets the stored T.
  template <class U>
  auto set(U&& a, U&& b) -> void {
    first_ = std::forward<U>(a);
    second_ = std::forward<U>(b);
    if (second_ < first_) {
      using std::swap;
      swap(first_, second_);
    }
  }

  auto swap(SortedPair<T>& p) -> void {
    using std::swap;
    swap(first_, p.first_);
    swap(second_, p.second_);
  }

  auto first() const -> const T& { return first_; }
  auto second() const -> const T& { return second_; }

 private:
  T first_;
  T second_;
};

template <class T>
inline std::ostream& operator<<(std::ostream& out, const SortedPair<T>& pair) {
  out << "(" << pair.first() << ", " << pair.second() << ")";
  return out;
}
template <class T>
inline bool operator<(const SortedPair<T>& a, const SortedPair<T>& b) {
  return std::tie(a.first(), a.second()) < std::tie(b.first(), b.second());
}
template <class T>
inline bool operator>(const SortedPair<T>& a, const SortedPair<T>& b) {
  return b < a;
}

template <class T>
inline bool operator<=(const SortedPair<T>& a, const SortedPair<T>& b) {
  return !(b < a);
}

template <class T>
inline bool operator>=(const SortedPair<T>& a, const SortedPair<T>& b) {
  return !(a < b);
}

template <class T>
inline bool operator==(const SortedPair<T>& a, const SortedPair<T>& b) {
  return !(b < a) && !(a < b);
}

}  // namespace sire::core
namespace std {
template <class T>
inline auto swap(sire::core::SortedPair<T>& a,
                 sire::core::SortedPair<T>& b) noexcept -> void {
  a.swap(b);
}

template <class T>
struct hash<sire::core::SortedPair<T>> {
  std::size_t operator()(const sire::core::SortedPair<T>& k) const {
    using std::hash;
    // Compute individual hash values for first,
    // second and combine them using XOR and bit shifting:
    return hash<T>()(k.first()) ^
           (hash<T>()(k.second()) << 1);
  }
};
}  // namespace std
#endif