#ifndef SIRE_IS_LESS_THAN_COMPARABLE_HPP_
#define SIRE_IS_LESS_THAN_COMPARABLE_HPP_
#include <type_traits>
namespace sire::core {
// 默认情况： 传入T，返回false_type
template <typename T, typename = void>
struct is_less_than_comparable : std::false_type {};

// 如果 T 有小于号的方法，则返回true_type
template <typename T>
struct is_less_than_comparable<
    T, typename std::enable_if_t<
           true, decltype(std::declval<T&>() < std::declval<T&>(), (void)0)>>
    : std::true_type {};
}  // namespace sire::core
#endif