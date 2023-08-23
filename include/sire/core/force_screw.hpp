#ifndef SIRE_FORCE_SCREW_HPP_
#define SIRE_FORCE_SCREW_HPP_
#include <cmath>
#include <sire_lib_export.h>
namespace sire::core::screw {
// 符号定义
// f: 3x1 纯力
// pe: 6x1 位置与欧拉角表示的朝向
// tau: 3x1 纯力偶
// fs:  6x1 力旋量 [f, tau]
auto SIRE_API s_fpm2fs(const double* f, const double* pm, double* fs_out) -> void;
}
#endif