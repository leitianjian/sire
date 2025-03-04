#ifndef SIRE_FORCE_SCREW_HPP_
#define SIRE_FORCE_SCREW_HPP_
#include <set>
#include <vector>

#include <sire_lib_export.h>

#include "sire/core/constants.hpp"
namespace sire::core::screw {
// 用于将空间力矢量转换为力旋量
// f: 3x1 纯力
// pe: 6x1 位置与欧拉角表示的朝向
// tau: 3x1 纯力偶
// fs:  6x1 力旋量 [f, tau]
auto SIRE_API s_fpm2fs(const double* f, const double* pm,
                       double* fs_out) -> void;

// 数除数组并返回数组，非原址操作
// numerator: 被除数
// denominators: 除数组
// n： 除数组大小
// zero_tol: 除数组的零值容忍范围，当除数小于这个阈值时，计算结果为零
// dv_out: 计算结果数组
auto SIRE_API s_dv(const double numerator, const double* denominators,
                   const sire::Size n, const double zero_tol,
                   double* dv_out) -> void;

// 数除数组并返回数组，原址操作
// numerator: 被除数
// denominators: 除数组及结果数组
// n： 除数组大小
// zero_tol: 除数组的零值容忍范围，默认设置为1e-5，
// 当除数小于这个阈值时，计算结果为零
auto SIRE_API s_dvi(const double numerator, double* denominators,
                    const sire::Size n, const double zero_tol) -> void;

auto SIRE_API s_safe_div(double number, double denominator,
                         double tol) -> double;

auto s_is_zeros(sire::Size m, sire::Size n, const double* v,
                double error) -> bool;
auto s_array_sum_abs_diff(sire::Size n, const double* A,
                          const double* B) -> double;
// type 0: norm(A, Inf)
// type 1: norm(A, 1)
// type 2: norm(A, 2)
auto matrix_norm(sire::Size m, sire::Size n, const double* A,
                 const int type) -> double;
auto squareMatrixRemoveLines(sire::Size n, const double* squareMatrix,
                             const std::set<sire::Size>& rmIdx,
                             std::vector<double>& result) -> void;
auto matrix_exp_pade(sire::Size n, const double* A, double* res) -> void;
auto matrixVectorComposeBack(sire::Size n, const double* A, const double* b, double* newA) -> void;
}  // namespace sire::core::screw
#endif