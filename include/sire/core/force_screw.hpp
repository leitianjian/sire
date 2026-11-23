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
/// Reusable Pade(6) storage. Solves denominator * result = numerator with
/// partial pivoting, then squares. Input/output may alias; not thread-safe.
class SIRE_API MatrixExpPadeWorkspace {
 public:
  explicit MatrixExpPadeWorkspace(sire::Size n);
  auto apply(const double* A, double* res, double t = 1.0) -> void;
 private:
  sire::Size n_;
  std::vector<double> scaled_, power_, numerator_, denominator_, scratch_;
};
/// Prepared exponential action for a fixed row-major matrix. Owns a copy of
/// A and reusable scratch buffers. apply() supports v==res; calls on the same
/// workspace must be serial. Create one workspace per contact-time solve.
class SIRE_API MatrixExpMultiplyWorkspace {
 public:
  MatrixExpMultiplyWorkspace(sire::Size n, const double* A);
  auto apply(const double* v, double* res, double t = 1.0) -> void;

 private:
  sire::Size n_;
  double shift_{0}, full_norm_{0}, shifted_norm_{0};
  std::vector<double> matrix_, shifted_, state_, term_, next_, pade_;
  MatrixExpPadeWorkspace pade_workspace_;
};

/// Compute res = exp(t*A)*v for a row-major n-by-n matrix. The scaled
/// Taylor path forms no matrix exponential; costly cases use Padé.
/// Uses double-precision norm bounds. v and res may alias; A is read-only.
/// n==0 is a no-op. Throws for invalid input or non-finite arithmetic.
auto SIRE_API matrix_exp_multiply(sire::Size n, const double* A,
                                 const double* v, double* res,
                                 double t = 1.0) -> void;
auto matrixVectorComposeBack(sire::Size n, const double* A, const double* b, double* newA) -> void;
}  // namespace sire::core::screw
#endif
