#include "sire/core/force_screw.hpp"

#include <cmath>

#include <aris/core/log.hpp>
#include <aris/dynamic/pose.hpp>
#include <aris/dynamic/screw.hpp>
#include <aris/ext/Array.hh>

#include "sire/core/constants.hpp"

namespace sire::core::screw {
/// Sire矩阵、旋量和向量相关的数学算法扩展 \n
///
///
/// # 一、矩阵表示（aris中的）：\n
///
///
/// 矩阵需要连续分布在内存中，例如矩阵 A 为 m x n 维：
/// [ a11 a12 ... a1n ]
/// | a21 a22 ... a2n |
/// | ... ...     ... |
/// [ am1 am2 ... amn ]
///
/// 在内存中可以沿着行来排列：
/// 内存位置：      1   2       n  n+1 n+2     2*n 2*n+1        m*n
/// 内存数据：   [ a11 a12 ... a1n a21 a22 ... a2n  a31 ... ... amn ]
///
/// 也可以沿着列来排列：
/// 内存位置：      1   2       m  m+1 m+2     2*m 2*m+1        m*n
/// 内存数据：   [ a11 a21 ... am1 a12 a22 ... am2  a13 ... ... amn ]
///
/// 延行排列的，叫做【行主元】，延列排列的，叫做【列主元】
///
/// 行主元用 RowMajor 类型表示，内含一个 Size 类型的成员
/// r_ld，它表示每一行共计有多少元素。上述矩阵的 行主元数为 n，即 r_ld =
/// n。一般来说，r_ld 可以大于 n，此时 A 仅仅为一个更大矩阵的子阵： \n
///    1   2       n    n+1 ... r_ld \n
/// [ a11 a12 ... a1n |  *  ...  *   ] \n
/// | a21 a22 ... a2n |  *  ...  *   | \n
/// | ... ...     ... |  *  ...  *   | \n
/// [ am1 am2 ... amn |  *  ...  *   ]
///
/// 【注】：上述表达中 * 表示这里占据内存，但不是矩阵 A 中的数据
///
/// 列主元同上，只不过内存沿着列方向分布。
///
/// 除行、列主元外，还可以有 【Stride】 分布，它的元素依次沿着 r_ld 和 c_ld
/// 方向分布，例如 某矩阵 A 为 3 x 4 维，在内存中按照 Stride{r_ld = 2, c_ld = 8}
/// 分布：\n
///
///            内存顺序 --->
/// [ a11  *  a12  *  a13  *  a14  * ] \n
/// | a21  *  a22  *  a23  *  a24  * | \n
/// [ a21  *  a22  *  a23  *  a24  * ] \n
///
/// 【Stride】中的 r_ld 表示 下一行元素距离当前位置的内存距离，c_ld 表示
/// 下一列元素距离当前位置的内存距离。
///
/// aris中默认用【行主元】，此时也可以用整数来表示内存分布，例如 5 等同于
/// RowMajor(5)
///
///
/// # 三、力旋量转换：\n
///
///
/// ## 1. 力矢转到力旋量：s_fpm2fs
///
/// Parameter: \n
/// f: 待转换的力矢量，三维矢量 double[3] \n
/// pm: f 矢量相对于原点的坐标变换 double[16] \n
/// fs: 转换后的力旋量 \n
/// s_fpm2fs(f, pm, fs_out)
///
using namespace aris::dynamic;
auto default_f() noexcept -> const double* {
  static const double value[3]{0, 0, 0};
  return value;
}
auto default_pe() noexcept -> const double* {
  static const double value[6]{0, 0, 0, 0, 0, 0};
  return value;
}
auto default_fs() noexcept -> const double* {
  static const double value[6]{0, 0, 0, 0, 0, 0};
  return value;
}
// auto default_pm() noexcept -> const double* {
//   static const double value[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
//   1}; return value;
// }
auto default_out() noexcept -> double* {
  static thread_local double value[36]{0};
  return value;
}
auto s_fpm2fs(const double* f, const double* pm, double* fs_out) -> void {
  f = f ? f : default_f();
  pm = pm ? pm : sire::default_pm;
  fs_out = fs_out ? fs_out : default_out();
  // 计算 fs (v, w) 的 v 部分
  s_pm_dot_v3(pm, f, fs_out);
  // 叉乘计算 fs (v, w) 的 w 部分
  s_c3(pm + 3, 4, fs_out, 1, fs_out + 3, 1);
}
// 数除向量，非原址操作
auto s_dv(const double numerator, const double* denominators, const Size n,
          const double zero_tol, double* dv_out) -> void {
  for (Size i = 0; i < n; ++i) {
    dv_out[i] =
        std::abs(denominators[i]) < zero_tol ? 0 : numerator / denominators[i];
  }
}
auto s_safe_div(double number, double denominator, double tol) -> double {
  return std::abs(denominator) < tol ? 0.0 : number / denominator;
}
// 数除向量，原址操作
auto s_dvi(const double numerator, double* denominators, const Size n,
           const double zero_tol) -> void {
  for (Size i = 0; i < n; ++i)
    denominators[i] =
        std::abs(denominators[i]) < zero_tol ? 0 : numerator / denominators[i];
}
// 判断数组全为零
auto s_is_zeros(sire::Size m, sire::Size n, const double* v, double error)
    -> bool {
  for (int i{0}; i < m; ++i) {
    for (int j{0}; j < n; ++j) {
      if (!s_is_equal(v[i * m + j], 0.0, error)) return false;
    }
  }
  return true;
}
auto matrix_norm(sire::Size m, sire::Size n, const double* A, const int type)
    -> double {
  double max{0};
  switch (type) {
    // 等价于 matlab 的 norm(A, Inf);
    case 0:
      // 元素取绝对值并行求和，并选取其中最大的
      max = 0;
      for (int i{0}; i < m; ++i) {
        double temp = 0;
        for (int j{0}; j < n; ++j) {
          temp += std::abs(A[i * n + j]);
        }
        max = std::max(temp, max);
      }
      return max;
    case 1:
      // 元素取绝对值并列求和，并选取其中最大的
      max = 0;
      for (int j{0}; j < n; ++j) {
        double temp = 0;
        for (int i{0}; i < m; ++i) {
          temp += std::abs(A[i * n + j]);
        }
        max = std::max(temp, max);
      }
      return max;
    case 2:
    default:
      THROW_FILE_LINE("Matrix Euclidean norm and others not implemented");
      return -1;
  }
}
auto s_array_sum_abs_diff(sire::Size n, const double* A, const double* B)
    -> double {
  double result{0};
  for (sire::Size i{0}; i < n; ++i) {
    result += std::abs(A[i] - B[i]);
  }
  return result;
}
auto squareMatrixRemoveLines(sire::Size n, const double* squareMatrix,
                             const std::set<sire::Size>& rmIdx,
                             std::vector<double>& result) -> void {
  sire::Size newSize = n - rmIdx.size();
  std::vector<sire::Size> selectedIdx(newSize);
  result.resize(newSize * newSize);
  for (sire::Size i{0}, idx{0}; i < n; ++i) {
    if (rmIdx.find(i) != rmIdx.end()) continue;
    selectedIdx[idx] = i;
    ++idx;
  }
  for (sire::Size i{0}; i < newSize; ++i) {
    for (sire::Size j{0}; j < newSize; ++j) {
      result[i * newSize + j] =
          squareMatrix[selectedIdx[i] * n + selectedIdx[j]];
    }
  }
}
// 基于Pade Approximation 计算矩阵指数的方法，res可以与 A相等（原址操作）
auto matrix_exp_pade(sire::Size n, const double* A, double* res) -> void {
  // 缩放，在 Matrix Computations, 3rd edition. Baltimore 1996.
  // 书中 p572 11.3.1部分有描述
  double j =
      std::max(0, 1 + (int)std::ceil(std::log2(matrix_norm(n, n, A, 0))));
  const sire::Size l = n * n;
  std::vector<double> sA(l);
  aris::dynamic::s_mc(n, n, A, sA.data());
  aris::dynamic::s_nm(n, n, 1 / std::pow(2, j), sA.data());  // sA = A / 2 ^ j;
  // Pade approximation
  std::vector<double> X(l), E(l), D(l);
  aris::dynamic::s_mc(n, n, sA.data(), X.data());  // X = sA;
  double c = 0.5;                                  // c = 1 / 2;
  aris::dynamic::s_eye(n, E.data());               // eye(size(A))
  aris::dynamic::s_eye(n, D.data());               // eye(size(A))
  aris::dynamic::s_ma(n, n, c, sA.data(),
                      E.data());  // E = eye(size(A)) + c * sA
  aris::dynamic::s_ma(n, n, -c, sA.data(),
                      D.data());  // E = eye(size(A)) - c * sA
  double q{6}, p{1};
  for (int k{2}; k <= q; ++k) {
    c = c * (q - k + 1) / (k * (2 * q - k + 1));
    std::vector<double> cX(l);
    aris::dynamic::s_mm(n, n, n, sA.data(), X.data(),
                        cX.data());                  // CX = sA * X
    aris::dynamic::s_mc(n, n, cX.data(), X.data());  // X = cX
    aris::dynamic::s_nm(n, n, c, cX.data());         // cX = c * X
    aris::dynamic::s_ma(n, n, cX.data(), E.data());  // E = E + cX;
    // D = D + (-1)^k * cX;
    aris::dynamic::s_ma(n, n, std::pow(-1, k), cX.data(), D.data());
  }
  quadprogpp::Matrix<double> e(E.data(), n, n);
  quadprogpp::Matrix<double> d(D.data(), n, n);
  // D x = E x = D^-1 * E;
  e = quadprogpp::dot_prod(quadprogpp::lu_inverse(d), e);
  for (int i{0}; i < j; ++i) {
    e = quadprogpp::dot_prod(e, e);  // E = E * E;
  }
  aris::dynamic::s_mc(n, n, e[0], res);
}

auto matrixVectorComposeBack(sire::Size n, const double* A, const double* b,
                             double* newA) -> void {
  for (sire::Size i{0}; i < n; ++i) {
    for (sire::Size j{0}; j < n; ++j) {
      newA[(n + 1) * i + j] = A[n * i + j];
    }
    newA[(n + 1) * i + n] = b[i];
  }
  // newA should be zero initilized otherwise should last column initialize to
  // zero
}
}  // namespace sire::core::screw