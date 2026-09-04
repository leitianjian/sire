#include "sire/core/force_screw.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <aris/core/log.hpp>
#include <aris/dynamic/pose.hpp>
#include <aris/dynamic/screw.hpp>
#include <aris/ext/Array.hh>

#include "sire/core/constants.hpp"
#include "sire/core/profiler.hpp"

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
MatrixExpPadeWorkspace::MatrixExpPadeWorkspace(sire::Size n) : n_(n) {
  if (n && n > std::numeric_limits<sire::Size>::max() / n)
    throw std::invalid_argument("Invalid Pade matrix size");
}

auto MatrixExpPadeWorkspace::apply(const double* A, double* res, double t) -> void {
  SIRE_PROFILE_SCOPE("core/matrix_exp_pade");
  const auto n = n_;
  if (!n) return;
  if (!A || !res || !std::isfinite(t))
    throw std::invalid_argument("Invalid Pade input");
  const auto size = n * n;
  // Lazy allocation: a Taylor-only action never allocates these matrices.
  scaled_.resize(size); power_.resize(size); numerator_.resize(size);
  denominator_.resize(size); scratch_.resize(size);
  double norm = 0;
  for (sire::Size i = 0; i < n; ++i) {
    double rowNorm = 0;
    for (sire::Size j = 0; j < n; ++j) {
      const double value = t * A[i * n + j];
      if (!std::isfinite(value)) throw std::overflow_error("Non-finite Pade input");
      scaled_[i * n + j] = value;
      rowNorm += std::abs(value);
    }
    norm = std::max(norm, rowNorm);
  }
  if (!std::isfinite(norm)) throw std::overflow_error("Pade norm overflow");
  // Preserve the old scaling rule, including its conservative extra factor 2.
  const int squarings = norm > 0 ? std::max(0, 1 + static_cast<int>(std::ceil(std::log2(norm)))) : 0;
  for (auto& value : scaled_) value = std::ldexp(value, -squarings);
  power_ = scaled_;
  for (sire::Size i = 0; i < size; ++i) {
    const double identity = i / n == i % n ? 1.0 : 0.0;
    numerator_[i] = identity + 0.5 * scaled_[i];
    denominator_[i] = identity - 0.5 * scaled_[i];
  }
  double coefficient = 0.5;
  for (int k = 2; k <= 6; ++k) {
    coefficient *= static_cast<double>(7 - k) / (k * (13 - k));
    aris::dynamic::s_mm(n, n, n, scaled_.data(), power_.data(), scratch_.data());
    power_.swap(scratch_);
    for (sire::Size i = 0; i < size; ++i) {
      const double term = coefficient * power_[i];
      numerator_[i] += term;
      denominator_[i] += k % 2 ? -term : term;
    }
  }
  {
    SIRE_PROFILE_SCOPE("core/matrix_exp_pade/solve");
    // Gaussian elimination with partial pivoting and n simultaneous RHS.
    // Reuse the numerator as the solution; never form the inverse.
    for (sire::Size k = 0; k < n; ++k) {
      sire::Size pivot = k;
      for (sire::Size i = k + 1; i < n; ++i)
        if (std::abs(denominator_[i*n+k]) > std::abs(denominator_[pivot*n+k])) pivot = i;
      if (denominator_[pivot*n+k] == 0.0)
        throw std::runtime_error("Singular Pade denominator");
      if (pivot != k) for (sire::Size j = 0; j < n; ++j) {
        std::swap(denominator_[k*n+j], denominator_[pivot*n+j]);
        std::swap(numerator_[k*n+j], numerator_[pivot*n+j]);
      }
      for (sire::Size i = k + 1; i < n; ++i) {
        const double factor = denominator_[i*n+k] / denominator_[k*n+k];
        denominator_[i*n+k] = 0;
        for (sire::Size j = k + 1; j < n; ++j)
          denominator_[i*n+j] -= factor * denominator_[k*n+j];
        for (sire::Size j = 0; j < n; ++j)
          numerator_[i*n+j] -= factor * numerator_[k*n+j];
      }
    }
    for (sire::Size row = n; row-- > 0;) {
      for (sire::Size k = row + 1; k < n; ++k)
        for (sire::Size j = 0; j < n; ++j)
          numerator_[row*n+j] -= denominator_[row*n+k] * numerator_[k*n+j];
      for (sire::Size j = 0; j < n; ++j) numerator_[row*n+j] /= denominator_[row*n+row];
    }
  }
  {
    SIRE_PROFILE_SCOPE("core/matrix_exp_pade/square");
    for (int k = 0; k < squarings; ++k) {
      aris::dynamic::s_mm(n, n, n, numerator_.data(), numerator_.data(), scratch_.data());
      numerator_.swap(scratch_);
    }
  }
  for (double value : numerator_)
    if (!std::isfinite(value)) throw std::overflow_error("Non-finite Pade result");
  std::copy(numerator_.begin(), numerator_.end(), res);
}

auto matrix_exp_pade(sire::Size n, const double* A, double* res) -> void {
  MatrixExpPadeWorkspace workspace(n);
  workspace.apply(A, res);
}

MatrixExpMultiplyWorkspace::MatrixExpMultiplyWorkspace(sire::Size n,
                                                       const double* A)
    : n_(n), pade_workspace_(n) {
  SIRE_PROFILE_SCOPE("core/matrix_exp_multiply/prepare");
  if (n == 0) return;
  if (!A || n > std::numeric_limits<sire::Size>::max() / n) {
    throw std::invalid_argument("Invalid matrix exponential action input");
  }
  for (sire::Size i = 0; i < n * n; ++i) {
    if (!std::isfinite(A[i])) {
      throw std::invalid_argument("Non-finite exponential action matrix");
    }
  }
  // Al-Mohy & Higham (2011), Algorithm 3.2: scaled Taylor action.
  // Use the exact 1-norm bound, not estimates of powers/eigenvalues. This
  // retains nonnormal and affine coupling, though it can overscale.
  // https://eprints.maths.manchester.ac.uk/1536/3/paper16.pdf
  matrix_.assign(A, A + n * n);
  shifted_ = matrix_;
  state_.resize(n);
  term_.resize(n);
  next_.resize(n);
  for (sire::Size i = 0; i < n; ++i) {
    shift_ += A[i * n + i] / static_cast<double>(n);
  }
  for (sire::Size j = 0; j < n; ++j) {
    double fullColumn = 0.0, shiftedColumn = 0.0;
    for (sire::Size i = 0; i < n; ++i) {
      fullColumn += std::abs(A[i * n + j]);
      if (i == j) shifted_[i * n + j] -= shift_;
      shiftedColumn += std::abs(shifted_[i * n + j]);
    }
    full_norm_ = std::max(full_norm_, fullColumn);
    shifted_norm_ = std::max(shifted_norm_, shiftedColumn);
  }
  if (!std::isfinite(shift_) || !std::isfinite(full_norm_) ||
      !std::isfinite(shifted_norm_)) {
    throw std::overflow_error("Exponential action norm overflow");
  }
}

auto MatrixExpMultiplyWorkspace::apply(const double* v, double* res,
                                       double t) -> void {
  SIRE_PROFILE_SCOPE("core/matrix_exp_multiply");
  const auto n = n_;
  if (n == 0) return;
  if (!v || !res || !std::isfinite(t)) {
    throw std::invalid_argument("Invalid exponential action vector/time");
  }
  auto& state = state_;
  std::copy(v, v + n, state.begin());
  bool zeroVector = true;
  for (double value : state) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Non-finite exponential action vector");
    }
    zeroVector = zeroVector && value == 0.0;
  }
  if (t == 0.0 || zeroVector) {
    std::copy(state.begin(), state.end(), res);
    return;
  }
  const double shift = t * shift_;
  const double fullNorm = std::abs(t) * full_norm_;
  const double shiftedNorm = std::abs(t) * shifted_norm_;
  if (!std::isfinite(shift) || !std::isfinite(fullNorm) ||
      !std::isfinite(shiftedNorm)) {
    throw std::overflow_error("Exponential action norm overflow");
  }

  // Subset of double-precision theta_m bounds (Higham, Table A.3;
  // Al-Mohy & Higham, Table 3.1). Minimize the upper bound m*s on matvecs.
  struct TaylorBound { int degree; double theta; };
  constexpr TaylorBound bounds[]{
      {1, 2.29e-16}, {2, 2.58e-8}, {3, 1.39e-5}, {4, 3.40e-4},
      {5, 2.40e-3}, {6, 9.07e-3}, {7, 2.38e-2}, {8, 5.00e-2},
      {9, 8.96e-2}, {10, 1.44e-1}, {15, 6.41e-1}, {20, 1.44},
      {25, 2.43}, {30, 3.54}, {35, 4.7}, {40, 6.0},
      {45, 7.2}, {50, 8.5}, {55, 9.9}};
  int degree = 0;
  double steps = 1.0, bestCost = std::numeric_limits<double>::infinity();
  // Keep the scalar trace factor representable, even for a scalar matrix.
  const double shiftSteps = std::max(1.0, std::ceil(std::abs(shift) / 500.0));
  for (const auto& bound : bounds) {
    const double count = std::max(shiftSteps,
                                  std::ceil(shiftedNorm / bound.theta));
    const double cost = count * bound.degree;
    if (cost < bestCost) {
      bestCost = cost;
      steps = count;
      degree = bound.degree;
    }
  }

  // Work heuristic only, not an error threshold: avoid excessive norm-only
  // scaling for stiff/nonnormal matrices. The fallback is independently timed.
  const double padeWork = std::max(
      512.0, 16.0 * static_cast<double>(n) *
                 (1.0 + std::ceil(std::log2(std::max(1.0, fullNorm)))));
  SIRE_PROFILE_PLOT("exp_action.time_interval", std::abs(t));
  SIRE_PROFILE_PLOT("exp_action.scaled_norm", shiftedNorm);
  SIRE_PROFILE_PLOT("exp_action.degree", static_cast<double>(degree));
  SIRE_PROFILE_PLOT("exp_action.scaling_steps", steps);
  if (!std::isfinite(bestCost) || bestCost > padeWork ||
      steps >= static_cast<double>(std::numeric_limits<int>::max())) {
    SIRE_PROFILE_PLOT("exp_action.pade_fallback", 1.0);
    SIRE_PROFILE_PLOT("exp_action.matvecs", 0.0);
    SIRE_PROFILE_SCOPE("core/matrix_exp_multiply/padeFallback");
    auto& scaled = pade_;
    scaled.resize(n * n);
    pade_workspace_.apply(matrix_.data(), scaled.data(), t);
    auto& output = next_;
    aris::dynamic::s_mm(n, 1, n, scaled.data(), state.data(), output.data());
    for (double value : output) {
      if (!std::isfinite(value)) {
        throw std::overflow_error("Non-finite Pade exponential action result");
      }
    }
    std::copy(output.begin(), output.end(), res);
    return;
  }

  {
    SIRE_PROFILE_SCOPE("core/matrix_exp_multiply/taylor");
    const int stepCount = static_cast<int>(steps);
    const double traceFactor = std::exp(shift / steps);
    const double stepTime = t / steps;
    double matvecCount = 0;
    constexpr double tolerance = 0.5 * std::numeric_limits<double>::epsilon();
    auto& term = term_;
    auto& next = next_;
    for (int step = 0; step < stepCount; ++step) {
      std::copy(state.begin(), state.end(), term.begin());
      double previousNorm = 0.0;
      for (double value : term) {
        previousNorm = std::max(previousNorm, std::abs(value));
      }
      for (int k = 1; k <= degree; ++k) {
        ++matvecCount;
        const double factor = stepTime / k;
        double termNorm = 0.0, sumNorm = 0.0;
        // Dedicated row-major matvec: fuse scaling, accumulation and norm
        // checks into the row loop. term stays unchanged until all rows have
        // been evaluated, so coupled components use the same Taylor term.
        for (sire::Size i = 0; i < n; ++i) {
          const double* row = shifted_.data() + i * n;
          double value = 0.0;
          for (sire::Size j = 0; j < n; ++j) value += row[j] * term[j];
          next[i] = value * factor;
          state[i] += next[i];
          if (!std::isfinite(next[i]) || !std::isfinite(state[i])) {
            throw std::overflow_error("Non-finite Taylor exponential action state");
          }
          termNorm = std::max(termNorm, std::abs(next[i]));
          sumNorm = std::max(sumNorm, std::abs(state[i]));
        }
        term.swap(next);
        // Test two consecutive terms; the selected degree bounds the tail.
        if (previousNorm + termNorm <= tolerance * sumNorm) break;
        previousNorm = termNorm;
      }
      for (double& value : state) {
        value *= traceFactor;
        if (!std::isfinite(value)) {
          throw std::overflow_error("Non-finite exponential action result");
        }
      }
    }
    std::copy(state.begin(), state.end(), res);
    SIRE_PROFILE_PLOT("exp_action.pade_fallback", 0.0);
    SIRE_PROFILE_PLOT("exp_action.matvecs", matvecCount);
  }
}

auto matrix_exp_multiply(sire::Size n, const double* A, const double* v,
                         double* res, double t) -> void {
  MatrixExpMultiplyWorkspace workspace(n, A);
  workspace.apply(v, res, t);
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
