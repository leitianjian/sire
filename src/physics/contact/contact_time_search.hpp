#pragma once

#include <cfloat>
#include <cmath>

#include "sire/core/constants.hpp"

#ifdef __CUDACC__
#define SIRE_CONTACT_TIME_HD __host__ __device__
#else
#define SIRE_CONTACT_TIME_HD
#endif

namespace sire::physics::contact::detail {

struct ContactTimeSpectrum {
  double rate;
  double frequency;
};

// Eigenvalues determine the sampling grid, while propagation retains the
// entire affine system and its coupling.
struct ContactTimeGrid {
  double max_step;
  double first_step;
  double tolerance;
};

SIRE_CONTACT_TIME_HD inline auto contactTimeGrid(double horizon,
                                                 ContactTimeSpectrum spectrum)
    -> ContactTimeGrid {
  double max_step = horizon / 32;
  if (spectrum.frequency > 0) {
    max_step = ::fmin(max_step, sire::PI / (16 * spectrum.frequency));
  }
  double first_step = max_step;
  if (spectrum.rate > 0) {
    first_step = ::fmin(max_step, 1 / (32 * spectrum.rate));
  }
  // Use a fraction of the current horizon, capped at 10 ns, with a floor
  // for additions performed at the scale of the global simulation clock.
  const double tolerance = ::fmax(
      64 * DBL_EPSILON * ::fmax(1.0, ::fabs(horizon)),
      ::fmin(1e-8, horizon * 1e-4));
  return {max_step, first_step, tolerance};
}

// Keep a chronological bracket for forward-only state propagation and a
// separate Brent-Dekker history ordered by residual.
class ContactTimeBracket {
 public:
  struct Trial {
    double time;
    bool bisected;
  };

  SIRE_CONTACT_TIME_HD ContactTimeBracket(double left, double right,
                                           double left_depth, double right_depth,
                                           double tolerance)
      : left_(left), right_(right), tolerance_(tolerance), a_(left), b_(right),
        fa_(left_depth), fb_(right_depth) {
    orderByResidual();
    c_ = d_ = a_;
    fc_ = fa_;
  }

  SIRE_CONTACT_TIME_HD auto left() const -> double { return left_; }
  SIRE_CONTACT_TIME_HD auto right() const -> double { return right_; }

  SIRE_CONTACT_TIME_HD auto needsRefinement(int iteration) const -> bool {
    return iteration < 80 && right_ - left_ > tolerance_;
  }

  SIRE_CONTACT_TIME_HD auto nextTrial(int iteration) -> Trial {
    const double midpoint = left_ + (right_ - left_) / 2;
    // Reserve the final 48 rounds for bisection. A zero-depth endpoint may
    // be a crossing or a persistent zero, so probe it once without accepting it.
    if (iteration >= 32) return {midpoint, true};
    if (!zero_probe_used_ && (fa_ == 0 || fb_ == 0)) {
      zero_probe_used_ = true;
      const double probe = (fa_ == 0 ? a_ : b_) + tolerance_ / 2;
      if (probe > left_ && probe < right_) return {probe, false};
    }
    if (fa_ == 0 || fb_ == 0 || fa_ == fb_) return {midpoint, true};

    const double scale = ::fmax(::fabs(fa_), ::fmax(::fabs(fb_), ::fabs(fc_)));
    const double va = fa_ / scale;
    const double vb = fb_ / scale;
    const double vc = fc_ / scale;
    double candidate;
    if (va != vc && vb != vc && va != vb) {
      candidate = b_ + (a_ - b_) * vb * vc / ((va - vb) * (va - vc))
                     + (c_ - b_) * va * vb / ((vc - va) * (vc - vb));
    } else {
      candidate = b_ - (b_ - a_) * vb / (vb - va);
    }
    const double guard = a_ + (b_ - a_) / 4;
    const double history = last_was_bisection_ ? ::fabs(b_ - c_) : ::fabs(c_ - d_);
    if (candidate > ::fmin(guard, b_) && candidate < ::fmax(guard, b_) &&
        history > tolerance_ && ::fabs(candidate - b_) < history / 2) {
      // Avoid repeating b due to rounding while keeping a strict bracket.
      const double trial = ::fabs(candidate - b_) < tolerance_ / 2
                               ? b_ + ::copysign(tolerance_ / 2, a_ - b_)
                               : candidate;
      if (trial > left_ && trial < right_) return {trial, false};
    }
    return {midpoint, true};
  }

  SIRE_CONTACT_TIME_HD auto update(Trial trial, double depth) -> void {
    const bool negative = depth < 0;
    if (negative) right_ = trial.time;
    else left_ = trial.time;
    d_ = c_;
    c_ = b_;
    fc_ = fb_;
    if ((fa_ < 0) != negative) {
      b_ = trial.time;
      fb_ = depth;
    } else {
      a_ = trial.time;
      fa_ = depth;
    }
    orderByResidual();
    last_was_bisection_ = trial.bisected;
  }

 private:
  SIRE_CONTACT_TIME_HD auto orderByResidual() -> void {
    if (::fabs(fa_) < ::fabs(fb_)) {
      const double old_a = a_;
      const double old_fa = fa_;
      a_ = b_;
      fa_ = fb_;
      b_ = old_a;
      fb_ = old_fa;
    }
  }

  double left_, right_, tolerance_;
  double a_, b_, fa_, fb_, c_, d_, fc_;
  bool last_was_bisection_{true};
  bool zero_probe_used_{false};
};

}  // namespace sire::physics::contact::detail

#undef SIRE_CONTACT_TIME_HD
