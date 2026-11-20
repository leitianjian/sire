#pragma once

#include <cmath>

#if defined(__CUDACC__)
#define SIRE_RIGID_BODY_HD __host__ __device__
#else
#define SIRE_RIGID_BODY_HD
#endif

namespace sire::physics::contact::cuda::detail {

// ARIS spatial vectors put linear/force components first, with angular/moment
// components last. Velocities and accelerations refer to the world origin.
SIRE_RIGID_BODY_HD inline void rigidCross3(const double* a, const double* b,
                                          double* out) {
  const double x = a[1] * b[2] - a[2] * b[1];
  const double y = a[2] * b[0] - a[0] * b[2];
  const double z = a[0] * b[1] - a[1] * b[0];
  out[0] = x;
  out[1] = y;
  out[2] = z;
}

SIRE_RIGID_BODY_HD inline void pointVelocity(const double* vs,
                                            const double* point, double* out) {
  double angular_velocity[3];
  rigidCross3(vs + 3, point, angular_velocity);
  for (int i = 0; i < 3; ++i) out[i] = vs[i] + angular_velocity[i];
}

SIRE_RIGID_BODY_HD inline void pointAcceleration(const double* vs,
                                                const double* as,
                                                const double* point,
                                                double* out) {
  double velocity[3], convective[3], angular_acceleration[3];
  pointVelocity(vs, point, velocity);
  rigidCross3(vs + 3, velocity, convective);
  rigidCross3(as + 3, point, angular_acceleration);
  for (int i = 0; i < 3; ++i) {
    out[i] = as[i] + convective[i] + angular_acceleration[i];
  }
}

SIRE_RIGID_BODY_HD inline void crossMotion(const double* vs,
                                          const double* motion, double* out) {
  double linear[3], angular[3], coupled[3];
  rigidCross3(vs + 3, motion, linear);
  rigidCross3(vs + 3, motion + 3, angular);
  rigidCross3(vs, motion + 3, coupled);
  for (int i = 0; i < 3; ++i) {
    out[i] = linear[i] + coupled[i];
    out[i + 3] = angular[i];
  }
}

SIRE_RIGID_BODY_HD inline void crossForce(const double* vs, const double* wrench,
                                         double* out) {
  double force[3], moment[3], coupled[3];
  rigidCross3(vs + 3, wrench, force);
  rigidCross3(vs + 3, wrench + 3, moment);
  rigidCross3(vs, wrench, coupled);
  for (int i = 0; i < 3; ++i) {
    out[i] = force[i];
    out[i + 3] = moment[i] + coupled[i];
  }
}

// The homogeneous bottom row is implicit. All matrices are row-major.
SIRE_RIGID_BODY_HD inline void composePose(const double* a, const double* b,
                                          double* out) {
  double composed[12];
  for (int i = 0; i < 12; i += 4) {
    for (int j = 0; j < 4; ++j) {
      composed[i + j] = a[i] * b[j] + a[i + 1] * b[4 + j] + a[i + 2] * b[8 + j];
    }
    composed[i + 3] += a[i + 3];
  }
  for (int i = 0; i < 12; ++i) out[i] = composed[i];
}

SIRE_RIGID_BODY_HD inline double rigidSinc(double x) {
  return x < 1e-8 ? 1.0 : ::sin(x) / x;
}

SIRE_RIGID_BODY_HD inline void screwPose(const double* ps, double* out) {
  const double x = ps[3], y = ps[4], z = ps[5];
  const double norm_squared = x * x + y * y + z * z;
  const double norm = ::sqrt(norm_squared);
  const double sinc = rigidSinc(norm);
  const double half_sinc = rigidSinc(0.5 * norm);
  const double cosc = 0.5 * half_sinc * half_sinc;

  const double ratio =
      norm < 1e-8 ? 0.0 : (ps[0] * x + ps[1] * y + ps[2] * z) / norm_squared;
  double perpendicular[3];
  rigidCross3(ps + 3, ps, perpendicular);
  double translation[3];
  for (int i = 0; i < 3; ++i) {
    const double parallel = ratio * ps[i + 3];
    translation[i] = parallel + (ps[i] - parallel) * sinc + perpendicular[i] * cosc;
  }

  out[0] = 1.0 - (y * y + z * z) * cosc;
  out[1] = -z * sinc + x * y * cosc;
  out[2] = y * sinc + x * z * cosc;
  out[3] = translation[0];
  out[4] = z * sinc + x * y * cosc;
  out[5] = 1.0 - (x * x + z * z) * cosc;
  out[6] = -x * sinc + y * z * cosc;
  out[7] = translation[1];
  out[8] = -y * sinc + x * z * cosc;
  out[9] = x * sinc + y * z * cosc;
  out[10] = 1.0 - (x * x + y * y) * cosc;
  out[11] = translation[2];
}

// Transform ARIS inertial parameters into a 6x6 inertia about the world origin.
SIRE_RIGID_BODY_HD inline void worldInertia(const double* iv, const double* pose,
                                           double* out) {
  const double mass = iv[0];
  const double p[3] = {pose[3], pose[7], pose[11]};
  const double inertia[9] = {iv[4], iv[7], iv[8], iv[7], iv[5],
                             iv[9], iv[8], iv[9], iv[6]};
  double rotated_first_moment[3], first_moment[3], rotated_inertia[9];
  for (int i = 0; i < 3; ++i) {
    const double* rotation = pose + 4 * i;
    rotated_first_moment[i] =
        rotation[0] * iv[1] + rotation[1] * iv[2] + rotation[2] * iv[3];
    first_moment[i] = rotated_first_moment[i] + mass * p[i];
    for (int j = 0; j < 3; ++j) {
      rotated_inertia[3 * i + j] =
          rotation[0] * inertia[j] + rotation[1] * inertia[3 + j] +
          rotation[2] * inertia[6 + j];
    }
  }

  for (int i = 0; i < 36; ++i) out[i] = 0.0;
  out[0] = out[7] = out[14] = mass;
  const double cx = first_moment[0], cy = first_moment[1], cz = first_moment[2];
  out[4] = out[24] = cz;
  out[5] = out[30] = -cy;
  out[9] = out[19] = -cz;
  out[11] = out[31] = cx;
  out[15] = out[20] = cy;
  out[16] = out[26] = -cx;

  for (int i = 0; i < 3; ++i) {
    const int j = (i + 1) % 3;
    const int k = (i + 2) % 3;
    const double* ri = rotated_inertia + 3 * i;
    const double* row = pose + 4 * i;
    out[21 + 7 * i] = ri[0] * row[0] + ri[1] * row[1] + ri[2] * row[2] +
                      (mass * (p[j] * p[j] + p[k] * p[k]) +
                       2.0 * (rotated_first_moment[j] * p[j] +
                              rotated_first_moment[k] * p[k]));
    for (int col = 0; col < i; ++col) {
      const double* other = pose + 4 * col;
      const double value =
          ri[0] * other[0] + ri[1] * other[1] + ri[2] * other[2] -
          (mass * (p[i] * p[col]) + rotated_first_moment[col] * p[i] +
           rotated_first_moment[i] * p[col]);
      out[21 + 6 * i + col] = out[21 + 6 * col + i] = value;
    }
  }
}

}  // namespace sire::physics::contact::cuda::detail

#undef SIRE_RIGID_BODY_HD
