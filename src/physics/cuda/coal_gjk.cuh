/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Geometry>

#include <cassert>
#include <cstddef>
#include <limits>

// Port of COAL 6f1194fe417dfae8143dc1d5d0737a5b8508f3be's GJK,
// by Jia Pan and contributors (src/narrowphase/gjk.cpp).
namespace sire::physics::contact::cuda::coal_port {

using Vec3 = Eigen::Vector3d;
using SolverScalar = double;
using Vec3ps = Vec3;

struct MinkowskiDifference;

struct SimplexVertex {
  Vec3 w, w0, w1;
};

template <class Shape>
class Gjk {
 public:
  using vertex_id_t = unsigned;
  using SimplexV = SimplexVertex;
  struct Simplex {
    SimplexV* vertex[4];
    vertex_id_t rank;
  };

  enum Status {
    DidNotRun,
    Failed,
    NoCollisionEarlyStopped,
    NoCollision,
    CollisionWithPenetrationInformation,
    Collision
  };

  EIGEN_DEVICE_FUNC explicit Gjk(std::size_t max_iterations_ = 128,
                                 SolverScalar tolerance_ = 2e-8)
      : max_iterations(max_iterations_), tolerance(tolerance_) {}
  Gjk(const Gjk&) = delete;
  Gjk& operator=(const Gjk&) = delete;

  EIGEN_DEVICE_FUNC Status evaluate(const Shape& shape_, const Vec3& guess);
  EIGEN_DEVICE_FUNC void getSupport(const Vec3& direction, SimplexV& vertex) const {
    shape->support(direction, vertex.w0, vertex.w1);
    vertex.w = vertex.w0 - vertex.w1;
  }
  EIGEN_DEVICE_FUNC bool encloseOrigin() {
    switch (simplex->rank) {
      case 1: return encloseOriginRank<1>();
      case 2: return encloseOriginRank<2>();
      case 3: return encloseOriginRank<3>();
      case 4: return encloseOriginRank<4>();
      default: return false;
    }
  }
  EIGEN_DEVICE_FUNC Simplex* getSimplex() const { return simplex; }
  EIGEN_DEVICE_FUNC SolverScalar getTolerance() const { return tolerance; }

  SolverScalar distance_upper_bound = (std::numeric_limits<SolverScalar>::max)();
  Status status = DidNotRun;
  const Shape* shape = nullptr;
  Vec3 ray;
  SolverScalar distance = 0;
  Simplex* simplex = nullptr;

 private:
  std::size_t max_iterations;
  SolverScalar tolerance;
  SimplexV store_v[4];
  SimplexV* free_v[4];
  vertex_id_t nfree;
  vertex_id_t current;
  Simplex simplices[2];
  std::size_t iterations;

  EIGEN_DEVICE_FUNC void removeVertex(Simplex& simplex) {
    free_v[nfree++] = simplex.vertex[--simplex.rank];
  }
  EIGEN_DEVICE_FUNC void appendVertex(Simplex& simplex, const Vec3& direction) {
    simplex.vertex[simplex.rank] = free_v[--nfree];
    getSupport(direction, *simplex.vertex[simplex.rank++]);
  }
  template <unsigned Rank>
  EIGEN_DEVICE_FUNC bool encloseOriginRank();
  EIGEN_DEVICE_FUNC bool projectLineOrigin(const Simplex& current, Simplex& next);
  EIGEN_DEVICE_FUNC bool projectTriangleOrigin(const Simplex& current, Simplex& next);
  EIGEN_DEVICE_FUNC bool projectTetrahedraOrigin(const Simplex& current, Simplex& next);
};

using GJK = Gjk<MinkowskiDifference>;

// Adapted from COAL src/narrowphase/gjk.cpp. Sire uses DefaultGJK, the
// default convergence criterion, and shapes without a swept-sphere radius.
template <class Shape>
EIGEN_DEVICE_FUNC typename Gjk<Shape>::Status Gjk<Shape>::evaluate(
    const Shape& shape_, const Vec3& guess) {
  SolverScalar alpha = 0;
  iterations = 0;
  free_v[0] = &store_v[0];
  free_v[1] = &store_v[1];
  free_v[2] = &store_v[2];
  free_v[3] = &store_v[3];
  nfree = 4;
  status = NoCollision;
  shape = &shape_;
  distance = 0;
  current = 0;
  simplices[current].rank = 0;

  SolverScalar ray_length = guess.norm();
  if (ray_length < tolerance) {
    ray = Vec3(-1, 0, 0);
    ray_length = 1;
  } else {
    ray = guess;
  }

  do {
    const vertex_id_t next = 1 - current;
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];
    if (ray_length < tolerance) {
      assert(ray_length > 0);
      status = Collision;
      distance = ray_length;
      break;
    }

    appendVertex(curr_simplex, -ray);
    const Vec3& support = curr_simplex.vertex[curr_simplex.rank - 1]->w;
    const SolverScalar omega = ray.dot(support) / ray.norm();
    if (omega > distance_upper_bound) {
      distance = omega;
      status = NoCollisionEarlyStopped;
      break;
    }

    alpha = alpha < omega ? omega : alpha;
    const SolverScalar diff = ray_length - alpha;
    if (iterations > 0 && diff - (tolerance + tolerance * ray_length) <= 0) {
      removeVertex(curr_simplex);
      distance = ray_length;
      status = distance < tolerance ? CollisionWithPenetrationInformation : NoCollision;
      break;
    }

    bool inside = false;
    switch (curr_simplex.rank) {
      case 1:
        assert(iterations == 0);
        ray = support;
        next_simplex.rank = 1;
        next_simplex.vertex[0] = curr_simplex.vertex[0];
        break;
      case 2:
        inside = projectLineOrigin(curr_simplex, next_simplex);
        break;
      case 3:
        inside = projectTriangleOrigin(curr_simplex, next_simplex);
        break;
      case 4:
        inside = projectTetrahedraOrigin(curr_simplex, next_simplex);
        break;
      default:
        assert(false);
        status = Failed;
        simplex = &curr_simplex;
        return status;
    }
    assert(nfree + next_simplex.rank == 4);
    current = next;
    ray_length = ray.norm();
    if (inside || ray_length == 0) {
      status = Collision;
      distance = ray_length;
      break;
    }
    status = (++iterations < max_iterations) ? status : Failed;
  } while (status == NoCollision);

  simplex = &simplices[current];
  assert(simplex->rank > 0 && simplex->rank < 5);
  return status;
}

// Resolve the four possible recursion levels at compile time for device code.
template <class Shape>
template <unsigned Rank>
EIGEN_DEVICE_FUNC bool Gjk<Shape>::encloseOriginRank() {
  Vec3 axis(Vec3::Zero());
  if constexpr (Rank == 1) {
    for (int index = 0; index < 3; ++index) {
      axis[index] = 1;
      appendVertex(*simplex, axis);
      if (encloseOriginRank<2>()) return true;
      removeVertex(*simplex);
      axis[index] = -1;
      appendVertex(*simplex, -axis);
      if (encloseOriginRank<2>()) return true;
      removeVertex(*simplex);
      axis[index] = 0;
    }
  } else if constexpr (Rank == 2) {
    const Vec3 edge = simplex->vertex[1]->w - simplex->vertex[0]->w;
    for (int index = 0; index < 3; ++index) {
      axis[index] = 1;
      const Vec3 perpendicular = edge.cross(axis);
      if (!perpendicular.isZero()) {
        appendVertex(*simplex, perpendicular);
        if (encloseOriginRank<3>()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -perpendicular);
        if (encloseOriginRank<3>()) return true;
        removeVertex(*simplex);
      }
      axis[index] = 0;
    }
  } else if constexpr (Rank == 3) {
    axis.noalias() = (simplex->vertex[1]->w - simplex->vertex[0]->w)
                         .cross(simplex->vertex[2]->w - simplex->vertex[0]->w);
    if (!axis.isZero()) {
      appendVertex(*simplex, axis);
      if (encloseOriginRank<4>()) return true;
      removeVertex(*simplex);
      appendVertex(*simplex, -axis);
      if (encloseOriginRank<4>()) return true;
      removeVertex(*simplex);
    }
  } else {
    return Eigen::numext::abs((simplex->vertex[0]->w - simplex->vertex[3]->w)
                                 .dot((simplex->vertex[1]->w - simplex->vertex[3]->w)
                                          .cross(simplex->vertex[2]->w - simplex->vertex[3]->w))) > 0;
  }
  return false;
}

template <class Simplex>
EIGEN_DEVICE_FUNC inline void originToPoint(const Simplex& current, unsigned a,
                          const Vec3ps& A, Simplex& next, Vec3ps& ray) {
  // A is the closest to the origin
  ray = A;
  next.vertex[0] = current.vertex[a];
  next.rank = 1;
}

template <class Simplex>
EIGEN_DEVICE_FUNC inline void originToSegment(const Simplex& current, unsigned a,
                            unsigned b, const Vec3ps& A,
                            const Vec3ps& B, const Vec3ps& AB,
                            const SolverScalar& ABdotAO, Simplex& next,
                            Vec3ps& ray) {
  // ray = - ( AB ^ AO ) ^ AB = (AB.B) A + (-AB.A) B
  ray = AB.dot(B) * A + ABdotAO * B;

  next.vertex[0] = current.vertex[b];
  next.vertex[1] = current.vertex[a];
  next.rank = 2;

  // To ensure backward compatibility
  ray /= AB.squaredNorm();
}

template <class Simplex>
EIGEN_DEVICE_FUNC inline bool originToTriangle(const Simplex& current, unsigned a,
                             unsigned b, unsigned c,
                             const Vec3ps& ABC, const SolverScalar& ABCdotAO,
                             Simplex& next, Vec3ps& ray) {
  next.rank = 3;
  next.vertex[2] = current.vertex[a];

  if (ABCdotAO == 0) {
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
    ray.setZero();
    return true;
  }
  if (ABCdotAO > 0) {  // Above triangle
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
  } else {
    next.vertex[0] = current.vertex[b];
    next.vertex[1] = current.vertex[c];
  }

  // To ensure backward compatibility
  ray = -ABCdotAO / ABC.squaredNorm() * ABC;
  return false;
}

template <class Shape>
EIGEN_DEVICE_FUNC bool Gjk<Shape>::projectLineOrigin(const Simplex& current, Simplex& next) {
  const vertex_id_t a = 1, b = 0;
  // A is the last point we added.
  const Vec3ps& A = current.vertex[a]->w;
  const Vec3ps& B = current.vertex[b]->w;

  const Vec3ps AB = B - A;
  const SolverScalar d = AB.dot(-A);
  assert(d <= AB.squaredNorm());

  if (d == 0) {
    // Two extremely unlikely cases:
    // - AB is orthogonal to A: should never happen because it means the support
    //   function did not do any progress and GJK should have stopped.
    // - A == origin
    // In any case, A is the closest to the origin
    originToPoint(current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
    return A.isZero();
  } else if (d < 0) {
    // A is the closest to the origin
    originToPoint(current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
  } else
    originToSegment(current, a, b, A, B, AB, d, next, ray);
  return false;
}

template <class Shape>
EIGEN_DEVICE_FUNC bool Gjk<Shape>::projectTriangleOrigin(const Simplex& current, Simplex& next) {
  const vertex_id_t a = 2, b = 1, c = 0;
  // A is the last point we added.
  const Vec3ps &A = current.vertex[a]->w, B = current.vertex[b]->w,
               C = current.vertex[c]->w;

  const Vec3ps AB = B - A, AC = C - A, ABC = AB.cross(AC);

  SolverScalar edgeAC2o = ABC.cross(AC).dot(-A);
  if (edgeAC2o >= 0) {
    SolverScalar towardsC = AC.dot(-A);
    if (towardsC >= 0) {  // Region 1
      originToSegment(current, a, c, A, C, AC, towardsC, next, ray);
      free_v[nfree++] = current.vertex[b];
    } else {  // Region 4 or 5
      SolverScalar towardsB = AB.dot(-A);
      if (towardsB < 0) {  // Region 5
        // A is the closest to the origin
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else  // Region 4
        originToSegment(current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    }
  } else {
    SolverScalar edgeAB2o = AB.cross(ABC).dot(-A);
    if (edgeAB2o >= 0) {  // Region 4 or 5
      SolverScalar towardsB = AB.dot(-A);
      if (towardsB < 0) {  // Region 5
        // A is the closest to the origin
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else  // Region 4
        originToSegment(current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    } else {
      return originToTriangle(current, a, b, c, ABC, ABC.dot(-A), next, ray);
    }
  }
  return false;
}

template <class Shape>
EIGEN_DEVICE_FUNC bool Gjk<Shape>::projectTetrahedraOrigin(const Simplex& current, Simplex& next) {
  // The code of this function was generated using doc/gjk.py
  const vertex_id_t a = 3, b = 2, c = 1, d = 0;
  const Vec3ps& A(current.vertex[a]->w);
  const Vec3ps& B(current.vertex[b]->w);
  const Vec3ps& C(current.vertex[c]->w);
  const Vec3ps& D(current.vertex[d]->w);
  const SolverScalar aa = A.squaredNorm();
  const SolverScalar da = D.dot(A);
  const SolverScalar db = D.dot(B);
  const SolverScalar dc = D.dot(C);
  const SolverScalar dd = D.dot(D);
  const SolverScalar da_aa = da - aa;
  const SolverScalar ca = C.dot(A);
  const SolverScalar cb = C.dot(B);
  const SolverScalar cc = C.dot(C);
  const SolverScalar& cd = dc;
  const SolverScalar ca_aa = ca - aa;
  const SolverScalar ba = B.dot(A);
  const SolverScalar bb = B.dot(B);
  const SolverScalar& bc = cb;
  const SolverScalar& bd = db;
  const SolverScalar ba_aa = ba - aa;
  const SolverScalar ba_ca = ba - ca;
  const SolverScalar ca_da = ca - da;
  const SolverScalar da_ba = da - ba;
  const Vec3ps a_cross_b = A.cross(B);
  const Vec3ps a_cross_c = A.cross(C);

  [[maybe_unused]] constexpr SolverScalar dummy_precision =
      3 * 1.490116119384765625e-8;

#define REGION_INSIDE()               \
  ray.setZero();                      \
  next.vertex[0] = current.vertex[d]; \
  next.vertex[1] = current.vertex[c]; \
  next.vertex[2] = current.vertex[b]; \
  next.vertex[3] = current.vertex[a]; \
  next.rank = 4;                      \
  return true;

  // clang-format off
  if (ba_aa <= 0) {                // if AB.AO >= 0 / a10
    if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / a10.a3
      if (ba * da_ba + bd * ba_aa - bb * da_aa <=
          0) {             // if (ADB ^ AB).AO >= 0 / a10.a3.a9
        if (da_aa <= 0) {  // if AD.AO >= 0 / a10.a3.a9.a12
          assert(da * da_ba + dd * ba_aa - db * da_aa <=
                 dummy_precision);  // (ADB ^ AD).AO >= 0 / a10.a3.a9.a12.a8
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
              0) {  // if (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.a4
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          } else {  // not (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.!a4
            // Region AB
            originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AB).AO >= 0
        } else {  // not AD.AO >= 0 / a10.a3.a9.!a12
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
              0) {  // if (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.a4
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
                0) {  // if (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5
              if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                  0) {  // if (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.a6
                // Region ACD
                originToTriangle(current, a, c, d, (C - A).cross(D - A),
                                 -D.dot(a_cross_c), next, ray);
                free_v[nfree++] = current.vertex[b];
              } else {  // not (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.!a6
                // Region AC
                originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              }  // end of (ACD ^ AC).AO >= 0
            } else {  // not (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.!a5
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            }  // end of (ABC ^ AC).AO >= 0
          } else {  // not (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.!a4
            // Region AB
            originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AB).AO >= 0
        }  // end of AD.AO >= 0
      } else {  // not (ADB ^ AB).AO >= 0 / a10.a3.!a9
        if (da * da_ba + dd * ba_aa - db * da_aa <=
            0) {  // if (ADB ^ AD).AO >= 0 / a10.a3.!a9.a8
          // Region ADB
          originToTriangle(current, a, d, b, (D - A).cross(B - A),
                           D.dot(a_cross_b), next, ray);
          free_v[nfree++] = current.vertex[c];
        } else {  // not (ADB ^ AD).AO >= 0 / a10.a3.!a9.!a8
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.!a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.!a7
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AD).AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        }  // end of (ADB ^ AD).AO >= 0
      }  // end of (ADB ^ AB).AO >= 0
    } else {                        // not ADB.AO >= 0 / a10.!a3
      if (C.dot(a_cross_b) <= 0) {  // if ABC.AO >= 0 / a10.!a3.a1
        if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
            0) {  // if (ABC ^ AB).AO >= 0 / a10.!a3.a1.a4
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                0) {  // if (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.!a6
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AC).AO >= 0
          } else {  // not (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.!a5
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AC).AO >= 0
        } else {  // not (ABC ^ AB).AO >= 0 / a10.!a3.a1.!a4
          // Region AB
          originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
          free_v[nfree++] = current.vertex[c];
          free_v[nfree++] = current.vertex[d];
        }  // end of (ABC ^ AB).AO >= 0
      } else {                        // not ABC.AO >= 0 / a10.!a3.!a1
        if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / a10.!a3.!a1.a2
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {             // not (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.!a6
            if (ca_aa <= 0) {  // if AC.AO >= 0 / a10.!a3.!a1.a2.!a6.a11
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {  // not AC.AO >= 0 / a10.!a3.!a1.a2.!a6.!a11
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of AC.AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        } else {  // not ACD.AO >= 0 / a10.!a3.!a1.!a2
          // Region Inside
          REGION_INSIDE()
        }  // end of ACD.AO >= 0
      }  // end of ABC.AO >= 0
    }  // end of ADB.AO >= 0
  } else {                          // not AB.AO >= 0 / !a10
    if (ca_aa <= 0) {               // if AC.AO >= 0 / !a10.a11
      if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / !a10.a11.a2
        if (da_aa <= 0) {           // if AD.AO >= 0 / !a10.a11.a2.a12
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7
              if (da * da_ba + dd * ba_aa - db * da_aa <=
                  0) {  // if (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.a8
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {  // not (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.!a8
                // Region AD
                originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }  // end of (ADB ^ AD).AO >= 0
            } else {  // not (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6
            assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                     -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                          // !a10.a11.a2.a12.!a6.!a7
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
                0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.a5
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.!a5
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            }  // end of (ABC ^ AC).AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        } else {  // not AD.AO >= 0 / !a10.a11.a2.!a12
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                0) {  // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.a6
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                       -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                            // !a10.a11.a2.!a12.a5.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.!a6
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AC).AO >= 0
          } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.!a5
            if (C.dot(a_cross_b) <=
                0) {  // if ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.a1
              assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                     dummy_precision);  // (ABC ^ AB).AO >= 0 /
                                        // !a10.a11.a2.!a12.!a5.a1.a4
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            } else {  // not ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.!a1
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                       -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                            // !a10.a11.a2.!a12.!a5.!a1.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of ABC.AO >= 0
          }  // end of (ABC ^ AC).AO >= 0
        }  // end of AD.AO >= 0
      } else {                        // not ACD.AO >= 0 / !a10.a11.!a2
        if (C.dot(a_cross_b) <= 0) {  // if ABC.AO >= 0 / !a10.a11.!a2.a1
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.a5
            // Region AC
            originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
            free_v[nfree++] = current.vertex[b];
            free_v[nfree++] = current.vertex[d];
          } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.!a5
            assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                   dummy_precision);  // (ABC ^ AB).AO >= 0 /
                                      // !a10.a11.!a2.a1.!a5.a4
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AC).AO >= 0
        } else {                         // not ABC.AO >= 0 / !a10.a11.!a2.!a1
          if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / !a10.a11.!a2.!a1.a3
            if (da * da_ba + dd * ba_aa - db * da_aa <=
                0) {  // if (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.a8
              // Region ADB
              originToTriangle(current, a, d, b, (D - A).cross(B - A),
                               D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.!a8
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of (ADB ^ AD).AO >= 0
          } else {  // not ADB.AO >= 0 / !a10.a11.!a2.!a1.!a3
            // Region Inside
            REGION_INSIDE()
          }  // end of ADB.AO >= 0
        }  // end of ABC.AO >= 0
      }  // end of ACD.AO >= 0
    } else {                           // not AC.AO >= 0 / !a10.!a11
      if (da_aa <= 0) {                // if AD.AO >= 0 / !a10.!a11.a12
        if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / !a10.!a11.a12.a3
          if (da * ca_da + dc * da_aa - dd * ca_aa <=
              0) {  // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7
            if (da * da_ba + dd * ba_aa - db * da_aa <=
                0) {  // if (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.a8
              assert(!(ba * da_ba + bd * ba_aa - bb * da_aa <=
                       -dummy_precision));  // Not (ADB ^ AB).AO >= 0 /
                                            // !a10.!a11.a12.a3.a7.a8.!a9
              // Region ADB
              originToTriangle(current, a, d, b, (D - A).cross(B - A),
                               D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.!a8
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of (ADB ^ AD).AO >= 0
          } else {  // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.!a7
            if (D.dot(a_cross_c) <=
                0) {  // if ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.a2
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <=
                     dummy_precision);  // (ACD ^ AC).AO >= 0 /
                                        // !a10.!a11.a12.a3.!a7.a2.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2
              if (C.dot(a_cross_b) <=
                  0) {  // if ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.a1
                assert(!(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                         -dummy_precision));  // Not (ABC ^ AB).AO >= 0 /
                                              // !a10.!a11.a12.a3.!a7.!a2.a1.!a4
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {  // not ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.!a1
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              }  // end of ABC.AO >= 0
            }  // end of ACD.AO >= 0
          }  // end of (ACD ^ AD).AO >= 0
        } else {                        // not ADB.AO >= 0 / !a10.!a11.a12.!a3
          if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / !a10.!a11.a12.!a3.a2
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.!a7
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <=
                     dummy_precision);  // (ACD ^ AC).AO >= 0 /
                                        // !a10.!a11.a12.!a3.a2.!a7.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not ACD.AO >= 0 / !a10.!a11.a12.!a3.!a2
            // Region Inside
            REGION_INSIDE()
          }  // end of ACD.AO >= 0
        }  // end of ADB.AO >= 0
      } else {  // not AD.AO >= 0 / !a10.!a11.!a12
        // Region A
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
        free_v[nfree++] = current.vertex[c];
        free_v[nfree++] = current.vertex[d];
      }  // end of AD.AO >= 0
    }  // end of AC.AO >= 0
  }  // end of AB.AO >= 0
  // clang-format on

#undef REGION_INSIDE
  return false;
}

}  // namespace sire::physics::contact::cuda::coal_port
