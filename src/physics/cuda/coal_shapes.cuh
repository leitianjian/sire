/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2018-2019, Centre National de la Recherche Scientifique
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

#include <cmath>
#include <limits>

#include <Eigen/Core>

#include "flat_ground_collision.hpp"

namespace sire::physics::contact::cuda::coal_port {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// The primitive support queries and box-sphere distance are from the pinned
// COAL narrowphase: support_functions.cpp, minkowski_difference.cpp and details.h.
// For this scene all swept-sphere radii are zero.
__host__ __device__ inline auto boxSupport(const Vec3& half_extent, const Vec3& dir,
                                          double negative_scale) -> Vec3 {
  constexpr double direction_tolerance = 1e-12;
  const Vec3 positive = (dir.array() > direction_tolerance).select(half_extent, 0);
  const Vec3 negative =
      (dir.array() < -direction_tolerance).select(-negative_scale * half_extent, 0);
  return positive + negative;
}

__host__ __device__ inline auto cylinderSupport(double radius, double half_length,
                                               const Vec3& dir) -> Vec3 {
  constexpr double inflate = 1.0 + 1e-10;
  constexpr double direction_tolerance = 1e-12;
  const bool aligned_z = dir.head<2>().isZero(direction_tolerance);
  if (aligned_z) half_length *= inflate;
  Vec3 support;
  if (dir[2] > direction_tolerance) {
    support[2] = half_length;
  } else if (dir[2] < -direction_tolerance) {
    support[2] = -half_length;
  } else {
    support[2] = 0.0;
    radius *= inflate;
  }
  if (aligned_z) {
    support.head<2>().setZero();
  } else {
    support.head<2>() = dir.head<2>().normalized() * radius;
  }
  return support;
}

struct MinkowskiDifference {
  Vec3 half_extent;
  GroundShape shape;
  Vec3 size;
  Mat3 rotation;
  Vec3 translation;
  bool identity;
  double negative_scale;

  __host__ __device__ auto support(const Vec3& dir, Vec3& first, Vec3& second) const -> void {
    first = boxSupport(half_extent, dir, negative_scale);
    const Vec3 opposite = identity ? Vec3(-dir) : Vec3(-rotation.transpose() * dir);
    second = shape == GroundShape::Box ? boxSupport(size, opposite, negative_scale)
                                       : cylinderSupport(size[0], size[1], opposite);
    if (!identity) second = rotation * second + translation;
  }
};

__host__ __device__ inline auto boxSphereDistance(
    const Vec3& half_extent, const Vec3& box_center, const Vec3& sphere_center, double radius,
    Vec3& point_box, Vec3& point_sphere, Vec3& normal) -> double {
  // The anchored box has an identity rotation. All operations in its frame
  // follow COAL boxSphereDistance, including the nearest-face tie order.
  const Vec3 local = sphere_center - box_center;
  point_box = box_center;
  bool outside = false;
  int axis = -1;
  double min_distance = (std::numeric_limits<double>::max)();
  for (int i = 0; i < 3; ++i) {
    if (local[i] < -half_extent[i]) {
      point_box[i] -= half_extent[i];
      outside = true;
    } else if (local[i] > half_extent[i]) {
      point_box[i] += half_extent[i];
      outside = true;
    } else {
      point_box[i] += local[i];
      const double face_distance = half_extent[i] - ::fabs(local[i]);
      if (!outside && face_distance < min_distance) {
        axis = i;
        min_distance = face_distance;
      }
    }
  }
  normal = point_box - sphere_center;
  const double witness_distance = normal.norm();
  double distance;
  if (outside) {
    distance = witness_distance - radius;
    normal /= -witness_distance;
  } else {
    normal.setZero();
    normal[axis] = local[axis] >= 0.0 ? 1.0 : -1.0;
    distance = -min_distance - radius;
  }
  point_sphere = sphere_center - radius * normal;
  if (!outside || distance <= 0.0) point_box = point_sphere - distance * normal;
  return distance;
}

}  // namespace sire::physics::contact::cuda::coal_port
