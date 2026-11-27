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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided
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
#include <cstdint>
#include <limits>

#include "coal_gjk.cuh"

// Port of Coal 6f1194fe417dfae8143dc1d5d0737a5b8508f3be's EPA in
// src/narrowphase/gjk.cpp and include/coal/narrowphase/gjk.h, with witness
// projection from include/coal/internal/intersect.hxx. Face indices replace
// pointers; the explicit stack preserves the recursive expansion order.
namespace sire::physics::contact::cuda::coal_port {

using EpaVector = Vec3;

enum class EpaStatus {
  Failed = 0,
  Valid = 1,
  AccuracyReached = 3,
  Degenerated = 2,
  NonConvex = 4,
  InvalidHull = 6,
  OutOfFaces = 8,
  OutOfVertices = 10,
  FallBack = 12,
};

struct EpaResult {
  EpaStatus status;
  double depth;
  EpaVector normal, w0, w1;
};

struct EpaWorkspace {
  static constexpr int max_iterations = 64;
  static constexpr int max_vertices = max_iterations + 4;
  static constexpr int max_faces = 2 * max_iterations + 4;

  struct Face {
    EpaVector n;
    double d;
    std::int16_t previous, next, adjacent_faces[3];
    std::uint8_t vertex_id[3], adjacent_edge[3], pass;
    bool ignore;
  };

  struct Frame {
    std::int16_t face;
    std::uint8_t edge, state;
  };

  SimplexVertex vertices[max_vertices];
  Face faces[max_faces];
  Frame stack[max_faces];
};

class EpaSolver {
 public:
  EIGEN_DEVICE_FUNC explicit EpaSolver(EpaWorkspace& workspace)
      : workspace_(workspace) {}

  template <typename Shape>
  EIGEN_DEVICE_FUNC auto evaluate(Gjk<Shape>& gjk, const EpaVector& guess) -> EpaResult {
    const auto& simplex = *gjk.getSimplex();
    const bool enclosed = gjk.encloseOrigin();
    if (!enclosed || simplex.rank != 4) return fallback(simplex, guess);

    stock_ = 0;
    hull_ = none;
    hull_count_ = 0;
    vertex_count_ = 4;
    status_ = EpaStatus::Valid;
    for (int i = 0; i < EpaWorkspace::max_faces; ++i) {
      workspace_.faces[i].previous = i - 1;
      workspace_.faces[i].next = i + 1;
    }
    workspace_.faces[EpaWorkspace::max_faces - 1].next = none;

    const auto& a = simplex.vertex[0]->w;
    const auto& b = simplex.vertex[1]->w;
    const auto& c = simplex.vertex[2]->w;
    const auto& d = simplex.vertex[3]->w;
    const bool swap = (a - d).dot((b - d).cross(c - d)) < 0;
    for (int i = 0; i < 4; ++i) {
      const auto& vertex = *simplex.vertex[swap && i < 2 ? 1 - i : i];
      workspace_.vertices[i] = vertex;
    }

    const int tetrahedron[]{newFace(0, 1, 2, true), newFace(1, 0, 3, true),
                            newFace(2, 1, 3, true), newFace(0, 2, 3, true)};
    if (hull_count_ != 4) return fallback(simplex, guess);
    bind(tetrahedron[0], 0, tetrahedron[1], 0);
    bind(tetrahedron[0], 1, tetrahedron[2], 0);
    bind(tetrahedron[0], 2, tetrahedron[3], 0);
    bind(tetrahedron[1], 1, tetrahedron[3], 2);
    bind(tetrahedron[1], 2, tetrahedron[2], 1);
    bind(tetrahedron[2], 2, tetrahedron[3], 1);

    int closest_face = findClosestFace();
    if (closest_face == none) return fallback(simplex, guess);
    Face outer = workspace_.faces[closest_face];
    int iteration = 0;
    for (; iteration < EpaWorkspace::max_iterations; ++iteration) {
      if (vertex_count_ >= EpaWorkspace::max_vertices) {
        status_ = EpaStatus::OutOfVertices;
        break;
      }
      Horizon horizon;
      const int pass = iteration + 1;
      auto& w = workspace_.vertices[vertex_count_++];
      auto& closest = workspace_.faces[closest_face];
      closest.pass = pass;
      gjk.getSupport(closest.n, w);
      const auto& vf1 = workspace_.vertices[closest.vertex_id[0]];
      const auto& vf2 = workspace_.vertices[closest.vertex_id[1]];
      const auto& vf3 = workspace_.vertices[closest.vertex_id[2]];
      const double fdist = closest.n.dot(w.w - vf1.w);
      const double limit = tolerance + tolerance * w.w.norm();
      if (fdist <= limit || (w.w - vf1.w).norm() <= limit ||
          (w.w - vf2.w).norm() <= limit || (w.w - vf3.w).norm() <= limit) {
        status_ = EpaStatus::AccuracyReached;
        break;
      }

      bool valid = true;
      for (int edge = 0; edge < 3 && valid; ++edge) {
        valid = expand(pass, w, closest.adjacent_faces[edge],
                       closest.adjacent_edge[edge], horizon);
      }
      if (!valid || horizon.count < 3) {
        if (valid) status_ = EpaStatus::InvalidHull;
        break;
      }
      bind(horizon.first, 2, horizon.current, 1);
      remove(hull_, closest_face);
      --hull_count_;
      append(stock_, closest_face);
      closest_face = findClosestFace();
      if (closest_face == none) {
        status_ = EpaStatus::InvalidHull;
        break;
      }
      outer = workspace_.faces[closest_face];
    }
    if (iteration == EpaWorkspace::max_iterations) status_ = EpaStatus::Failed;

    EpaResult result{status_, outer.d, outer.n,
                     EpaVector::Zero(), EpaVector::Zero()};
    const auto& v0 = workspace_.vertices[outer.vertex_id[0]];
    const auto& v1 = workspace_.vertices[outer.vertex_id[1]];
    const auto& v2 = workspace_.vertices[outer.vertex_id[2]];
    const EpaVector weights = triangleWeights(v0.w, v1.w, v2.w);
    for (int i = 0; i < 3; ++i) {
      const auto& vertex = workspace_.vertices[outer.vertex_id[i]];
      result.w0 += weights[i] * vertex.w0;
      result.w1 += weights[i] * vertex.w1;
    }
    if ((result.w0 - result.w1).norm() >
        Eigen::NumTraits<double>::dummy_precision()) {
      if (result.depth >= 0) result.normal = (result.w0 - result.w1).normalized();
      else result.normal = (result.w1 - result.w0).normalized();
    }
    return result;
  }

 private:
  using Face = EpaWorkspace::Face;
  static constexpr int none = -1;
  static constexpr double tolerance = 1e-6;

  struct Horizon {
    int current = none;
    int first = none;
    int count = 0;
  };

  template <typename Simplex>
  EIGEN_DEVICE_FUNC static auto fallback(const Simplex& simplex,
                                          const EpaVector& guess) -> EpaResult {
    EpaResult result{EpaStatus::FallBack, 0, -guess, EpaVector::Zero(),
                     EpaVector::Zero()};
    const double norm = result.normal.norm();
    if (norm > 0) result.normal /= norm;
    else result.normal = EpaVector(1, 0, 0);
    if (simplex.rank > 0) {
      result.w0 = simplex.vertex[0]->w0;
      result.w1 = simplex.vertex[0]->w1;
    }
    return result;
  }

  EIGEN_DEVICE_FUNC void append(int& root, int id) {
    auto& face = workspace_.faces[id];
    face.previous = none;
    face.next = root;
    if (root != none) workspace_.faces[root].previous = id;
    root = id;
  }

  EIGEN_DEVICE_FUNC void remove(int& root, int id) {
    const auto& face = workspace_.faces[id];
    if (face.next != none) workspace_.faces[face.next].previous = face.previous;
    if (face.previous != none) workspace_.faces[face.previous].next = face.next;
    if (id == root) root = face.next;
  }

  EIGEN_DEVICE_FUNC void bind(int first, int first_edge, int second,
                                int second_edge) {
    auto& a = workspace_.faces[first];
    auto& b = workspace_.faces[second];
    a.adjacent_edge[first_edge] = second_edge;
    a.adjacent_faces[first_edge] = second;
    b.adjacent_edge[second_edge] = first_edge;
    b.adjacent_faces[second_edge] = first;
  }

  EIGEN_DEVICE_FUNC auto newFace(int id_a, int id_b, int id_c,
                                  bool force = false) -> int {
    const int id = stock_;
    if (id == none) {
      status_ = EpaStatus::OutOfFaces;
      return none;
    }
    remove(stock_, id);
    append(hull_, id);
    ++hull_count_;
    auto& face = workspace_.faces[id];
    face.pass = 0;
    face.vertex_id[0] = id_a;
    face.vertex_id[1] = id_b;
    face.vertex_id[2] = id_c;
    const auto& a = workspace_.vertices[id_a].w;
    const auto& b = workspace_.vertices[id_b].w;
    const auto& c = workspace_.vertices[id_c].w;
    face.n = (b - a).cross(c - a);
    if (face.n.norm() > Eigen::NumTraits<double>::epsilon()) {
      face.n.normalize();
      const double a_dot_nab = a.dot((b - a).cross(face.n));
      const double b_dot_nbc = b.dot((c - b).cross(face.n));
      const double c_dot_nca = c.dot((a - c).cross(face.n));
      face.ignore = !(a_dot_nab >= -tolerance && b_dot_nbc >= -tolerance &&
                      c_dot_nca >= -tolerance);
      face.d = face.ignore ? (std::numeric_limits<double>::max)() : a.dot(face.n);
      if (face.d >= -tolerance || force) return id;
      status_ = EpaStatus::NonConvex;
    } else {
      status_ = EpaStatus::Degenerated;
    }
    remove(hull_, id);
    --hull_count_;
    append(stock_, id);
    return none;
  }

  EIGEN_DEVICE_FUNC auto findClosestFace() const -> int {
    int closest = none;
    double distance = (std::numeric_limits<double>::max)();
    for (int id = hull_; id != none; id = workspace_.faces[id].next) {
      const auto& face = workspace_.faces[id];
      if (face.ignore) continue;
      const double squared = face.d * face.d;
      if (squared < distance) {
        closest = id;
        distance = squared;
      }
    }
    return closest;
  }

  EIGEN_DEVICE_FUNC auto expand(int pass, const SimplexVertex& w, int face,
                                 int edge, Horizon& horizon) -> bool {
    auto* stack = workspace_.stack;
    int size = 1;
    stack[0] = {static_cast<std::int16_t>(face), static_cast<std::uint8_t>(edge), 0};
    const double precision = 3 * ::sqrt(std::numeric_limits<double>::epsilon());
    while (size > 0) {
      auto& frame = stack[size - 1];
      auto& current = workspace_.faces[frame.face];
      const int next = (frame.edge + 1) % 3;
      const int previous = (frame.edge + 2) % 3;
      if (frame.state == 0) {
        if (current.pass == pass) {
          status_ = EpaStatus::InvalidHull;
          return false;
        }
        const auto& vf = workspace_.vertices[current.vertex_id[frame.edge]];
        if (current.n.dot(w.w - vf.w) < precision) {
          const int new_face = newFace(current.vertex_id[next],
                                       current.vertex_id[frame.edge], vertex_count_ - 1);
          if (new_face == none) return false;
          bind(new_face, 0, frame.face, frame.edge);
          if (horizon.current != none) bind(new_face, 2, horizon.current, 1);
          else horizon.first = new_face;
          horizon.current = new_face;
          ++horizon.count;
          --size;
          continue;
        }
        current.pass = pass;
      }
      if (frame.state < 2) {
        const int child = frame.state++ == 0 ? next : previous;
        if (size == EpaWorkspace::max_faces) {
          status_ = EpaStatus::InvalidHull;
          return false;
        }
        stack[size++] = {current.adjacent_faces[child], current.adjacent_edge[child], 0};
      } else {
        remove(hull_, frame.face);
        --hull_count_;
        append(stock_, frame.face);
        --size;
      }
    }
    return true;
  }

  EIGEN_DEVICE_FUNC static auto triangleWeights(const EpaVector& a,
                                                 const EpaVector& b,
                                                 const EpaVector& c) -> EpaVector {
    const EpaVector* vertex[]{&a, &b, &c};
    const EpaVector delta[]{a - b, b - c, c - a};
    const EpaVector normal = delta[0].cross(delta[1]);
    const double length = normal.squaredNorm();
    EpaVector weights = EpaVector::Zero();
    double distance = -1;
    if (length > 0) {
      for (int i = 0; i < 3; ++i) {
        if (vertex[i]->dot(delta[i].cross(normal)) > 0) {
          const int j = (i + 1) % 3;
          const EpaVector line = *vertex[j] - *vertex[i];
          const double squared = line.squaredNorm();
          const double t = -vertex[i]->dot(line);
          const double u = t >= squared ? 1 : (t <= 0 ? 0 : t / squared);
          const double candidate = t >= squared ? vertex[j]->squaredNorm()
              : (t <= 0 ? vertex[i]->squaredNorm()
                        : (*vertex[i] + line * u).squaredNorm());
          if (distance < 0 || candidate < distance) {
            distance = candidate;
            weights[i] = 1 - u;
            weights[j] = u;
            weights[(j + 1) % 3] = 0;
          }
        }
      }
      if (distance < 0) {
        const double d = a.dot(normal);
        const double scale = ::sqrt(length);
        const EpaVector projection = normal * (d / length);
        weights[0] = delta[1].cross(b - projection).norm() / scale;
        weights[1] = delta[2].cross(c - projection).norm() / scale;
        weights[2] = 1 - weights[0] - weights[1];
      }
    }
    return weights;
  }

  EpaWorkspace& workspace_;
  EpaStatus status_;
  int stock_, hull_, hull_count_, vertex_count_;
};

}  // namespace sire::physics::contact::cuda::coal_port
