//
// Created by ZHOUYC on 2022/8/5.
//

#ifndef COLLISION_H
#define COLLISION_H

#include <sire_lib_export.h>
#include <assimp/Importer.hpp>   // C++ importer interface
#include <assimp/postprocess.h>  // Post processing flags
#include <assimp/scene.h>        // Output data structure
#include <hpp/fcl/BV/AABB.h>
#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <iostream>

using namespace hpp;

namespace sire {

class SIRE_API Collision {
 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;

 private:
  Collision();
  ~Collision();
  Collision(const std::string& ee_model_path);
  Collision(const Collision&) = delete;
  Collision& operator=(const Collision&) = delete;

 public:
  static auto instance(const std::string& ee_model_path =
                           "./ee.STL") -> Collision&;
  auto CalCollision(std::array<double, 7 * 7> linpq, size_t& num_contacts)
      -> void;
};
}  // namespace sire
#endif  // ROBOT_OCC_COLLISION_H