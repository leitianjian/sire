//
// Created by ZHOUYC on 2022/8/5.
//

#ifndef ROBOT_OCC_COLLISION_H
#define ROBOT_OCC_COLLISION_H

#include <aris_sim_lib_export.h>
#include <assimp/Importer.hpp>   // C++ importer interface
#include <assimp/postprocess.h>  // Post processing flags
#include <assimp/scene.h>        // Output data structure
#include <hpp/fcl/BV/AABB.h>
#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <iostream>
#include <memory>

using namespace hpp;

namespace aris_sim {
struct TriangleAndVertices {
  std::vector<fcl::Vec3f> vertices_;
  std::vector<fcl::Triangle> triangles_;
};

struct Loader {
  Loader(const std::string& resource_path);
  ~Loader();

  void load(const std::string& resource_path);

  Assimp::Importer* importer;
  aiScene const* scene;
};

void meshFromAssimpScene(
    const fcl::Vec3f& scale, const aiScene* scene,
    const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>>& mesh);

auto ARIS_SIM_API InitCollision(const std::string& resource_path) -> void;
auto ARIS_SIM_API Collision(float x_, float y_, 
                            float z_, size_t& num_contacts_) -> void;
}  // namespace aris_sim

#endif  // ROBOT_OCC_COLLISION_H
