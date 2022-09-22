//
// Created by ZHOUYC on 2022/8/5.
//

#ifndef ROBOT_OCC_COLLISION_H
#define ROBOT_OCC_COLLISION_H

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
auto SIRE_API InitCollision(const std::string& resource_path) -> void;
auto SIRE_API Collision(float x_, float y_, float z_, size_t& num_contacts_)
    -> void;
}  // namespace sire

#endif  // ROBOT_OCC_COLLISION_H
