//
// Created by ZHOUYC on 2022/8/5.
//

#ifndef ROBOT_OCC_COLLISION_H
#define ROBOT_OCC_COLLISION_H

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "fcl/fcl.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/math/bv/AABB.h"
#include <fcl/math/bv/OBBRSS.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace fcl;

namespace zyc {
    struct TriangleAndVertices {
        std::vector<fcl::Vector3f> vertices_;
        std::vector<fcl::Triangle> triangles_;
    };

    struct  Loader {
        Loader();
        ~Loader();

        void load(const std::string&);

        Assimp::Importer* importer;
        aiScene const* scene;
    };

    void buildMesh(const fcl::Vector3f& scale, const aiScene* scene,
        unsigned vertices_offset,
        TriangleAndVertices& tv);

    void meshFromAssimpScene(
        const fcl::Vector3f& scale, const aiScene* scene,
        const shared_ptr<fcl::BVHModel<OBBRSSf> >& mesh);

    void fclCollision();
    auto InitCollision()->void;
    auto Collision(float x_, float y_, float z_, size_t& num_contacts_)->void;

}//  namespace zyc

#endif //ROBOT_OCC_COLLISION_H
