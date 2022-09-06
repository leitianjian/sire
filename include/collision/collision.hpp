//
// Created by ZHOUYC on 2022/8/5.
//

#ifndef ROBOT_OCC_COLLISION_H
#define ROBOT_OCC_COLLISION_H

#include <iostream>
#include <memory>

#include <assimp/scene.h>           // Output data structure
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/postprocess.h>     // Post processing flags

#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/math/bv/OBBRSS.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/math/bv/AABB.h>

namespace aris_sim {
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
        const std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf> >& mesh);

    void fclCollision();
    auto InitCollision()->void;
    auto Collision(float x_, float y_, float z_, size_t& num_contacts_)->void;

}

#endif //ROBOT_OCC_COLLISION_H
