//
// Created by ZHOUYC on 2022/8/5.
//
#include "aris_sim/collision/collision.hpp"
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <mutex>
#include <thread>

using namespace hpp;

namespace aris_sim {
static std::thread collision_thread_;
static std::mutex collision_mutex_;
static float x{0.0}, y{0.0}, z{0.0};
static int num_contacts{0};

Loader::Loader() : importer(new Assimp::Importer()) {
  // set list of ignored parameters (parameters used for rendering)
  importer->SetPropertyInteger(
      AI_CONFIG_PP_RVC_FLAGS,
      aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
          aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
          aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_TEXTURES |
          aiComponent_TEXCOORDS | aiComponent_MATERIALS | aiComponent_NORMALS);

  // remove LINES and POINTS
  importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                               aiPrimitiveType_LINE | aiPrimitiveType_POINT);
}

Loader::~Loader() {
  if (importer) delete importer;
}

void Loader::load(const std::string& resource_path) {
  scene = importer->ReadFile(
      resource_path.c_str(),
      aiProcess_SortByPType | aiProcess_Triangulate |
          aiProcess_RemoveComponent | aiProcess_ImproveCacheLocality |
          aiProcess_FindDegenerates | aiProcess_JoinIdenticalVertices);

  if (!scene) {
    const std::string exception_message(
        std::string("Could not load resource ") + resource_path +
        std::string("\n") + importer->GetErrorString() + std::string("\n") +
        "Hint: the mesh directory may be wrong.");
    throw std::invalid_argument(exception_message);
  }

  if (!scene->HasMeshes())
    throw std::invalid_argument(std::string("No meshes found in file ") +
                                resource_path);
}

unsigned recurseBuildMesh(const fcl::Vec3f& scale, const aiScene* scene,
                          const aiNode* node, unsigned int vertices_offset,
                          TriangleAndVertices& tv) {
  if (!node) return 0;

  aiMatrix4x4 transform = node->mTransformation;
  aiNode* pnode = node->mParent;
  while (pnode) {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != nullptr) {
      transform = pnode->mTransformation * transform;
    }
    pnode = pnode->mParent;
  }

  unsigned nbVertices = 0;
  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
      aiVector3D p = input_mesh->mVertices[j];
      //          p *= transform;
      tv.vertices_.push_back(
          fcl::Vec3f(p.x * scale[0], p.y * scale[1], p.z * scale[2]));
    }

    // add the indices
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace& face = input_mesh->mFaces[j];
      assert(face.mNumIndices == 3 && "The size of the face is not valid.");

      tv.triangles_.push_back(
          fcl::Triangle(static_cast<std::size_t>(vertices_offset) + face.mIndices[0],
                        static_cast<std::size_t>(vertices_offset) + face.mIndices[1],
                        static_cast<std::size_t>(vertices_offset) + face.mIndices[2]));
    }
    nbVertices += input_mesh->mNumVertices;
  }

  for (uint32_t i = 0; i < node->mNumChildren; ++i) {
    nbVertices +=
        recurseBuildMesh(scale, scene, node->mChildren[i], nbVertices, tv);
  }
  return nbVertices;
}

void buildMesh(const fcl::Vec3f& scale, const aiScene* scene,
               unsigned vertices_offset, TriangleAndVertices& tv) {
  recurseBuildMesh(scale, scene, scene->mRootNode, vertices_offset, tv);
}

void meshFromAssimpScene(
    const fcl::Vec3f& scale, const aiScene* scene,
    const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>>& mesh) {
  TriangleAndVertices tv;

  mesh->beginModel();

  buildMesh(scale, scene, (unsigned)mesh->num_vertices, tv);
  mesh->addSubModel(tv.vertices_, tv.triangles_);

  mesh->endModel();
}

auto InitCollision() -> void {
  if (collision_thread_.joinable()) {
    return;
  } else {
    collision_thread_ = std::thread([]() {
      // Configure robot_ee geometry.
      Loader scene;
      scene.load("C:/Users/ZHOUYC/Desktop/ee_4.stl");
      typedef fcl::BVHModel<fcl::OBBRSS> Model;
      std::shared_ptr<Model> geom = std::make_shared<Model>();
      fcl::Vec3f scale{1, 1, 1};
      meshFromAssimpScene(scale, scene.scene, geom);
      geom->computeLocalAABB();
      fcl::Transform3f X_WBV = fcl::Transform3f::Identity();

      // Configure sphere geometry.
      const fcl::FCL_REAL r = 30;
      auto sphere_geometry = std::make_shared<fcl::Sphere>(r);
      // Poses of the geometry.
      fcl::Transform3f X_WS = fcl::Transform3f::Identity();
      X_WS.translation() << 167, 0, 0;
      fcl::CollisionObject Sphere(sphere_geometry, X_WS);
      sphere_geometry->computeLocalAABB();

      // thread
      while (num_contacts == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        X_WBV.translation() << -5.1 + x, -20.2 + y, -20.2 + z;
        //      X_WBV.linear() << 0,0,1, 0,1,0, -1,0,0;//y axis 90
        auto Robot = new fcl::CollisionObject(geom, X_WBV);

        // Compute collision - single contact and enable contact.
        fcl::CollisionRequest Request(fcl::CollisionRequestFlag::CONTACT, 1);
        // Request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        fcl::CollisionResult Result;
        // Calculate collision
        num_contacts = fcl::collide(&Sphere, Robot, Request, Result);
        std::vector<fcl::Contact> contacts;
        Result.getContacts(contacts);

        // set the distance request structure, here we just use the default
        // setting
        fcl::DistanceRequest request;
        // result will be returned via the collision result structure
        fcl::DistanceResult result;
        // perform distance test
        fcl::distance(&Sphere, Robot, request, result);
        if (num_contacts) {
          std::cout << "num " << contacts.size() << " contacts found"
                    << std::endl;
          for (const fcl::Contact& contact : contacts) {
            std::cout << "collision position at : " << contact.pos[0] << " "
                      << contact.pos[1] << " " << contact.pos[2] << std::endl;
            std::cout << "min_distance: " << result.min_distance << std::endl;
          }
        }
        delete Robot;
      }
    });
  }
}

auto Collision(float x_, float y_, float z_, size_t& num_contacts_) -> void {
  std::lock_guard<std::mutex> guard(collision_mutex_);

  x = x_;
  y = y_;
  z = z_;
  num_contacts_ = num_contacts;
};

}  // namespace aris_sim
