//
// Created by ZHOUYC on 2022/8/5.
//

#ifndef SIRE_COLLISION_HPP_
#define SIRE_COLLISION_HPP_

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

namespace sire::physics::collision {
// using namespace hpp;
class SIRE_API Collision {
 public:
  static auto instance(const std::string& robot_stl_path = "./") -> Collision&;
  auto CalCollision(std::array<double, 7 * 7> linpq, size_t& num_contacts)
      -> void;
  auto GetFileName(const std::string& path, std::vector<std::string>& files)
      -> void;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
  Collision();
  ~Collision();
  Collision(const std::string& ee_model_path);
  Collision(const Collision&) = delete;
  Collision& operator=(const Collision&) = delete;
};
}  // namespace sire::physics::collision
#endif  // COLLISION_H