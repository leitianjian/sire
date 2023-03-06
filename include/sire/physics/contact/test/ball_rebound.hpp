//
// Created by ZHOUYC on 2023/3/4.
//

#ifndef SIRE_BALL_REBOUND_HPP
#define SIRE_BALL_REBOUND_HPP

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <sire_lib_export.h>

namespace sire::physics::contact {
// using namespace hpp;
class SIRE_API BallRebound {
 public:
  static auto instance(const std::string& robot_stl_path = "./")
      -> BallRebound&;
  auto CalContact(std::array<double, 7>&, size_t&) -> void;
  auto GetFileName(const std::string& path, std::vector<std::string>& files)
      -> void;
  auto ContactSolver(std::array<double,7>,std::array<double,7>) -> std::array<double, 7>;

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
  BallRebound();
  ~BallRebound();
  BallRebound(const std::string& ee_model_path);
  BallRebound(const BallRebound&) = delete;
  BallRebound& operator=(const BallRebound&) = delete;
};
}  // namespace sire::physics::contact

#endif	
