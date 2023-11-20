#include "sire/cam_backend/cam_backend.hpp"

#include <iostream>

#include <gtest/gtest.h>

#include <aris.hpp>

#include "sire/core/constants.hpp"
using namespace aris::dynamic;
using namespace sire::cam_backend;
GTEST_TEST(CAMBackend, mapAngleToSymRangeTest) {
  std::cout << sire::PI / 6 << " " << sire::PI / 2 << " "
            << mapAngleToSymRange(sire::PI / 6, sire::PI / 2) << std::endl;
  // sire::cam_backend::CamBackend backend();
}