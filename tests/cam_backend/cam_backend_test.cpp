#include "sire/cam_backend/cam_backend.hpp"

#include <iostream>

#include <gtest/gtest.h>

#include <aris.hpp>

#include "sire/core/constants.hpp"
using namespace aris::dynamic;
using namespace sire::cam_backend;
GTEST_TEST(CAMBackend, mapAngleToSymRangeTest) {
  double temp = sire::PI / 180;
  std::cout << temp * 181 << " " << temp * 179 << " "
            << mapAngleToSymRange(-temp * 179, sire::PI) << " "
            << mapAngleToSymRange(-temp * 181, sire::PI) << " " << temp * 91
            << " " << temp * 89 << mapAngleToSymRange(-temp * 91, sire::PI / 2)
            << " " << mapAngleToSymRange(-temp * 89, sire::PI / 2) << " "
            << mapAngleToSymRange(temp * 89, sire::PI / 2) << " "
            << mapAngleToSymRange(temp * 91, sire::PI / 2) << " " << std::endl;

  sire::cam_backend::CamBackend backend;
  //backend.init();
}