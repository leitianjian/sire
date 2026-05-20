#include "sire/core/geometry/height_field_shape.hpp"

#include <algorithm>
#include <stdexcept>

#include "aris/dynamic/math_matrix.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "sire/ext/stb_image.h"

namespace sire::geometry {

// Set and Access
auto HeightFieldShape::setHeights(int nrow, int ncol,
                                  const std::vector<double>& heights) -> void {
  nrow_ = nrow;
  ncol_ = ncol;
  heights_ = heights;
}
auto HeightFieldShape::setNRow(int nrow) -> void { nrow_ = nrow; }
auto HeightFieldShape::nrow() const -> int { return nrow_; }
auto HeightFieldShape::setNCol(int ncol) -> void { ncol_ = ncol; }
auto HeightFieldShape::ncol() const -> int { return ncol_; }
// Load from MuJoCo PNG
void HeightFieldShape::loadMujocoPNG(const char* png_path, double half_x,
                                     double half_y, double scale_z) {
  // 1. Load image
  int width, height, channels;
  stbi_uc* img = stbi_load(png_path, &width, &height, &channels, 0);
  if (!img) {
    throw std::runtime_error("Failed to load PNG: " + std::string(png_path));
  }

  // 2. Extract grayscale data
  ncol_ = width;
  nrow_ = height;
  std::vector<float> raw(nrow_ * ncol_);
  // coal from left top is same to mujoco, so just invert in meshcat vis;
  for (int r = 0; r < nrow_; ++r) {
    // int src_row = nrow_ - 1 - r;
    for (int c = 0; c < ncol_; ++c) {
      int idx = r * ncol_ + c;
      raw[r * ncol_ + c] = static_cast<float>(img[idx * channels]);
    }
  }
  stbi_image_free(img);

  // 3. Min-max normalization
  auto result = std::minmax_element(raw.begin(), raw.end());
  float min_val = *result.first;
  float max_val = *result.second;
  if (max_val >= min_val) {
    for (float& val : raw) {
      val = (val - min_val) / (max_val - min_val);
    }
  } else {
    for (float& val : raw) {
      val = (val - min_val);
    }
  }

  // 4. Store absolute heights
  x_dim_ = 2.0 * half_x;
  y_dim_ = 2.0 * half_y;
  heights_.resize(nrow_ * ncol_);
  for (int i = 0; i < nrow_ * ncol_; ++i) {
    heights_[i] = static_cast<double>(raw[i]) * scale_z;
  }
  // std::cout << "HeightFieldShape loaded from PNG: " << png_path
  //           << ", size: " << nrow_ << "x" << ncol_ << ", min_val: " << min_val
  //           << ", max_val: " << max_val << std::endl;
  // aris::dynamic::dsp(nrow_, ncol_, heights_.data());
}
// Constructor and Destructor
HeightFieldShape::HeightFieldShape(double x_dim, double y_dim, int nrow,
                                   int ncol, double min_height,
                                   const std::vector<double>& heights)
    : ShapeBase(ShapeTag<HeightFieldShape>()),
      x_dim_(x_dim),
      y_dim_(y_dim),
      nrow_(nrow),
      ncol_(ncol),
      min_height_(min_height),
      heights_(heights) {}

HeightFieldShape::~HeightFieldShape() = default;

}  // namespace sire::geometry