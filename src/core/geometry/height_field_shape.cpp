#include "sire/core/geometry/height_field_shape.hpp"

#include <algorithm>

#include <aris/core/reflection.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "sire/ext/stb_image.h"

namespace sire::geometry {

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
auto HeightFieldShape::loadMujocoPNG(const char* png_path, double half_x,
                                     double half_y, double scale_z) -> void {
  // 1. 读取 PNG 并归一化到 [0,1]
  int width, height, channels;
  unsigned char* data = stbi_load(png_path, &width, &height, &channels, 0);
  if (!data) throw std::runtime_error("Failed to load PNG");

  int ncol = width;   // X 方向点数
  int nrow = height;  // Y 方向点数

  std::vector<double> norm(nrow * ncol);
  for (int i = 0; i < nrow * ncol; ++i) {
    if (channels >= 3) 
      norm[i] = data[i * channels] / 255.0;  // 取红色通道
    else 
      norm[i] = data[i] / 255.0;  // 灰度
  }
  stbi_image_free(data);

  // 3. 构建绝对高度矩阵 (行主序 → Eigen 默认列主序，逐元素赋值)
  // CoalHeightFieldParams result;
  x_dim_ = 2.0 * half_x;
  y_dim_ = 2.0 * half_y;
  nrow_ = nrow;
  ncol_ = ncol;

  heights_.resize(nrow, ncol);
  for (int r = 0; r < nrow; ++r) 
    for (int c = 0; c < ncol; ++c) 
      heights_[r * ncol + c] = norm[r * ncol + c] * scale_z;
}
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
