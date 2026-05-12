#ifndef SIRE_HEIGHT_FIELD_SHAPE_HPP_
#define SIRE_HEIGHT_FIELD_SHAPE_HPP_

#include <stdexcept>
#include <vector>

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class HeightFieldShape final : public ShapeBase {
 private:
  double x_dim_{1.0};
  double y_dim_{1.0};
  int nrow_{1};
  int ncol_{1};
  double min_height_{0};
  std::vector<double> heights_{0};

 public:
  auto setXDim(double x_dim) -> void { x_dim_ = x_dim; };
  auto setYDim(double y_dim) -> void { y_dim_ = y_dim; };
  auto xDim() const -> double { return x_dim_; };
  auto yDim() const -> double { return y_dim_; };

  auto setHeights(int nrow, int ncol, const std::vector<double>& heights)
      -> void;
  auto heights() const -> const std::vector<double>& { return heights_; };
  auto heights() -> std::vector<double>& {
    return const_cast<std::vector<double>&>(
        static_cast<const HeightFieldShape*>(this)->heights());
  };
  auto setNRow(int nrow) -> void;
  auto nrow() const -> int;
  auto setNCol(int ncol) -> void;
  auto ncol() const -> int;
  auto setMinHeight(double min_height) -> void { min_height_ = min_height; };
  auto minHeight() const -> double { return min_height_; };
  auto loadMujocoPNG(const char* png_path, double half_x, double half_y,
                     double scale_z) -> void;

  explicit HeightFieldShape(double x_dim = 1.0, double y_dim = 1.0,
                            int nrow = 1, int ncol = 1, double min_height = 0,
                            const std::vector<double>& heights = {0});
  virtual ~HeightFieldShape();
};
}  // namespace sire::geometry

#endif
