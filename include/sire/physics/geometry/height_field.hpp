#ifndef SIRE_COLLISION_HEIGHT_FIELD_HPP_
#define SIRE_COLLISION_HEIGHT_FIELD_HPP_

#include "sire/core/geometry/height_field_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/geometry/collision_adapter.hpp"

namespace sire::physics::geometry {
class HeightField
    : public CollisionAdapter<HeightField, sire::geometry::HeightFieldShape> {
 private:
  double scaleZ_ {1};
  std::string filePath_ {""};

 public:
  auto init() -> void override;
  explicit HeightField(double x_dim = 1, double y_dim = 1, int nrow = 1,
                       int ncol = 1, double min_height = 0,
                       const std::vector<double>& heights = {0},
                       int part_id = 0, bool is_dynamic = false,
                       const double* prt_pm = nullptr,
                       const std::string& material = "m1",
                       const std::string& propStr = "{}");
  virtual ~HeightField();
  SIRE_DECLARE_MOVE_CTOR(HeightField)
  auto filePath() const -> std::string { return filePath_; }
  auto setFilePath(const std::string& filePath) -> void {
    filePath_ = filePath;
  }
  auto scaleZ() const -> double { return scaleZ_; }
  auto setScaleZ(double scaleZ) -> void { scaleZ_ = scaleZ; }

  // 类内部使用的to_json from_json的声明
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO

  // nlohammn::json j = o;的时候会自动调用的to_json from_json的声明
  SIRE_DECLARE_JSON_FRIEND_TWO(HeightField)
};
}  // namespace sire::physics::geometry
#endif