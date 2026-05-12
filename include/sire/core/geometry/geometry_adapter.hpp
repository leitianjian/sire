#ifndef SIRE_GEOMETRY_ADAPTER_HPP_
#define SIRE_GEOMETRY_ADAPTER_HPP_

#include <aris/core/reflection.hpp>
#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/core/geometry/shape_calculator.hpp"
#include "sire/core/sire_decl_def_macro.hpp"

namespace sire::geometry {
/**
 * A generic adapter that wraps any ShapeBase into a GeometryOnPart.
 * This eliminates the need to rewrite GeometryOnPart subclasses for every new shape.
 * 
 * Usage for a new shape, e.g., MyShape:
 * class MyGeometry : public GeometryAdapter<MyGeometry, MyShape> {
 * public:
 *   // Expose constructors, register reflection, etc.
 *   using GeometryAdapter::GeometryAdapter;
 * };
 */
template <typename Derived, typename ShapeType>
class GeometryAdapter : public GeometryOnPart {
 public:
  ShapeType typedShape;
  
  auto virtual shape() -> ShapeBase* override { return &typedShape; }
  
  template <typename... Args>
  explicit GeometryAdapter(int part_id = 0, bool is_dynamic = false,
                           const double* prt_pm = nullptr, Args&&... args)
      : GeometryOnPart(prt_pm, part_id, is_dynamic),
        typedShape(std::forward<Args>(args)...) {}

  virtual ~GeometryAdapter() = default;

  // We explicitly define to_json here so derived classes share this logic.
  // Note: We don't use SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO directly 
  // to avoid friend injection issues in templates without specifying the type.
  auto to_json(nlohmann::json& j) const -> void override {
    GeometryOnPart::to_json(j);
    ShapeToName cal;
    typedShape.Reify(&cal);
    j["shape_type"] = cal.string();
    
    // For specific properties, derived classes should override to_json 
    // or we rely purely on reflection to dump properties automatically if Aris supports it.
    // If not, derived classes can override:
    // void to_json(nlohmann::json& j) const override { 
    //   GeometryAdapter::to_json(j); 
    //   j["my_prop"] = typedShape.my_prop(); 
    // }
  }

  // To allow nlohmann::json j = o;
  friend auto to_json(nlohmann::json& j, const Derived& o) -> void {
    o.to_json(j);
  }
};

}  // namespace sire::geometry
#endif
