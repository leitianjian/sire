#include "sire/core/geometry/shape_base.hpp"

#include <aris/core/reflection.hpp>

namespace sire::geometry {
ShapeBase::~ShapeBase() {}

void ShapeBase::Reify(ShapeCalculator* calculator, void* user_data) const {
  calculator_(*this, calculator, user_data);
}

std::unique_ptr<ShapeBase> ShapeBase::Clone() const { return cloner_(*this); }

// template <typename S>
// ShapeBase::ShapeBase(ShapeTag<S>)
}  // namespace sire::geometry
