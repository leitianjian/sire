#ifndef SIRE_SHAPE_BASE_HPP_
#define SIRE_SHAPE_BASE_HPP_

#include <typeinfo>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/geometry/shape_calculator.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {

/** Simple struct for instantiating the type-specific Shape functionality.
 A class derived from the Shape class will invoke the parent's constructor as
 Shape(ShapeTag<DerivedShape>()). */
template <typename ShapeType>
struct ShapeTag {};

/** The base interface for all shape specifications. It has no public
  constructor and cannot be instantiated directly. The Shape class has two
  key properties:

   - it is cloneable, and
   - it can be "reified" (see ShapeCalculator).

  When you add a new subclass of Shape to Drake, you must:

  1. add a virtual function ImplementGeometry() for the new shape in
     ShapeCalculator that invokes the ThrowUnsupportedGeometry method, and add
 to the test for it in shape_specification_test.cc.
  2. implement ImplementGeometry in derived ShapeCalculator to continue support
     if desired, otherwise ensure unimplemented functions are not hidden in new
     derivations of ShapeCalculator with `using`, for example, `using
     ShapeCalculator::ImplementGeometry`. Existing subclasses should already
 have this.

  Otherwise, you might get a runtime error. We do not have an automatic way to
  enforce them at compile time.

 Note that the Shape class hierarchy is closed to third-party extensions. All
 Shape classes must be defined within Drake directly (and in this h/cc file
 pair in particular).
 */
class ShapeBase {
 public:
  virtual ~ShapeBase();

  /** Causes this description to be reified in the given `calculator`. Each
   concrete subclass must invoke the single, matching method on the calculator.
   Provides optional user-data (cast as a void*) for the calculator to consume.
 */
  void Reify(ShapeCalculator* calculator, void* user_data = nullptr) const;

  /** Creates a unique copy of this shape. */
  std::unique_ptr<ShapeBase> Clone() const;

 protected:
  // This is *not* in the public section. However, this allows the children to
  // also use this macro, but precludes the possibility of external users
  // slicing Shapes.
  ARIS_DEFINE_BIG_FOUR(ShapeBase)

  /** Constructor available for derived class construction. A derived class
   should invoke this in its initialization list, passing a ShapeTag
   instantiated on its derived type, e.g.:

   ```
   class MyShape final : public Shape {
    public:
     MyShape() : Shape(ShapeTag<MyShape>()) {}
     ...
   };
   ```

   The base class provides infrastructure for cloning and reification. To work
   and to maintain sanity, we place the following requirements on derived
   classes:

   1. they must have a public copy constructor,
   2. they must be marked as final, and
   3. their constructors must invoke the parent constructor with a ShapeTag
      instance (as noted above), and
   4. The ShapeCalculator class must be extended to include an invocation of
      ShapeCalculator::ImplementGeometry() on the derived Shape class.

   @tparam S    The derived shape class. It must derive from Shape. */
  template <typename S>
  explicit ShapeBase(ShapeTag<S> tag) {
    static_assert(std::is_base_of_v<ShapeBase, S>,
                  "Concrete shapes *must* be derived from the Shape class");
    cloner_ = [](const ShapeBase& shape_arg) {
      SIRE_DEMAND(typeid(shape_arg) == typeid(S));
      const S& derived_shape = static_cast<const S&>(shape_arg);
      return std::unique_ptr<ShapeBase>(new S(derived_shape));
    };
    calculator_ = [](const ShapeBase& shape_arg, ShapeCalculator* calculator,
                     void* user_data) {
      SIRE_DEMAND(typeid(shape_arg) == typeid(S));
      const S& derived_shape = static_cast<const S&>(shape_arg);
      calculator->ImplementGeometry(derived_shape, user_data);
    };
  }

 private:
  std::function<std::unique_ptr<ShapeBase>(const ShapeBase&)> cloner_;
  std::function<void(const ShapeBase&, ShapeCalculator*, void*)> calculator_;
};

// enum ShapeType { GEOM_GENERAL, GEOM_BOX, GEOM_SPHERE, GEOM_MESH };

// NLOHMANN_JSON_SERIALIZE_ENUM(ShapeType, {{GEOM_GENERAL, "general"},
//                                          {GEOM_BOX, "box"},
//                                          {GEOM_SPHERE, "sphere"},
//                                          {GEOM_MESH, "mesh"}})

// class SIRE_API ShapeBase {
//  private:
//   ShapeType type_;

//  public:
//   auto shapeType() -> ShapeType& { return type_; };
//   auto shapeType() const -> const ShapeType { return type_; };
//   auto setShapeType(ShapeType type_in) -> void { type_ = type_in; };
//   explicit ShapeBase(ShapeType type = ShapeType::GEOM_GENERAL) :
//   type_(type){};
// };
}  // namespace sire::geometry

#endif