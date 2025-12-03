#ifndef SIRE_BOX_SHAPE_HPP_
#define SIRE_BOX_SHAPE_HPP_

#include <array>

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
/** Definition of a box. The box is centered on the origin of its canonical
 frame with its dimensions aligned with the frame's axes. The size of the box
 is given by three sizes. */
class BoxShape final : public ShapeBase {
 public:
  ARIS_DECLARE_BIG_FOUR(BoxShape)

  /** Constructs a box with the given `width`, `depth`, and `height`, which
   specify the box's dimension along the canonical x-, y-, and z-axes,
   respectively.
   @throws std::exception if `width`, `depth` or `height` are not strictly
   positive. */
  BoxShape(double length, double width, double height);

  /** Constructs a box with a vector of measures: width, depth, and height --
   the box's dimensions along the canonical x-, y-, and z-axes, respectively.
   @throws std::exception if the measures are not strictly positive. */
  explicit BoxShape(const double* measures);
  virtual ~BoxShape();

  /** Constructs a cube with the given `edge_size` for its width, depth, and
   height. */
  static BoxShape MakeCube(double edge_size);

  /** Returns the box's dimension along the x axis. */
  double length() const { return side_[0]; }

  /** Returns the box's dimension along the y axis. */
  double width() const { return side_[1]; }

  /** Returns the box's dimension along the z axis. */
  double height() const { return side_[2]; }

  auto setSide(double* side_in) -> void;
  auto setSide(double length, double width, double height) -> void;
  /** Returns the box's dimensions. */
  auto side() const -> const double* { return side_; };
  auto length() -> double&;
  // auto virtual length() const -> double;
  auto width() -> double&;
  // auto virtual width() const -> double;
  auto height() -> double&;
  // auto virtual height() const -> double;

 private:
  double side_[3];
};

// class SIRE_API BoxShape : public ShapeBase {
//  private:
//   std::array<double, 3> side_{0.1, 0.1, 0.1};

//  public:
//   auto virtual setSide(double* side_in) -> void;
//   auto virtual setSide(double length, double width, double height) -> void;
//   auto virtual sidePtr() const -> const double*;
//   auto virtual side() const -> const std::array<double, 3>;
//   auto virtual length() -> double&;
//   auto virtual length() const -> double;
//   auto virtual width() -> double&;
//   auto virtual width() const -> double;
//   auto virtual height() -> double&;
//   auto virtual height() const -> double;

//   explicit BoxShape(double length = 0.1, double width = 0.1,
//                     double height = 0.1);
//   explicit BoxShape(double* side_in);
//   virtual ~BoxShape();
// };
}  // namespace sire::geometry

#endif