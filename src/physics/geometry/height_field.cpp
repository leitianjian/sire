#include "sire/physics/geometry/height_field.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>
#include <filesystem>

#include <coal/BVH/BVH_model.h>
#include <coal/hfield.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::physics::geometry {

/**
 * 验证文件路径是否满足：
 * 1. 文件后缀为 .png（不区分大小写）
 * 2. 路径存在（且不是目录/设备等）
 * 3. 文件可打开读取
 * 返回 true 表示通过所有检查。
 */
bool validatePngFile(const std::string& filePath_) {
    namespace fs = std::filesystem;
    std::error_code ec;

    // ---------- 1. 后缀检查 ----------
    fs::path p(filePath_);
    std::string ext = p.extension().string();  // 带点号，如 ".png"
    
    // 统一转为小写再比较，实现不区分大小写
    std::string ext_lower = ext;
    std::transform(ext_lower.begin(), ext_lower.end(), ext_lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (ext_lower != ".png") {
        // std::cerr << "文件后缀不是 .png: " << filePath_ << std::endl;
        return false;
    }

    // ---------- 2. 路径存在性检查 ----------
    if (!fs::exists(filePath_, ec)) {
        // std::cerr << "路径不存在";
        // if (ec)
        //     std::cerr << " (" << ec.message() << ")";
        // std::cerr << ": " << filePath_ << std::endl;
        return false;
    }

    // 额外确认：是常规文件（不是目录）
    if (!fs::is_regular_file(filePath_, ec)) {
        // std::cerr << "路径不是常规文件";
        // if (ec)
        //     std::cerr << " (" << ec.message() << ")";
        // std::cerr << ": " << filePath_ << std::endl;
        return false;
    }

    // ---------- 3. 文件可打开（可读）检查 ----------
    // std::ifstream file(filePath_, std::ios::binary);
    // if (!file.is_open()) {
    //     // std::cerr << "文件无法打开: " << filePath_ << std::endl;
    //     return false;
    // }
    // // 可选：立即关闭，仅验证是否可读
    // file.close();

    return true;  // 所有检查通过
}

SIRE_DEFINE_TO_JSON_HEAD(HeightField) {
  GeometryOnPart::to_json(j);
  j["x_dim"] = typedShape.xDim();
  j["y_dim"] = typedShape.yDim();
  j["nrow"] = typedShape.nrow();
  j["ncol"] = typedShape.ncol();
  j["min_height"] = typedShape.minHeight();
  j["heights"] = typedShape.heights();
}

auto HeightField::init() -> void {
  if (validatePngFile(filePath_) && scaleZ_ > 0) {
    typedShape.loadMujocoPNG(filePath_.c_str(), typedShape.xDim() / 2,
                           typedShape.yDim() / 2, scaleZ_);
  }

  // 1. 创建一个列主序的 Eigen 矩阵（注意：矩阵尺寸为 nrow x ncol）
  int nrow = typedShape.nrow(), ncol = typedShape.ncol();
  coal::MatrixXs mat(nrow, ncol);
  auto& heights_ = typedShape.heights();
  for (int r = 0; r < nrow; ++r)
    for (int c = 0; c < ncol; ++c) mat(r, c) = heights_[r * ncol + c];

  resetCollisionObject(new coal::CollisionObject(
      std::make_shared<coal::HeightField<coal::AABB>>(
          typedShape.xDim(), typedShape.yDim(), mat, typedShape.minHeight()),
      getCoalTransform()));
}
HeightField::HeightField(double x_dim, double y_dim, int nrow, int ncol,
                         double min_height, const std::vector<double>& heights,
                         int part_id, bool is_dynamic, const double* prt_pm,
                         const std::string& material,
                         const std::string& propStr)
    : CollisionAdapter(part_id, is_dynamic, prt_pm, material, propStr, x_dim,
                       y_dim, nrow, ncol, min_height, heights) {}
HeightField::~HeightField() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(HeightField)

ARIS_REGISTRATION {
  auto setXDim = [](HeightField* geo, double x_dim) -> void {
    geo->typedShape.setXDim(x_dim);
  };
  auto setYDim = [](HeightField* geo, double y_dim) -> void {
    geo->typedShape.setYDim(y_dim);
  };
  auto getXDim = [](HeightField* geo) -> double {
    return geo->typedShape.xDim();
  };
  auto getYDim = [](HeightField* geo) -> double {
    return geo->typedShape.yDim();
  };

  auto setNRow = [](HeightField* geo, int nrow) -> void {
    geo->typedShape.setNRow(nrow);
  };
  auto getNRow = [](HeightField* geo) -> int {
    return geo->typedShape.nrow();
  };
  auto setNCol = [](HeightField* geo, int ncol) -> void {
    geo->typedShape.setNCol(ncol);
  };
  auto getNCol = [](HeightField* geo) -> int {
    return geo->typedShape.ncol();
  };

  auto setMinHeight = [](HeightField* geo, double min_height) -> void {
    geo->typedShape.setMinHeight(min_height);
  };
  auto getMinHeight = [](HeightField* geo) -> double {
    return geo->typedShape.minHeight();
  };

  aris::core::class_<HeightField>("HeightField")
      .inherit<CollidableGeometry>()
      .prop("x_dim", &setXDim, &getXDim)
      .prop("y_dim", &setYDim, &getYDim)
      // .prop("nrow", &setNRow, &getNRow)
      // .prop("ncol", &setNCol, &getNCol)
      .prop("min_height", &setMinHeight, &getMinHeight)
      .prop("scale_z", &HeightField::setScaleZ, &HeightField::scaleZ)
      .prop("file", &HeightField::setFilePath, &HeightField::filePath)
      ;
}
}  // namespace sire::physics::geometry