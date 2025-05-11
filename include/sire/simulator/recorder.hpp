#ifndef SIRE_RECORDER_HPP_
#define SIRE_RECORDER_HPP_
#include <array>
#include <vector>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>
#include "sire/physics/common/point_pair_contact_info.hpp"
#include "sire/core/constants.hpp"

namespace sire::simulator {
/**
 * 1. 假定只有Model中的数据需要被记录以支持仿真回放功能
 * 2. 假定Model中只有ForcePool的结构会发生变化。
 * 3. 假定运动过程中，机器人本身结构不会发生变化，环境不会发生变化
 *
 * 其实需要做的事情与Model的CopyConstructor并无二质，
 * 所以可以写一个接受两个Model指针的方法src复制到dest，然后dest init。
 */
class Record {
 public:
  double timeIndex;
  std::vector<std::array<double, 7>> prtPqs;
  std::vector<std::array<double, 6>> prtVs;
  std::vector<std::array<double, 6>> prtAs;
  std::vector<sire::physics::common::PointPairContactInfo> contactInfos;
};

class Recorder : public aris::core::NamedObject {
 public:
  sire::Size timeDuration;
  sire::Size recordSize;
  std::vector<double> timeIndices;
  std::vector<Record> records;
  auto record(double time, aris::dynamic::Model& model, const std::vector<sire::physics::common::PointPairContactInfo>& contactInfos) -> void;
  auto reset() -> void {
    timeIndices.clear();
    records.clear();
    recordSize = 0;
    timeDuration = 0;
  }
  Recorder() = default;
  virtual ~Recorder() = default;
};
}  // namespace sire::simulator
#endif