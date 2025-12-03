#ifndef SIRE_RECORDER_HPP_
#define SIRE_RECORDER_HPP_
#include <array>
#include <vector>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"

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
  double dt;
  std::vector<std::array<double, 7>> prtPqs;
  std::vector<std::array<double, 6>> prtVs;
  std::vector<std::array<double, 6>> prtAs;
  std::vector<sire::physics::common::PointPairContactInfo> contactInfos;
  std::vector<double> interestedData;
};

class Recorder : public aris::core::NamedObject {
 public:
  sire::Size timeDuration;
  sire::Size recordSize;
  std::vector<double> timeIndices;
  std::vector<double> dts;  // 每个记录的时间间隔
  std::vector<Record> records;
  auto addRecord(double time) -> void;
  auto recordModelState(aris::dynamic::Model& model) -> void;
  auto recordDt(double dt) -> void;
  auto recordContactInfo(
      const std::vector<sire::physics::common::PointPairContactInfo>&
          contactInfos) -> void;
  auto setInterestedDataSize(sire::Size size) -> void;
  auto recordInterestedData(sire::Size idx, double data) -> void;
  auto record(double time, double dt, aris::dynamic::Model& model,
              const std::vector<sire::physics::common::PointPairContactInfo>&
                  contactInfos) -> void;
  SIRE_DECLARE_JSON_INTER_TWO
  auto reset() -> void {
    timeIndices.clear();
    dts.clear();
    records.clear();
    recordSize = 0;
    timeDuration = 0;
  }
  Recorder() = default;
  virtual ~Recorder() = default;
};
}  // namespace sire::simulator
#endif