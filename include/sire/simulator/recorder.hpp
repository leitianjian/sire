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
#include "sire/ext/json.hpp"

namespace sire::simulator {

/// Per-contact-pair solver result — lightweight, Python-friendly.
struct ContactPairResult {
  sire::PartId geomIdA;
  sire::PartId geomIdB;
  double force_W[3];   ///< world-frame contact force (N)
  double point_W[3];   ///< world-frame contact point (m)

  auto to_json(nlohmann::json& j) const -> void {
    j["geomIdA"] = geomIdA;
    j["geomIdB"] = geomIdB;
    j["force_W"] = {force_W[0], force_W[1], force_W[2]};
    j["point_W"] = {point_W[0], point_W[1], point_W[2]};
  }
};

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
  std::vector<double> singleComponentForces;
  std::vector<std::array<double, 6>> generalForces;
  std::vector<sire::physics::common::PointPairContactInfo> contactInfos;
  std::vector<sire::physics::common::PenetrationAsPointPair> penetrationPairs;
  std::vector<ContactPairResult> contactPairResults;  ///< per-pair forces in world frame
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
          contactInfos,
      sire::Size n) -> void;
  auto recordPenetrationPairs(
      const std::vector<sire::physics::common::PenetrationAsPointPair>&
          penetrationPairs) -> void;
  auto recordContactPairResults(
      const std::vector<ContactPairResult>& results) -> void;
  auto setHistoryEnabled(bool enabled) -> void {
    if (history_enabled_ == enabled) return;
    history_enabled_ = enabled;
    reset();
  }
  auto historyEnabled() const noexcept -> bool { return history_enabled_; }
  auto latestContactPairResults() const noexcept
      -> const std::vector<ContactPairResult>& {
    return latest_contact_pair_results_;
  }
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
    latest_contact_pair_results_.clear();
    recordSize = 0;
    timeDuration = 0;
  }
  Recorder() = default;
  virtual ~Recorder() = default;

 private:
  // Full model snapshots are needed only for playback/visualization.  RL
  // workers can disable them and retain just the latest lightweight contact
  // result, avoiding allocator growth across long multi-threaded runs.
  bool history_enabled_{true};
  std::vector<ContactPairResult> latest_contact_pair_results_;
};
}  // namespace sire::simulator
#endif
