#include "sire/simulator/recorder.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"
namespace sire::simulator {
auto Recorder::record(double time, aris::dynamic::Model& model, const std::vector<sire::physics::common::PointPairContactInfo>& contactInfos) -> void {
  auto& prtPool = model.partPool();
  sire::Size prtSize = prtPool.size();
  Record cr;
  cr.timeIndex = time;
  cr.prtPqs.resize(prtSize);
  cr.prtVs.resize(prtSize);
  cr.prtAs.resize(prtSize);
  for (sire::Size i{0}; i < prtSize; ++ i) {
    auto& prt = prtPool[i];
    prt.getPq(cr.prtPqs[i].data());
    prt.getVs(cr.prtVs[i].data());
    prt.getAs(cr.prtAs[i].data());
  }
  cr.contactInfos = contactInfos;
  records.push_back(std::move(cr));
  timeIndices.push_back(time);
}
}  // namespace sire::Simulator