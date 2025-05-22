#include "sire/simulator/recorder.hpp"

#include "sire/physics/common/point_pair_contact_info.hpp"
namespace sire::simulator {
auto Recorder::record(
    double time, aris::dynamic::Model& model,
    const std::vector<sire::physics::common::PointPairContactInfo>&
        contactInfos) -> void {
  auto& prtPool = model.partPool();
  sire::Size prtSize = prtPool.size();
  Record cr;
  cr.timeIndex = time;
  cr.prtPqs.resize(prtSize);
  cr.prtVs.resize(prtSize);
  cr.prtAs.resize(prtSize);
  for (sire::Size i{0}; i < prtSize; ++i) {
    auto& prt = prtPool[i];
    prt.getPq(cr.prtPqs[i].data());
    prt.getVs(cr.prtVs[i].data());
    prt.getAs(cr.prtAs[i].data());
  }
  cr.contactInfos = contactInfos;
  records.push_back(std::move(cr));
  timeIndices.push_back(time);
}
SIRE_DEFINE_TO_JSON_HEAD(Recorder) {
  nlohmann::json part_pq_json = nlohmann::json::array();  // 外层数组，长度=records.size()
  nlohmann::json contact_info_json = nlohmann::json::array();  // 外层数组，长度=records.size()
  for (const auto& record : records) {  // 遍历每个 Record
    // 处理 partPqs 数据
    nlohmann::json prt_pq_array;  // 每个 Record 的 prtPqs 数组
    for (const auto& pq : record.prtPqs) {  // 遍历 prtPqs 中的每个 std::array
      prt_pq_array.push_back(std::vector<double>(pq.begin(), pq.end()));
    }
    part_pq_json.push_back(prt_pq_array);  // 将当前 Record 数据加入外层数组

    // 处理 contactInfos 数据
    nlohmann::json contact_info_array;  // 每个 Record 的 contactInfos 数组
    for (const auto& contact : record.contactInfos) {  // 遍历 contactInfos 中的每个 PointPairContactInfo
      nlohmann::json contact_json;
      contact_json["partId_A"] = contact.partId_A();
      contact_json["partId_B"] = contact.partId_B();
      contact_json["contact_force"] = std::vector<double>(contact.contact_force(), contact.contact_force() + 6);
      contact_json["contact_point_pe"] = std::vector<double>(contact.contact_point_pe(), contact.contact_point_pe() + 6);
      contact_json["separation_speed"] = contact.separation_speed();
      contact_json["slip_speed"] = contact.slip_speed();
      contact_json["contact_force_vector"] = std::vector<double>(contact.contact_force_vector(), contact.contact_force_vector() + 3);
      contact_info_array.push_back(contact_json);
    }
    contact_info_json.push_back(contact_info_array);  // 将当前 Record 的 contactInfos 数据加入外层数组
  }

  j["partpq"] = part_pq_json;             // 添加 partpq 数据
  j["timeindex"] = timeIndices;  // 添加 timeindex 数据
  j["contact_info"] = contact_info_json;  // 添加 contact_info 数据
}
}  // namespace sire::simulator