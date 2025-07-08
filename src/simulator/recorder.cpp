#include "sire/simulator/recorder.hpp"

#include "sire/physics/common/point_pair_contact_info.hpp"
namespace sire::simulator {
auto Recorder::record(
    double time, double dt, aris::dynamic::Model& model,
    const std::vector<sire::physics::common::PointPairContactInfo>&
        contactInfos) -> void {
  if (model.forwardDynamics()) {
    std::cout << "Model forward dynamics failed." << std::endl;
  }
  auto& prtPool = model.partPool();
  sire::Size prtSize = prtPool.size();
  Record cr;
  cr.timeIndex = time;
  cr.dt = dt;
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
  dts.push_back(dt);
}
SIRE_DEFINE_TO_JSON_HEAD(Recorder) {
  nlohmann::json part_pq_json = nlohmann::json::array();  // 外层数组，长度=records.size()
  nlohmann::json part_vs_json = nlohmann::json::array();  // 外层数组，长度=records.size()
  nlohmann::json part_as_json = nlohmann::json::array();  // 外层数组，长度=records.size()
  nlohmann::json contact_info_json = nlohmann::json::array();  // 外层数组，长度=records.size()
  for (const auto& record : records) {  // 遍历每个 Record
    // 处理 partPqs 数据
    nlohmann::json prt_pq_array;  // 每个 Record 的 prtPqs 数组
    for (const auto& pq : record.prtPqs) {  // 遍历 prtPqs 中的每个 std::array
      prt_pq_array.push_back(std::vector<double>(pq.begin(), pq.end()));
    }
    part_pq_json.push_back(prt_pq_array);  // 将当前 Record 数据加入外层数组

    // 处理 partVs 数据
    nlohmann::json prt_vs_array;  // 每个 Record 的 prtVs 数组
    for (const auto& vs : record.prtVs) {  // 遍历 prtVs 中的每个 std::array
      prt_vs_array.push_back(std::vector<double>(vs.begin(), vs.end()));
    }
    part_vs_json.push_back(prt_vs_array);  // 将当前 Record 数据加入外层数组

    // 处理 partAs 数据
    nlohmann::json prt_as_array;  // 每个 Record 的 prtAs 数组
    for (const auto& as : record.prtAs) {  // 遍历 prtAs 中的每个 std::array
      prt_as_array.push_back(std::vector<double>(as.begin(), as.end()));
    }
    part_as_json.push_back(prt_as_array);  // 将当前 Record 数据加入外层数组

    // 处理 contactInfos 数据
    nlohmann::json contact_info_array;  // 每个 Record 的 contactInfos 数组
    for (const auto& contact : record.contactInfos) {  // 遍历 contactInfos 中的每个 PointPairContactInfo
      nlohmann::json contactJson;  // 用于存储每个 PointPairContactInfo 的 JSON 对象
      contact.to_json(contactJson);
      contact_info_array.push_back(contactJson);
    }
    contact_info_json.push_back(contact_info_array);  // 将当前 Record 的 contactInfos 数据加入外层数组
  }

  j["partPq"] = part_pq_json;  // 添加 partpq 数据
  j["partVs"] = part_vs_json;  // 添加 partpq 数据
  j["partAs"] = part_as_json;  // 添加 partpq 数据
  j["timeIndex"] = timeIndices;  // 添加 timeindex 数据
  j["dts"] = dts;  // 添加 timeindex 数据
  j["contactInfo"] = contact_info_json;  // 添加 contact_info 数据
}
}  // namespace sire::simulator