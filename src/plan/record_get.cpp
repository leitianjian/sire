#include "sire/plan/record_get.hpp"

#include <array>
#include <iostream>
#include <tuple>
#include <vector>

#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
// #include "sire/ext/tinyxml2.h"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/server/interface.hpp"
#include "sire/simulator/recorder.hpp"

namespace sire::plan {
struct GetParam {
  std::vector<std::vector<double>> part_pq;
  std::vector<std::vector<double>> part_vs;
  std::vector<double> motors_a;
  std::vector<double> motors_f;
  std::vector<double> motors_v;
  std::vector<double> motors_p;
  int state_code;
  bool is_cs_started;
  std::string currentPlan;
  int currentPlanId;
};

void printVector(const std::vector<sire::simulator::Record>& vec) {
  for (const auto& record : vec) {
    std::cout << "Time Index: " << record.timeIndex << "\n";

    std::cout << "prtPqs: [\n";
    for (const auto& pq : record.prtPqs) {
      std::cout << "  [ ";
      for (double val : pq) {
        std::cout << val << " ";
      }
      std::cout << "]\n";
    }
    std::cout << "]\n";

    std::cout << "prtVs: [\n";
    for (const auto& v : record.prtVs) {
      std::cout << "  [ ";
      for (double val : v) {
        std::cout << val << " ";
      }
      std::cout << "]\n";
    }
    std::cout << "]\n";

    std::cout << "prtAs: [\n";
    for (const auto& a : record.prtAs) {
      std::cout << "  [ ";
      for (double val : a) {
        std::cout << val << " ";
      }
      std::cout << "]\n";
    }
    std::cout << "]\n";

    std::cout << "----------------------\n";
  }
}

auto RecordGet::prepareNrt() -> void {
  option() |=
      NOT_PRINT_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  auto& cs = *controlServer();
  auto& middleware = dynamic_cast<middleware::SireMiddleware&>(cs.middleWare());
  auto& simulator = middleware.simulationLoop();
  auto& recorder = simulator.recorder();

  nlohmann::json part_pq_json = nlohmann::json::array();  // 外层数组，长度=records.size()
  nlohmann::json contact_info_json = nlohmann::json::array();  // 外层数组，长度=records.size()

  for (const auto& record : recorder.records) {  // 遍历每个 Record
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
      contact_info_array.push_back(contact_json);
    }
    contact_info_json.push_back(contact_info_array);  // 将当前 Record 的 contactInfos 数据加入外层数组
  }

  std::vector<std::pair<std::string, std::any>> out_param;
  ret() = out_param;

  nlohmann::json output_json;
  output_json["partpq"] = part_pq_json;             // 添加 partpq 数据
  output_json["timeindex"] = recorder.timeIndices;  // 添加 timeindex 数据
  output_json["contact_info"] = contact_info_json;  // 添加 contact_info 数据

  // 将 JSON 数据写入文件
  std::ofstream outfile("record.json");  // 创建并打开文件
  if (outfile.is_open()) {
    outfile << output_json.dump(4) << std::endl;
  } else {
    std::cerr << "Fail to open file" << std::endl;
  }
}

// auto RecordGet::collectNrt() -> void {}

RecordGet::RecordGet(const std::string& name) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"record_get\">"
                            "</Command>");
}

ARIS_REGISTRATION {
  aris::core::class_<RecordGet>("RecordGet").inherit<aris::plan::Plan>();
}
}  // namespace sire::plan