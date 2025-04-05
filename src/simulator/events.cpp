#include "sire/simulator/events.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <set>

#include "log/easyloggingConfig.hpp"

#include <aris/core/log.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/screw.hpp>

#include "sire/core/event_manager.hpp"
#include "sire/core/sire_fixed_joint.hpp"
#include "sire/ext/json.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/utils.hpp"

namespace sire::simulator {
using namespace sire::physics;
auto process_impact_threshold(aris::dynamic::Model* m,
                              double impact_threshold_insert,
                              double impact_threshold_remove)
    -> std::pair<double, double> {
  double min_mass = std::numeric_limits<double>::infinity();
  for (auto& part : m->partPool()) {
    min_mass = min_mass < part.prtIv()[0] ? min_mass : part.prtIv()[0];
  }
  min_mass = min_mass <= 0 ? 1.0 : min_mass;
  double max_allow_velocity_change_dt_insert = 5000.0;
  double impact_threshold_1 = min_mass * max_allow_velocity_change_dt_insert;
  impact_threshold_insert = impact_threshold_insert < impact_threshold_1
                                ? impact_threshold_insert
                                : impact_threshold_1;
  double max_allow_velocity_change_dt_remove = 10.0;
  double impact_threshold_2 = min_mass * max_allow_velocity_change_dt_remove;
  impact_threshold_remove = impact_threshold_remove < impact_threshold_2
                                ? impact_threshold_remove
                                : impact_threshold_2;
  return std::make_pair(impact_threshold_insert, impact_threshold_remove);
}
// 需要跳过除了接触力的其他力导致的大impact
auto cpt_prt_contact_impact(simulator::SimulationLoop* s)
    -> std::vector<double> {
  SIRE_DEMAND(s != nullptr);
  aris::dynamic::Model* m = s->model();

  std::vector<std::array<double, 6>> part_fnet(m->partPool().size(), {0.0});

  for (sire::Size i = 0; i < m->partPool().size(); ++i) {
    auto& contact_force = m->forcePool().at(i + m->motionPool().size());
    double fsI[6], fsJ[6];
    contact_force.cptGlbFs(fsI, fsJ);
    if (&contact_force.makI()->fatherPart() != &m->ground())
      aris::dynamic::s_va(
          6, fsI, part_fnet[contact_force.makI()->fatherPart().id()].data());
    if (&contact_force.makJ()->fatherPart() != &m->ground())
      aris::dynamic::s_va(
          6, fsJ, part_fnet[contact_force.makJ()->fatherPart().id()].data());
  }
  double dt = s->deltaT();
  std::vector<double> result(m->partPool().size());
  for (sire::PartId i = 0; i < m->partPool().size(); ++i) {
    // 跳过ground
    if (i != m->ground().id()) {
      double part_impact = dt * aris::dynamic::s_norm(3, part_fnet[i].data()) /
                           m->partPool().at(i).prtIv()[0];
      result[i] = part_impact;
      // aris::dynamic::s_nv(6, dt, part_fnet[i].data());
    } else {
      result[i] = 0;
    }
  }
  return result;
}
auto process_penetration_depth_and_maintain_impact_set(
    simulator::SimulationLoop* simulator_ptr) -> double {
  physics::PhysicsEngine* engine_ptr = simulator_ptr->physicsEnginePtr();
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();

  // physicsEngine ptr -> handleContact()
  engine_ptr->updateGeometryLocationFromModel();
  std::vector<common::PenetrationAsPointPair> pairs;
  // 碰撞检测
  engine_ptr->cptPointPairPenetration(pairs);

  using ContactPairMap = std::unordered_map<core::SortedPair<sire::PartId>,
                                            core::ContactPairValue>;
  ContactPairMap& contact_pair_map = manager_ptr->contactPairMap();
  // 1. 修改表二
  // 根据碰撞信息结合碰撞点的记录更新表二的碰撞点记录和碰撞信息 depth-init_depth
  // Map中有的，vector中没有，就删除
  for (ContactPairMap::iterator it = contact_pair_map.begin();
       it != contact_pair_map.end();) {
    if (auto search = std::find_if(pairs.begin(), pairs.end(),
                                   [it](common::PenetrationAsPointPair& pair) {
                                     return it->first ==
                                            core::SortedPair<sire::PartId>(
                                                pair.id_A, pair.id_B);
                                   });
        search == pairs.end()) {
      contact_pair_map.erase(it++);
    } else {
      ++it;
    }
  }
  // Vector中有的，Map中没有，就插入，先不修改新加入点的穿深，计算一个huge_impact_prt,
  // 再修改穿深进行积分。记录没有减去穿深的新加入点的index
  std::vector<common::PenetrationAsPointPair*> new_contacts_ptr;
  for (auto& pair : pairs) {
    if (auto search = contact_pair_map.find({pair.id_A, pair.id_B});
        search == contact_pair_map.end()) {
      contact_pair_map.insert({{pair.id_A, pair.id_B}, {pair.depth, false}});
      new_contacts_ptr.push_back(&pair);
    } else {
      auto& contact_pair_value = contact_pair_map[{pair.id_A, pair.id_B}];
      pair.depth -= contact_pair_value.init_penetration_depth_;
      if (pair.depth < 0) {
        // 更新记录的初始穿深
        contact_pair_value.init_penetration_depth_ += pair.depth;
        // 记录为新的碰撞点
        // new_contacts_ptr.push_back(&pair);
      }
      // if (contact_pair_value.is_depth_smaller_than_init_depth_) {
      //   // 更新initial depth
      //   // 如果又开始接触，需要开始处理
      //   if (pair.depth > 0) {
      //     contact_pair_value.is_depth_smaller_than_init_depth_ = false;
      //     new_contacts_ptr.push_back(&pair);
      //   } else {
      //     // 将depth一直设置为零，并更新记录的initial depth
      //     contact_pair_value.init_penetration_depth_ += pair.depth;
      //     pair.depth = 0;
      //   }
      // } else {
      //   // 修改已经在表二中的点的穿深（已经清理过不在vector中的记录）
      //   if (pair.depth < 0) {
      //     contact_pair_value.is_depth_smaller_than_init_depth_ = true;
      //     contact_pair_value.init_penetration_depth_ += pair.depth;
      //   }
      // }
    }
  }
  // if (contact_pair_map.size() != 0) {
  //   double vs[6], as[6];
  //   simulator_ptr->model()->partPool().at(1).getVs(vs);
  //   simulator_ptr->model()->partPool().at(1).getAs(as);
  //   std::cout << "init_depth="
  //             << contact_pair_map.at({0, 1}).init_penetration_depth_ << " "
  //             << vs[2] << " " << as[2] << " ";
  // }
  // if (pairs.size() != 0)
  //   std::cout << " pair_depth2=" << pairs.at(0).depth << " ";

  std::vector<common::PointPairContactInfo> contact_info;
  // 接触求解，得到接触力
  // TODO: 对于第一次求解没必要使用多点接触求解方法，直接用最基本的就行了，
  // 这个只是后面消除穿深的参考
  double nextSuggestDt{-1};
  // nextSuggestDt = engine_ptr->cptContactInfo(pairs, contact_info);
  DLOG(DEBUG) << "next suggest dt1: " << nextSuggestDt;
  // 重置上一时刻关节和forcePool设置的力
  // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
  engine_ptr->resetPartContactForce();
  // 根据接触信息将力设置回model的forcePool
  engine_ptr->cptGlbForceByContactInfo(contact_info);
  // 计算每个杆件碰撞力和加速度和dt的数据，判断是否碰撞，碰撞需要缩小步长
  double impact_threshold_insert =
      simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
          "impact_threshold_insert", 5);
  double impact_threshold_remove =
      simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
          "impact_threshold_remove", 0.2);
  std::tie(impact_threshold_insert, impact_threshold_remove) =
      process_impact_threshold(simulator_ptr->model(), impact_threshold_insert,
                               impact_threshold_remove);
  std::vector<double> prt_impact_result = cpt_prt_contact_impact(simulator_ptr);
  // if (contact_pair_map.size() != 0)
  //   std::cout << "prt_impact=" << prt_impact_result[1] << " ";
  std::set<sire::PartId> impact_prts_insert, impact_prts_remove;
  for (sire::Size i = 0; i < prt_impact_result.size(); ++i) {
    if (prt_impact_result[i] > impact_threshold_insert)
      impact_prts_insert.insert(i);
    if (prt_impact_result[i] < impact_threshold_remove)
      impact_prts_remove.insert(i);
  }
  impact_prts_remove.erase(simulator_ptr->model()->ground().id());
  // 2. 修改表一（添加新的prt）
  using ImpactedPrtSet = std::unordered_set<sire::PartId>;
  ImpactedPrtSet& impacted_prt_set = manager_ptr->impactedPrtSet();
  for (sire::PartId impact_prt : impact_prts_insert) {
    impacted_prt_set.insert(impact_prt);
    // if (!simulator_ptr->contactPairManager()->hasImpactedPrt(impact_prt)) {
    //   // 1. 过大冲击的杆件没有被记录
    //   //
    //   需要检测碰撞列表是否有新的碰撞需要加入，如果没有就不需要记录杆件（修改表1）
    //   // 修改表一
    //   if (auto search = std::find_if(
    //           contact_info.begin(), contact_info.end(),
    //           [&impact_prt](common::PointPairContactInfo& contact) {
    //             return contact.partId_A() == impact_prt ||
    //                    contact.partId_B() == impact_prt;
    //           });
    //       search != contact_info.end()) {
    //     impacted_prt_set.insert(impact_prt);
    //   }
    // }
  }
  // 3. 修改表一（移除记录）
  for (ImpactedPrtSet::iterator it = impacted_prt_set.begin();
       it != impacted_prt_set.end();) {
    if (auto search = impact_prts_remove.find(*it);
        search != impact_prts_remove.end()) {
      impacted_prt_set.erase(it++);
    } else {
      ++it;
    }
  }
  // if (impacted_prt_set.empty()) {
  //   for (auto& contact_pair : contact_pair_map)
  //     contact_pair.second.is_depth_smaller_than_init_depth_ = true;
  // }
  if (new_contacts_ptr.size() != 0) {
    // 减去 init_depth，再计算F
    for (auto new_contact : new_contacts_ptr) {
      new_contact->depth -=
          contact_pair_map[{new_contact->id_A, new_contact->id_B}]
              .init_penetration_depth_;
    }
    contact_info.clear();
    // 接触求解，得到接触力
    // nextSuggestDt = engine_ptr->cptContactInfo(pairs, contact_info);
    DLOG(DEBUG) << "next suggest dt2: " << nextSuggestDt;
    // 重置上一时刻关节和forcePool设置的力
    // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
    engine_ptr->resetPartContactForce();
    // 根据接触信息将力设置回model的forcePool
    engine_ptr->cptGlbForceByContactInfo(contact_info);
  }
  return nextSuggestDt;
  // if (pairs.size() != 0) {
  //   double vs[6];
  //   simulator_ptr->model()->partPool().at(1).getVs(vs);
  //   std::cout << pairs.at(0).depth << " " << impacted_prt_set.size() << " "
  //             << vs[2] << " " << std::endl;
  // }
}
static std::fstream file;
auto initLog() -> void {
  auto file_name =
      aris::core::defaultLogDirectory() /
      ("state-log--" +
       aris::core::logFileTimeFormat(std::chrono::system_clock::now()) + "--");
  file.open(file_name.string() + ".txt", std::ios::out | std::ios::trunc);
}
auto logCurrentState(double time, double realtime_rate,
                     simulator::SimulationLoop* base) -> void {
  std::vector<std::array<double, 6>> parts_pe;
  std::vector<std::array<double, 6>> parts_vs;
  for (int i = 0; i < base->model()->partPool().size(); ++i) {
    std::array<double, 6> buffer_pe{0}, buffer_vs{0};
    base->model()->partPool().at(i).getPe(buffer_pe.data());
    base->model()->partPool().at(i).getVs(buffer_vs.data());
    parts_pe.push_back(buffer_pe);
    parts_vs.push_back(buffer_vs);
  }

  file << time << " " << realtime_rate << " " << nlohmann::json(parts_pe).dump()
       << " " << nlohmann::json(parts_vs).dump() << std::endl;
}

auto InitTrigger::trigger(simulator::SimulationLoop*) -> void {
  // manager->addEvent();
}
auto InitEvent::init() -> void {}
auto InitHandler::init(simulator::SimulationLoop* simulator) -> void {
  simulator_ptr = simulator;
}
auto InitHandler::handle(core::EventBase* e) -> bool {
  InitEvent* event_ptr = dynamic_cast<InitEvent*>(e);
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  process_penetration_depth_and_maintain_impact_set(simulator_ptr);
  // 之后就可以正常积分

  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  step_event->eventProp().addProp(
      "dt", manager_ptr->impactedPrtSet().empty()
                ? simulator_ptr->deltaT()
                : simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
                      "shrink_dt", 1e-5));
  simulator_ptr->eventManager().addEvent(std::move(step_event));

  return true;
  // Add Step Trigger to trigger list in Event Manager
  // EventManager ptr
}
auto StepEvent::init() -> void {}
auto StepHandler::init(simulator::SimulationLoop* simulator) -> void {
  simulator_ptr = simulator;
}
auto StepHandler::handle(core::EventBase* e) -> bool {
  // 积分到当前event记录的时间
  double dt = e->eventProp().getPropValue("dt");
  simulator_ptr->integratorPoolPtr()->at(0).step(dt);
  // std::cout << "dt=" << dt << " ";
  simulator_ptr->timer().updateSimTime(dt);
  // logCurrentState(simulator_ptr->timer().simTime(),
  //                 simulator_ptr->timer().realtimeRate(), simulator_ptr);
  // if (dt == 0.0000001) std::cout << "dt=" << dt << " ";
  StepEvent* event_ptr = dynamic_cast<StepEvent*>(e);
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  // 如果这一轮检测没有碰撞点（时间步长与标准步长一致），就清除记录到的穿深（防止过穿的）
  // 感觉就不该清除，可以让穿深往上修正而不是一直往下，也不需要impactPrtSet标识是否要缩短时间步长，时间步长由碰撞求解控制。
  if (e->eventProp().getPropValueOrDefault("clearInitDepth", 0.0)) {
    DLOG(DEBUG) << "Clear record initial depth";
    manager_ptr->contactPairMap().clear();
  }
  double suggestDt =
      process_penetration_depth_and_maintain_impact_set(simulator_ptr);
  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  double nextDt =
      manager_ptr->impactedPrtSet().empty()
          ? simulator_ptr->deltaT()
          : simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
                "shrink_dt", 1e-5);
  if (suggestDt > 0 && suggestDt < nextDt) {
    nextDt = suggestDt;
  }
  DLOG_IF(suggestDt > 0, DEBUG)
      << "dt: " << nextDt << " suggestDt: " << suggestDt;
  step_event->eventProp().addProp("dt", nextDt);
  if (nextDt == simulator_ptr->deltaT() &&
      manager_ptr->impactedPrtSet().empty()) {
    step_event->eventProp().addProp("clearInitDepth", 1);
  } else {
    step_event->eventProp().addProp("clearInitDepth", 0);
  }
  DLOG(DEBUG) << "impacted prt set empty: "
              << manager_ptr->impactedPrtSet().empty();
  // step_event->eventProp().addProp(
  //     "dt", manager_ptr->impactedPrtSet().empty()
  //               ? simulator_ptr->deltaT()
  //               :
  //               simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
  //                     "shrink_dt", 1e-5));
  // if (!manager_ptr->impactedPrtSet().empty()) {
  //   std::cout
  //       << "shrinked dt "
  //       << simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
  //              "shrink_dt", 1e-7)
  //       << " "
  //       << manager_ptr->contactPairMap().at({0, 1}).init_penetration_depth_
  //       << std::endl;
  // }
  simulator_ptr->eventManager().addEvent(std::move(step_event));

  return true;
}
auto process_penetration_depth_and_maintain_impact_set2(
    simulator::SimulationLoop* simulator_ptr,
    std::vector<common::PointPairContactInfo>& contact_info_out) -> double {
  physics::PhysicsEngine* engine_ptr = simulator_ptr->physicsEnginePtr();
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();

  // physicsEngine ptr -> handleContact()
  engine_ptr->updateGeometryLocationFromModel();
  std::vector<common::PenetrationAsPointPair> pairs;
  // 碰撞检测
  engine_ptr->cptPointPairPenetration(pairs);

  using ContactPairMap = std::unordered_map<core::SortedPair<sire::PartId>,
                                            core::ContactPairValue>;
  ContactPairMap& contact_pair_map = manager_ptr->contactPairMap();
  // 1. 修改表二
  // 根据碰撞信息结合碰撞点的记录更新表二的碰撞点记录和碰撞信息 depth-init_depth
  // Map中有的，vector中没有，就删除
  for (ContactPairMap::iterator it = contact_pair_map.begin();
       it != contact_pair_map.end();) {
    if (auto search = std::find_if(pairs.begin(), pairs.end(),
                                   [it](common::PenetrationAsPointPair& pair) {
                                     return it->first ==
                                            core::SortedPair<sire::PartId>(
                                                pair.id_A, pair.id_B);
                                   });
        search == pairs.end()) {
      contact_pair_map.erase(it++);
    } else {
      ++it;
    }
  }
  // Vector中有的，Map中没有，就插入，先不修改新加入点的穿深，计算一个huge_impact_prt,
  // 再修改穿深进行积分。记录没有减去穿深的新加入点的index
  std::vector<common::PenetrationAsPointPair*> new_contacts_ptr;
  for (auto& pair : pairs) {
    DLOG(DEBUG) << "contact detected id: " << pair.id_A << " " << pair.id_B
                << " depth: " << pair.depth;
    if (auto search = contact_pair_map.find({pair.id_A, pair.id_B});
        search == contact_pair_map.end()) {
      contact_pair_map.insert({{pair.id_A, pair.id_B}, {pair.depth, false}});
      // new_contacts_ptr.push_back(&pair);
      pair.depth = 0;
    } else {
      auto& contact_pair_value = contact_pair_map[{pair.id_A, pair.id_B}];
      pair.depth -= contact_pair_value.init_penetration_depth_;
      if (pair.depth < 0) {
        // 更新记录的初始穿深
        contact_pair_value.init_penetration_depth_ += pair.depth;
      }
    }
  }
  std::vector<common::PointPairContactInfo> contact_info;
  // 接触求解，得到接触力
  // TODO: 对于第一次求解没必要使用多点接触求解方法，直接用最基本的就行了，
  // 这个只是后面消除穿深的参考
  double nextSuggestDt{pairs.size() ? -1.0 : 0.0};
  // nextSuggestDt = engine_ptr->cptContactInfo(pairs, contact_info_out);
  // 重置上一时刻关节和forcePool设置的力
  // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
  engine_ptr->resetPartContactForce();
  // 根据接触信息将力设置回model的forcePool
  engine_ptr->cptGlbForceByContactInfo(contact_info_out);
  return nextSuggestDt;
}
auto process_penetration_depth_and_maintain_impact_set3(
    simulator::SimulationLoop* simulator_ptr) -> double {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };

  physics::PhysicsEngine* engine_ptr = simulator_ptr->physicsEnginePtr();
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  aris::dynamic::Model* model_ptr = simulator_ptr->model();
  // 积分后调整一下

  // physicsEngine ptr -> handleContact()
  engine_ptr->updateGeometryLocationFromModel();
  std::vector<common::PenetrationAsPointPair> pairs;
  // 碰撞检测
  engine_ptr->cptPointPairPenetration(pairs);
  if (pairs.size() == 0) {
    manager_ptr->contactPairMap().clear();
    engine_ptr->resetPartContactForce();
    return 0.0;
  }
  // ----------------- 计算碰撞产生的初速度与碰撞点的坐标系
  // ---------------------- 每个碰撞点构建的坐标系保存的位置，使用pm保存
  const sire::Size num_contacts = pairs.size();
  std::vector<std::array<double, 16>> T_C_vec;
  T_C_vec.resize(num_contacts);
  for (sire::Size i{0}; i < num_contacts; ++i) {
    // 使用 nhat_AB_w 构建当前碰撞点的 T 矩阵
    aris::dynamic::s_sov_axes2pm(
        pairs[i].p_WC.data(), pairs[i].nhat_AB_W.data(),
        pairs[i].nhat_AB_W.data(), T_C_vec[i].data(), "zx");
  }

  std::vector<std::array<double, 3>> v0W_vec;  // 世界坐标系接触点速度。
  std::vector<std::array<double, 3>> v0C_vec;  // 世界坐标系接触点速度。
  v0W_vec.resize(2 * num_contacts);
  v0C_vec.resize(2 * num_contacts);
  engine_ptr->cptContactVelocityAB(pairs, v0W_vec);
  for (sire::Size i{0}; i < num_contacts; ++i) {
    aris::dynamic::s_inv_pm_dot_v3(T_C_vec[i].data(), v0W_vec[2 * i].data(),
                                   v0C_vec[2 * i].data());
    aris::dynamic::s_inv_pm_dot_v3(T_C_vec[i].data(), v0W_vec[2 * i + 1].data(),
                                   v0C_vec[2 * i + 1].data());
  }

  using ContactPairMap = std::unordered_map<core::SortedPair<sire::PartId>,
                                            core::ContactPairValue>;
  ContactPairMap& contact_pair_map = manager_ptr->contactPairMap();
  // 1. 修改表二
  // 根据碰撞信息结合碰撞点的记录更新表二的碰撞点记录和碰撞信息 depth-init_depth
  // Map中有的，vector中没有，就删除
  for (ContactPairMap::iterator it = contact_pair_map.begin();
       it != contact_pair_map.end();) {
    if (auto search = std::find_if(pairs.begin(), pairs.end(),
                                   [it](common::PenetrationAsPointPair& pair) {
                                     return it->first ==
                                            core::SortedPair<sire::PartId>(
                                                pair.id_A, pair.id_B);
                                   });
        search == pairs.end()) {
      contact_pair_map.erase(it++);
    } else {
      ++it;
    }
  }
  // Vector中有的，Map中没有，就插入
  // 再修改穿深进行积分。记录没有减去穿深的新加入点的index
  std::vector<sire::Size> adjustPositionPairIdx;
  for (sire::Size i{0}; i < pairs.size(); ++i) {
    auto& pair = pairs[i];
    DLOG(DEBUG) << "contact detected id: " << pair.id_A << " " << pair.id_B
                << " depth: " << pair.depth;
    if (auto search = contact_pair_map.find({pair.id_A, pair.id_B});
        search == contact_pair_map.end()) {
      contact_pair_map.insert({{pair.id_A, pair.id_B}, {pair.depth, false}});
      adjustPositionPairIdx.push_back(i);
      // new_contacts_ptr.push_back(&pair);
      pair.depth = 0;
    }
  }

  auto& jointPool = model_ptr->jointPool();
  auto& partPool = model_ptr->partPool();
  sire::Size tempJointIdxOffset = jointPool.size();
  std::map<sire::Size, sire::Size> updatedMakerPoolSize;
  for (sire::Size idx{0}; idx < adjustPositionPairIdx.size(); ++idx) {
    sire::Size i{adjustPositionPairIdx[idx]};
    const auto& pair = pairs[i];
    const auto& vCa = v0C_vec[2 * i];
    const auto& vCb = v0C_vec[2 * i + 1];
    const double sumVnAbs = (std::abs(vCa[2]) + std::abs(vCb[2]));

    Eigen::Matrix<double, 3, 1> ab_depth_vec = pair.p_WCa - pair.p_WCb;
    Eigen::Matrix<double, 3, 1> dpb =
        (std::abs(vCb[2]) / sumVnAbs) * ab_depth_vec;
    Eigen::Matrix<double, 3, 1> new_cp =
        (std::abs(vCb[2]) / sumVnAbs) * ab_depth_vec + pair.p_WCb;
    Eigen::Matrix<double, 3, 1> dpa =
        (std::abs(vCa[2]) / sumVnAbs) * ab_depth_vec;
    // 一个prt多个接触点怎么办？
    // 先不管
    auto* geometry_A = engine_ptr->queryGeometryPoolById(pairs[i].id_A);
    auto* geometry_B = engine_ptr->queryGeometryPoolById(pairs[i].id_B);
    SIRE_ASSERT(geometry_A != nullptr && geometry_B != nullptr);

    auto& prtA = model_ptr->partPool().at(geometry_A->partId());
    auto pmA = const_cast<aris::dynamic::double4x4&>(prtA.pm());
    pmA[0][3] -= dpa[0];
    pmA[1][3] -= dpa[1];
    pmA[2][3] -= dpa[2];
    auto& prtB = model_ptr->partPool().at(geometry_B->partId());
    auto pmB = const_cast<aris::dynamic::double4x4&>(prtB.pm());
    pmB[0][3] += dpb[0];
    pmB[1][3] += dpb[1];
    pmB[2][3] += dpb[2];
    T_C_vec[i][3] = new_cp[0];
    T_C_vec[i][7] = new_cp[1];
    T_C_vec[i][11] = new_cp[2];

    sire::core::FixedJoint::add2ModelAbs(model_ptr, prtA, prtB, new_cp.data(),
                                         new_cp.data());
    DLOG(DEBUG) << "new_cp: " << new_cp.transpose();

    auto lowerBoundPrt = updatedMakerPoolSize.lower_bound(prtA.id());
    if (lowerBoundPrt != updatedMakerPoolSize.end() &&
        !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, prtA.id()))) {
      ++lowerBoundPrt->second;
    } else {
      updatedMakerPoolSize.insert(
          lowerBoundPrt,
          std::map<sire::Size, sire::Size>::value_type(prtA.id(), 1));
    }
    lowerBoundPrt = updatedMakerPoolSize.lower_bound(prtB.id());
    if (lowerBoundPrt != updatedMakerPoolSize.end() &&
        !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, prtB.id()))) {
      ++lowerBoundPrt->second;
    } else {
      updatedMakerPoolSize.insert(
          lowerBoundPrt,
          std::map<sire::Size, sire::Size>::value_type(prtB.id(), 1));
    }
    jointPool[i + tempJointIdxOffset].resetModel(model_ptr);
    jointPool[i + tempJointIdxOffset].setId(i + tempJointIdxOffset);
    init_interaction(jointPool[i + tempJointIdxOffset], model_ptr);
  }

  // 调用 FK solver 的内存分配方法重新分配内存
  model_ptr->solverPool().at(1).allocateMemory();
  model_ptr->forwardKinematics();

  for (sire::Size i = 0; i < adjustPositionPairIdx.size(); ++i) {
    jointPool.pop_back();
  }
  for (auto const& [prtIdx, jntNum] : updatedMakerPoolSize) {
    for (sire::Size i = 0; i < jntNum; ++i) {
      partPool[prtIdx].markerPool().pop_back();
    }
  }
  model_ptr->solverPool().at(1).allocateMemory();

  std::vector<common::PointPairContactInfo> contact_info;
  // 接触求解，得到接触力
  // TODO: 对于第一次求解没必要使用多点接触求解方法，直接用最基本的就行了，
  // 这个只是后面消除穿深的参考
  double nextSuggestDt{pairs.size() ? -1.0 : 0.0};
  // nextSuggestDt = engine_ptr->cptContactInfo(pairs, contact_info);
  // 重置上一时刻关节和forcePool设置的力
  // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
  engine_ptr->resetPartContactForce();
  // 根据接触信息将力设置回model的forcePool
  engine_ptr->cptGlbForceByContactInfo(contact_info);
  return nextSuggestDt;
}
auto adjustPenetrationPosition(
    simulator::SimulationLoop* simulator_ptr,
    std::vector<common::PenetrationAsPointPair>& pairs,
    std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<sire::Size>& adjustPositionPairIdx) {
  physics::PhysicsEngine* engine_ptr = simulator_ptr->physicsEnginePtr();
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  aris::dynamic::Model* model_ptr = simulator_ptr->model();
  auto& partPool = model_ptr->partPool();
  // ----------------- 计算碰撞产生的初速度与碰撞点的坐标系 ------------------
  //  每个碰撞点构建的坐标系保存的位置，使用pm保存
  const sire::Size num_contacts = pairs.size();

  std::vector<std::array<double, 3>> v0W_vec;  // 世界坐标系接触点速度。
  std::vector<std::array<double, 3>> v0C_vec;  // 世界坐标系接触点速度。
  v0W_vec.resize(2 * num_contacts);
  v0C_vec.resize(2 * num_contacts);
  engine_ptr->cptContactVelocityAB(pairs, v0W_vec);
  for (sire::Size i{0}; i < num_contacts; ++i) {
    aris::dynamic::s_inv_pm_dot_v3(T_C_vec[i].data(), v0W_vec[2 * i].data(),
                                   v0C_vec[2 * i].data());
    aris::dynamic::s_inv_pm_dot_v3(T_C_vec[i].data(), v0W_vec[2 * i + 1].data(),
                                   v0C_vec[2 * i + 1].data());
  }

  std::map<sire::Size, sire::Size> updatedMakerPoolSize;
  std::vector<std::array<double, 3>> movement;
  auto safe_div = [](double number, double denominator) -> double {
    return denominator == 0.0 ? 0.0 : number / denominator;
  };
  for (sire::Size idx{0}; idx < adjustPositionPairIdx.size(); ++idx) {
    sire::Size i{adjustPositionPairIdx[idx]};
    const auto& pair = pairs[i];
    const auto& vCa = v0C_vec[2 * i];
    const auto& vCb = v0C_vec[2 * i + 1];
    const double sumVnAbs = (std::abs(vCa[2]) + std::abs(vCb[2]));

    Eigen::Matrix<double, 3, 1> ab_depth_vec = pair.p_WCa - pair.p_WCb;
    Eigen::Matrix<double, 3, 1> dpb =
        safe_div(std::abs(vCb[2]), sumVnAbs) * ab_depth_vec;
    Eigen::Matrix<double, 3, 1> new_cp =
        safe_div(std::abs(vCb[2]), sumVnAbs) * ab_depth_vec + pair.p_WCb;
    Eigen::Matrix<double, 3, 1> dpa =
        safe_div(std::abs(vCa[2]), sumVnAbs) * ab_depth_vec;
    // 一个prt多个接触点怎么办？
    // 先不管
    auto* geometry_A = engine_ptr->queryGeometryPoolById(pairs[i].id_A);
    auto* geometry_B = engine_ptr->queryGeometryPoolById(pairs[i].id_B);
    SIRE_ASSERT(geometry_A != nullptr && geometry_B != nullptr);

    if (geometry_A->partId() != model_ptr->ground().id()) {
      movement.push_back({-dpa[0], -dpa[1], -dpa[2]});
    }

    if (geometry_B->partId() != model_ptr->ground().id()) {
      movement.push_back({dpb[0], dpb[1], dpb[2]});
    }
  }
  double sum[3]{0}, mean[3]{0};
  for (sire::Size i{0}; i < movement.size(); ++i) {
    sum[0] += movement[i][0];
    sum[1] += movement[i][1];
    sum[2] += movement[i][2];
  }

  mean[0] = safe_div(sum[0], movement.size());
  mean[1] = safe_div(sum[1], movement.size());
  mean[2] = safe_div(sum[2], movement.size());

  for (sire::Size i{1}; i < partPool.size(); ++i) {
    auto& prt = partPool.at(i);
    auto pm = const_cast<aris::dynamic::double4x4&>(prt.pm());
    pm[0][3] += mean[0];
    pm[1][3] += mean[1];
    pm[2][3] += mean[2];
  }
  for (sire::Size idx{0}; idx < adjustPositionPairIdx.size(); ++idx) {
    sire::Size i{adjustPositionPairIdx[idx]};
    const auto& pair = pairs[i];
    T_C_vec[i][3] += mean[0];
    T_C_vec[i][7] += mean[1];
    T_C_vec[i][11] += mean[2];
  }
}
auto process_penetration_depth_and_maintain_impact_set4(
    simulator::SimulationLoop* simulator_ptr,
    std::vector<common::PenetrationAsPointPair>& pairs,
    std::vector<std::array<double, 16>>& T_C_vec) -> void {
  physics::PhysicsEngine* engine_ptr = simulator_ptr->physicsEnginePtr();
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  aris::dynamic::Model* model_ptr = simulator_ptr->model();
  auto& partPool = model_ptr->partPool();

  // 1. 修改contact_pair_map
  using ContactPairMap = std::unordered_map<core::SortedPair<sire::PartId>,
                                            core::ContactPairValue>;
  ContactPairMap& contact_pair_map = manager_ptr->contactPairMap();
  // 根据碰撞信息结合碰撞点的记录更新contact_pair_map的碰撞点记录和碰撞信息
  // depth-init_depth 删除Map中有的记录但是没有被碰撞检测到的
  for (ContactPairMap::iterator it = contact_pair_map.begin();
       it != contact_pair_map.end();) {
    if (auto search = std::find_if(pairs.begin(), pairs.end(),
                                   [it](common::PenetrationAsPointPair& pair) {
                                     return it->first ==
                                            core::SortedPair<sire::PartId>(
                                                pair.id_A, pair.id_B);
                                   });
        search == pairs.end()) {
      contact_pair_map.erase(it++);
    } else {
      ++it;
    }
  }
  // Vector中有的，Map中没有，就插入
  // 再修改穿深进行积分。记录没有减去穿深的新加入点的index
  bool newContact{false};
  std::vector<sire::Size> adjustPositionPairIdx;
  for (sire::Size i{0}; i < pairs.size(); ++i) {
    auto& pair = pairs[i];
    DLOG(DEBUG) << "contact detected id: " << pair.id_A << " " << pair.id_B
                << " depth: " << pair.depth;
    if (auto search = contact_pair_map.find({pair.id_A, pair.id_B});
        search == contact_pair_map.end()) {
      contact_pair_map.insert({{pair.id_A, pair.id_B}, {pair.depth, false}});
      newContact = true;
      adjustPositionPairIdx.push_back(i);
      pair.depth = 0;
    } else {
      auto& contact_pair_value = contact_pair_map[{pair.id_A, pair.id_B}];
      pair.depth -= contact_pair_value.init_penetration_depth_;
      if (pair.depth < 0) {
        // 更新记录的初始穿深
        contact_pair_value.init_penetration_depth_ += pair.depth;
      }
    }
  }

  if (newContact) {
    adjustPenetrationPosition(simulator_ptr, pairs, T_C_vec,
                              adjustPositionPairIdx);
  }
}
auto InitEvent1::init() -> void {}
auto InitHandler1::init(simulator::SimulationLoop* simulator) -> void {
  simulator_ptr = simulator;
}
auto InitHandler1::handle(core::EventBase* e) -> bool {
  InitEvent1* event_ptr = dynamic_cast<InitEvent1*>(e);
  physics::PhysicsEngine* engine_ptr = simulator_ptr->physicsEnginePtr();
  aris::dynamic::Model* model_ptr = simulator_ptr->model();
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  SIRE_ASSERT(engine_ptr != nullptr);
  SIRE_ASSERT(model_ptr != nullptr);
  SIRE_ASSERT(manager_ptr != nullptr);
  simulator_ptr->timer().reset();
  // initLog();
  // logCurrentState(0, 1, simulator_ptr);
  engine_ptr->updateGeometryLocationFromModel();
  std::vector<common::PenetrationAsPointPair> pairs;
  // 碰撞检测
  engine_ptr->cptPointPairPenetration(pairs);

  double nextSuggestDt{pairs.size() ? -1.0 : 0.0};
  std::vector<common::PointPairContactInfo> contact_info;
  if (pairs.size() != 0) {
    std::vector<std::array<double, 16>> T_C_vec;
    cptContactFrame(pairs, T_C_vec);

    process_penetration_depth_and_maintain_impact_set4(simulator_ptr, pairs,
                                                       T_C_vec);

    nextSuggestDt = engine_ptr->cptContactInfo(pairs, T_C_vec, contact_info);
    // 重置上一时刻关节和forcePool设置的力
    // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
    engine_ptr->resetPartContactForce();
    // 根据接触信息将力设置回model的forcePool
    engine_ptr->cptGlbForceByContactInfo(contact_info);

  } else {
    manager_ptr->contactPairMap().clear();
    engine_ptr->resetPartContactForce();
  }
  // 记录模型状态和接触信息
  simulator_ptr->recorder().record(simulator_ptr->timer().simTime(),
                                   *simulator_ptr->model(), contact_info);
  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  double nextDt = simulator_ptr->deltaT();

  if (nextSuggestDt > 0 && nextSuggestDt < simulator_ptr->deltaT()) {
    nextDt = nextSuggestDt;
  }
  DLOG_IF(nextSuggestDt > 0, DEBUG)
      << "dt: " << nextDt << " suggestDt: " << nextSuggestDt;
  step_event->eventProp().addProp("dt", nextDt);
  simulator_ptr->eventManager().addEvent(std::move(step_event));
  return true;
}

auto StepEvent1::init() -> void {}
auto StepHandler1::init(simulator::SimulationLoop* simulator) -> void {
  simulator_ptr = simulator;
}
auto StepHandler1::handle(core::EventBase* e) -> bool {
  // 积分到当前 event 记录的时间
  double dt = e->eventProp().getPropValue("dt");
  DLOG(DEBUG) << "-------------- integrate with dt " << dt << " --------------";
  simulator_ptr->integratorPoolPtr()->at(0).step(dt);
  simulator_ptr->timer().updateSimTime(dt);

  physics::PhysicsEngine* engine_ptr = simulator_ptr->physicsEnginePtr();
  aris::dynamic::Model* model_ptr = simulator_ptr->model();
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  SIRE_ASSERT(engine_ptr != nullptr);
  SIRE_ASSERT(model_ptr != nullptr);
  SIRE_ASSERT(manager_ptr != nullptr);

  // initLog();
  // logCurrentState(0, 1, simulator_ptr);
  engine_ptr->updateGeometryLocationFromModel();
  std::vector<common::PenetrationAsPointPair> pairs;
  // 碰撞检测
  engine_ptr->cptPointPairPenetration(pairs);

  double nextSuggestDt{pairs.size() ? -1.0 : 0.0};
  std::vector<common::PointPairContactInfo> contact_info;
  if (pairs.size() != 0) {
    std::vector<std::array<double, 16>> T_C_vec;
    cptContactFrame(pairs, T_C_vec);

    process_penetration_depth_and_maintain_impact_set4(simulator_ptr, pairs,
                                                       T_C_vec);

    nextSuggestDt = engine_ptr->cptContactInfo(pairs, T_C_vec, contact_info);
    // 重置上一时刻关节和forcePool设置的力
    // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
    engine_ptr->resetPartContactForce();
    // 根据接触信息将力设置回model的forcePool
    engine_ptr->cptGlbForceByContactInfo(contact_info);
  } else {
    manager_ptr->contactPairMap().clear();
    engine_ptr->resetPartContactForce();
  }
  // 记录模型状态和接触信息
  simulator_ptr->recorder().record(simulator_ptr->timer().simTime(),
                                   *simulator_ptr->model(), contact_info);
  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  double nextDt = simulator_ptr->deltaT();

  if (nextSuggestDt > 0 && nextSuggestDt < simulator_ptr->deltaT()) {
    nextDt = nextSuggestDt;
  }
  DLOG_IF(nextSuggestDt > 0, DEBUG)
      << "dt: " << nextDt << " suggestDt: " << nextSuggestDt;
  step_event->eventProp().addProp("dt", nextDt);
  simulator_ptr->eventManager().addEvent(std::move(step_event));
  return true;
}
ARIS_REGISTRATION {
  // core::EventRegister<InitEvent>::registration("initial", 0);
  // core::EventRegister<StepEvent>::registration("step", 1);
  // core::HandlerRegister<InitHandler>::registration("initial", 0);
  // core::HandlerRegister<StepHandler>::registration("step", 1);
  core::EventRegister<InitEvent1>::registration("initial1", 0);
  core::EventRegister<StepEvent1>::registration("step1", 1);
  core::HandlerRegister<InitHandler1>::registration("initial1", 0);
  core::HandlerRegister<StepHandler1>::registration("step1", 1);
}
}  // namespace sire::simulator