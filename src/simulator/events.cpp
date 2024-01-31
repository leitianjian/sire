#include "sire/simulator/events.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <set>

#include <aris/dynamic/model.hpp>
#include <aris/dynamic/screw.hpp>

#include "sire/core/event_manager.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

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
auto cpt_prt_contact_impact(simulator::SimulatorBase* s)
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
    simulator::SimulatorBase* simulator_ptr) -> void {
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
  // if (pairs.size() != 0) {
  //   for (auto& pair : pairs) {
  //     if (pair.id_B == 1) {
  //       std::cout << "pair_depth=" << pair.depth << " ";
  //     }
  //   }
  // }
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
        // 将穿深设置为0，受到重力
        pair.depth = 0;
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
  engine_ptr->cptContactInfo(pairs, contact_info);
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
    engine_ptr->cptContactInfo(pairs, contact_info);
    // 重置上一时刻关节和forcePool设置的力
    // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
    engine_ptr->resetPartContactForce();
    // 根据接触信息将力设置回model的forcePool
    engine_ptr->cptGlbForceByContactInfo(contact_info);
  }
  // if (pairs.size() != 0) {
  //   double vs[6];
  //   simulator_ptr->model()->partPool().at(1).getVs(vs);
  //   std::cout << pairs.at(0).depth << " " << impacted_prt_set.size() << " "
  //             << vs[2] << " " << std::endl;
  // }
}
auto InitTrigger::trigger(simulator::SimulatorBase*) -> void {
  // manager->addEvent();
}
auto InitEvent::init() -> void {}
auto InitHandler::init(simulator::SimulatorBase* simulator) -> void {
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
                      "shrink_dt", 1e-7));
  simulator_ptr->eventManager().addEvent(std::move(step_event));

  return true;
  // Add Step Trigger to trigger list in Event Manager
  // EventManager ptr
}
auto StepEvent::init() -> void {}
auto StepHandler::init(simulator::SimulatorBase* simulator) -> void {
  simulator_ptr = simulator;
}
static double cumulate_time = 0;
auto StepHandler::handle(core::EventBase* e) -> bool {
  // 积分到当前event记录的时间
  double dt = e->eventProp().getPropValue("dt");
  simulator_ptr->integratorPoolPtr()->at(0).step(dt);
  simulator_ptr->timer().updateSimTime(dt);
  StepEvent* event_ptr = dynamic_cast<StepEvent*>(e);
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  process_penetration_depth_and_maintain_impact_set(simulator_ptr);
  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  step_event->eventProp().addProp(
      "dt", manager_ptr->impactedPrtSet().empty()
                ? simulator_ptr->deltaT()
                : simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
                      "shrink_dt", 1e-7));
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
  // Add Step Trigger to trigger list in Event Manager
  // EventManager ptr
}

auto InitEvent1::init() -> void {}
auto InitHandler1::init(simulator::SimulatorBase* simulator) -> void {
  simulator_ptr = simulator;
}
auto InitHandler1::handle(core::EventBase* e) -> bool {
  InitEvent1* event_ptr = dynamic_cast<InitEvent1*>(e);
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
auto StepEvent1::init() -> void {}
auto StepHandler1::init(simulator::SimulatorBase* simulator) -> void {
  simulator_ptr = simulator;
}
auto StepHandler1::handle(core::EventBase* e) -> bool {
  // 积分到当前event记录的时间
  double dt = e->eventProp().getPropValue("dt");
  simulator_ptr->integratorPoolPtr()->at(0).step(dt);
  // std::cout << "dt=" << dt << " ";
  simulator_ptr->timer().updateSimTime(dt);
  // if (dt == 0.0000001) std::cout << "dt=" << dt << " ";
  StepEvent* event_ptr = dynamic_cast<StepEvent*>(e);
  core::ContactPairManager* manager_ptr = simulator_ptr->contactPairManager();
  if (dt == simulator_ptr->deltaT()) {
    manager_ptr->contactPairMap().clear();
  }
  process_penetration_depth_and_maintain_impact_set(simulator_ptr);
  // std::cout << std::endl;

  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  step_event->eventProp().addProp(
      "dt", manager_ptr->impactedPrtSet().empty()
                ? simulator_ptr->deltaT()
                : simulator_ptr->getGlobalVariablePool().getPropValueOrDefault(
                      "shrink_dt", 1e-5));
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
  // Add Step Trigger to trigger list in Event Manager
  // EventManager ptr
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