#ifndef SIRE_POINT_PAIR_CONTACT_INFO_HPP_
#define SIRE_POINT_PAIR_CONTACT_INFO_HPP_
#include <algorithm>

#include <coal/data_types.h>

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::physics::common {
using namespace coal;
// 全局接触点数据池 (存放整个仿真生命周期内所有的点)
// 所有标量都在一个巨长无比的内存条里，Cache-line 命中率极高
struct ContactDataPool {
    std::vector<sire::Size> id_A;
    std::vector<sire::Size> id_B;
    std::vector<double> initial_depth;
    std::vector<double> initial_velocity;
    std::vector<double> modified_depth;
    std::vector<double> modified_velocity;
};

// 全局帧结构数据池
struct FrameDataPool {
    std::vector<double> current_time;
    std::vector<double> suggest_dt;
    std::vector<double> min_time;
    
    // 关键设计：通过 offset(起始索引) 和 count(数量) 去 ContactDataPool 中取数据
    std::vector<sire::Size> contact_start_idx; 
    std::vector<sire::Size> contact_count;
};

// 物理上下文的数据集 (挂载在物理引擎的 imp_ 内部)
struct ContactDatabase {
    FrameDataPool frames;
    ContactDataPool contacts;
};

// 纯数据行为：将当前帧的物理计算结果写入 Database
inline void log_contact_frame_system(
    ContactDatabase& db,
    double time, double dt, double min_time, double stiffScale,
    const std::vector<common::PenetrationAsPointPair>& pairs,
    const std::vector<sire::Size>& preservedIdx,
    const std::vector<double>& realDepth,
    const std::vector<double>& initial_x0,
    const std::vector<double>& modified_x0) 
{
    sire::Size n = preservedIdx.size();
    
    // 1. 记录这一帧的宏观数据（各执行 1 把 push_back，且往往触发连续内存的隐式扩容）
    db.frames.current_time.push_back(time);
    db.frames.suggest_dt.push_back(dt);
    db.frames.min_time.push_back(min_time);
    
    // 2. 划定区间：获取当前全局点池的尾部，这就是当前帧接触点写入的起点
    sire::Size start_idx = db.contacts.id_A.size();
    db.frames.contact_start_idx.push_back(start_idx);
    db.frames.contact_count.push_back(n);

    if (n == 0) return;

    // 3. 一次性为超级数组分配当帧要用的内存，这避免了碎片的频繁 alloc
    sire::Size new_size = start_idx + n;
    db.contacts.id_A.resize(new_size);
    db.contacts.id_B.resize(new_size);
    db.contacts.initial_depth.resize(new_size);
    db.contacts.initial_velocity.resize(new_size);
    db.contacts.modified_depth.resize(new_size);
    db.contacts.modified_velocity.resize(new_size);

    // 4. SIMD 极其友好的连续内存灌入
    for (sire::Size i = 0; i < n; ++i) {
        sire::Size dst_idx = start_idx + i;
        const auto& p = pairs[preservedIdx[i]];
        
        db.contacts.id_A[dst_idx] = p.id_A;
        db.contacts.id_B[dst_idx] = p.id_B;
        db.contacts.initial_depth[dst_idx]    = realDepth[i] * stiffScale;
        db.contacts.initial_velocity[dst_idx] = initial_x0[i + n];
        db.contacts.modified_depth[dst_idx]   = modified_x0[i] * stiffScale;
        db.contacts.modified_velocity[dst_idx]= modified_x0[i + n];
    }
}

// // 纯数据转换函数：只在需要的时候把 POD 组装成 JSON
// nlohmann::json buildContactLogJson(const ContactDatabase& log_data) {
//     nlohmann::json root = nlohmann::json::array();

//     for (const auto& frame : log_data.frames) {
//         nlohmann::json frame_json;
//         frame_json["time"] = frame.current_time;
//         frame_json["dt"] = frame.suggest_dt;
//         frame_json["min_time"] = frame.min_time;
        
//         nlohmann::json contacts = nlohmann::json::array();
//         for (sire::Size i = 0; i < frame.count; ++i) {
//             contacts.push_back({
//                 {"id_A", frame.id_A[i]},
//                 {"id_B", frame.id_B[i]},
//                 {"initialDepth", frame.initial_depth[i]},
//                 {"initialVel", frame.initial_velocity[i]},
//                 {"modifiedDepth", frame.modified_depth[i]},
//                 {"modifiedVel", frame.modified_velocity[i]}
//             });
//         }
//         frame_json["contacts"] = std::move(contacts);
//         root.push_back(std::move(frame_json));
//     }
    
//     return root;
// }

class PointPairContactInfo {
 public:
  PointPairContactInfo(sire::PartId partId_A, sire::PartId partId_B,
                       const double* pe_WC, double separation_speed,
                       double slip_speed,
                       const PenetrationAsPointPair& point_pair,
                       const double* fs_WC, const double* f_WC,
                       const double* fs_WC_vel = nullptr,
                       const double* f_WC_vel = nullptr)
      : partId_A_(partId_A),
        partId_B_(partId_B),
        separation_speed_(separation_speed),
        slip_speed_(slip_speed),
        point_pair_(point_pair) {
    std::copy(pe_WC, pe_WC + 6, pe_WC_);
    std::copy(fs_WC, fs_WC + 6, fs_WC_);
    std::copy(f_WC, f_WC + 3, f_WC_);
    if (fs_WC_vel == nullptr)
      std::fill(fs_WC_vel_, fs_WC_vel_ + 6, 0.0);
    else
      std::copy(fs_WC_vel, fs_WC_vel + 6, fs_WC_vel_);
    
    if (f_WC_vel == nullptr)
      std::fill(f_WC_vel_, f_WC_vel_ + 3, 0.0);
    else  
      std::copy(f_WC_vel, f_WC_vel + 3, f_WC_vel_);
  };
  ~PointPairContactInfo() {};
  SIRE_DEFINE_TO_JSON_HEAD(PointPairContactInfo) {
    j["partId_A"] = partId_A_;
    j["partId_B"] = partId_B_;
    j["contactWrench"] = std::vector<double>(fs_WC_, fs_WC_ + 6);
    j["contactWrenchVel"] = std::vector<double>(fs_WC_vel_, fs_WC_vel_ + 6);
    j["contactForce"] = std::vector<double>(f_WC_, f_WC_ + 3);
    j["contactForceVel"] = std::vector<double>(f_WC_vel_, f_WC_vel_ + 3);
    j["contact_point_pe"] = std::vector<double>(pe_WC_, pe_WC_ + 6);
    j["separation_speed"] = separation_speed_;
    point_pair_.to_json(j["point_pair"]);
    j["slip_speed"] = slip_speed_;
  }
  ARIS_DEFINE_BIG_FOUR(PointPairContactInfo);

  sire::PartId partId_A() const { return partId_A_; }

  sire::PartId partId_B() const { return partId_B_; }

  const double* contact_point_pe() const { return pe_WC_; };

  double separation_speed() const { return separation_speed_; };

  double slip_speed() const { return slip_speed_; };

  const PenetrationAsPointPair& point_pair() const { return point_pair_; }

  const double* contact_force() const { return fs_WC_; };

  const double* contact_force_vector() const { return f_WC_; };

  const double* contact_force_vel() const { return fs_WC_vel_; };

  const double* contact_force_vel_vector() const { return f_WC_vel_; };

 private:
  /** The id of the first geometry in the contact. */
  sire::PartId partId_A_;
  /** The id of the second geometry in the contact. */
  sire::PartId partId_B_;
  /** Contact point position euler angle 313 in world frame*/
  double pe_WC_[6];
  /** The penetration depth. Should be positive*/
  double separation_speed_;
  double slip_speed_;
  PenetrationAsPointPair point_pair_;
  /** Contact force screw from A to B in world frame. */
  double fs_WC_[6];
  /** Contact force vetor in world frame*/
  double f_WC_[3];
  // contact force for update velocity only
  double fs_WC_vel_[6];
  // contact force for update velocity only
  double f_WC_vel_[3];
};
}  // namespace sire::physics::common
#endif