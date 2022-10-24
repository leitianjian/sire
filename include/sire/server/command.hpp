#ifndef COMMAND_H_
#define COMMAND_H_

#include <aris.hpp>

namespace sire::server {
class Get : public aris::core::CloneObject<Get, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void;
  auto virtual collectNrt() -> void;
  explicit Get(const std::string& name = "Get_plan");
};

class GetForce : public aris::core::CloneObject<GetForce, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void;
  auto virtual collectNrt() -> void;
  explicit GetForce(const std::string& name = "GetForce_plan");
};

	/// \brief 将机器人从轴空间移动到某个位姿。
///
///
/// ### 参数定义 ###
///
/// 指定目标位姿，可以使用以下参数：
/// + 位置与四元数，例如位置 xyz 为[0 , 0.5 , 1.1]，姿态为原始姿态[0 , 0 , 0 ,
/// 1]：“mvj --pe={0,0.5,1.1,0,0,0,1}”
/// + 位姿矩阵，位姿仍然如上：“mvj --pm={1,0,0,0,0,1,0,0.5,0,0,1,1.1,0,0,0,1}”
/// + 位置与欧拉角，位置姿态仍然如上：“mvj --pe={0,0.5,1.1,0,0,0}”
///
/// 此外，还可以指定位置和角度的单位，长度单位默认为 m ，角度默认为 rad ：
/// + 指定位置单位，例如将单位设置为 m （米）：“mvj --pq={0,0.5,1.1,0,0,0,1}
/// --pos_unit=m”
/// + 指定角度单位，例如将单位设置为 rad ：“mvj --pe={0,0.5,1.1,0,0,0}
/// --ori_unit=rad”
///
/// 还可以指定欧拉角的种类，可以是 321 313 123 212 ...
/// 等任意类型的欧拉角，默认为 321 的欧拉角
/// + 指定欧拉角为 123 的，“mvj --pe={0,0.5,1.1,0,0,0} --ori_unit=rad
/// --eul_type=321”
///
/// 指定关节速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为0.1
/// + 指定所有电机的速度都为0.5：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5”
/// + 指定所有电机的速度：“mvj --pe={0,0.5,1.1,0,0,0}
/// --joint_vel={0.2,0.2,0.2,0.3,0.3,0.3}”
///
/// 指定关节加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
/// + 指定所有电机的加速度都为0.3：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5
/// --joint_acc=0.3”
/// + 指定所有电机的加速度：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5
/// --joint_acc={0.2,0.2,0.2,0.3,0.3,0.3}”
///
/// 指定关节减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
/// + 指定所有电机的加速度都为0.3：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5
/// --joint_dec=0.3”
/// + 指定所有电机的加速度：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5
/// --joint_dec={0.2,0.2,0.2,0.3,0.3,0.3}”
///
class SireMoveJ
    : public aris::core::CloneObject<SireMoveJ, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual executeRT() -> int override;

  virtual ~SireMoveJ();
  explicit SireMoveJ(const std::string& name = "move_j");
  ARIS_DECLARE_BIG_FOUR(SireMoveJ);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}
#endif