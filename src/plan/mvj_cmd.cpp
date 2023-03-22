#include "sire/plan/mvj_cmd.hpp"

#include "sire/controller/controller_sensor.hpp"
#include "sire/ext/json.hpp"
#include "sire/server/interface.hpp"

#define CHECK_PARAM_STRING                           \
  "		<UniqueParam default=\"check_all\">"            \
  "			<Param name=\"check_all\"/>"                   \
  "			<Param name=\"check_none\"/>"                  \
  "			<GroupParam>"                                  \
  "				<UniqueParam default=\"check_enable\">"       \
  "					<Param name=\"check_enable\"/>"              \
  "					<Param name=\"not_check_enable\"/>"          \
  "				</UniqueParam>"                               \
  "				<UniqueParam default=\"check_pos\">"          \
  "					<Param name=\"check_pos\"/>"                 \
  "					<Param name=\"not_check_pos\"/>"             \
  "					<GroupParam>"                                \
  "						<UniqueParam "                              \
  "default=\"check_pos_max\">"                       \
  "							<Param "                                   \
  "name=\"check_pos_max\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_pos_max\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_min\">"                       \
  "							<Param "                                   \
  "name=\"check_pos_min\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_pos_min\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_continuous\">"                \
  "							<Param "                                   \
  "name=\"check_pos_continuous\"/>"                  \
  "							<Param "                                   \
  "name=\"not_check_pos_continuous\"/>"              \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_continuous_second_order\">"   \
  "							<Param "                                   \
  "name=\"check_pos_continuous_second_order\"/>"     \
  "							<Param "                                   \
  "name=\"not_check_pos_continuous_second_order\"/>" \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_following_error\">"           \
  "							<Param "                                   \
  "name=\"check_pos_following_error\"/>"             \
  "							<Param "                                   \
  "name=\"not_check_pos_following_error\"/>"         \
  "						</UniqueParam>"                             \
  "					</GroupParam>"                               \
  "				</UniqueParam>"                               \
  "				<UniqueParam default=\"check_vel\">"          \
  "					<Param name=\"check_vel\"/>"                 \
  "					<Param name=\"not_check_vel\"/>"             \
  "					<GroupParam>"                                \
  "						<UniqueParam "                              \
  "default=\"check_vel_max\">"                       \
  "							<Param "                                   \
  "name=\"check_vel_max\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_vel_max\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_vel_min\">"                       \
  "							<Param "                                   \
  "name=\"check_vel_min\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_vel_min\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_vel_continuous\">"                \
  "							<Param "                                   \
  "name=\"check_vel_continuous\"/>"                  \
  "							<Param "                                   \
  "name=\"not_check_vel_continuous\"/>"              \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_vel_following_error\">"           \
  "							<Param "                                   \
  "name=\"check_vel_following_error\"/>"             \
  "							<Param "                                   \
  "name=\"not_check_vel_following_error\"/>"         \
  "						</UniqueParam>"                             \
  "					</GroupParam>"                               \
  "				</UniqueParam>"                               \
  "			</GroupParam>"                                 \
  "		</UniqueParam>"

namespace sire::plan {
auto set_check_option(
    const std::map<std::string_view, std::string_view>& cmd_params,
    aris::plan::Plan& plan) -> void {
  for (const auto& cmd_param : cmd_params) {
    if (cmd_param.first == "check_all") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_POS_MIN |
                    aris::plan::Plan::NOT_CHECK_POS_MAX |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                    aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
                    aris::plan::Plan::NOT_CHECK_VEL_MIN |
                    aris::plan::Plan::NOT_CHECK_VEL_MAX |
                    aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
    } else if (cmd_param.first == "check_none") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MIN |
                  aris::plan::Plan::NOT_CHECK_POS_MAX |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                  aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
                  aris::plan::Plan::NOT_CHECK_VEL_MIN |
                  aris::plan::Plan::NOT_CHECK_VEL_MAX |
                  aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_enable") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_ENABLE);
    } else if (cmd_param.first == "not_check_enable") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_ENABLE;
    } else if (cmd_param.first == "check_pos") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_POS_MIN |
                    aris::plan::Plan::NOT_CHECK_POS_MAX |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                    aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
    } else if (cmd_param.first == "not_check_pos") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MIN |
                  aris::plan::Plan::NOT_CHECK_POS_MAX |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                  aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_vel") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_VEL_MIN |
                    aris::plan::Plan::NOT_CHECK_VEL_MAX |
                    aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
    } else if (cmd_param.first == "not_check_vel") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_MIN |
                  aris::plan::Plan::NOT_CHECK_VEL_MAX |
                  aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_pos_min") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_MIN;
    } else if (cmd_param.first == "not_check_pos_min") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MIN;
    } else if (cmd_param.first == "check_pos_max") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_MAX;
    } else if (cmd_param.first == "not_check_pos_max") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MAX;
    } else if (cmd_param.first == "check_pos_continuous") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
    } else if (cmd_param.first == "not_check_pos_continuous") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
    } else if (cmd_param.first == "check_pos_continuous_second_order") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    } else if (cmd_param.first == "not_check_pos_continuous_second_order") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    } else if (cmd_param.first == "check_pos_following_error") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    } else if (cmd_param.first == "not_check_pos_following_error") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_vel_min") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_MIN;
    } else if (cmd_param.first == "not_check_vel_min") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_MIN;
    } else if (cmd_param.first == "check_vel_max") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_MAX;
    } else if (cmd_param.first == "not_check_vel_max") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_MAX;
    } else if (cmd_param.first == "check_vel_continuous") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
    } else if (cmd_param.first == "not_check_vel_continuous") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
    } else if (cmd_param.first == "check_vel_following_error") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    } else if (cmd_param.first == "not_check_vel_following_error") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    }
  }
}

auto check_eul_validity(std::string_view eul_type) -> bool {
  if (eul_type.size() < 3) return false;

  for (int i = 0; i < 3; ++i)
    if (eul_type[i] > '3' || eul_type[i] < '1') return false;

  if (eul_type[0] == eul_type[1] || eul_type[1] == eul_type[2]) return false;

  return true;
}

auto find_pq(const std::map<std::string_view, std::string_view>& params,
             aris::plan::Plan& plan, double* pq_out) -> bool {
  double pos_unit;
  auto pos_unit_found = params.find("pos_unit");
  if (pos_unit_found == params.end())
    pos_unit = 1.0;
  else if (pos_unit_found->second == "m")
    pos_unit = 1.0;
  else if (pos_unit_found->second == "mm")
    pos_unit = 0.001;
  else if (pos_unit_found->second == "cm")
    pos_unit = 0.01;
  else
    THROW_FILE_LINE("");

  for (const auto& cmd_param : params) {
    if (cmd_param.first == "pq") {
      auto pq_mat = plan.matrixParam(cmd_param.first);
      if (pq_mat.size() != 7) THROW_FILE_LINE("");
      aris::dynamic::s_vc(7, pq_mat.data(), pq_out);
      aris::dynamic::s_nv(3, pos_unit, pq_out);
      return true;
    } else if (cmd_param.first == "pm") {
      auto pm_mat = plan.matrixParam(cmd_param.first);
      if (pm_mat.size() != 16) THROW_FILE_LINE("");
      aris::dynamic::s_pm2pq(pm_mat.data(), pq_out);
      aris::dynamic::s_nv(3, pos_unit, pq_out);
      return true;
    } else if (cmd_param.first == "pe") {
      double ori_unit;
      auto ori_unit_found = params.find("ori_unit");
      if (ori_unit_found == params.end())
        ori_unit = 1.0;
      else if (ori_unit_found->second == "rad")
        ori_unit = 1.0;
      else if (ori_unit_found->second == "degree")
        ori_unit = aris::PI / 180.0;
      else
        THROW_FILE_LINE("");

      std::string eul_type;
      auto eul_type_found = params.find("eul_type");
      if (eul_type_found == params.end())
        eul_type = "321";
      else if (check_eul_validity(eul_type_found->second.data()))
        eul_type = eul_type_found->second;
      else
        THROW_FILE_LINE("");

      auto pe_mat = plan.matrixParam(cmd_param.first);
      if (pe_mat.size() != 6) THROW_FILE_LINE("");
      aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
      aris::dynamic::s_pe2pq(pe_mat.data(), pq_out, eul_type.data());
      aris::dynamic::s_nv(3, pos_unit, pq_out);
      return true;
    }
  }

  THROW_FILE_LINE("No pose input");
}

struct SireMoveJParam {
  std::vector<double> joint_vel, joint_acc, joint_dec, ee_pq, joint_pos_begin,
      joint_pos_end;
  std::vector<sire::Size> total_count;
};
struct SireMoveJ::Imp {};
auto SireMoveJ::prepareNrt() -> void {
  set_check_option(cmdParams(), *this);

  SireMoveJParam mvj_param;

  // find model index
  int model_index;
  auto model_index_found = cmdParams().find("model_index");
  if (model_index_found != cmdParams().end())
    model_index = int32Param("model_index");
  else
    model_index = 0;
  auto m = dynamic_cast<aris::dynamic::MultiModel*>(modelBase());
  if (m != nullptr) {
    model_index = model_index < m->subModels().size() ? model_index : 0;
    this->setModelBase(&m->subModels().at(model_index)); 
  }

  // find ee pq //
  mvj_param.ee_pq.resize(7);
  find_pq(cmdParams(), *this, mvj_param.ee_pq.data());

  mvj_param.joint_pos_begin.resize(model()->motionPool().size(), 0.0);
  mvj_param.joint_pos_end.resize(model()->motionPool().size(), 0.0);
  mvj_param.total_count.resize(model()->motionPool().size(), 0);

  // find joint acc/vel/dec/
  for (const auto& cmd_param : cmdParams()) {
    auto c = controller();
    if (cmd_param.first == "joint_acc") {
      mvj_param.joint_acc.clear();
      mvj_param.joint_acc.resize(model()->motionPool().size(), 0.0);

      auto acc_mat = matrixParam(cmd_param.first);
      if (acc_mat.size() == 1)
        std::fill(mvj_param.joint_acc.begin(), mvj_param.joint_acc.end(),
                  acc_mat.toDouble());
      else if (acc_mat.size() == model()->motionPool().size())
        std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.joint_acc.begin());
      else
        THROW_FILE_LINE("");

      for (int i = 0; i < controller()->motorPool().size(); ++i)
        mvj_param.joint_acc[i] *= controller()->motorPool()[i].maxAcc();

      // check value validity //
      for (sire::Size i = 0;
           i < std::min(model()->motionPool().size(), c->motorPool().size());
           ++i)
        if (mvj_param.joint_acc[i] <= 0 ||
            mvj_param.joint_acc[i] > c->motorPool()[i].maxAcc())
          THROW_FILE_LINE("");
    } else if (cmd_param.first == "joint_vel") {
      mvj_param.joint_vel.clear();
      mvj_param.joint_vel.resize(model()->motionPool().size(), 0.0);

      auto vel_mat = matrixParam(cmd_param.first);
      if (vel_mat.size() == 1)
        std::fill(mvj_param.joint_vel.begin(), mvj_param.joint_vel.end(),
                  vel_mat.toDouble());
      else if (vel_mat.size() == model()->motionPool().size())
        std::copy(vel_mat.begin(), vel_mat.end(), mvj_param.joint_vel.begin());
      else
        THROW_FILE_LINE("");

      for (int i = 0; i < controller()->motorPool().size(); ++i)
        mvj_param.joint_vel[i] *= controller()->motorPool()[i].maxVel();

      // check value validity //
      for (sire::Size i = 0;
           i < std::min(model()->motionPool().size(), c->motorPool().size());
           ++i)
        if (mvj_param.joint_vel[i] <= 0 ||
            mvj_param.joint_vel[i] > c->motorPool()[i].maxVel())
          THROW_FILE_LINE("");
    } else if (cmd_param.first == "joint_dec") {
      mvj_param.joint_dec.clear();
      mvj_param.joint_dec.resize(model()->motionPool().size(), 0.0);

      auto dec_mat = matrixParam(cmd_param.first);
      if (dec_mat.size() == 1)
        std::fill(mvj_param.joint_dec.begin(), mvj_param.joint_dec.end(),
                  dec_mat.toDouble());
      else if (dec_mat.size() == model()->motionPool().size())
        std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_dec.begin());
      else
        THROW_FILE_LINE("");

      for (int i = 0; i < controller()->motorPool().size(); ++i)
        mvj_param.joint_dec[i] *= controller()->motorPool()[i].maxAcc();

      // check value validity //
      for (sire::Size i = 0;
           i < std::min(model()->motionPool().size(), c->motorPool().size());
           ++i)
        if (mvj_param.joint_dec[i] <= 0 ||
            mvj_param.joint_dec[i] > c->motorPool()[i].maxAcc())
          THROW_FILE_LINE("");
    }
  }

  this->param() = mvj_param;

  for (auto& option : motorOptions())
    option |= aris::plan::Plan::USE_TARGET_POS;

  std::vector<std::pair<std::string, std::any>> ret_value;
  ret() = ret_value;
}
auto SireMoveJ::executeRT() -> int {
  auto mvj_param = std::any_cast<SireMoveJParam>(&this->param());

  // 取得起始位置 //
  double p, v, a;
  static sire::Size max_total_count;
  if (count() == 1) {
    // begin pos //
    model()->getInputPos(mvj_param->joint_pos_begin.data());

    // inverse kinematic //
    auto& gm = model()->generalMotionPool().at(0);
    double end_pe321[6]{0.0};
    aris::dynamic::s_pq2pe(mvj_param->ee_pq.data(), end_pe321, "321");

    gm.setP(end_pe321);
    if (model()->solverPool().at(0).kinPos()) return -1;

    // compute max count //
    for (sire::Size i = 0; i < std::min(controller()->motorPool().size(),
                                        model()->motionPool().size());
         ++i) {
      mvj_param->joint_pos_end[i] = *model()->motionPool()[i].p();
      aris::plan::moveAbsolute(
          static_cast<double>(count()), mvj_param->joint_pos_begin[i],
          mvj_param->joint_pos_end[i], mvj_param->joint_vel[i] / 1000,
          mvj_param->joint_acc[i] / 1000 / 1000,
          mvj_param->joint_dec[i] / 1000 / 1000, p, v, a,
          mvj_param->total_count[i]);
    }

    max_total_count = *std::max_element(mvj_param->total_count.begin(),
                                        mvj_param->total_count.end());
  }

  for (sire::Size i = 0; i < std::min(controller()->motorPool().size(),
                                      model()->motionPool().size());
       ++i) {
    aris::plan::moveAbsolute(
        static_cast<double>(count()) * mvj_param->total_count[i] /
            max_total_count,
        mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
        mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000,
        mvj_param->joint_dec[i] / 1000 / 1000, p, v, a,
        mvj_param->total_count[i]);

    model()->setInputPosAt(p, i);
    model()->setInputVelAt(v, i);
    model()->setInputAccAt(a, i);
    model()->forwardKinematics();
    model()->forwardKinematicsVel();
    model()->forwardKinematicsAcc();
  }

  return max_total_count == 0 ? 0 : static_cast<int>(max_total_count - count());
}
SireMoveJ::~SireMoveJ() = default;
SireMoveJ::SireMoveJ(const std::string& name) : imp_(new Imp) {
  aris::core::fromXmlString(
      command(),
      "<Command name=\"sire_mvj\">"
      "	<GroupParam>"
      "		<Param name=\"model_index\" default=\"0\"/>"
      "		<Param name=\"pos_unit\" default=\"m\"/>"
      "		<UniqueParam default=\"pq\">"
      "			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
      "			<Param name=\"pm\" "
      "default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
      "			<GroupParam>"
      "				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
      "				<Param name=\"ori_unit\" default=\"rad\"/>"
      "				<Param name=\"eul_type\" default=\"321\"/>"
      "			</GroupParam>"
      "		</UniqueParam>"
      "		<Param name=\"joint_acc\" default=\"0.1\"/>"
      "		<Param name=\"joint_vel\" default=\"0.1\"/>"
      "		<Param name=\"joint_dec\" default=\"0.1\"/>" CHECK_PARAM_STRING
      "	</GroupParam>"
      "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(SireMoveJ);

ARIS_REGISTRATION {
  aris::core::class_<SireMoveJ>("SireMoveJ").inherit<aris::plan::Plan>();
}
}  // namespace sire::plan