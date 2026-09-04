from .legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

try:
    from .legged_robot import LeggedRobot
except ModuleNotFoundError as error:
    if error.name != "mujoco":
        raise
    LeggedRobot = None

__all__ = ["LeggedRobot", "LeggedRobotCfg", "LeggedRobotCfgPPO"]
