from .base.legged_robot_sire import LeggedRobotSire  # Sire production env
from .go2 import GO2RoughCfg, GO2RoughCfgPPO, GO2Threshold, GO2ThresholdCfg, GO2ThresholdCfgPPO

try:
    from .base.legged_robot import LeggedRobot  # MuJoCo comparison only
except ModuleNotFoundError as error:
    if error.name != "mujoco":
        raise
    LeggedRobot = None

__all__ = ["LeggedRobot", "LeggedRobotSire", "GO2RoughCfg", "GO2RoughCfgPPO", "GO2Threshold", "GO2ThresholdCfg", "GO2ThresholdCfgPPO"]
