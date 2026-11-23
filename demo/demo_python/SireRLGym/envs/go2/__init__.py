from .go2_config import GO2RoughCfg, GO2RoughCfgPPO
from .go2_threshold_config import GO2ThresholdCfg, GO2ThresholdCfgPPO

try:
    from .go2_threshold import GO2Threshold
except ModuleNotFoundError as error:
    if error.name != "mujoco":
        raise
    GO2Threshold = None

__all__ = ["GO2RoughCfg", "GO2RoughCfgPPO", "GO2Threshold", "GO2ThresholdCfg", "GO2ThresholdCfgPPO"]
