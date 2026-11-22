"""香橙派 5 行人检测与雷达-视觉融合。

检测只保留 person。融合在雷达平面上做时间对齐、外参投影和门控关联。
"""

from .fusion import FusionEngine, FusionTrack
from .config import load_config

__all__ = ["FusionEngine", "FusionTrack", "load_config"]
