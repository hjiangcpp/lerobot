from .config import RobotConfig
from .robot import Robot
from .utils import make_robot_from_config

# Import all robot configurations to ensure they are registered with draccus
from . import so100_follower  # noqa: F401
from . import so101_follower  # noqa: F401
from . import koch_follower  # noqa: F401
from . import lekiwi  # noqa: F401
try:
    from . import stretch3  # noqa: F401
except ImportError:
    pass  # stretch3 requires stretch_body which may not be installed
from . import viperx  # noqa: F401
from . import hope_jr  # noqa: F401
from . import bi_so100_follower  # noqa: F401
