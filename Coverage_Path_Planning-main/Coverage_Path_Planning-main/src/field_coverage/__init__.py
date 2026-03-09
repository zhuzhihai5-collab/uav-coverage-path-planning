"""
农田覆盖路径规划包
用于自主农田覆盖路径规划的综合Python包
"""

# 是否支持可视化功能
VISUALIZATION_AVAILABLE = False

try:
    # 导入核心模块
    from .core.coordinates import GPSCoordinate, UTMCoordinate, CoordinateTransformer
    from .core.field import Field, FieldBoundary
    from .core.waypoint import Waypoint, WaypointSequence
    from .algorithms.boustrophedon import BoustrophedonPlanner
    from .main import FieldCoveragePlanner
    from .io.csv_handler import CSVHandler
    from .utils import validation

    # 尝试导入可视化模块（可选依赖）
    try:
        from .visualization.field_plotter import FieldPlotter, create_coverage_visualization
        VISUALIZATION_AVAILABLE = True
    except ImportError:
        VISUALIZATION_AVAILABLE = False
        print("警告：未安装matplotlib，无法使用可视化功能")

except ImportError as e:
    print(f"警告：部分依赖缺失: {e}")

# 包的基本信息
__version__ = "1.0.0"
__author__ = "davoud nikkhouy"
__email__ = "davoudnikkhouy@gmail.com"

# 对外公开的接口
__all__ = [
    'GPSCoordinate', 'UTMCoordinate', 'CoordinateTransformer',
    'Field', 'FieldBoundary', 'Waypoint', 'WaypointSequence',
    'BoustrophedonPlanner', 'FieldCoveragePlanner',
    'CSVHandler', 'validation', 'VISUALIZATION_AVAILABLE'
]

# 如果支持可视化，则加入接口
if VISUALIZATION_AVAILABLE:
    __all__.extend(['FieldPlotter', 'create_coverage_visualization'])


# 下面是备用导入方案（当依赖缺失时提供最小功能）
__version__ = "0.1.0"
__author__ = "Your Name"
__email__ = "your.email@example.com"

try:
    from .core.coordinates import GPSCoordinate, UTMCoordinate, CoordinateTransformer
    from .core.waypoint import Waypoint, WaypointSequence
    from .core.field import Field
    from .algorithms.boustrophedon import BoustrophedonPlanner
    from .io.csv_handler import CSVHandler
    from .main import FieldCoveragePlanner
except ImportError as e:
    import warnings
    warnings.warn(f"部分依赖缺失: {e}，请安装requirements.txt获取完整功能")

    # 尝试导入最基础功能
    try:
        from .core.coordinates import GPSCoordinate
    except ImportError:
        GPSCoordinate = None

    FieldCoveragePlanner = None
    Field = None

# 最小功能对外接口
__all__ = [
    "Field",
    "Waypoint", 
    "WaypointSequence",
    "GPSCoordinate",
    "UTMCoordinate", 
    "CoordinateTransformer",
    "BoustrophedonPlanner",
    "CSVHandler",
    "FieldCoveragePlanner",
]
