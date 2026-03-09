"""
主农田覆盖路径规划器类，负责协调所有组件。
"""

from typing import Optional, List, Dict, Any
from pathlib import Path

from .core.field import Field
from .core.waypoint import WaypointSequence
from .algorithms.boustrophedon import BoustrophedonPlanner
from .io.csv_handler import CSVHandler
from .io.ros_handler import ROSHandler
from .utils.validation import validate_field_parameters, validate_waypoint_sequence


class FieldCoveragePlanner:
    """
    农田覆盖路径规划的主类。

    该类提供了高级接口，用于：
    - 从 CSV 文件或 ROS 主题加载农田边界
    - 生成最优覆盖路径
    - 将航点导出为 CSV 文件
    - 验证输入和输出
    """

    def __init__(self,
                 swath_width: float = 2.0,
                 overlap: float = 0.1,
                 turn_radius: float = 2.0,
                 speed: float = 2.0,
                 algorithm: str = 'boustrophedon'):
        """
        初始化农田覆盖路径规划器。

        参数:
            swath_width: 作业条带的宽度（单位：米）
            overlap: 条带间的重叠比例 (0.0 到 1.0)
            turn_radius: 最小转弯半径（单位：米）
            speed: 默认航点速度（单位：米/秒）
            algorithm: 覆盖算法 ('boustrophedon  牛耕试', 'spiral  螺旋', 等)
        """
        self.swath_width = swath_width
        self.overlap = overlap
        self.turn_radius = turn_radius
        self.speed = speed
        self.algorithm = algorithm  #算法

        # 初始化组件
        self.csv_handler = CSVHandler()
        self.ros_handler = None  # 按需初始化

        # 初始化算法
        if algorithm == 'boustrophedon':
            self.planner = BoustrophedonPlanner(
                swath_width=swath_width,
                overlap=overlap,
                turn_radius=turn_radius,
                speed=speed
            )
        else:
            raise ValueError(f"不支持的算法: {algorithm}")

    def load_field_from_csv(self,
                           csv_file_path: str,
                           hole_files: Optional[List[str]] = None,
                           field_id: Optional[str] = None,
                           validate: bool = True) -> Field:
        """
        从 CSV 文件加载农田边界。

        参数:
            csv_file_path: 包含农田边界坐标的 CSV 文件路径
            hole_files: 可选的包含孔洞边界的 CSV 文件列表
            field_id: 可选的农田标识符
            validate: 是否验证农田参数

        返回:
            从 CSV 加载的 Field 对象

        异常:
            ValueError: 如果验证失败或文件格式无效
        """
        try:
            # 从 CSV 加载农田
            if hole_files:
                field = self.csv_handler.read_field_with_holes_csv(
                    main_boundary_file=csv_file_path,
                    hole_files=hole_files,
                    field_id=field_id
                )
            else:
                # 使用 CSVHandler 提供的读取函数，内部已包含格式校验
                coordinates = self.csv_handler.read_field_boundary_csv(csv_file_path, validate=True)
                field = Field.from_gps_coordinates(
                    coordinates=coordinates,
                    field_id=field_id or Path(csv_file_path).stem
                )

            # 如果请求则验证农田参数
            if validate:
                errors = validate_field_parameters(
                    field=field,
                    swath_width=self.swath_width,
                    overlap=self.overlap,
                    min_turn_radius=self.turn_radius
                )
                if errors:
                    raise ValueError(f"农田验证失败: {'; '.join(errors)}")

            return field

        except Exception as e:
            raise ValueError(f"从 CSV 加载农田失败: {str(e)}")

    def load_field_from_ros(self,
                          topic_name: str = '/field_boundary',
                          timeout: float = 30.0) -> Optional[Field]:
        """
        从 ROS 主题加载农田边界。

        参数:
            topic_name: 农田边界的 ROS 主题名称
            timeout: 等待消息的最大时间（单位：秒）

        返回:
            如果收到消息则返回 Field 对象，超时则返回 None
        """
        if self.ros_handler is None:
            self.ros_handler = ROSHandler()

        return self.ros_handler.wait_for_field_boundary(topic_name, timeout)

    def generate_coverage_path(self,
                             field: Field,
                             direction: Optional[float] = None,
                             optimize_direction: bool = True,
                             optimization_step: float = 15.0,
                             validate_output: bool = True) -> WaypointSequence:
        """
        为农田生成覆盖路径。

        参数:
            field: 为其生成覆盖路径的农田
            direction: 覆盖方向（单位：度）。如果为 None，则使用最优方向。
            optimize_direction: 是否优化覆盖方向
            optimization_step: 优化时的角度步长（单位：度，默认: 15.0）
            validate_output: 是否验证生成的航点

        返回:
            包含覆盖路径的 WaypointSequence 对象

        异常:
            ValueError: 如果路径生成或验证失败
        """
        try:
            # 生成覆盖路径（如果方向为 None，规划器将自动优化方向）
            waypoints = self.planner.plan_coverage(
                field=field,
                direction=direction,
                optimization_step=optimization_step
            )

            # 如果请求则验证输出
            if validate_output:
                errors = validate_waypoint_sequence(
                    waypoints=waypoints,
                    max_speed=self.speed * 2,  # 允许一些容差
                    max_heading_change=90.0
                )
                if errors:
                    print(f"警告: 航点验证出现问题: {'; '.join(errors)}")

            return waypoints

        except Exception as e:
            raise ValueError(f"生成覆盖路径失败: {str(e)}")

    def export_waypoints(self,
                        waypoints: WaypointSequence,
                        output_file: str,
                        include_metadata: bool = False) -> None:
        """
        将航点导出为 CSV 文件。

        参数:
            waypoints: 要导出的 WaypointSequence 对象
            output_file: 输出 CSV 文件路径
            include_metadata: 是否包含航点元数据
        """
        try:
            self.csv_handler.write_waypoints_csv(
                waypoints=waypoints,
                file_path=output_file,
                include_metadata=include_metadata
            )
        except Exception as e:
            raise ValueError(f"导出航点失败: {str(e)}")

    def publish_waypoints_ros(self,
                            waypoints: WaypointSequence,
                            topic_name: str = '/waypoints') -> None:
        """
        将航点发布到 ROS 主题。

        参数:
            waypoints: 要发布的 WaypointSequence 对象
            topic_name: 航点的 ROS 主题名称
        """
        if self.ros_handler is None:
            self.ros_handler = ROSHandler()

        self.ros_handler.publish_waypoints(waypoints, topic_name)

    def generate_field_report(self,
                            field: Field,
                            waypoints: WaypointSequence,
                            output_file: str) -> Dict[str, Any]:
        """
        生成全面的农田覆盖报告。

        参数:
            field: 已规划的农田
            waypoints: 生成的航点
            output_file: 详细报告的输出文件路径

        返回:
            包含报告摘要的字典
        """
        try:
            # 计算基本指标
            field_area = field.calculate_area()
            total_distance = waypoints.total_distance()
            total_time = waypoints.total_time()

            # 计算覆盖指标
            if hasattr(self.planner, 'calculate_coverage_area'):
                coverage_area = self.planner.calculate_coverage_area(waypoints)
                coverage_efficiency = (coverage_area / field_area) * 100
            else:
                # 简化的覆盖计算
                estimated_coverage = min(field_area, total_distance * self.swath_width)
                coverage_efficiency = (estimated_coverage / field_area) * 100

            # 准备报告数据
            report_summary = {
                'field_id': field.field_id,
                'field_area_m2': field_area,
                'field_perimeter_m': field.calculate_perimeter(),
                'total_path_distance_m': total_distance,
                'estimated_time_min': total_time / 60,
                'number_of_waypoints': len(waypoints),
                'coverage_efficiency_percent': coverage_efficiency,
                'swath_width_m': self.swath_width,
                'overlap_percent': self.overlap * 100,
                'algorithm': self.algorithm
            }

            # 将详细报告写入 CSV
            parameters = {
                'swath_width': self.swath_width,
                'overlap': self.overlap,
                'turn_radius': self.turn_radius,
                'speed': self.speed,
                'algorithm': self.algorithm
            }

            self.csv_handler.write_coverage_report_csv(
                field=field,
                waypoints=waypoints,
                parameters=parameters,
                file_path=output_file
            )

            return report_summary

        except Exception as e:
            raise ValueError(f"生成农田报告失败: {str(e)}")

    def validate_configuration(self) -> List[str]:
        """
        验证当前规划器配置。

        返回:
            验证错误信息列表（如果有效则为空列表）
        """
        errors = []

        # 验证条带宽度
        if self.swath_width <= 0:
            errors.append(f"无效的条带宽度: {self.swath_width}")

        # 验证重叠率
        if not (0.0 <= self.overlap < 1.0):
            errors.append(f"无效的重叠率: {self.overlap}")

        # 验证转弯半径
        if self.turn_radius <= 0:
            errors.append(f"无效的转弯半径: {self.turn_radius}")

        # 验证速度
        if self.speed <= 0:
            errors.append(f"无效的速度: {self.speed}")

        return errors

    def set_parameters(self,
                      swath_width: Optional[float] = None,
                      overlap: Optional[float] = None,
                      turn_radius: Optional[float] = None,
                      speed: Optional[float] = None) -> None:
        """
        更新规划器参数。

        参数:
            swath_width: 新的条带宽度（单位：米）
            overlap: 新的重叠比例
            turn_radius: 新的转弯半径（单位：米）
            speed: 新的默认速度（单位：米/秒）
        """
        if swath_width is not None:
            self.swath_width = swath_width
            self.planner.swath_width = swath_width

        if overlap is not None:
            self.overlap = overlap
            self.planner.overlap = overlap

        if turn_radius is not None:
            self.turn_radius = turn_radius
            self.planner.turn_radius = turn_radius

        if speed is not None:
            self.speed = speed
            self.planner.speed = speed

    def get_parameters(self) -> Dict[str, Any]:
        """
        获取当前规划器参数。

        返回:
            包含当前参数的字典
        """
        return {
            'swath_width': self.swath_width,
            'overlap': self.overlap,
            'turn_radius': self.turn_radius,
            'speed': self.speed,
            'algorithm': self.algorithm
        }

    def process_field_file(self,
                         input_csv: str,
                         output_csv: str,
                         direction: Optional[float] = None,
                         field_id: Optional[str] = None) -> Dict[str, Any]:
        """
        完整工作流程：加载农田，生成路径，导出航点。

        参数:
            input_csv: 包含农田边界的输入 CSV 文件
            output_csv: 航点的输出 CSV 文件
            direction: 覆盖方向（None 表示最优方向）
            field_id: 可选的农田标识符

        返回:
            包含处理摘要的字典
        """
        try:
            # 加载农田
            field = self.load_field_from_csv(input_csv, field_id=field_id)

            # 生成覆盖路径
            waypoints = self.generate_coverage_path(field, direction=direction)

            # 导出航点
            self.export_waypoints(waypoints, output_csv)

            # 生成摘要
            summary = {
                'input_file': input_csv,
                'output_file': output_csv,
                'field_area_m2': field.calculate_area(),
                'waypoints_generated': len(waypoints),
                'total_distance_m': waypoints.total_distance(),
                'estimated_time_min': waypoints.total_time() / 60,
                'success': True
            }

            return summary

        except Exception as e:
            return {
                'input_file': input_csv,
                'output_file': output_csv,
                'error': str(e),
                'success': False
            }