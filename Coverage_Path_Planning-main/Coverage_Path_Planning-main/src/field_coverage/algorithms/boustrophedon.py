"""
实现 Boustrophedon（牛耕式）覆盖路径规划算法，支持方向处理与自动优化。
"""

import math
from typing import List, Tuple, Optional
from shapely.geometry import LineString, Point
import numpy as np

from ..core.field import Field
from ..core.waypoint import Waypoint, WaypointSequence, WaypointType
from ..core.coordinates import GPSCoordinate, UTMCoordinate, CoordinateTransformer


class BoustrophedonPlanner:
    """
    实现 Boustrophedon（牛耕式）覆盖路径规划，支持方向处理与自动优化。
    """

    def __init__(self,
                 swath_width: float = 2.0,
                 overlap: float = 0.1,
                 turn_radius: float = 2.0,
                 speed: float = 2.0):
        """
        初始化 Boustrophedon 路径规划器。

        参数:
            swath_width: 每条作业条带的宽度（单位：米）
            overlap: 条带之间的重叠比例（0.0 到 1.0 之间）
            turn_radius: 最小转弯半径（单位：米）
            speed: 航点默认速度（单位：米/秒）
        """
        self.swath_width = swath_width
        self.overlap = overlap
        self.turn_radius = turn_radius
        self.speed = speed
        self.coordinate_transformer = CoordinateTransformer()

    def plan_coverage(self,
                     field: Field,
                     direction: Optional[float] = None,
                     start_point: Optional[UTMCoordinate] = None,
                     optimization_step: float = 15.0) -> WaypointSequence:
        """
        生成符合农田边界的 Boustrophedon 覆盖路径。

        参数:
            field: 需要覆盖的农田对象
            direction: 覆盖方向（角度，0°=正北，90°=正东）。若为 None，则自动优化方向。
            start_point: 覆盖起始点（可选）
            optimization_step: 自动优化时的角度步长（单位：度，默认 15.0°）

        返回:
            包含所有覆盖航点的 WaypointSequence 对象
        """
        # 确定覆盖方向
        if direction is None:
            print("🔍 正在优化覆盖方向...")
            direction = self.optimize_direction(field, step_size=optimization_step)
            print(f"✓ 找到最优方向: {direction:.1f}°")
        else:
            print(f"📐 使用指定方向: {direction:.1f}°")

        # 获取农田的边界框和工作区域多边形
        min_bbox, max_bbox = field.get_bounding_box()
        field_polygon = field.get_working_area_polygon(use_utm=True)

        # 使用农田自带的坐标转换器
        transformer = field.coordinate_transformer

        # 计算有效条带间距（考虑重叠）
        effective_swath_width = self.swath_width * (1 - self.overlap)

        waypoints = []
        waypoint_id = 1

        # 将方向转换为弧度用于计算
        direction_rad = math.radians(direction)

        # 计算扫描线方向（垂直于覆盖方向）
        scan_direction_rad = direction_rad + math.pi / 2

        # 获取农田边界范围
        minx, miny, maxx, maxy = field_polygon.bounds
        field_center_x = (minx + maxx) / 2
        field_center_y = (miny + maxy) / 2

        # 计算覆盖整个农田所需的延伸长度（取对角线作为保守估计）
        diagonal = math.sqrt((maxx - minx)**2 + (maxy - miny)**2)

        # 估算所需扫描线数量
        field_width_in_scan_dir = diagonal  # 保守估计
        num_lines = int(field_width_in_scan_dir / effective_swath_width) + 2

        # 生成垂直于覆盖方向的扫描线
        for i in range(-num_lines//2, num_lines//2 + 1):
            # 计算当前扫描线相对于中心的偏移量
            offset = i * effective_swath_width

            # 计算扫描线中心点坐标
            scan_center_x = field_center_x + offset * math.cos(scan_direction_rad)
            scan_center_y = field_center_y + offset * math.sin(scan_direction_rad)

            # 创建一条远超农田边界的扫描线
            line_half_length = diagonal

            # 扫描线起点和终点（沿覆盖方向延伸）
            start_x = scan_center_x - line_half_length * math.cos(direction_rad)
            start_y = scan_center_y - line_half_length * math.sin(direction_rad)
            end_x = scan_center_x + line_half_length * math.cos(direction_rad)
            end_y = scan_center_y + line_half_length * math.sin(direction_rad)

            # 创建扫描线并求其与农田边界的交集
            coverage_line = LineString([(start_x, start_y), (end_x, end_y)])
            intersection = field_polygon.intersection(coverage_line)

            if intersection.is_empty:
                continue

            # 处理不同类型的交集结果
            if hasattr(intersection, 'geoms'):  # 多线段（MultiLineString）
                line_segments = list(intersection.geoms)
            else:  # 单一线段（LineString）
                line_segments = [intersection] if hasattr(intersection, 'coords') else []

            # 处理每一段有效线段
            for segment in line_segments:
                if not hasattr(segment, 'coords'):
                    continue

                coords = list(segment.coords)
                if len(coords) < 2:
                    continue

                # 根据 Boustrophedon 模式确定行进方向
                line_index = i + num_lines//2  # 转换为从 0 开始的索引
                if line_index % 2 == 0:
                    # 偶数行：正向（保持原始顺序）
                    heading = direction
                    start_coord = coords[0]
                    end_coord = coords[-1]
                else:
                    # 奇数行：反向（实现“来回”效果）
                    heading = (direction + 180) % 360
                    start_coord = coords[-1]
                    end_coord = coords[0]

                # 为该线段生成航点
                line_waypoints = self._create_line_waypoints(
                    start_coord[0], start_coord[1],
                    end_coord[0], end_coord[1],
                    transformer, waypoint_id, heading
                )
                waypoints.extend(line_waypoints)
                waypoint_id += len(line_waypoints)

        return WaypointSequence(waypoints)

    def _create_line_waypoints(self,
                             x1: float, y1: float,
                             x2: float, y2: float,
                             transformer: CoordinateTransformer,
                             start_id: int,
                             heading: float,
                             sampling_distance: float = 10.0) -> List[Waypoint]:
        """
        在一条直线上生成航点序列。

        参数:
            x1, y1: 起点 UTM 坐标
            x2, y2: 终点 UTM 坐标
            transformer: 坐标转换器
            start_id: 起始航点 ID
            heading: 航向（角度）
            sampling_distance: 航点采样间隔（单位：米）

        返回:
            该线段上的航点列表
        """
        waypoints = []

        # 计算线段长度和方向
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx**2 + dy**2)

        if length < sampling_distance:
            # 线段很短，只生成起点和终点
            points = [(x1, y1), (x2, y2)]
        else:
            # 沿线段均匀采样
            num_points = max(2, int(length / sampling_distance) + 1)
            points = []

            for i in range(num_points):
                t = i / (num_points - 1)
                x = x1 + t * dx
                y = y1 + t * dy
                points.append((x, y))

        # 创建航点对象
        for i, (x, y) in enumerate(points):
            utm_coord = UTMCoordinate(
                easting=x, northing=y,
                zone=transformer.utm_zone,
                hemisphere=transformer.hemisphere
            )

            gps_coord = transformer.utm_to_gps(utm_coord)

            waypoint = Waypoint(
                gps_coordinate=gps_coord,
                utm_coordinate=utm_coord,
                heading=heading,
                speed=self.speed,
                waypoint_type=WaypointType.COVERAGE,
                waypoint_id=start_id + i
            )
            waypoints.append(waypoint)

        return waypoints

    def optimize_direction(self, field: Field, step_size: float = 15.0) -> float:
        """
        寻找使总路径长度最小的最优覆盖方向。
        通过测试多个方向，选择总行程最短的一个。

        参数:
            field: 待优化的农田对象
            step_size: 测试方向的角度步长（单位：度，默认 15.0°）

        返回:
            最优方向（角度，范围 0°~179°）
        """
        # 生成测试方向列表（0° 到 180°-step_size）
        max_angle = 180
        test_directions = []
        angle = 0
        while angle < max_angle:
            test_directions.append(angle)
            angle += step_size

        print(f"  正在测试 {len(test_directions)} 个方向（步长: {step_size}°）...")

        # 第一轮：计算各方向的路径长度
        results = []
        best_direction = 0
        min_path_length = float('inf')

        for direction in test_directions:
            try:
                waypoints = self._generate_test_coverage(field, direction)
                path_length = waypoints.total_distance()
                results.append((direction, path_length))

                if path_length < min_path_length:
                    min_path_length = path_length
                    best_direction = direction
            except Exception as e:
                print(f"    方向 {direction:6.1f}°: 出错 - {e}")
                results.append((direction, None))
                continue

        # 第二轮：打印结果，并标记最优方向
        for direction, path_length in results:
            if path_length is not None:
                marker = " ⭐" if direction == best_direction else ""
                print(f"    方向 {direction:6.1f}°: {path_length:8.1f}m 总路径{marker}")

        print(f"  最优方向: {best_direction}°，总路径长度: {min_path_length:.1f}m")
        return best_direction

    def _generate_test_coverage(self, field: Field, direction: float) -> WaypointSequence:
        """
        为方向优化生成测试用的覆盖路径（简化版，提高速度）。
        """
        # 获取农田多边形和坐标转换器
        field_polygon = field.get_working_area_polygon(use_utm=True)
        transformer = field.coordinate_transformer
        effective_swath_width = self.swath_width * (1 - self.overlap)

        waypoints = []
        waypoint_id = 1

        # 转换方向为弧度
        direction_rad = math.radians(direction)
        scan_direction_rad = direction_rad + math.pi / 2

        # 获取农田边界
        minx, miny, maxx, maxy = field_polygon.bounds
        field_center_x = (minx + maxx) / 2
        field_center_y = (miny + maxy) / 2
        diagonal = math.sqrt((maxx - minx)**2 + (maxy - miny)**2)

        # 计算扫描线数量
        field_width_in_scan_dir = diagonal
        num_lines = int(field_width_in_scan_dir / effective_swath_width) + 2

        # 生成扫描线
        for i in range(-num_lines//2, num_lines//2 + 1):
            offset = i * effective_swath_width
            scan_center_x = field_center_x + offset * math.cos(scan_direction_rad)
            scan_center_y = field_center_y + offset * math.sin(scan_direction_rad)

            line_half_length = diagonal
            start_x = scan_center_x - line_half_length * math.cos(direction_rad)
            start_y = scan_center_y - line_half_length * math.sin(direction_rad)
            end_x = scan_center_x + line_half_length * math.cos(direction_rad)
            end_y = scan_center_y + line_half_length * math.sin(direction_rad)

            coverage_line = LineString([(start_x, start_y), (end_x, end_y)])
            intersection = field_polygon.intersection(coverage_line)

            if intersection.is_empty:
                continue

            if hasattr(intersection, 'geoms'):
                line_segments = list(intersection.geoms)
            else:
                line_segments = [intersection] if hasattr(intersection, 'coords') else []

            for segment in line_segments:
                if not hasattr(segment, 'coords'):
                    continue

                coords = list(segment.coords)
                if len(coords) < 2:
                    continue

                line_index = i + num_lines//2
                if line_index % 2 == 0:
                    heading = direction
                    start_coord = coords[0]
                    end_coord = coords[-1]
                else:
                    heading = (direction + 180) % 360
                    start_coord = coords[-1]
                    end_coord = coords[0]

                # 使用更大的采样距离以加快测试速度
                line_waypoints = self._create_line_waypoints(
                    start_coord[0], start_coord[1],
                    end_coord[0], end_coord[1],
                    transformer, waypoint_id, heading, 20.0
                )
                waypoints.extend(line_waypoints)
                waypoint_id += len(line_waypoints)

        return WaypointSequence(waypoints)

    def calculate_coverage_area(self, waypoints: WaypointSequence) -> float:
        """
        估算航点序列所覆盖的总面积。

        参数:
            waypoints: 覆盖航点序列

        返回:
            估算的覆盖面积（单位：平方米）
        """
        coverage_waypoints = waypoints.get_coverage_waypoints()

        if len(coverage_waypoints) < 2:
            return 0.0

        total_distance = 0.0
        for i in range(len(coverage_waypoints) - 1):
            total_distance += coverage_waypoints[i].distance_to(coverage_waypoints[i + 1])

        return total_distance * self.swath_width