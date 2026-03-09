"""
用于田块覆盖路径规划的几何工具函数。
"""

import math
from typing import List, Tuple, Optional
from shapely.geometry import Polygon, Point, LineString, MultiLineString
from shapely.ops import linemerge, unary_union
import numpy as np


def calculate_polygon_area(coordinates: List[Tuple[float, float]]) -> float:
    """
    使用鞋带公式计算多边形面积。

    参数:
        coordinates: (x, y) 坐标元组列表

    返回:
        多边形面积
    """
    if len(coordinates) < 3:
        return 0.0

    area = 0.0
    n = len(coordinates)

    for i in range(n):
        j = (i + 1) % n
        area += coordinates[i][0] * coordinates[j][1]
        area -= coordinates[j][0] * coordinates[i][1]

    return abs(area) / 2.0


def calculate_polygon_centroid(coordinates: List[Tuple[float, float]]) -> Tuple[float, float]:
    """
    计算多边形的几何中心（质心）。

    参数:
        coordinates: (x, y) 坐标元组列表

    返回:
        质心的 (x, y) 坐标
    """
    if len(coordinates) < 3:
        raise ValueError("多边形至少需要 3 个坐标点")

    area = calculate_polygon_area(coordinates)
    if area == 0:
        raise ValueError("多边形面积为 0")

    cx = 0.0
    cy = 0.0
    n = len(coordinates)

    for i in range(n):
        j = (i + 1) % n
        factor = coordinates[i][0] * coordinates[j][1] - coordinates[j][0] * coordinates[i][1]
        cx += (coordinates[i][0] + coordinates[j][0]) * factor
        cy += (coordinates[i][1] + coordinates[j][1]) * factor

    factor = 1.0 / (6.0 * area)
    return (cx * factor, cy * factor)


def point_to_line_distance(point: Tuple[float, float],
                          line_start: Tuple[float, float],
                          line_end: Tuple[float, float]) -> float:
    """
    计算点到线段的垂直距离。

    参数:
        point: 点的 (x, y) 坐标
        line_start: 线段起点 (x, y)
        line_end: 线段终点 (x, y)

    返回:
        点到线段的距离
    """
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end

    # 计算线段长度的平方
    line_length_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2

    if line_length_sq == 0:
        # 如果线段退化为一个点
        return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

    # 参数 t 表示在线段上的最近点位置
    t = max(0, min(1, ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / line_length_sq))

    # 线段上的最近点
    closest_x = x1 + t * (x2 - x1)
    closest_y = y1 + t * (y2 - y1)

    # 点到最近点的距离
    return math.sqrt((x0 - closest_x) ** 2 + (y0 - closest_y) ** 2)


def line_polygon_intersection(line_start: Tuple[float, float],
                            line_end: Tuple[float, float],
                            polygon_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    计算直线与多边形的交点。

    参数:
        line_start: 直线起点
        line_end: 直线终点
        polygon_coords: 多边形坐标列表

    返回:
        交点坐标列表
    """
    line = LineString([line_start, line_end])
    polygon = Polygon(polygon_coords)

    intersection = polygon.boundary.intersection(line)

    points = []
    if hasattr(intersection, 'geoms'):
        # 存在多个交点
        for geom in intersection.geoms:
            if geom.geom_type == 'Point':
                points.append((geom.x, geom.y))
    elif intersection.geom_type == 'Point':
        points.append((intersection.x, intersection.y))

    return points


def create_parallel_lines(base_line: LineString,
                         distance: float,
                         num_lines: int,
                         side: str = 'both') -> List[LineString]:
    """
    生成与基准线平行的多条平行线。

    参数:
        base_line: 基准线
        distance: 相邻平行线之间的距离
        num_lines: 每侧生成的平行线数量
        side: 'left'（左侧）、'right'（右侧）或 'both'（两侧）

    返回:
        平行线 LineString 对象列表
    """
    parallel_lines = []

    if side in ['left', 'both']:
        for i in range(1, num_lines + 1):
            offset_line = base_line.parallel_offset(i * distance, 'left')
            if isinstance(offset_line, LineString):
                parallel_lines.append(offset_line)
            elif isinstance(offset_line, MultiLineString):
                parallel_lines.extend(list(offset_line.geoms))

    if side in ['right', 'both']:
        for i in range(1, num_lines + 1):
            offset_line = base_line.parallel_offset(i * distance, 'right')
            if isinstance(offset_line, LineString):
                parallel_lines.append(offset_line)
            elif isinstance(offset_line, MultiLineString):
                parallel_lines.extend(list(offset_line.geoms))

    return parallel_lines


def clip_line_to_polygon(line: LineString, polygon: Polygon) -> List[LineString]:
    """
    将线段裁剪到多边形内部。

    参数:
        line: 需要裁剪的 LineString
        polygon: 裁剪目标多边形

    返回:
        裁剪后的线段列表
    """
    intersection = polygon.intersection(line)

    lines = []
    if hasattr(intersection, 'geoms'):
        for geom in intersection.geoms:
            if geom.geom_type == 'LineString':
                lines.append(geom)
    elif intersection.geom_type == 'LineString':
        lines.append(intersection)

    return lines


def calculate_line_angle(start_point: Tuple[float, float],
                        end_point: Tuple[float, float]) -> float:
    """
    计算线段的角度（单位：度）。

    参数:
        start_point: 起点 (x, y)
        end_point: 终点 (x, y)

    返回:
        角度（0 = 北，90 = 东）
    """
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]

    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)

    # 转换为导航角度（0 = 北，90 = 东）
    nav_angle = (90 - angle_deg) % 360

    return nav_angle


def rotate_point(point: Tuple[float, float],
                center: Tuple[float, float],
                angle_deg: float) -> Tuple[float, float]:
    """
    围绕指定中心点旋转一个点。

    参数:
        point: 需要旋转的点
        center: 旋转中心
        angle_deg: 旋转角度（度）

    返回:
        旋转后的点坐标
    """
    angle_rad = math.radians(angle_deg)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)

    # 平移到原点
    x = point[0] - center[0]
    y = point[1] - center[1]

    # 旋转
    rotated_x = x * cos_angle - y * sin_angle
    rotated_y = x * sin_angle + y * cos_angle

    # 平移回原位置
    return (rotated_x + center[0], rotated_y + center[1])


def simplify_polygon(coordinates: List[Tuple[float, float]],
                    tolerance: float = 1.0) -> List[Tuple[float, float]]:
    """
    简化多边形，去除冗余点。

    参数:
        coordinates: 多边形坐标列表
        tolerance: 简化容差（单位：米）

    返回:
        简化后的多边形坐标列表
    """
    if len(coordinates) <= 3:
        return coordinates

    polygon = Polygon(coordinates)
    simplified = polygon.simplify(tolerance, preserve_topology=True)

    return list(simplified.exterior.coords[:-1])  # 去除重复的最后一个点


def calculate_turning_radius(speed: float, max_lateral_acceleration: float = 2.0) -> float:
    """
    根据速度和横向加速度限制计算最小转弯半径。

    参数:
        speed: 速度（m/s）
        max_lateral_acceleration: 最大横向加速度（m/s²）

    返回:
        最小转弯半径（米）
    """
    if max_lateral_acceleration <= 0:
        raise ValueError("最大横向加速度必须为正数")

    return speed ** 2 / max_lateral_acceleration


def generate_turn_waypoints(start_point: Tuple[float, float],
                          end_point: Tuple[float, float],
                          turn_radius: float,
                          num_points: int = 10) -> List[Tuple[float, float]]:
    """
    在两个点之间生成平滑转弯航点。

    参数:
        start_point: 转弯起点
        end_point: 转弯终点
        turn_radius: 转弯半径
        num_points: 生成的航点数量

    返回:
        转弯航点列表
    """
    if num_points < 2:
        return [start_point, end_point]

    # 为简化实现，这里生成直线插值点
    # 在真实实现中应生成圆弧航点
    waypoints = []

    for i in range(num_points):
        t = i / (num_points - 1)
        x = start_point[0] + t * (end_point[0] - start_point[0])
        y = start_point[1] + t * (end_point[1] - start_point[1])
        waypoints.append((x, y))

    return waypoints