"""
用于农田覆盖路径规划的验证工具函数。
"""

import math
from typing import List, Tuple, Optional, Dict, Any
from ..core.coordinates import GPSCoordinate
from ..core.field import Field
from ..core.waypoint import WaypointSequence


def validate_gps_coordinates(coordinates: List[GPSCoordinate]) -> List[str]:
    """
    验证一组 GPS 坐标。

    参数:
        coordinates: 需要验证的 GPS 坐标列表

    返回:
        验证错误信息列表（如果有效则为空列表）
    """
    errors = []

    if not coordinates:
        errors.append("坐标列表为空")
        return errors

    if len(coordinates) < 3:
        errors.append("至少需要 3 个坐标点才能形成多边形")

    for i, coord in enumerate(coordinates):
        try:
            # 如果坐标无效，将抛出 ValueError
            _ = GPSCoordinate(coord.latitude, coord.longitude)
        except ValueError as e:
            errors.append(f"索引 {i} 处的坐标无效: {str(e)}")

    # 检查是否存在连续重复点
    for i in range(len(coordinates) - 1):
        if (abs(coordinates[i].latitude - coordinates[i + 1].latitude) < 1e-8 and
            abs(coordinates[i].longitude - coordinates[i + 1].longitude) < 1e-8):
            errors.append(f"索引 {i} 和 {i + 1} 处存在连续重复坐标")

    return errors


def validate_polygon_closure(coordinates: List[GPSCoordinate],
                             tolerance: float = 1e-6) -> bool:
    """
    检查多边形是否正确闭合。

    参数:
        coordinates: GPS 坐标列表
        tolerance: 坐标比较容差

    返回:
        如果多边形闭合则返回 True，否则返回 False
    """
    if len(coordinates) < 4:  # 至少 3 个唯一点 + 1 个闭合点
        return False

    first = coordinates[0]
    last = coordinates[-1]

    return (abs(first.latitude - last.latitude) < tolerance and
            abs(first.longitude - last.longitude) < tolerance)


def validate_polygon_self_intersection(coordinates: List[Tuple[float, float]]) -> List[str]:
    """
    检查多边形是否存在自相交。

    参数:
        coordinates: (x, y) 坐标列表

    返回:
        验证错误信息列表
    """
    errors = []
    n = len(coordinates)

    if n < 4:
        return errors

    # 检查每条边与所有非相邻边是否相交
    for i in range(n - 1):
        line1_start = coordinates[i]
        line1_end = coordinates[i + 1]

        for j in range(i + 2, n - 1):
            if j == n - 2 and i == 0:
                # 跳过最后一条边与第一条边的比较（它们共享顶点）
                continue

            line2_start = coordinates[j]
            line2_end = coordinates[j + 1]

            if _lines_intersect(line1_start, line1_end, line2_start, line2_end):
                errors.append(f"边 {i}-{i+1} 与 边 {j}-{j+1} 发生自相交")

    return errors


def _lines_intersect(p1: Tuple[float, float], p2: Tuple[float, float],
                     p3: Tuple[float, float], p4: Tuple[float, float]) -> bool:
    """
    检查两条线段是否相交。

    参数:
        p1, p2: 第一条线段端点
        p3, p4: 第二条线段端点

    返回:
        若相交则返回 True，否则返回 False
    """
    def _ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    return _ccw(p1, p3, p4) != _ccw(p2, p3, p4) and _ccw(p1, p2, p3) != _ccw(p1, p2, p4)


def validate_field_parameters(field: Field,
                              swath_width: float,
                              overlap: float = 0.0,
                              min_turn_radius: float = 1.0) -> List[str]:
    """
    验证农田与覆盖参数。

    参数:
        field: 需要验证的农田对象
        swath_width: 作业幅宽（米）
        overlap: 重叠率（0.0 到 1.0）
        min_turn_radius: 最小转弯半径（米）

    返回:
        验证错误信息列表
    """
    errors = []

    if swath_width <= 0:
        errors.append(f"作业幅宽无效: {swath_width}，必须为正数")

    if not (0.0 <= overlap < 1.0):
        errors.append(f"重叠率无效: {overlap}，必须在 0.0 到 1.0 之间")

    if min_turn_radius <= 0:
        errors.append(f"最小转弯半径无效: {min_turn_radius}，必须为正数")

    try:
        field_area = field.calculate_area()
        min_bbox, max_bbox = field.get_bounding_box()
        field_width = max_bbox.easting - min_bbox.easting
        field_height = max_bbox.northing - min_bbox.northing
        min_dimension = min(field_width, field_height)

        if swath_width > min_dimension:
            errors.append(
                f"作业幅宽 ({swath_width}m) 大于田块最小边长 ({min_dimension:.1f}m)"
            )

        if swath_width > min_dimension / 2:
            errors.append(
                f"警告: 作业幅宽 ({swath_width}m) 超过田块最小边长的一半"
            )

    except Exception as e:
        errors.append(f"计算田块尺寸时发生错误: {str(e)}")

    return errors


def validate_waypoint_sequence(waypoints: WaypointSequence,
                               max_speed: float = 10.0,
                               max_heading_change: float = 45.0) -> List[str]:
    """
    验证航点序列的可行性。

    参数:
        waypoints: 航点序列
        max_speed: 最大允许速度（m/s）
        max_heading_change: 相邻航点最大允许航向变化（度）

    返回:
        验证错误信息列表
    """
    errors = []

    if len(waypoints) == 0:
        errors.append("航点序列为空")
        return errors

    for i, waypoint in enumerate(waypoints):
        if waypoint.speed > max_speed:
            errors.append(f"航点 {i+1}: 速度 {waypoint.speed} 超过最大允许值 {max_speed}")

        if waypoint.speed <= 0:
            errors.append(f"航点 {i+1}: 速度无效 {waypoint.speed}")

        if not (0 <= waypoint.heading < 360):
            errors.append(f"航点 {i+1}: 航向无效 {waypoint.heading}")

        if i > 0:
            prev_heading = waypoints[i-1].heading
            heading_change = abs(waypoint.heading - prev_heading)

            if heading_change > 180:
                heading_change = 360 - heading_change

            if heading_change > max_heading_change and not (160 <= heading_change <= 180):
                errors.append(
                    f"航点 {i+1}: 与前一航点航向变化异常 {heading_change:.1f}°"
                )

    return errors


def validate_coverage_completeness(field: Field,
                                   waypoints: WaypointSequence,
                                   swath_width: float,
                                   min_coverage: float = 0.95) -> Dict[str, Any]:
    """
    验证航点是否实现充分覆盖。

    参数:
        field: 目标田块
        waypoints: 航点序列
        swath_width: 作业幅宽
        min_coverage: 最小覆盖率要求（0.0 到 1.0）

    返回:
        覆盖分析结果字典
    """
    try:
        field_area = field.calculate_area()

        total_distance = waypoints.total_distance()
        estimated_coverage_area = total_distance * swath_width

        coverage_percentage = min(1.0, estimated_coverage_area / field_area)

        result = {
            'coverage_percentage': coverage_percentage,
            'field_area_m2': field_area,
            'estimated_covered_area_m2': estimated_coverage_area,
            'meets_minimum': coverage_percentage >= min_coverage,
            'total_path_distance_m': total_distance,
            'efficiency': coverage_percentage * field_area / total_distance if total_distance > 0 else 0
        }

        return result

    except Exception as e:
        return {
            'error': str(e),
            'coverage_percentage': 0.0,
            'meets_minimum': False
        }


def validate_input_csv_format(file_path: str) -> List[str]:
    """
    验证用于田块边界输入的 CSV 文件格式。

    参数:
        file_path: CSV 文件路径

    返回:
        验证错误信息列表
    """
    errors = []

    try:
        import pandas as pd

        df = pd.read_csv(file_path)

        required_columns = ['latitude', 'longitude']
        missing_columns = [col for col in required_columns if col not in df.columns]

        if missing_columns:
            errors.append(f"缺少必要列: {missing_columns}")
            return errors

        if len(df) == 0:
            errors.append("CSV 文件为空")
            return errors

        if len(df) < 3:
            errors.append("CSV 文件必须至少包含 3 对坐标")

        for i, row in df.iterrows():
            try:
                lat = float(row['latitude'])
                lon = float(row['longitude'])

                if not (-90 <= lat <= 90):
                    errors.append(f"第 {i+1} 行: 纬度无效 {lat}")

                if not (-180 <= lon <= 180):
                    errors.append(f"第 {i+1} 行: 经度无效 {lon}")

            except (ValueError, TypeError):
                errors.append(f"第 {i+1} 行: 纬度或经度为非数值类型")

    except FileNotFoundError:
        errors.append(f"文件未找到: {file_path}")
    except Exception as e:
        errors.append(f"读取 CSV 文件时发生错误: {str(e)}")

    return errors