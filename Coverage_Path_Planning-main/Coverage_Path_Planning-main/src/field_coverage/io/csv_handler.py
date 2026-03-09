"""
CSV 文件处理模块，用于输入与输出操作。
"""

import csv
from pathlib import Path
from typing import List, Optional, Dict, Any
import pandas as pd

from ..core.coordinates import GPSCoordinate
from ..core.field import Field, FieldBoundary
from ..core.waypoint import WaypointSequence, Waypoint, WaypointType
from ..utils.validation import validate_input_csv_format


class CSVHandler:
    """用于处理 CSV 文件的读写操作。"""

    @staticmethod
    def read_field_boundary_csv(file_path: str,
                               validate: bool = True) -> List[GPSCoordinate]:
        """
        从 CSV 文件中读取田块边界的 GPS 坐标。

        参数:
            file_path: 包含 GPS 坐标的 CSV 文件路径
            validate: 是否对 CSV 文件格式进行验证

        返回:
            表示田块边界的 GPS 坐标列表

        异常:
            ValueError: 当 CSV 格式不合法时抛出
            FileNotFoundError: 当文件不存在时抛出
        """
        # 如果需要，先验证文件格式
        if validate:
            errors = validate_input_csv_format(file_path)
            if errors:
                raise ValueError(f"CSV 格式错误: {'; '.join(errors)}")

        coordinates = []

        try:
            df = pd.read_csv(file_path)

            for _, row in df.iterrows():
                coordinate = GPSCoordinate(
                    latitude=float(row['latitude']),
                    longitude=float(row['longitude'])
                )
                coordinates.append(coordinate)

            return coordinates

        except Exception as e:
            raise ValueError(f"读取 CSV 文件时发生错误: {str(e)}")

    @staticmethod
    def read_field_with_holes_csv(main_boundary_file: str,
                                 hole_files: Optional[List[str]] = None,
                                 field_id: Optional[str] = None) -> Field:
        """
        从 CSV 文件读取完整田块定义（包括主边界和内部障碍）。

        参数:
            main_boundary_file: 主边界 CSV 文件路径
            hole_files: 内部障碍（洞）CSV 文件路径列表
            field_id: 可选的田块编号

        返回:
            包含主边界和洞信息的 Field 对象
        """
        # 读取主边界
        main_coords = CSVHandler.read_field_boundary_csv(main_boundary_file)
        main_boundary = FieldBoundary(gps_coordinates=main_coords)

        # 读取洞（如果提供）
        holes = []
        if hole_files:
            for hole_file in hole_files:
                hole_coords = CSVHandler.read_field_boundary_csv(hole_file)
                hole_boundary = FieldBoundary(
                    gps_coordinates=hole_coords,
                    is_hole=True
                )
                holes.append(hole_boundary)

        return Field(
            main_boundary=main_boundary,
            holes=holes,
            field_id=field_id or Path(main_boundary_file).stem
        )

    @staticmethod
    def write_waypoints_csv(waypoints: WaypointSequence,
                           file_path: str,
                           include_metadata: bool = False) -> None:
        """
        将航点序列写入 CSV 文件。

        参数:
            waypoints: 需要导出的航点序列
            file_path: 输出 CSV 文件路径
            include_metadata: 是否包含航点的附加元数据
        """
        headers = [
            'waypoint_id',
            'latitude',
            'longitude',
            'heading',
            'speed',
            'waypoint_type'
        ]

        if include_metadata:
            all_metadata_keys = set()
            for waypoint in waypoints:
                all_metadata_keys.update(waypoint.metadata.keys())
            headers.extend(sorted(all_metadata_keys))

        with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(headers)

            for waypoint in waypoints:
                row = [
                    waypoint.waypoint_id,
                    waypoint.gps_coordinate.latitude,
                    waypoint.gps_coordinate.longitude,
                    waypoint.heading,
                    waypoint.speed,
                    waypoint.waypoint_type.value
                ]

                if include_metadata:
                    for key in sorted(all_metadata_keys):
                        row.append(waypoint.metadata.get(key, ''))

                writer.writerow(row)

    @staticmethod
    def read_waypoints_csv(file_path: str) -> WaypointSequence:
        """
        从 CSV 文件读取航点数据。

        参数:
            file_path: 航点 CSV 文件路径

        返回:
            从文件中加载的 WaypointSequence 对象
        """
        waypoints = []

        try:
            df = pd.read_csv(file_path)

            for _, row in df.iterrows():
                gps_coord = GPSCoordinate(
                    latitude=float(row['latitude']),
                    longitude=float(row['longitude'])
                )

                waypoint_type = WaypointType.COVERAGE
                if 'waypoint_type' in row:
                    try:
                        waypoint_type = WaypointType(row['waypoint_type'])
                    except ValueError:
                        waypoint_type = WaypointType.COVERAGE

                metadata = {}
                excluded_cols = {
                    'waypoint_id', 'latitude', 'longitude',
                    'heading', 'speed', 'waypoint_type'
                }
                for col in df.columns:
                    if col not in excluded_cols and pd.notna(row[col]):
                        metadata[col] = row[col]

                waypoint = Waypoint(
                    gps_coordinate=gps_coord,
                    heading=float(row.get('heading', 0.0)),
                    speed=float(row.get('speed', 2.0)),
                    waypoint_type=waypoint_type,
                    waypoint_id=int(row.get('waypoint_id', 0)),
                    metadata=metadata
                )

                waypoints.append(waypoint)

            return WaypointSequence(waypoints)

        except Exception as e:
            raise ValueError(f"读取航点 CSV 文件时发生错误: {str(e)}")

    @staticmethod
    def write_field_summary_csv(field: Field, file_path: str) -> None:
        """
        将田块统计信息写入 CSV 文件。

        参数:
            field: 需要统计的田块对象
            file_path: 输出 CSV 文件路径
        """
        summary_data = {
            'field_id': [field.field_id],
            'area_m2': [field.calculate_area()],
            'perimeter_m': [field.calculate_perimeter()],
            'num_holes': [len(field.holes)],
            'boundary_points': [len(field.main_boundary.gps_coordinates)],
        }

        gps_centroid, utm_centroid = field.get_centroid()
        summary_data['centroid_latitude'] = [gps_centroid.latitude]
        summary_data['centroid_longitude'] = [gps_centroid.longitude]

        min_bbox, max_bbox = field.get_bounding_box()
        summary_data['bbox_min_easting'] = [min_bbox.easting]
        summary_data['bbox_min_northing'] = [min_bbox.northing]
        summary_data['bbox_max_easting'] = [max_bbox.easting]
        summary_data['bbox_max_northing'] = [max_bbox.northing]

        summary_data['optimal_direction_deg'] = [field.calculate_optimal_direction()]

        df = pd.DataFrame(summary_data)
        df.to_csv(file_path, index=False)

    @staticmethod
    def write_coverage_report_csv(field: Field,
                                 waypoints: WaypointSequence,
                                 parameters: Dict[str, Any],
                                 file_path: str) -> None:
        """
        生成覆盖规划报告并写入 CSV 文件。

        参数:
            field: 被规划的田块
            waypoints: 生成的航点序列
            parameters: 使用的规划参数
            file_path: 输出 CSV 文件路径
        """
        report_data = []

        field_area = field.calculate_area()
        total_distance = waypoints.total_distance()
        total_time = waypoints.total_time()

        report_data.append({
            'metric': '田块面积 (m²)',
            'value': field_area,
            'unit': 'm²'
        })

        report_data.append({
            'metric': '田块周长 (m)',
            'value': field.calculate_perimeter(),
            'unit': 'm'
        })

        report_data.append({
            'metric': '路径总长度 (m)',
            'value': total_distance,
            'unit': 'm'
        })

        report_data.append({
            'metric': '预计飞行时间 (s)',
            'value': total_time,
            'unit': 's'
        })

        report_data.append({
            'metric': '航点数量',
            'value': len(waypoints),
            'unit': 'count'
        })

        df = pd.DataFrame(report_data)
        df.to_csv(file_path, index=False)