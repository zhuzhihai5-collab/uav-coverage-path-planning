from typing import List, Sequence, Union

from ..core.waypoint import Waypoint, WaypointSequence


def _clone_waypoint(wp: Waypoint) -> Waypoint:
    """创建一个 Waypoint 的浅拷贝，避免修改原对象。"""
    return Waypoint(
        gps_coordinate=wp.gps_coordinate,
        utm_coordinate=wp.utm_coordinate,
        heading=wp.heading,
        speed=wp.speed,
        waypoint_type=wp.waypoint_type,
        waypoint_id=None,
        metadata=dict(wp.metadata) if wp.metadata else {},
    )


def split_path_by_battery(
    waypoints: Union[WaypointSequence, Sequence[Waypoint]],
    max_distance: float,
) -> List[WaypointSequence]:
    """
    按无人机最大航程，对覆盖路径进行续航约束拆分（贪心）。

    约束：
        current_path_distance + next_distance + return_distance <= max_distance
    其中：
        next_distance   = 当前 mission 末尾点 -> 下一个点
        return_distance = 下一个点 -> 起飞点（全局第一个点）
    """
    if max_distance <= 0:
        raise ValueError("max_distance 必须为正数（米）。")

    if isinstance(waypoints, WaypointSequence):
        original_wps: List[Waypoint] = list(waypoints.waypoints)
    else:
        original_wps = list(waypoints)

    if not original_wps:
        return []

    if len(original_wps) == 1:
        return [WaypointSequence([_clone_waypoint(original_wps[0])])]

    home_wp = original_wps[0]
    missions: List[WaypointSequence] = []

    current_mission_wps: List[Waypoint] = [_clone_waypoint(original_wps[0])]
    current_distance: float = 0.0
    last_wp_original: Waypoint = original_wps[0]

    for idx in range(1, len(original_wps)):
        next_wp_original = original_wps[idx]

        next_segment_dist = last_wp_original.distance_to(next_wp_original)
        return_dist = next_wp_original.distance_to(home_wp)

        if next_segment_dist + return_dist > max_distance and current_distance == 0.0:
            raise ValueError(
                f"max_distance={max_distance} 太小，"
                f"单段距离 {next_segment_dist:.2f} m + 返航 {return_dist:.2f} m 已超出。"
            )

        if current_distance + next_segment_dist + return_dist <= max_distance:
            current_mission_wps.append(_clone_waypoint(next_wp_original))
            current_distance += next_segment_dist
            last_wp_original = next_wp_original
        else:
            missions.append(WaypointSequence(current_mission_wps))

            current_mission_wps = [_clone_waypoint(last_wp_original)]
            current_distance = 0.0

            next_segment_dist = last_wp_original.distance_to(next_wp_original)
            return_dist = next_wp_original.distance_to(home_wp)

            if next_segment_dist + return_dist > max_distance:
                raise ValueError(
                    f"max_distance={max_distance} 太小，"
                    f"分段后仍无法容纳从某段起点到下一个点再返航的距离。"
                )

            current_mission_wps.append(_clone_waypoint(next_wp_original))
            current_distance += next_segment_dist
            last_wp_original = next_wp_original

    if current_mission_wps:
        missions.append(WaypointSequence(current_mission_wps))

    return missions