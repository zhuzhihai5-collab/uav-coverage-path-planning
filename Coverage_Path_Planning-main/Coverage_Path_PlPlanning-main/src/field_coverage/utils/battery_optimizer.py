"""
根据无人机最大航程，对完整覆盖路径进行续航约束拆分。

设计目标：
- 不修改原有 Boustrophedon 算法结构，只对生成的完整路径做后处理。
- 支持 WaypointSequence 或 Waypoint 列表输入。
- 引入返航约束：确保从当前任务最后一个点可以在剩余电量内返回起飞点。
"""

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

    参数:
        waypoints: 完整覆盖路径，可以是 WaypointSequence 或 Waypoint 对象列表
        max_distance: 无人机最大可飞行距离（米），包含作业 + 返航

    返回:
        missions: List[WaypointSequence]，每个为一个子任务路径

    约束:
        对每个 mission 内，沿路径累积距离 current_path_distance，
        对任意将要加入的下一个航点 next_wp，必须满足：
            current_path_distance + next_distance + return_distance <= max_distance

        其中：
            next_distance   = 当前 mission 末尾航点 到 next_wp 的距离
            return_distance = next_wp 到起飞点（整条路径的第一个航点）的距离
    """
    if max_distance <= 0:
        raise ValueError("max_distance 必须为正数（米）。")

    # 统一为 list，不直接操作原对象
    if isinstance(waypoints, WaypointSequence):
        original_wps: List[Waypoint] = list(waypoints.waypoints)
    else:
        original_wps = list(waypoints)

    if not original_wps:
        return []

    if len(original_wps) == 1:
        return [WaypointSequence([_clone_waypoint(original_wps[0])])]

    # 起飞点（全局固定），用于计算返航距离
    home_wp = original_wps[0]

    missions: List[WaypointSequence] = []

    # 当前 mission 中的拷贝航点及累积距离
    current_mission_wps: List[Waypoint] = [_clone_waypoint(original_wps[0])]
    current_distance: float = 0.0
    last_wp_original: Waypoint = original_wps[0]

    for idx in range(1, len(original_wps)):
        next_wp_original = original_wps[idx]

        # 相邻段距离
        next_segment_dist = last_wp_original.distance_to(next_wp_original)
        # 从下一个点返回起飞点的距离
        return_dist = next_wp_original.distance_to(home_wp)

        # 如果当前 mission 为空（只包含起点），且单段+返航都超续航，说明配置不合理
        if next_segment_dist + return_dist > max_distance and current_distance == 0.0:
            raise ValueError(
                f"max_distance={max_distance} 太小，"
                f"单段距离 {next_segment_dist:.2f} m + 返航 {return_dist:.2f} m 已超出。"
            )

        # 判断是否还能把 next_wp 放进当前 mission
        if current_distance + next_segment_dist + return_dist <= max_distance:
            current_mission_wps.append(_clone_waypoint(next_wp_original))
            current_distance += next_segment_dist
            last_wp_original = next_wp_original
        else:
            # 结束当前 mission，开启新 mission
            missions.append(WaypointSequence(current_mission_wps))

            # 新 mission 从上一个路径点开始，保持路径连贯
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

