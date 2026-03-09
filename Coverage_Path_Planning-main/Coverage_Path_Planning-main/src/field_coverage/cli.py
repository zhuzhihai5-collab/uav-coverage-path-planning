"""
农田覆盖路径规划器的命令行界面（CLI）入口模块。

【核心作用说明】
本模块提供用户友好的命令行工具，将复杂的路径规划流程封装为简单指令：
1. 通过参数配置规划参数（条带宽度、重叠率等）
2. 自动加载农田边界数据（支持CSV/ROS）
3. 智能生成最优覆盖路径（牛耕式算法）
4. 一键导出航点文件 + 生成可视化报告
5. 支持配置文件管理，避免重复输入参数

【设计亮点】
✅ 智能配置优先级：硬编码默认值 < YAML配置文件 < 命令行参数（后者覆盖前者）
✅ 路径自动解析：自动定位项目根目录，解决相对路径混乱问题
✅ 用户体验优化：详细进度提示 + 可视化反馈 + 错误精准定位
✅ 生产级健壮性：全流程异常处理 + 输入验证 + 目录自动创建
"""

import argparse
import yaml
import os
from pathlib import Path
from typing import Optional, Dict, Any

try:
    from .main import FieldCoveragePlanner
    from . import VISUALIZATION_AVAILABLE
    if VISUALIZATION_AVAILABLE:
        from .visualization.field_plotter import create_coverage_visualization
except ImportError:
    # 处理直接运行脚本时的模块导入问题（开发调试场景）
    import sys
    sys.path.append(str(Path(__file__).parent))
    from main import FieldCoveragePlanner


def find_project_root() -> Path:
    """
    智能定位项目根目录（解决跨环境路径问题）。

    【工作原理】
    1. 优先从当前工作目录向上搜索标志性文件（setup.py/requirements.txt等）
    2. 若失败，尝试从脚本所在位置向上回溯
    3. 最终兜底：返回当前工作目录

    【为什么重要】
    确保配置文件、示例数据、输出目录的路径解析始终正确，
    避免因运行位置不同导致的"文件找不到"错误。

    返回:
        项目根目录的Path对象
    """
    # 从当前工作目录开始搜索
    current = Path.cwd()

    # 项目根目录的标志性文件列表
    markers = ['setup.py', 'requirements.txt', 'README.md', 'config/defaults.yaml']

    # 逐级向上检查父目录
    for path in [current] + list(current.parents):
        if any((path / marker).exists() for marker in markers):
            return path

    # 备用方案：从脚本位置回溯
    script_dir = Path(__file__).parent
    for path in [script_dir.parent.parent.parent, script_dir.parent.parent, script_dir.parent]:
        if any((path / marker).exists() for marker in markers):
            return path

    # 最终兜底方案
    return current


def load_config(config_path: str = "config/defaults.yaml") -> Dict[str, Any]:
    """
    从YAML配置文件加载规划参数（实现参数集中管理）。

    【配置优先级说明】
    命令行参数 > YAML配置文件 > 硬编码默认值
    此函数负责加载第二优先级的YAML配置

    参数:
        config_path: 相对于项目根目录的YAML配置文件路径

    返回:
        配置字典（若文件不存在或解析失败则返回空字典）

    【实用技巧】
    在项目根目录创建config/defaults.yaml，可预设常用参数：
    ```yaml
    swath_width: 2.5
    overlap: 0.15
    input_file: "data/my_field.csv"
    ```
    """
    try:
        project_root = find_project_root()
        config_file = project_root / config_path

        if not config_file.exists():
            return {}  # 配置文件非必需，静默处理

        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            return config if config else {}

    except Exception as e:
        # 配置加载失败不应阻断主流程，仅记录空配置
        return {}


def create_parser() -> argparse.ArgumentParser:
    """
    创建智能参数解析器（融合三重配置源）。

    【配置融合逻辑】
    1. 加载项目根目录的config/defaults.yaml（用户自定义默认值）
    2. 与硬编码默认值合并（config优先级更高）
    3. 将相对路径转换为绝对路径（避免路径解析错误）
    4. 生成带友好提示的命令行参数解析器

    【用户价值】
    - 首次使用：直接运行`field-coverage`，自动使用示例数据
    - 日常使用：修改YAML配置，避免重复输入参数
    - 精细调整：通过命令行参数临时覆盖配置
    """

    # 定位项目根目录（用于路径解析）
    project_root = find_project_root()

    # 加载YAML配置（用户自定义默认值）
    config = load_config()

    # 硬编码兜底默认值（相对路径，后续会转绝对路径）
    hardcoded_defaults = {
        'input_file': str(project_root / "data/example_field.csv"),
        'output_file': str(project_root / "output/coverage_result.csv"),
        'swath_width': 3.0,
        'overlap': 0.1,
        'turn_radius': 2.0,
        'speed': 2.0,
        'direction': None,
        'optimization_step': 15.0,
        'field_id': None,
        'algorithm': 'boustrophedon',
        'report': False,
        'plot': True,
        'plot_output': str(project_root / "output/field_coverage_plot.png"),
        'show_plot': False,
        'validate': True,
        'verbose': True,
        # 新增：无人机最大航程（米），为 None 时不启用续航拆分
        'max_distance': None,
    }

    # 将YAML中的相对路径转为绝对路径（关键！避免路径错误）
    if config:
        if 'input_file' in config and not os.path.isabs(config['input_file']):
            config['input_file'] = str(project_root / config['input_file'])
        if 'output_file' in config and not os.path.isabs(config['output_file']):
            config['output_file'] = str(project_root / config['output_file'])
        if 'plot_output' in config and not os.path.isabs(config['plot_output']):
            config['plot_output'] = str(project_root / config['plot_output'])

    # 合并配置：YAML配置覆盖硬编码默认值
    defaults = {**hardcoded_defaults, **config}

    # 创建带丰富帮助信息的解析器
    parser = argparse.ArgumentParser(
        description="🌾 农田覆盖路径规划器 - 为农业机械生成高效、无遗漏的作业路径",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例：
  # 快速开始（使用示例农田数据，自动生成路径图）
  field-coverage
  
  # 基础用法：指定输入输出文件
  field-coverage my_field.csv path_output.csv
  
  # 自定义作业参数（条带宽度3米，重叠率15%）
  field-coverage field.csv output.csv --swath-width 3.0 --overlap 0.15
  
  # 精细优化方向（5度步长搜索最优方向，更精准但稍慢）
  field-coverage --optimization-step 5
  
  # 生成带可视化图表的报告
  field-coverage --report --plot --plot-output report.png
        """
    )

    # 位置参数：输入/输出文件（带智能默认值提示）
    parser.add_argument(
        "input_file",
        type=Path,
        nargs='?',
        default=defaults['input_file'],
        help=f"包含农田边界的CSV文件路径（GPS坐标，经度,纬度格式）。默认: {defaults['input_file']}"
    )

    parser.add_argument(
        "output_file",
        type=Path,
        nargs='?',
        default=defaults['output_file'],
        help=f"生成的航点CSV输出路径。默认: {defaults['output_file']}"
    )

    # 核心规划参数
    parser.add_argument(
        "--swath-width", "-w",
        type=float,
        default=defaults['swath_width'],
        help=f"作业条带宽度（米），决定机械单次作业覆盖宽度。默认: {defaults['swath_width']}"
    )

    parser.add_argument(
        "--overlap", "-o",
        type=float,
        default=defaults['overlap'],
        help=f"条带重叠比例（0.1=10%），防止漏耕。默认: {defaults['overlap']}"
    )

    parser.add_argument(
        "--turn-radius", "-r",
        type=float,
        default=defaults['turn_radius'],
        help=f"机械最小转弯半径（米），影响路径平滑度。默认: {defaults['turn_radius']}"
    )

    parser.add_argument(
        "--speed", "-s",
        type=float,
        default=defaults['speed'],
        help=f"航点默认作业速度（米/秒）。默认: {defaults['speed']}"
    )

    parser.add_argument(
        "--max-distance", "-m",
        type=float,
        default=defaults['max_distance'],
        help="无人机最大航程（米，含返航）。设置后会按续航约束自动把整条路径拆分成多个飞行任务。"
    )

    parser.add_argument(
        "--direction", "-d",
        type=float,
        default=defaults['direction'],
        help="指定覆盖方向（度）：0=正北，90=正东。不指定时自动计算最优方向（推荐）"
    )

    parser.add_argument(
        "--optimization-step",
        type=float,
        default=defaults['optimization_step'],
        help=f"方向优化步长（度）。值越小越精准但计算慢（默认{defaults['optimization_step']}°）"
    )

    parser.add_argument(
        "--field-id",
        type=str,
        default=defaults['field_id'],
        help="农田唯一标识符（默认使用输入文件名）"
    )

    parser.add_argument(
        "--algorithm", "-a",
        choices=['boustrophedon'],  # 未来可扩展螺旋式等算法
        default=defaults['algorithm'],
        help=f"覆盖路径算法（当前仅支持牛耕式）。默认: {defaults['algorithm']}"
    )

    # 高级功能开关
    parser.add_argument('--report', action='store_true', default=defaults['report'],
                       help='生成详细覆盖报告（含面积效率、作业时间等）')
    parser.add_argument('--plot', action='store_true', default=defaults['plot'],
                       help=f'生成农田覆盖可视化图（默认: {defaults["plot"]}）')
    parser.add_argument('--plot-output', type=str, default=defaults['plot_output'],
                       help=f'可视化图片保存路径。默认: {defaults["plot_output"]}')
    parser.add_argument('--show-plot', action='store_true', default=defaults['show_plot'],
                       help='在生成图片的同时弹出窗口显示（适合交互调试）')
    parser.add_argument('--no-show-plot', dest='show_plot', action='store_false',
                       help='禁止弹出窗口（适合服务器无GUI环境）')

    parser.add_argument(
        "--validate",
        action='store_true',
        default=defaults['validate'],
        help=f"生成后验证航点合理性（检查速度突变、航向突变等）。默认: {defaults['validate']}"
    )

    parser.add_argument(
        "--verbose", "-v",
        action='store_true',
        default=defaults['verbose'],
        help=f"显示详细执行过程（强烈推荐首次使用时开启）。默认: {defaults['verbose']}"
    )

    return parser


def main():
    """
    CLI主执行流程（端到端工作流）。

    【执行流程】
    1️⃣ 解析参数（融合三重配置源）
    2️⃣ 初始化规划器（带参数校验）
    3️⃣ 自动创建输出目录（避免"目录不存在"错误）
    4️⃣ 加载农田边界（含格式验证）
    5️⃣ 生成覆盖路径（智能方向优化）
    6️⃣ 验证航点质量（可选）
    7️⃣ 导出航点CSV
    8️⃣ 生成报告/可视化（按需）

    【错误处理设计】
    - FileNotFoundError：精准提示缺失文件路径
    - 全局异常捕获：打印堆栈（verbose模式）便于调试
    - 非阻塞警告：如验证问题仅提示不中断流程
    """
    parser = create_parser()
    args = parser.parse_args()

    try:
        # 显示配置来源（增强用户信任感）
        if args.verbose:
            project_root = find_project_root()
            config = load_config()
            config_file = project_root / "config/defaults.yaml"

            if config and config_file.exists():
                print(f"📋 已加载配置文件: {config_file}")
            else:
                print("📋 未找到配置文件，使用内置默认参数")
            print(f"📁 项目根目录: {project_root}")
            print()

        # 初始化规划核心引擎
        planner = FieldCoveragePlanner(  #从这里调用了主类
            swath_width=args.swath_width,
            overlap=args.overlap,
            turn_radius=args.turn_radius,
            speed=args.speed,
            algorithm=args.algorithm
        )

        # 确保输出目录存在（用户体验关键点！）
        output_dir = Path(args.output_file).parent
        output_dir.mkdir(parents=True, exist_ok=True)

        if args.verbose:
            print("⚙️  初始化农田覆盖规划器...")
            print(f"  • 作业条带宽度: {args.swath_width} 米")
            print(f"  • 条带重叠率: {args.overlap*100:.1f}%")
            print(f"  • 最小转弯半径: {args.turn_radius} 米")
            print(f"  • 作业速度: {args.speed} 米/秒")
            print(f"  • 算法: {args.algorithm}（牛耕式往返路径）")
            print()

        # 加载农田边界数据
        field_id = args.field_id or args.input_file.stem
        if args.verbose:
            print(f"📍 正在加载农田边界: {args.input_file}")

        field = planner.load_field_from_csv(
            csv_file_path=str(args.input_file),
            field_id=field_id
        )

        if args.verbose:
            print(f"  ✓ 农田ID: {field.field_id}")
            print(f"  ✓ 农田面积: {field.calculate_area():.1f} 平方米")
            print(f"  ✓ 农田周长: {field.calculate_perimeter():.1f} 米")
            print()

        # 生成覆盖路径（核心算法执行）
        if args.verbose:
            print("GenerationStrategy 生成覆盖路径...")
            if args.direction is not None:
                print(f"  • 使用指定方向: {args.direction}°")
            else:
                print(f"  • 智能优化方向中（步长: {args.optimization_step}°）...")

        waypoints = planner.generate_coverage_path(
            field,
            direction=args.direction,
            optimization_step=args.optimization_step
        )

        # 根据电池续航对路径进行拆分（可选）
        missions = None
        if args.max_distance is not None:
            if args.verbose:
                print(f"\n🔋 按最大航程 {args.max_distance} m 对路径进行续航约束拆分...")
            try:
                from .utils.battery_optimizer import split_path_by_battery
                missions = split_path_by_battery(waypoints, args.max_distance)
                if args.verbose:
                    print(f"  ✓ 已拆分为 {len(missions)} 个飞行任务 (mission)")
            except Exception as e:
                print(f"⚠️  续航拆分失败，将继续使用完整路径: {e}")
                missions = None

        # 航点质量验证（安全网）
        if args.validate:
            from .utils.validation import validate_waypoint_sequence
            validation_results = validate_waypoint_sequence(waypoints, max_speed=args.speed*2)
            if validation_results and args.verbose:
                print(f"⚠️  航点验证提示: {'; '.join(validation_results)}")

        # 计算关键指标
        total_distance = waypoints.total_distance()
        estimated_time = total_distance / args.speed / 60  # 转换为分钟

        if args.verbose:
            print(f"  ✓ 生成航点数量: {len(waypoints.waypoints)} 个")
            print(f"  ✓ 总作业路径长度: {total_distance:.1f} 米")
            print(f"  ✓ 预估作业时间: {estimated_time:.1f} 分钟")
            print()

        # 导出航点文件（交付物）
        if args.verbose:
            print(f"📤 正在导出航点至: {args.output_file}")

        # 导出完整路径
        planner.export_waypoints(waypoints, str(args.output_file))

        # 如果有续航拆分结果，则按 mission 额外导出子任务文件
        if missions is not None:
            from pathlib import Path as _Path
            base_path = _Path(args.output_file)
            for idx, mission_seq in enumerate(missions, start=1):
                mission_file = base_path.with_name(f"{base_path.stem}_mission{idx}{base_path.suffix}")
                if args.verbose:
                    print(f"📤 正在导出子任务 mission{idx} 至: {mission_file}")
                planner.export_waypoints(mission_seq, str(mission_file))

        # 生成详细报告（生产环境推荐）
        if args.report:
            report_path = str(args.output_file).replace('.csv', '.report.csv')
            if args.verbose:
                print(f"📊 生成详细覆盖报告: {report_path}")

            report_summary = planner.generate_field_report(
                field=field,
                waypoints=waypoints,
                output_file=report_path
            )

            # 打印报告摘要（关键指标一目了然）
            print("\n📌 覆盖报告摘要:")
            print("-" * 40)

            # 补充规划参数到报告
            report_summary.update({
                '条带宽度(米)': args.swath_width,
                '重叠率(%)': args.overlap * 100,
                '算法': args.algorithm
            })

            for key, value in report_summary.items():
                if isinstance(value, float):
                    # 格式化数值：面积/距离保留1位小数，百分比保留2位
                    if 'percent' in key or '效率' in key:
                        print(f"{key}: {value:.2f}%")
                    else:
                        print(f"{key}: {value:.1f}")
                else:
                    print(f"{key}: {value}")

        # 生成可视化图表（直观验证路径合理性）
        if args.plot:
            if not VISUALIZATION_AVAILABLE:
                print("⚠️  警告: 可视化功能不可用。请安装matplotlib: pip install matplotlib")
            else:
                if args.verbose:
                    print("\n🖼️  正在生成农田覆盖可视化图...")

                plot_path = args.plot_output
                if plot_path is None and args.output_file:
                    plot_path = str(args.output_file).replace('.csv', '_coverage.png')

                # 确保图表输出目录存在
                if plot_path:
                    plot_dir = Path(plot_path).parent
                    plot_dir.mkdir(parents=True, exist_ok=True)

                try:
                    # 如果有续航拆分结果，则按 mission 多颜色绘制
                    if 'missions' in locals() and missions is not None and len(missions) > 0:
                        from .visualization.field_plotter import FieldPlotter

                        plotter = FieldPlotter()
                        title = f"Coverage Plan - {field.field_id} (Missions)"
                        plotter.setup_plot(title)

                        # 画农田边界
                        plotter.plot_field_boundary(field)

                        # 为不同 mission 准备颜色
                        color_cycle = ["red", "blue", "green", "purple", "orange", "cyan"]

                        for idx, mission_seq in enumerate(missions, start=1):
                            color = color_cycle[(idx - 1) % len(color_cycle)]
                            label_prefix = f"Mission {idx}"
                            plotter.plot_coverage_path(
                                mission_seq,
                                color=color,
                                linewidth=1.5,
                                show_waypoints=False,
                                label_prefix=label_prefix,
                            )

                        # 统计信息仍基于完整路径
                        plotter.add_statistics(field, waypoints, total_distance, estimated_time)
                        plotter.add_legend()

                        if plot_path:
                            plotter.save_plot(plot_path)

                        if args.show_plot:
                            plotter.show()
                        else:
                            plotter.close()
                    else:
                        # 没有 mission 拆分时，使用原来的单路径可视化
                        create_coverage_visualization(
                            field=field,
                            waypoints=waypoints,
                            swath_width=args.swath_width,
                            total_distance=total_distance,
                            estimated_time=estimated_time,
                            output_path=plot_path,
                            show=args.show_plot,
                        )

                    if plot_path and args.verbose:
                        print(f"✓ 可视化图表已保存: {plot_path}")
                except Exception as e:
                    print(f"⚠️  警告: 图表生成失败: {e}")

        # 最终成功提示（用户获得感）
        print("\n✅ 农田覆盖路径规划成功完成！")
        print(f"  📥 输入农田: {args.input_file}")
        print(f"  📤 输出航点: {args.output_file}")
        print(f"  📍 生成航点: {len(waypoints.waypoints)} 个")
        print(f"  💡 提示: 使用--report生成详细报告，--plot生成可视化图表")

    except FileNotFoundError:
        print(f"❌ 错误: 输入文件 '{args.input_file}' 不存在，请检查路径")
        return 1
    except Exception as e:
        print(f"❌ 执行出错: {e}")
        if args.verbose:
            import traceback
            print("\n【详细错误堆栈】")
            traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    # 退出码规范：0=成功，1=失败（符合Unix惯例）
    exit(main())