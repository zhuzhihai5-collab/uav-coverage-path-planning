#!/usr/bin/env python3
"""
田间覆盖规划器的简易运行脚本。

此脚本允许您在未安装包的情况下运行田间覆盖规划器。
它会自动将 src 目录添加到 Python 路径中并运行命令行界面 (CLI)。
"""

import sys
from pathlib import Path

# 将 src 目录添加到 Python 路径
project_root = Path(__file__).parent
src_path = project_root / "src"
sys.path.insert(0, str(src_path)) # src_path 指向项目根目录下的 src 文件夹 所以，Python 会直接进入 src/field_coverage/ 这个文件夹。

# 导入并运行命令行界面 (CLI)
from field_coverage.cli import main  #去找到 src/field_coverage/cli.py 文件，加载它，并把里面定义的 main 函数 拿过来。

if __name__ == "__main__":
    sys.exit(main())