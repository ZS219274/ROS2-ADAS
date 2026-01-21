#!/bin/bash

# 自动运行脚本：先启动planning模块，等待4秒后启动control模块

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2环境
source install/setup.bash

echo "=========================================="
echo "开始启动ROS2-ADAS系统"
echo "=========================================="

# 启动planning模块（后台运行）
echo "[1/2] 启动planning模块..."
ros2 launch planning planning_launch.py > /dev/null 2>&1 &
PLANNING_PID=$!
echo "planning模块已启动 (PID: $PLANNING_PID)"

# 等待4秒
echo "[等待] 等待4秒让planning模块初始化..."
sleep 4

# 启动control模块（前台运行，可以看到输出）
echo "[2/2] 启动control模块..."
echo "=========================================="
ros2 launch control control_launch.py

# 如果control模块退出，清理planning进程
echo ""
echo "=========================================="
echo "control模块已退出，清理planning进程..."
kill $PLANNING_PID 2>/dev/null
echo "清理完成"
echo "=========================================="

