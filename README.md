# ADAS-PnC

基于ROS2的自动驾驶辅助系统（Advanced Driver Assistance System）

## 系统要求

- **操作系统**: Ubuntu 22.04 (推荐)
- **ROS2版本**: ROS2 Humble
- **编译器**: GCC 11+ 或 Clang 14+

## 安装指南

### 1. 安装ROS2 Humble

推荐使用**鱼香ROS一键安装**脚本：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

按照提示选择安装ROS2 Humble版本。

或者手动安装，参考[ROS2官方安装文档](https://docs.ros.org/en/humble/Installation.html)。

### 2. 安装依赖

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libyaml-cpp-dev \
    python3-colcon-common-extensions
```

## 工程获取
```bash
 git clone https://github.com/ZS219274/ROS2-ADAS.git  -b PnC
```

## 编译

在工程根目录下执行编译命令：

```bash
cd /path/to/ASD-PnC
colcon build
```

编译完成后，source工作空间：

```bash
source install/setup.bash
```

## 运行

使用提供的自动运行脚本启动系统：

```bash
./autoRun.sh
```

该脚本会：
1. 自动source ROS2环境
2. 先启动planning模块（后台运行）
3. 等待4秒让planning模块初始化
4. 启动control模块（前台运行）
5. 当control模块退出时，自动清理planning进程

## 项目框架

```
ROS2-ADAS/
├── src/                          # 源代码目录
│   ├── base_msgs/               # 基础消息定义包
│   │   └── msg/                 # 自定义消息类型
│   │
│   ├── planning/                # 规划模块
│   │   ├── src/
│   │   │   ├── common/         # 通用工具和算法
│   │   │   ├── decision_center/ # 决策中心
│   │   │   ├── global_planner/ # 全局路径规划
│   │   │   ├── local_planner/  # 局部路径规划
│   │   │   ├── reference_line/ # 参考线生成
│   │   │   ├── vehicle_info/   # 车辆信息处理
│   │   │   ├── planning_process/ # 规划流程管理
│   │   │   ├── pnc_map_creator/  # 地图创建工具
│   │   │   ├── move_cmd/       # 运动指令生成
│   │   │   └── test/           # 测试代码
│   │   ├── config/             # 配置文件
│   │   ├── launch/             # Launch文件
│   │   ├── rviz/               # RViz配置文件
│   │   └── urdf/               # URDF模型文件
│   │
│   ├── control/                 # 控制模块
│   │   ├── src/
│   │   │   ├── common/         # 通用组件
│   │   │   │   └── lqr_controller/ # LQR控制器
│   │   │   └── controller/     # 控制器实现
│   │   ├── config/             # 控制器配置文件
│   │   └── launch/             # Launch文件
│   │
│   └── data_plot/              # 数据可视化模块
│
├── third_party/                 # 第三方依赖库
│   ├── include/                # 头文件
│   │   ├── eigen-3.4.0/       # Eigen线性代数库
│   │   └── OsqpEigen/         # OSQP优化求解器Eigen接口
│   └── lib/                    # 编译后的库文件
│
├── build/                       # 编译输出目录
├── install/                     # 安装目录
├── log/                         # 编译日志目录
├── autoRun.sh                   # 自动运行脚本
└── README.md                    # 本文件
```

## 模块说明

### base_msgs
定义系统中使用的自定义消息类型，包括车辆状态、路径点、控制指令等。

### planning（规划模块）
负责路径规划和决策：
- **decision_center**: 决策中心，负责高层决策逻辑
- **global_planner**: 全局路径规划器
- **local_planner**: 局部路径规划器，处理动态避障
- **reference_line**: 参考线生成和优化
- **vehicle_info**: 车辆状态信息处理
- **planning_process**: 规划流程管理和协调
- **pnc_map_creator**: 规划与控制地图创建工具
- **move_cmd**: 运动指令生成

### control（控制模块）
负责车辆底层控制：
- **controller**: 控制器实现，包括横向和纵向控制
- **common/lqr_controller**: LQR（线性二次调节器）控制器

### data_plot（数据可视化）
用于实时数据可视化和调试。

## 依赖库

- **Eigen 3.4.0**: 线性代数计算库
- **OsqpEigen**: OSQP二次规划求解器的Eigen接口
- **yaml-cpp**: YAML配置文件解析库

## 许可证

TODO: License declaration

## 维护者

- Wu_xiaoxiao (89206437@qq.com)
