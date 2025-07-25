# Integrated Robot System

ROS2集成机器人系统，包含RM75机械臂、DH AG95夹爪和RealSense相机的完整集成方案。

## 系统组成

- **机械臂**: RM75 7自由度机械臂
- **夹爪**: DH AG95 电动夹爪
- **相机**: RealSense D435i 深度相机
- **控制**: MoveIt2 路径规划 + 直接话题控制

## 功能特性

- ✅ **完整的URDF模型** - 机械臂、夹爪、相机集成
- ✅ **MoveIt2集成** - 机械臂路径规划和可视化
- ✅ **独立夹爪控制** - 通过话题直接控制夹爪
- ✅ **实时状态同步** - 所有关节状态实时显示
- ✅ **碰撞检测** - 完整的碰撞检测配置
- ✅ **RViz可视化** - 完整的机器人状态可视化

## 系统要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

## 安装和使用

### 1. 克隆仓库
```bash
git clone <repository_url>
cd rm_gripper_ws
```

### 2. 安装依赖
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 构建
```bash
colcon build
source install/setup.bash
```

### 4. 启动系统
```bash
ros2 launch integrated_robot_bringup integrated_robot_bringup.launch.py
```

## 包结构

```
src/
├── integrated_robot/           # 主要集成包
│   ├── robot_description/      # 机器人描述和配置
│   └── robot_bringup/         # 启动文件和脚本
├── ros2_rm_robot/             # RM机械臂驱动包
└── dh_ag95_ros2/              # DH夹爪驱动包
```

## 主要话题

### 控制话题
- `/joint_states` - 机械臂关节状态
- `/gripper/joint_states` - 夹爪关节状态
- `/gripper/gripper_controller/gripper_cmd` - 夹爪控制命令

### 状态话题
- `/robot_state_publisher/joint_states` - 合并后的关节状态
- `/tf` - TF变换信息
- `/move_group/display_planned_path` - MoveIt规划路径

## 配置说明

### MoveIt配置
- **规划组**: `rm_group` (机械臂7个关节)
- **显示组**: `gripper_display_group` (夹爪关节，仅显示)
- **控制器**: 只控制机械臂，夹爪独立控制

### 关节状态合并
- `joint_states_merger.py` 负责合并机械臂和夹爪的关节状态
- 确保TF树完整和RViz正确显示

## 故障排除

### 常见问题
1. **MoveIt显示零位状态**: 检查关节状态话题是否正确发布
2. **夹爪状态不更新**: 确认夹爪驱动正常运行
3. **规划失败**: 检查碰撞检测配置和起始状态

### 调试命令
```bash
# 检查关节状态
ros2 topic echo /robot_state_publisher/joint_states

# 检查TF树
ros2 run tf2_tools view_frames

# 检查MoveIt节点
ros2 node info /move_group
```

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 许可证

[添加你的许可证信息] 