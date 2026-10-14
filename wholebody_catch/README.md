# wholebody_catch：无人机机械臂视觉抓取实验

基于 `astra_wall_control` 的 `my` 携臂无人机模型，使用 **FastLIO2 定位、腕部 RGB-D 定目标、机身 xyz/yaw 与五轴机械臂联合控制、双指接触抓取**。
适用环境沿用 ROS2 Foxy / Gazebo Classic / PX4 SITL / MAVROS。模型沿用原包；抓取入口启用基础控制器新增的可选机体速度阻尼。

默认感知使用 [SAM 2.1](https://github.com/facebookresearch/sam2) 分割与对齐深度，不要求颜色标记、类别标签或目标 CAD；卡尔曼滤波关联同一物体。每次选择工作区域内一个尺寸适合夹爪的候选，不提供按语义指定物品的功能。
默认测试场景仍是托台上的 40 mm、15 g 球体，有真实重力、碰撞和摩擦；另提供不同颜色、尺寸和位置的球、方块、圆柱场景生成器。
控制和视觉不读取世界中的物体位置，没有固定关节吸附、目标瞬移或真值定位。
抓取策略采用预规划轨迹和接触反馈，不使用强化学习，也不要求全程看见物体。
当前抓取候选是可见表面的低速 +X 侧向抓取启发式，尚不是通用六自由度抓取或避障规划。球体和方块各一次通过独立物理验收，其中方块在末段暂停视觉后仍完成抓取；重复成功率和实机迁移尚未验收。

## 构建和启动

本机已下载 SAM 2.1 tiny 权重并验证 CUDA 推理。迁移机器时，先准备 Python 3.11、PyTorch >= 2.5 / torchvision >= 0.20 以及兼容显卡驱动，再下载固定版本资源：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros2_ws
export CATCH_PYTHON="$HOME/miniconda3/envs/torch/bin/python"
bash src/wholebody_catch/scripts/setup_perception.sh
```

脚本将源码、Python 辅助依赖和权重放在 `dependencies/`，核对权重 SHA256，不升级现有 PyTorch/CUDA。ROS Foxy 仍使用系统 Python 3.8，推理在独立的 Python 3.11 进程中运行。
首次或更新源码后，在工作区构建：

```bash
cd /home/li/AstraDroneOpen/AstraDrone_ros2_ws
source /opt/ros/foxy/setup.bash
# 依赖包此前已构建时，复用它们：
source install/setup.bash
colcon build --packages-select astra_wall_control wholebody_catch --symlink-install
source install/setup.bash
ros2 launch wholebody_catch catch.launch.py
```

已构建后，一条命令启动仿真、FastLIO2、视觉抓取和可视化：

```bash
cd ~/AstraDroneOpen/AstraDrone_ros2_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 launch wholebody_catch catch.launch.py
```

加 `evaluate:=true` 同时启动独立物理验收，报告默认写入 `/tmp/wholebody_catch_validation.json`；
可用 `evaluation_output:=/绝对路径/结果.json` 指定位置。验收运行最长 300 秒墙钟时间，结束后不会关闭可视化。

依赖未构建时，使用 `colcon build --packages-up-to wholebody_catch --symlink-install`，并先按工作区既有说明准备 PX4 SITL、Livox 仿真插件及 ROS 依赖。
默认 `auto_arm:=true`，等待 FastLIO2、飞控预检和关节反馈就绪后自动解锁起飞至约 1.6 m。
`auto_arm:=false` 时由操作者进入 OFFBOARD 并解锁，随后抓取仍自动执行。

```bash
ros2 launch wholebody_catch catch.launch.py gui:=false rviz:=false
```

`inference_python:=/绝对路径/python` 可覆盖推理解释器。`perception:=sphere` 保留旧的已知半径红球检测基线，仅用于对应红球场景。

生成无标记方块试验，目标位置不会传给控制器：

```bash
ros2 run wholebody_catch make_catch_scene.py --shape box --seed 7 --output /tmp/catch_box.world
ros2 launch wholebody_catch catch.launch.py world:=/tmp/catch_box.world evaluate:=true
```

`--shape` 支持 `sphere`、`box`、`cylinder`；种子控制 32–42 mm 尺寸及托台位置。一次试验放置一个目标，换场景前先关闭上一套仿真。

腕部深度相机仍需要可用的 Gazebo 图形渲染环境；`gui:=false` 只关闭客户端。
`lidar_downsample:=4` 沿用原包设置，PX4 运行目录默认 `/tmp/wholebody_catch_px4`，可用 `rootfs:=...` 更改。
端口沿用原包，**同一时间只运行一个 PX4/Gazebo 实验**。

## 实验过程

1. 启动 Gazebo/PX4/MAVROS；15 秒后启动 FastLIO2，18 秒后启动外部定位桥，35 秒后启动全身控制器。
   延时用于沿用原包的落地静止初始化流程，真正解锁仍依赖定位稳定、反馈新鲜度和正常预检。
2. 原地起飞，等待视觉连续稳定观测，不会先飞向场景中写死的目标位置。
3. RGB、depth、CameraInfo 在 25 ms 内同步；读取曝光时间的机体和关节反馈插值，按原 URDF 的腕部相机光学外参转换至 `map`。
   `map` 沿用原控制器的出生点对齐坐标系；不直接将 FastLIO2 的 `camera_init` 当作 `map`。
4. 到目标前方 12 cm 的预抓取位置，确认工具方向和速度。根据此处观测生成五次时间缩放的直线接近轨迹，起止速度和加速度为零，标称峰值速度 2.5 cm/s。
5. 默认锁定此轨迹终点，避免夹指遮挡使分割中心漂移。目标丢失时，只要距终点小于 4.5 cm、距离最后一次真实观测不足 5 秒、跟踪偏差小于 2 cm、工具角度小于 0.10 rad，继续执行既定轨迹。闭爪仍要求轨迹走完、末端误差持续小于 6 mm 且速度小于 1.2 cm/s；预测值不会冒充新观测。
6. 双侧接触稳定 0.4 秒后抬升 16 cm。允许遮挡下用持续双侧接触、末端位移和稳定性验证，状态标记为 `contact_and_tool_motion`；有新鲜视觉时还必须满足物体抬升与夹爪邻近判据，标记为 `vision_and_contact`。二者都不替代独立物理真值验收。
7. 成功后悬停持物，继续监测接触，不自动松爪或执行搬运投放。

阶段：`WAIT_FLIGHT → SEARCH → PREGRASP → INSERT → CLOSE → LIFT → VERIFY → SUCCESS`。
各执行阶段有超时。遮挡不满足上述轨迹继续条件时进入 `RECOVER`：先停推进，再沿已走过的短路段退回，确认同一目标后重新预抓取；默认最多两次，恢复超时则中止。
定位/飞行反馈异常、目标明显跳变、时钟回退或抬升后接触丢失均锁存 `ABORT`。闭爪后不要求视觉重新出现。
盲行假设目标静止、近距离路径空旷；尚未实现障碍物距离场和动态物体预测抓取。
中止时保留夹爪命令，若仍在 OFFBOARD 则请求 `AUTO.LAND`；不会重新解锁或夺回人工切换的飞行模式。
核心节点退出会关闭整套仿真。

```bash
ros2 topic echo /wholebody_catch/status
ros2 topic echo /wholebody_catch/vision_status
ros2 service call /wholebody_catch/abort std_srvs/srv/Trigger '{}'
```

## 节点和接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/livox/lidar`, `/livox/imu` | 原模型传感器 | FastLIO2 输入 |
| `/Odometry` | `nav_msgs/Odometry` | 激光惯导定位，原控制器及 MAVROS 桥使用 |
| `/my/arm/camera/image_raw` | `sensor_msgs/Image` | 腕部 RGB |
| `/my/arm/camera/depth/image_raw` | `sensor_msgs/Image` | 对齐深度，32FC1 米或 16UC1 毫米 |
| `/my/arm/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/wholebody_catch/target` | `geometry_msgs/PointStamped` | 视觉抓取点，`map`，保留图像时间戳 |
| `/wholebody_catch/ee_goal` | `geometry_msgs/PoseStamped` | 抓取任务发往全身控制器的末端目标 |
| `/wholebody_catch/arm_reference` | `sensor_msgs/JointState` | 全身控制器输出，经重映射供任务节点合并夹爪 |
| `/my/arm/position_command` | `sensor_msgs/JointState` | 唯一最终机械臂命令，保留五轴和重力前馈，夹指以限速轨迹覆盖 |
| `/my/arm/gripper_left_contact`, `/my/arm/gripper_right_contact` | `gazebo_msgs/ContactsState` | 双侧目标接触 |
| `/wholebody_catch/tactile/left`, `/wholebody_catch/tactile/right` | `std_msgs/Bool` | 实机触觉适配器入口，`contact_source:=tactile` 时使用 |
| `/wholebody_catch/status`, `/wholebody_catch/vision_status` | `std_msgs/String` | JSON 状态及诊断 |
| `/wholebody_catch/eval/object`, `/wall_eval/body` | `nav_msgs/Odometry` | 仅独立验收脚本读取的物理真值 |

不要同时运行原包的其他控制 launch，也不要向上述目标/执行器话题发送竞争命令。
RViz 显示实际模型、末端参考、视觉抓取点和腕部画面；交互目标输入已关闭。
原 `/wall_control/telemetry` 的夹指 **参考值** 仍是原控制器的开度，任务合并后的夹指参考应查看 `/my/arm/position_command`；实际关节状态始终来自编码器。

## 参数和模型复用

`config/catch.yaml` 设置预抓取距离、接近/抬升速度、抬升量、遮挡距离/时限和夹指行程。旧 `catch_vision.target_radius` 仅用于 `perception:=sphere`。
`grasp_tip_offset=0.024 m` 表示抓取中心位于当前模型指尖 TCP 后方 24 mm，换夹爪时按实际工具标定修改；不是目标的已知位置。
`refine_approach=false` 默认冻结预抓取终点；设为 `true` 才允许推进期间最多 1 cm/s 的视觉终点修正，适合仍能完整看见物体的相机布局。
`object_vision` 的 `workspace_min/max`、`max_width`、`surface_insertion` 定义搜索区域、夹爪可夹宽度和可见表面后的浅插入量。它不恢复被遮住的完整物体几何，当前默认面向小型刚性物体。
原模型双指正值张开、负值闭合；当前张开 `0.018 m`，闭合指令 `-0.018 m`，原插件用受限 PD 力驱动，碰到球后实际关节不会穿过球体。
闭合位置是行程终点；任务节点另外将每根夹指的指令限制在实测位置向内最多 0.8 mm，避免夹到物体后继续积累位置误差。该限制基于新鲜编码器反馈，不是力传感器闭环。
接触允许最长 80 ms 的空样本间隙，以处理碰撞求解的短时抖动；消息本身必须保持新鲜。任务与独立验收都保留掉物检测。
目前未增加载荷质量自适应或接触力控制。
抓取入口设置基础控制器 `wholebody_velocity_damping=0.7 s`，在机体位置参考中减去实测速度乘以该系数；原包其他入口默认仍为零。
同时启用 `measured_motion_compensation`：末端距参考小于 8 cm 时，用实测机体速度重新分配机械臂速度，补偿飞控尚未实现的机体运动；保持关节速度、加速度和位置限位。远距离接近保持原来的朝向优先级，避免相机过早偏离目标。
启动等待超过 `startup_timeout`（默认 180 秒墙钟时间），或起飞已解锁但飞行遥测过期，也会中止；降落服务超时后会重试。

复用资源：原 `models/my`、`iris_arm_base`、MID360、URDF、`run_sim.sh`、FastLIO2 配置、飞控参数和 `wall_control.py`。
完整的接近路径需位于空旷空间；本实验没有沿用走廊模式的碰撞规划，也未做动态避障。

## 实机迁移入口

```bash
ros2 launch wholebody_catch perception.launch.py
```

该入口仅启动感知，默认订阅 RealSense 彩色图、对齐到彩色的深度图及相机内参，并查询曝光时间的 `map → camera_color_optical_frame` TF。话题、光学帧和世界帧均可通过同名 launch 参数修改。
它不需要当前无人机 URDF；腕部安装的相机 TF 应由标定外参与编码器生成，机体 TF 应由实际定位系统提供。真实时间戳、相机标定和工具尺寸仍不可省略。
任务状态机独立于 ROS/模型，但现有飞行及机械臂输出仍是本机的 PX4/五轴全身控制适配器。更换机器人需对接末端位姿、关节反馈和控制适配器，触觉 `Bool` 必须代表可靠夹持接触并持续更新。尚未开展实机飞行和抓取验收。

## 检查与物理验收

不构建、也不读取工作区编译文件即可运行算法检查：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/wholebody_catch:src/astra_wall_control \
  OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m unittest discover -s src/wholebody_catch/tests -v
```

ROS 消息检查只使用系统 ROS 和源码，不启动 Gazebo/PX4：

```bash
source /opt/ros/foxy/setup.bash
ROS_DOMAIN_ID=84 PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=src/wholebody_catch:src/astra_wall_control:$PYTHONPATH \
  OPENBLAS_NUM_THREADS=1 /usr/bin/python3 src/wholebody_catch/tests/check_ros_interfaces.py
```

物理验收时，**在启动 launch 前或刚启动时**，在相同 ROS domain 的另一个终端运行：

```bash
ros2 run wholebody_catch evaluate_catch.py --seconds 300 --output /tmp/wholebody_catch_validation.json
```

验收脚本要求观测到闭爪前基线，在任务 `SUCCESS` 后独立检查真实物体升高至少 12 cm、真实物体距真实末端小于 4 cm、持续双侧目标接触 2 秒。
双侧接触直接来自两个 Gazebo 接触传感器，验收不以任务节点上报的接触布尔值替代传感器检查。
超时、缺反馈、无初始基线、中止均返回非零；保存真实物体和末端轨迹。
算法、ROS 消息或 launch 解析通过不等于物理抓取验收通过。实际完成情况见 `PROGRESS.md`。

重复末段无视觉试验时，在仿真终端将启动输出保存到日志，并打开验收：

```bash
ros2 launch wholebody_catch catch.launch.py world:=/tmp/catch_box.world evaluate:=true > /tmp/catch_launch.log 2>&1
```

另一终端 source 相同工作区并使用相同 `ROS_DOMAIN_ID`，运行：

```bash
python3 src/wholebody_catch/tests/check_occluded_sitl.py /tmp/catch_launch.log
```

该故障注入脚本仅适用于 SITL：接近轨迹剩余不足 2 秒且跟踪误差小于 12 mm 时暂停本 launch 的视觉进程，任务结束后恢复；单独检查 `/tmp/wholebody_catch_validation.json` 的 `passed` 才算物理验收通过。第 12 轮的阶段事件、真实物体轨迹和确切方块场景保存在 `docs/validation/`。
# wholebodycatch
