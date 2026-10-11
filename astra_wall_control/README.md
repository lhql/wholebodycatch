# ROS2 机械臂无人机墙面定点

本包当前提供以下三个任务入口，持续开发状态见 [PROGRESS.md](PROGRESS.md)。
三个入口均包含 Gazebo、PX4、MAVROS、LIO、唯一控制器和可选 RViz，**一次只运行一个**。
三个入口已完成本轮独立 Gazebo 真值验收；结果、曲线与 RViz 截图见 [验收记录](docs/validation/README.md)。
既有起飞问题和后续适用范围见下文。

| 入口 | 起飞后行为 | 目标含义 |
| --- | --- | --- |
| `manual_flight.launch.py` | 起飞至 1.6 m，图形滑条给定状态，正运动学显示末端 | 机体 xyz/yaw 和七关节分别控制 |
| `wholebody_corridor.launch.py` | 在安全走廊内跟踪多段五次最小 jerk 曲线 | 携臂无人机机体位置，绕过静态障碍 |
| `wholebody_point.launch.py` | 末端先到点，再在限定范围内运动机体并联合补偿 | 世界坐标固定末端点 + 有界机体运动 |

构建并启动（以下三条 launch 命令选择一条）：

```bash
cd /home/li/AstraDroneOpen/AstraDrone_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select astra_wall_control --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
OPENBLAS_NUM_THREADS=1 ros2 launch astra_wall_control manual_flight.launch.py
OPENBLAS_NUM_THREADS=1 ros2 launch astra_wall_control wholebody_corridor.launch.py
OPENBLAS_NUM_THREADS=1 ros2 launch astra_wall_control wholebody_point.launch.py
```

默认同时打开 RViz 和中文只读监控面板。通用参数：`gui:=false`、`rviz:=false`、`auto_arm:=false`、`lidar_downsample:=4`。
`dashboard:=false` 可单独关闭监控面板；默认跟随 `rviz` 开关。
前两个入口用 `takeoff_height:=1.6` 设置机体起飞高度；第三个入口的起飞高度由初始末端目标和机械臂偏移计算，`goal_height:=1.4` 设置二维点击的末端高度。
默认自动等待定位和预检、进入 OFFBOARD、解锁起飞。`auto_arm:=false` 时由操作者切换 OFFBOARD 并解锁后继续任务。

### 1. 空中独立控制

这是**给定机体/关节状态 → 正运动学显示末端**的入口，不会用逆运动学将机械臂拉向独立末端点。
默认打开 **Astra · 正运动学滑条控制** 窗口：11 条横向滑条分别设置机体 X/Y/Z/偏航和七个关节，
可以在机体工作范围与 URDF 关节限位内任意组合，也可在右侧数值框输入精确值并按回车。
等待起飞完成（APPROACH/HOLD、OFFBOARD、已解锁）后启用控制；拖动时实时计算面板中的 FK 末端位置和姿态，
**松开滑条后发送目标**，RViz 随后显示整机目标姿态。旋转单位 rad，伸缩/夹指单位 m。
FK 目标预览采用水平机体和目标偏航；实际末端读数来自包含机体姿态扰动的控制反馈。
“读取控制器当前目标”按钮用于与 RViz/键盘设置的目标同步，本身不发指令。
关闭滑条窗口后仍保持已接受的目标；`sliders:=false` 可关闭自动启动，独立打开可运行
`ros2 run astra_wall_control manual_sliders.py --ros-args -p use_sim_time:=true`。
滑条独立于 `rviz`/`dashboard` 开关，无图形环境时也需指定 `sliders:=false`。

原有 RViz 操作也可继续使用：
在 RViz 选择 **Interact**：拖动机体目标的 XYZ 箭头和偏航圆环，松开后发送机体目标；
右击目标或橙色关节控件，从菜单选择七个关节之一。旋转关节拖圆环，伸缩/夹指关节拖箭头。
拖动期间紫色半透明整机和 `FK` 标记显示目标状态及正运动学预测末端；
**松开鼠标才发送关节状态**，越限拒收。实心机身、橙色机械臂与绿色末端表示实际反馈。
选中的控件位于该关节的真实运动轴处，一次只显示一个关节的操作控件，避免七组圆环重叠。

也保留键盘输入：起飞完成后，在另一个已 source 环境的终端运行：

```bash
ros2 run astra_wall_control manual_teleop.py
```

键盘 `i/k` 控制地图 X、`j/l` 控制地图 Y、`r/f` 升降、`u/o` 偏航；空格停止，`q` 退出键盘程序。
按 `1..7` 选择关节，`+/-` 修改关节目标。关节顺序是 yaw、shoulder、extension、wrist、roll、左夹指、右夹指；
旋转关节每次 0.03 rad，伸缩与夹指每次 0.005 m。机械臂保持已接受的姿态，机体继续悬停或执行飞行命令。
持续按键通过终端自动重复发送；按键间断会停止速度输入，300 ms 后控制器保持当前位置。退出键盘程序不等于降落。

也可用 RViz 的 Interact 拖动 XYZ 设置机体位置，或用 2D Goal Pose 设置平面位置和偏航（高度采用 `takeoff_height`）。
接口为 `/body_goal` (`PoseStamped`, map)、`/manual/body_velocity` (`TwistStamped`, map，必须带仿真时间戳)、
`/manual/arm_goal` (`JointState`，全部七个唯一关节名和合法 position，不带 velocity/effort)。
机体位置目标范围 x∈[-1,3]、y∈[-1.5,1.5]、z∈[0.8,2.0]；速度限制为 xy 0.18 m/s、z 0.15 m/s、yaw 0.3 rad/s。
这些用户接口由控制器统一输出到执行器；不要在运行时直接向底层 `/my/arm/position_command` 发布竞争命令。

### 2. 携臂全身安全走廊导航

世界中橙色障碍占据 x∈[1.2,1.6]、y∈[-0.7,0.7]，默认路线从其 +Y 侧绕行。
起飞进入 APPROACH/HOLD 后，用 RViz Interact 拖动**机体**目标，或点击 2D Goal Pose。
推荐首次目标 `(2.7, 0, 1.6)`；也可精确发送：

```bash
ros2 topic pub --once /body_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.7, y: 0.0, z: 1.6}, orientation: {w: 1.0}}}"
```

半透明框是已按全身包络和跟踪余量收缩后的**机体中心允许区域**；绿色球是全身包络，黄色曲线是规划轨迹。
走廊包括机身、旋翼、起落架、机械臂和夹爪，机械臂保持运输姿态，航向保持 0。
采用类似 MINCO 的最小控制努力思路：多段五次 Bezier 曲线最小化 jerk 积分，段间保持 C2 连续；
通过控制点留在对应凸走廊内，保证整段连续曲线的空间包含关系。计算速度、加速度多项式极值后统一拉长时间，
每轴速度上限 0.12 m/s、加速度上限 0.18 m/s²。它不是外部 MINCO 库，也没有全局时间分配优化。
只有实测位置进入相邻区域交集后才切换；跟踪落后超过 0.12 m 时暂停轨迹时钟，面板显示暂停状态。
暂停会改变实际执行时序，不能把名义轨迹的 jerk 连续性当作扰动下实际飞行的 jerk 保证。
换目标会从当前位置重规划，采用静止端点边界；没有宣称非零速度换轨也能保持无缝 C2 连续。
不连通或靠近障碍的目标会拒收，保留原任务。
实测位置离开当前安全区域则退出导航并请求降落。安全依赖配套静态地图和实际跟踪误差；尚无动态避障。
修改 `corridor_file` 时必须同步匹配世界障碍，不能把未经验证的自由空间配置用于不同世界。

### 3. RViz 末端全身定点

等待 APPROACH/HOLD 后，选择 Interact，拖动红色目标 XYZ 箭头并松开，设置三维**末端**目标。
也可使用 2D Goal Pose 或 `/ee_goal`；二维箭头的偏航不作为末端姿态命令。
初始到点后，等待机体进入范围中心的内半区且估计速度低于 0.025 m/s，再默认开始小幅平移/偏航运动，
从当前机体参考平滑接入，同时让末端工具中心尽量保持在地图中的原目标点。
绿色球为测量计算的末端位置，`EE error` 为末端误差；机身 xyz/yaw 和五轴机械臂联合求解，夹爪保持开度。
末端位置优先于机体摆动，关节加中位姿态正则；摆动时工具朝向是较弱约束，避免过强朝向约束消耗左右移动所需的自由度。
已按用户确认的定义实现：**末端工具中心固定、机械臂关节尽量靠近中位姿态**。
横滚/俯仰不是独立的位置控制输入：由欠驱动四旋翼飞行环产生，其测量扰动由机械臂联合补偿。

默认机体参考范围相对末端目标对应的标称机体中心为 `±[0.06, 0.06, 0.05] m`，偏航 `±0.20 rad`；
默认摆动幅度 `[0.025, 0.035, 0.02] m`、偏航 `0.08 rad`，基准周期 32 s。实际能完成的幅度还受机械臂可达性限制。
末端估计误差 >4 cm 会暂停摆动相位并优先恢复定点；范围约束约束控制参考，物理惯性/估计误差仍需实测验收。
机体速度输出转换为 PX4 位置指令时加入实测速度阻尼，抑制携臂飞行的滞后与反作用放大。
默认参数已完成两个基准周期（约 64 s）的独立真值检查，工具误差最大 1.73 cm，未观测到机体越界。
RViz 紫色小框显示机体允许范围；紫色机体参考控件可给定范围内的偏移，同时保持红色末端目标不变。
右击末端或机体参考控件的菜单可暂停/恢复机体运动。新的机体偏移会平滑接入并启用摆动，越界拒收。

```bash
ros2 launch astra_wall_control wholebody_point.launch.py \
  hold_body_motion:=true body_motion_period:=32.0 \
  body_motion_bounds:='[0.06, 0.06, 0.05, 0.20]' \
  body_motion_amplitude:='[0.025, 0.035, 0.02, 0.08]'
```

`hold_body_motion:=false` 只进行原地末端定点。独立接口为 `/wholebody/body_goal` (`PoseStamped`, map) 与
`/wholebody/motion_enabled` (`Bool`)；改变机体参考不会改变 `/ee_goal` 的末端目标。
这是低速微分全身运动学控制，加上机械臂重力前馈和 PX4 飞行内环；详细实现及旧入口说明如下。

### 可视化与图例

三个入口默认打开 RViz 和中文只读监控面板。走廊入口采用任务总览视角，另外两个采用机械臂近景。
RViz 左侧可单独开关以下图层：

| 图层 | 显示内容 |
| --- | --- |
| Measured vehicle and arm | 根据 SDF/URDF 碰撞几何绘制的机体、四旋翼、起落架、各连杆和夹爪；实际姿态、机体/工具坐标轴 |
| Whole-body safety envelope | 全身包络球及轮廓；第三模式还显示有界机体运动框 |
| World obstacles | 与场景文件一致的障碍和房间边界 |
| Safe corridors and waypoints | 当前段绿色、下一段琥珀色、其他段青色，路段编号和连接点 |
| Planned and flown trajectories | 黄色规划曲线、蓝色机体历史轨迹、绿色末端轨迹、紫色飞控设定点历史 |
| Targets and tracking errors | 红色目标/误差线，手动模式还有紫色 FK 整机预览与预测末端 |
| Flight status | 阶段、误差、轨迹进度；失去反馈时变为 STALE，整机显示变灰 |

原始未收缩自由空间边界作为可选图层，默认关闭以免遮挡。历史轨迹最多 1,500 点，仿真时钟回退会清空。
监控面板显示机体/末端误差曲线、xyz 实际值和目标值、走廊净空与进度、七关节位置/目标/速度/力矩、
目标接受/拒收和故障原因。旋转轴单位 rad、rad/s、N·m；伸缩与夹指单位 m、m/s、N。
面板只读，所有实际状态来自控制反馈；这些显示不能替代独立真值验收。

Marker 图层位于 `/aerial_visualization/*`，`body_path`、`ee_path`、`reference_path`、`planned_path`
同时提供 `nav_msgs/Path`；TF 提供 `map → aerial/*` 的实际机体/连杆姿态。

### 检查与续做

在本包目录、source 完构建后的环境后运行：

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=74 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tests/check_ros_modes.py
ROS_DOMAIN_ID=74 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tests/check_ros_goals.py
```

物理验收脚本需与已启动的 launch 使用同一 ROS domain。选择对应的 `--mode manual|corridor|wholebody`：

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tests/sitl_modes.py \
  --mode corridor --seconds 280 --output /tmp/corridor_validation.json
```

该脚本会在起飞后自动发送测试目标，采集独立 Gazebo 真值。手动/走廊要求连续 8 秒位置误差 <10 cm；
手动模式还检查关节到达，走廊模式还检查已观测路径的全身包络与静态障碍净空。
末端模式要求机体实际 xyz/yaw 在配置范围内、真实末端误差 <3.5 cm、机体跟踪误差 <8 cm，
默认持续至少两个摆动基准周期；还必须观测到三轴各 >1 cm、偏航 >0.04 rad 和机械臂关节 >0.03 的实际变化，
防止静态悬停误判为联合控制成功。摆动期间观测到真实机体超界会使整次验收失败，不能恢复后重新累计来掩盖越界。
失败返回非零，控制器停止反馈超过 5 秒立即失败。
同时记录 `/Odometry`、PX4 位置和独立真值到 `.jsonl`，便于定位中途故障。

可用以下入口自动启动、验收和清理一次仿真（需要没有其他 PX4 占用 UDP 端口）：

```bash
python3 tests/run_sitl.py --mode wholebody --goal-source rviz --seconds 300 --output-dir /tmp/astra_wbc_check
python3 tests/check_lidar_geometry.py /tmp/astra_wbc_check/lidar_ground.npz
MPLCONFIGDIR=/tmp/astra_plot python3 tests/plot_sitl.py /tmp/astra_wbc_check/result.json /tmp/astra_wbc_check/evidence.png
```

`run_sitl.py` 默认 ROS domain 75、独立 Gazebo master 11365；日志、PX4 rootfs 和结果均放在指定目录。
`--goal-source rviz` 通过 InteractiveMarker 松开反馈话题发送目标，验证 RViz 服务端到控制器的真实 ROS 链路；
这是自动反馈消息测试，不等于人工鼠标拖动验收。
绘图脚本不需要 ROS，输出真实轨迹、真实/估计误差、机体范围或定位偏差、关节补偿或障碍净空。
原 `observe.py` 针对旧末端任务，不能直接用于判定手动关节动作成功。
最新完成项、实际误差及未验证事项见 [PROGRESS.md](PROGRESS.md)；下次开发会通过 `AGENTS.md` 先读取该进度。

从 ROS1 `my` 模型移植的五轴机械臂 + 双指夹爪，运行于本工作区的
ROS2 Foxy / Gazebo Classic / PX4 SITL / MAVROS。模型几何、质量和关节限位沿用原机型；机体使用本包独立的
`iris_arm_base` 动力模型，适配机械臂负载。ROS1 插件全部替换为 ROS2 接口。

```bash
cd /home/li/AstraDroneOpen/AstraDrone_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select astra_wall_control --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
OPENBLAS_NUM_THREADS=1 ros2 launch astra_wall_control wall.launch.py gui:=true
```

启动后自动等待定位、正常预检、解锁、起飞、接近蓝色墙面的红点并保持。
默认墙面为世界坐标 x=2 m，红点 (2, 0, 1.4) m。机身参考由该目标和
中间机械臂姿态计算，约 (1.5597, 0, 1.5965) m，航向沿 +X。
`auto_arm:=false` 可仅启动仿真和观测。仅针对 SITL 的独立启动入口，
不要与其他使用同一 Gazebo/PX4/MAVROS 端口的仿真同时启动。

控制使用激光惯导位置、PX4 姿态和关节编码器；Gazebo 真值只用于独立验收。
本包 MID360 射线从光学原点发出：现有共享 Livox 插件的自定义 `Init` 没有初始化 Gazebo 内部最小距离，
非零射线起点会导致返回距离漏加该偏移，原配置产生 0.2 m 径向缩短。现在仅将本包模型射线起点改为 0，
FAST-LIO 仍过滤 0.5 m 内近点。修复前后真实点云、五面墙/顶面检查结果保存在 `tests/data/lidar_geometry_*`，
没有用 Gazebo 位置补偿控制器定位，也没有修改共享插件。
所有测试场景都带有 z=3.2 m 的半透明顶面，为 MID360 提供高度可观测的水平面。
激光惯导在落地静止后初始化，避免落地冲击污染重力估计。
该机型 `SYS_HAS_MAG=0`，因此使用 `EKF2_EV_CTRL=11`，融合视觉位置、高度和航向，
`EKF2_HGT_REF=3`。启动时加载独立参数文件，未绕过解锁预检。

全身控制以机身 xyz/yaw + 五轴机械臂构成九维控制变量：

- 解析几何雅可比和有界最小二乘，末端位置权重最高，兼顾工具朝向、机身位置和航向。
- 横滚/俯仰是测量的欠驱动状态，由 PX4 飞行环控制；其扰动由机械臂补偿。
- 限制关节速度、加速度、接近关节限位的速度及机身到墙面间距。
- 根据 URDF 质量与质心计算各关节重力前馈；Gazebo 物理步内执行限力矩 PD。
- 仅通过 `Joint::SetForce` 驱动，机械臂反作用力由物理引擎传递到机身。
- 50 Hz 定时设定值发布；计算步长使用仿真时间，避免低实时率加快动作。
- 使用当前 PX4 局部位置与 LIO 位置间的变换发送设定点，避免 EKF 原点变化移动世界目标。
- 定位异常、反馈过期、控制失败后退出任务；退出 OFFBOARD 或解除武装后不会自动抢回控制或重新解锁。

这套控制针对低速末端定点。不是接触力控制器，不包含全身刚体逆动力学 MPC，
也不宣称欠驱动四旋翼能独立跟踪任意六维机身位姿。

话题：

| 话题 | 内容 |
| --- | --- |
| `/my/arm/joint_states` | 七轴实际位置、速度、施加力矩 |
| `/my/arm/position_command` | 全七轴位置 + 可选重力前馈 effort |
| `/my/arm/effort_command` | 全七轴纯力矩模式 |
| `/my/arm/wrist_wrench` | 腕部力传感器 |
| `/wall_control/telemetry` | 阶段、目标、误差和计算耗时 JSON |
| `/wall_eval/body` | 只供验收的 Gazebo 真值 |

命令必须包含全部七个唯一关节名，仿真时间戳，有限数值；过期命令拒收。
250 ms 没收到新命令时，关节驱动保持当前姿态，清除力矩前馈。

```bash
PYTHONPATH=src/astra_wall_control OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m unittest discover -s src/astra_wall_control/tests -v
ros2 run astra_wall_control observe.py --seconds 150 --output log/wall_validation.json
```

验收脚本从独立机身真值和关节反馈计算末端位置，记录完整 JSON/JSONL。
通过条件：最后连续约 20 秒处于 HOLD，真实末端误差 < 3.5 cm，真实机身误差 < 8 cm。
脚本失败返回非零状态，不能把命令已发布或估计误差较小当成物理验收成功。

模型来源：`PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/my`；
关节插件移植自其中 `src/arm_control.cpp`。URDF 坐标树来自同目录 `arm.urdf`。
场景改编自 ROS1 `aerial_arm_bounce_demo/worlds/rebound.world`。
ROS2 接触插件参数依据 [gazebo_plugins 官方文档](https://docs.ros.org/en/rolling/p/gazebo_plugins/generated/classgazebo__plugins_1_1GazeboRosBumper.html)。

空中 RViz 末端目标测试：

```bash
cd /home/li/AstraDroneOpen/AstraDrone_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select astra_wall_control --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
OPENBLAS_NUM_THREADS=1 ros2 launch astra_wall_control air_test.launch.py
```

**2026-09-29 历史验证状态（最新进度见 PROGRESS.md）：** launch 构建、算法测试、ROS 目标接口和 RViz 加载检查通过。
当时本机 SITL 未通过稳定起飞及末端物理到达验收。
该轮同时修正了 `flight.json` 中三项无效的 PX4 积分限幅参数名，
但这项修正本身未解决起飞问题。详细记录见 [验证记录](tests/air_test_validation.md)。

这个入口启动独立的 Gazebo / PX4 / MAVROS / LIO、控制器和 RViz。
空中场景去掉中间的墙面和立柱，保留外围墙体及半透明顶面供激光定位。默认自动起飞，
先接近末端初始点 `(2, 0, 1.4)`，再等待你更新目标。
等待 RViz 状态出现 `APPROACH` 或 `HOLD` 后，选择工具栏 **Interact**，
拖动红色目标的 XYZ 箭头，松开鼠标发送三维末端目标。
绿色球是反馈计算出的实际末端，整机碰撞几何显示机体、旋翼、起落架及机械臂实际状态；
显示的 `EE error` 是末端位置误差，连续满足到达条件后显示 `HOLD`。
这些显示使用控制器反馈，并非独立 Gazebo 真值验收。

也可以点击 **2D Goal Pose**，点击并拖动设置 x/y；末端 z 使用
`goal_height`（默认 1.4 m，可用 `goal_height:=1.6` 修改）。箭头朝向不用于控制：
本测试保持工具朝向 +X、机身航向约 0，只给末端位置目标。
飞机目标位置自动扣除机械臂的标称末端偏移，飞机与机械臂共同调整使末端到达。

目标坐标系为 `map`：出生点对齐的 ENU 米制坐标，地面 z=0。
目标范围 x∈[-1,3]、y∈[-1.5,1.5]、z∈[0.8,2.0]；越界、非有限值、
其他坐标系以及起飞完成前的目标会拒收。中止后发送目标不会重新解锁。
需要精确输入三维坐标时，也可发布同一个接口：

```bash
ros2 topic pub --once /ee_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 1.5}, orientation: {w: 1.0}}}"
```

`gui:=false` 关闭 Gazebo 窗口，`rviz:=false` 关闭 RViz；`auto_arm:=false`
仅准备系统，由你手动进入 OFFBOARD 并解锁后执行任务。该入口只用于无障碍空间的
SITL 低速定点，不包含路径规划和障碍物绕行；不要与 `wall.launch.py` 同时运行。
`/air_test/goal/update` 为交互标记，`/air_test/markers` 为状态显示，
`/goal_pose` 为 RViz 二维目标，`/ee_goal` 为三维末端目标。

动力与控制修正：

- 整机质量约 2.192 kg。旧动力最大总推力约 28.27 N，推重比约 1.31，
  机械臂前伸后的姿态修正容易令单个旋翼饱和。
- 本包 `models/iris_arm_base` 保持机体质量、惯量及几何，旋翼推力系数改为
  `8.54858e-6 N/(rad/s)^2`，最高转速仍为 1100 rad/s。
  最大总推力约 41.38 N，推重比约 1.92；悬停指令初值为 0.70。
  这是仿真动力系统的调整，不代表原实物电机具有相同能力。
- PX4 控制分配中的旋翼 x/y 位置与 SDF 实际几何保持一致。
  正确的积分限幅参数为 `MC_RR_INT_LIM`、`MC_PR_INT_LIM`、`MC_YR_INT_LIM`。
- 全身控制器输出的机身速度通过 0.7 秒位置前瞻发送给 PX4，避免在 PX4
  位置环外重复积分引起接近目标后的振荡。机械臂仍使用有界速度积分和重力前馈。
- 所有模型修改均位于本包，不覆盖共享的 PX4 / ROS1 模型。
