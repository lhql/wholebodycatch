# 空中末端目标入口验证（2026-09-29）

已完成：

- 在 `/tmp/astra_air_build`、`/tmp/astra_air_install` 独立构建成功，launch 参数可解析。
- 7 项单元测试通过：雅可比、重力、墙面收敛、空中目标反向切换及加速度限制、非法输入和目标范围。
- `check_ros_goals.py` 在独立 ROS domain 验证目标回调、起飞前拒收、坐标系拒收、中止后拒收、二维目标高度、交互标记发布与关闭。
- RViz 实际加载 `config/air_test.rviz`，未报告插件加载错误。未进行人工拖动验收。

物理验收未通过：

- 默认配置及修正积分参数名后的配置都能进入 OFFBOARD 并解锁，但起飞未收敛。
- 修正参数名后的独立 Gazebo 机身真值 z 仍约 0.499 m。
- 当前 PX4 源码使用 `MC_RR_INT_LIM`、`MC_PR_INT_LIM`、`MC_YR_INT_LIM`；旧配置中的
  `MC_ROLLRATE_IMAX`、`MC_PITCHRATE_IMAX`、`MC_YAWRATE_IMAX` 不存在。已修正项目配置，
  并通过运行时 `px4-param show` 确认实际生效。
- 在 `/tmp` 临时降低积分增益与限幅的实验发生姿态失稳，没有写入项目默认配置。
- 因未进入 HOLD，未完成实际飞行中的连续目标切换验收；算法收敛与接口测试不能替代物理验收。

本轮运行日志：`/tmp/astra_air_sitl_full.log`、`/tmp/astra_air_sitl_fixed.log`、
`/tmp/astra_air_sitl_tuned.log`；真值验收：`/tmp/astra_air_validation.json`、
`/tmp/astra_air_fixed_validation.json`。这些临时文件可能被系统清理。

复查命令（先 source ROS 和构建后的工作区）：

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=74 /usr/bin/python3 tests/check_ros_goals.py
```

`check_ros_goals.py` 使用本包源码回调与已安装 URDF，不启动飞行仿真。
