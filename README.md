# ROS 2 基础 · 第六次作业（mini_sentry 哨兵机器人）

算法组《ROS 2 基础》讲义配套课后作业的实现。讲义只描述了 `mini_sentry` 的行为、没有给源码，本仓库是从零搭出来并**在本机 Ubuntu 虚拟机里真实编译、真实运行验证过**的可编译工作空间（ROS 2 Lyrical）。

## 目录

- [一、四道作业做了什么](#一四道作业做了什么)
- [二、编译与运行](#二编译与运行)
- [三、工程结构](#三工程结构)
- [四、四题实测数据](#四四题实测数据)
- [五、发行版差异（Humble / Jazzy / Lyrical）](#五发行版差异humble--jazzy--lyrical)
- [六、踩过的坑](#六踩过的坑)
- [七、资料](#七资料)

## 一、四道作业做了什么

| # | 作业要求 | 承载节点 | 实现要点 |
|---|----------|----------|----------|
| 1 | 最后 60 s 回防、血量低于阈值回防 | `sentry_brain` | 订阅 `/game_status`，`rclcpp_action` 异步客户端，100 ms 一决策；回防时**精确取消**巡逻目标再改道补给点 |
| 2 | 速度参数在线修改、非法值要拒绝 | `cmd_test` | `linear_x` / `angular_z` / `max_speed`，`add_on_set_parameters_callback` 校验范围 |
| 3 | 录 bag、回放看轨迹 | `scripts/record_demo.sh` / `replay_demo.sh` | 脚本自动适配新旧 `rosbag2` 命令行语法 |
| 4 | 订阅图像、画框、再发布 | `image_debug` | `SensorDataQoS().keep_last(1)` 匹配相机、`cv_bridge` 取图后 clone 再画、保留原 header 发布到 `/image_debug` |

配套节点：`chassis_sim`（差速底盘 + `/odom` + `/tf`）、`referee_sim`（裁判系统，发 `/game_status`、提供 `set_mode` 服务）、`goto_server`（`GoToPoint` 动作服务端，按限速走直线并发布 `/cmd_vel`）、`tf_demo`（tf 广播/监听示例）。接口包 `mini_sentry_interfaces` 提供 `GameStatus.msg`、`SetMode.srv`、`GoToPoint.action`。

## 二、编译与运行

```bash
source /opt/ros/lyrical/setup.bash      # Humble 换成 /opt/ros/humble/setup.bash
cd ros2-mini-sentry-homework            # 仓库根目录就是工作空间根目录
colcon build --parallel-workers 2
source install/setup.bash
```

一键起全套仿真（底盘 + 裁判 + 动作服务端 + 决策节点）：

```bash
ros2 launch mini_sentry sentry.launch.py
# 调试用：不模拟掉血、比赛时长 90 s
ros2 launch mini_sentry sentry.launch.py simulate_damage:=false match_duration:=90.0
```

作业 2 单独验证参数：

```bash
ros2 run mini_sentry cmd_test          # 终端 A：参数服务端
ros2 run mini_sentry chassis_sim       # 终端 B：底盘，观察 /cmd_vel
ros2 param set /cmd_test linear_x 0.3
ros2 param set /cmd_test linear_x 5.0  # 期望被拒：超过 max_speed
```

作业 3 录制与回放：

```bash
bash scripts/record_demo.sh            # 录 40 s 到 ./bag
bash scripts/replay_demo.sh            # 回放，rviz2 里看轨迹
```

作业 4 图像链路：

```bash
ros2 run image_tools cam2image --ros-args -p burger_mode:=true
ros2 run mini_sentry image_debug
ros2 run rqt_image_view rqt_image_view   # 选 /image_debug
```

## 三、工程结构

```
ros2-mini-sentry-homework/
├── src/
│   ├── mini_sentry_interfaces/          # 接口包：msg / srv / action
│   │   ├── msg/GameStatus.msg           # 比赛剩余时间、血量、状态
│   │   ├── srv/SetMode.srv              # 巡逻 / 回防 / 待命
│   │   └── action/GoToPoint.action
│   └── mini_sentry/                     # 主包，7 个可执行节点
│       ├── src/chassis_sim.cpp
│       ├── src/referee_sim.cpp
│       ├── src/goto_server.cpp
│       ├── src/sentry_brain.cpp
│       ├── src/cmd_test.cpp
│       ├── src/tf_demo.cpp
│       ├── src/image_debug.cpp
│       ├── launch/sentry.launch.py
│       └── config/sentry.yaml, sentry.rviz
├── scripts/
│   ├── record_demo.sh
│   └── replay_demo.sh
└── docs/
    ├── ROS2基础知识点清单.html          # 讲义知识点整理
    └── mini_sentry_ws 作业说明与验证.pdf # 说明 + 全部源码的离线版
```

## 四、四题实测数据

验证环境：VMware 虚拟机 `Ubuntu 64 位 (2)`，Ubuntu 26.04.1 LTS，ROS 2 **Lyrical**（`/opt/ros/lyrical`），2 核 / 5 GB，`rmw_fastrtps_cpp`。

**编译**：`colcon build --parallel-workers 2`，两个包全部 Finished（接口包约 1 分 25 秒，主包约 1 分 11 秒，增量 30–80 秒）。

**作业 1**：4 个航点 `(1,1) → (4,1) → (4,3) → (1,3)` 循环，每段 1.7–3.4 s；剩余 59 s 时打印「进入回防：回补给点 (0.0, 0.0)」，取消巡逻目标后回到 `(0,0)` 打印「已回到补给点，原地待命」。日志里进入回防 1 次、发目标 1 次、回点 1 次、CANCELED 0 次。`set_mode {mode:1}` 返回 `success=True`。

**作业 2**：`/cmd_vel` 初值 `linear_x 0.6 / angular_z 0.8`；改 `linear_x 0.3` 立即生效；设 `5.0` 被拒并返回「`Setting parameter failed: linear_x 超限，|v| 不能大于 max_speed=2.000000`」，原值不变；再改 `angular_z 2.0` 生效。

**作业 3**：短录 40 s 得到 37.5 s / 1.9 MiB / 3938 条消息（`/odom` 1871、`/tf` 1878、`/game_status` 187、`/tf_static` 2）；回放期间 `/odom` 稳定 50 Hz，按 bag 里的 odom 重建出完整矩形巡逻路线。

**作业 4**：`/image_debug` 30.3 Hz，QoS 为 `BEST_EFFORT` / `KEEP_LAST(1)`，`frame_id=camera_frame`，320×240 `bgr8`；`box_width` 在线改到 400 后框变大；抓帧上可见居中绿框与帧号文字。

## 五、发行版差异（Humble / Jazzy / Lyrical）

| 项目 | 处理方式 |
|------|----------|
| CMake 宏 | Lyrical 已移除 `ament_target_dependencies`，改用 `target_link_libraries(<exe> PUBLIC rclcpp::rclcpp …)`；接口包链接 `mini_sentry_interfaces::mini_sentry_interfaces`；`cmake_minimum_required` 提到 3.10 |
| 头文件 | `cv_bridge/cv_bridge.h` 只有 `.hpp` → `__has_include` 条件包含；`tf2_ros` 系列头文件优先 `.hpp`；`tf2/LinearMath/Quaternion.h` 不存在 → 直接按 `yaw` 构造 `q.z/q.w` |
| `package.xml` | 同一依赖不能同时写 `<depend>` 与 `<exec_depend>`，否则 buildfarm 报重复 |
| `rosbag2` | 话题必须用 `--topics`；默认存储由 sqlite3 改为 mcap（脚本显式 `-s sqlite3`）；`-d` 变成单文件最长时长会切片，限时录制改用 `timeout -s INT`；回放倍速是 `--rate 2.0` |
| 参数 | `referee_sim` 原来只在构造函数读一次参数，改为 `tick()` 内 `get_parameter` 每帧重读，`ros2 param set` 才能在线生效 |

## 六、踩过的坑

- **回防目标打架**：`async_cancel_all_goals()` 会把刚发的新目标一起取消，日志出现「目标未完成(code=5)」抖动。改成 `async_cancel_goal(goal_handle_)` 精确取消 + 新目标延后 3 个 tick（0.3 s）+ 目标序号自增作废旧回调，抖动归零。
- **手动切模式被覆盖**：加 `auto_retreat_` 标记，手动用 service 切回防/待命后，tick 不再自动改回巡逻。
- **车停住时 `ros2 topic echo --once /cmd_vel` 会一直阻塞**（没有新消息就不返回），改用 `timeout` 包一层。
- **`pip install colcon-common-extensions` 依赖冲突**（ResolutionImpossible），改用 `apt-get install python3-colcon-common-extensions`。

## 七、资料

- `docs/ROS2基础知识点清单.html`：讲义《ROS 2 基础》的知识点整理（话题/服务/动作/QoS/参数/tf/launch/rosbag2 等）。
- `docs/mini_sentry_ws 作业说明与验证.pdf`：本文档的离线版，附录含全部源码，便于打印或提交。
- `docs/工程说明与实测日志.md`：更细的节点行为、话题/服务/动作表、参数说明和逐条实测日志。
