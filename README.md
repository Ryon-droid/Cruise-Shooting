# CRAIC2026 巡航射击小车工程

本工程是面向 CRAIC2026 机器人任务挑战赛“目标射击场景”的 ROS Melodic/catkin 工作空间，核心目标是在 3.6m x 3.6m 场地内完成自主导航、语音任务识别、视觉/AR 靶标识别、自动瞄准射击和终点返回。

## 赛道规则摘要

- 比赛方式：线上。
- 场地尺寸：`3.6m x 3.6m`，四周围栏高度 `30cm`。
- 起点/终点：各 `40cm x 40cm`。
- 任务点：1-3 共三个任务点，每个任务点为 `38cm x 32cm`。
- 靶标：1 号任务点对应环形计分靶，2 号任务点对应旋转靶，3 号任务点对应移动靶。
- 靶距：环形靶距离任务点水平距离 `190cm`；旋转靶和移动靶距离任务点水平距离 `150cm`。
- 靶心高度：离地 `26cm`。
- 挡板：任务点之间由长 `120cm`、高 `30cm` 的挡板隔离。
- 任务信息：比赛开始后通过语音告诉机器人旋转靶和移动靶标号。
- 射击限制：机器人到达任务点后自行瞄准射击，瞄准射击过程中不可进入禁行区域。
- 时间限制：单场不超过 `2min`；裁判宣布开始后 `30s` 未运动则比赛结束。
- 结束条件：进入终点、触碰围挡、队员进入场地、主动示意结束、程序长时间无状态变化等。
- 排名规则：同分时按比赛终止前用时排序，用时更短排名更优。

主要得分点：

| 项目 | 分值 |
| --- | --- |
| 语音发布任务信息 | 10 |
| 到达任务点 1 | 10 |
| 击中 1 前方环形标靶 | 20 |
| 到达任务点 2 | 10 |
| 击倒 2 前方任务标靶 | 15 |
| 到达任务点 3 | 10 |
| 击倒 3 前方任务标靶 | 15 |
| 到达终点区域 | 10 |
| 技术文档或现场答辩 | 10 |

## 工程方案

本工程当前采用“地图导航 + 视觉闭环瞄准 + 串口射击”的方案：

```text
语音启动/识别目标编号
  -> 导航到任务点 1
  -> 识别圆形靶并转向瞄准
  -> 串口开火
  -> 导航/开环后退到任务点 2
  -> AR 识别旋转靶编号并瞄准
  -> 串口开火
  -> 导航/过渡到任务点 3
  -> AR 识别移动靶编号并动态提前射击
  -> 后退/横移到终点
```

主流程脚本：

```text
src/robot_slam/scripts/shoot_2025_new.py
```

主启动文件：

```text
src/robot_slam/launch/multi_goal_shoot_2025.launch
shall/2026_shoot.sh
```

## 目录结构

```text
.
├── shall/                         # 一键启动脚本
│   ├── 1-gmapping.sh              # 建图
│   ├── 2-navigation.sh            # 导航
│   └── 2026_shoot.sh              # 比赛流程
├── src/
│   ├── abot_base/
│   │   ├── abot_bringup/           # 底盘、雷达、IMU、模型启动
│   │   ├── abot_imu/               # IMU 节点和 RawImu 消息
│   │   ├── abot_model/             # URDF 模型
│   │   └── lidar_filters/          # 雷达滤波
│   ├── robot_slam/                 # 主导航/射击比赛包
│   ├── shoot_cmd/                  # C++ 射击串口控制
│   ├── track_tag/                  # USB 摄像头与 AR tag 识别
│   ├── abot_find/                  # find_object_2d
│   ├── TTS_audio/                  # TTS 服务
│   ├── robot_voice/voice_demo/     # 语音相关示例
│   ├── abot_object_detect/         # HSV、人脸、目标检测
│   ├── abot_vlm/                   # 视觉大模型识别
│   └── hector_slam/                # hector_slam 源码
├── build/                          # catkin 生成目录
└── devel/                          # catkin 开发空间
```

日常修改优先集中在：

- `src/robot_slam/scripts/shoot_2025_new.py`
- `src/robot_slam/launch/multi_goal_shoot_2025.launch`
- `src/robot_slam/params/carto/*.yaml`
- `src/robot_slam/maps/shoot_lab.yaml`
- `src/track_tag/launch/*.launch`

## 运行环境

默认车载环境：

- ROS：`/opt/ros/melodic/setup.bash`
- 工作空间：`/home/abot/8C8PE4`
- 主系统：Ubuntu + ROS Melodic
- 图形界面：`gnome-terminal`、`DISPLAY=:0`
- Python：底盘/ROS 脚本混用 Python 2 和 Python 3，语音识别脚本使用 conda 环境

默认硬件设备：

| 设备 | 默认路径 |
| --- | --- |
| 底盘串口 | `/dev/abot` |
| 激光雷达 | `/dev/rplidar` |
| 射击串口 | `/dev/shoot` |
| USB 摄像头 | `/dev/video0` |

## API 密钥替换

本工程中部分语音、TTS 和视觉大模型模块需要外部平台 API 密钥。为了保持完赛代码的运行方式，上传版保留原代码中的“直接在文件里配置变量”的写法，只把真实密钥替换为 `YOUR_...` 占位符。下载到车载端后，按原位置把占位符替换成自己的平台密钥即可。

先用下面命令检查密钥位置：

```bash
rg -n "appid|token|YI_KEY|DOUBAO_KEY|ARK_API_KEY|login_params|api_key|Bearer" src
```

需要重点检查这些文件：

| 文件 | 需要替换的内容 | 用途 |
| --- | --- | --- |
| `src/TTS_audio/scripts/TTS.py` | `appid`、`token`、`cluster`、`voice_type` | 火山/字节 TTS WebSocket 播报 |
| `src/abot_vlm/scripts/API_KEY.py` | `YI_KEY` | 零一万物 `yi-vision` 视觉模型 |
| `src/abot_vlm/scripts/API_KEY_DOUBAO.py` | `DOUBAO_KEY` | 豆包视觉模型 |
| `src/abot_vlm/scripts/doubao_openAI.py` | `ARK_API_KEY` 环境变量 | 豆包 OpenAI 兼容接口 |
| `src/robot_voice/src/iat_publish.cpp` | `login_params` 中的 `appid` 等 | 科大讯飞语音识别 |
| `src/robot_voice/src/tts_subscribe.cpp` | `login_params` 中的 `appid` 等 | 科大讯飞 TTS |
| `src/robot_voice/src/voice_assistant.cpp` | `login_params` 中的 `appid` 等 | 语音助手 |

按原代码方式替换占位符：

```python
# src/abot_vlm/scripts/API_KEY.py
YI_KEY = "YOUR_YI_API_KEY"

# src/abot_vlm/scripts/API_KEY_DOUBAO.py
DOUBAO_KEY = "YOUR_DOUBAO_API_KEY"
```

```python
# src/TTS_audio/scripts/TTS.py
appid = "YOUR_VOLC_TTS_APPID"
token = "YOUR_VOLC_TTS_TOKEN"
cluster = "volcano_tts"
voice_type = "YOUR_VOLC_TTS_VOICE_TYPE"
```

科大讯飞语音模块仍然使用原来的 `login_params` 字符串形式：

```cpp
const char* login_params = "appid = YOUR_XFYUN_APPID, work_dir = .";
```

修改 `src/robot_voice/src/*.cpp` 中的 `login_params` 后，需要重新编译：

```bash
cd ~/8C8PE4
catkin_make
source devel/setup.bash
```

注意：`src/abot_vlm/scripts/doubao_openAI.py` 原本就是通过 `ARK_API_KEY` 环境变量读取豆包/火山方舟密钥，该文件保持原写法。如果使用这个脚本，运行前设置：

```bash
export ARK_API_KEY="YOUR_ARK_API_KEY"
```

## 本地模型文件

语音识别模型权重 `src/robot_slam/scripts/paraformer-zh/model.pt` 约 840MB，超过 GitHub 普通仓库单文件限制，已在 `.gitignore` 中排除。部署到小车时，需要把该模型文件手动放回：

```text
src/robot_slam/scripts/paraformer-zh/model.pt
```

如果模型路径变化，需要同步修改：

```text
src/robot_slam/scripts/2026_shoot_demo.py
src/robot_slam/scripts/demo.py
```

## 编译

```bash
cd ~/8C8PE4
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```

脚本权限：

```bash
chmod +x src/robot_slam/scripts/*.py
chmod +x src/abot_base/abot_bringup/scripts/*.py
chmod +x src/TTS_audio/scripts/TTS.py
```

## 一键启动

建图：

```bash
bash shall/1-gmapping.sh
```

普通导航：

```bash
bash shall/2-navigation.sh
```

巡航射击比赛流程：

```bash
bash shall/2026_shoot.sh
```

`2026_shoot.sh` 会启动：

- `roscore`
- `abot_bringup robot_with_imu.launch`
- `abot_bringup shoot.launch`
- `robot_slam navigation_shoot.launch`
- `track_tag usb_cam_with_calibration.launch`
- `track_tag ar_track_camera.launch`
- `find_object_2d find_object_2d_shoot.launch`
- `TTS_audio TTS.py`
- `robot_slam 2026_shoot_target.py`
- `robot_slam/scripts/2026_shoot_demo.py`
- `robot_slam multi_goal_shoot_2025.launch`

注意：`shoot_2025_new.py` 会直接打开 `/dev/shoot` 串口；`abot_bringup shoot.launch` 也会打开同一串口。如果出现串口占用、开火异常或随机失效，优先二选一：保留主流程直接串口，关闭 `abot_bringup shoot.launch`；或改主流程为发布 `/shoot`，只由 `shoot.py` 控制串口。

## 手动启动顺序

调试时建议分终端启动，便于定位问题：

```bash
roscore
```

```bash
source ~/8C8PE4/devel/setup.bash
roslaunch abot_bringup robot_with_imu.launch
```

```bash
source ~/8C8PE4/devel/setup.bash
roslaunch robot_slam navigation_shoot.launch
roslaunch robot_slam view_nav.launch
```

```bash
source ~/8C8PE4/devel/setup.bash
roslaunch track_tag usb_cam_with_calibration.launch
roslaunch track_tag ar_track_camera.launch
```

```bash
source ~/8C8PE4/devel/setup.bash
roslaunch find_object_2d find_object_2d_shoot.launch
```

```bash
source ~/8C8PE4/devel/setup.bash
rosrun TTS_audio TTS.py
rosrun robot_slam 2026_shoot_target.py
```

```bash
source ~/8C8PE4/devel/setup.bash
cd ~/8C8PE4/src/robot_slam/scripts
/home/abot/anaconda3/envs/py39/bin/python 2026_shoot_demo.py
```

```bash
source ~/8C8PE4/devel/setup.bash
roslaunch robot_slam multi_goal_shoot_2025.launch
```

## 关键节点与话题

| 名称 | 作用 |
| --- | --- |
| `/cmd_vel` | 底盘速度控制 |
| `/odom` | 融合后里程计 |
| `/scan_filtered` | 雷达滤波结果 |
| `/move_base` | 导航 action server |
| `/initialpose` | AMCL 初始位姿 |
| `/object_position` | 圆形靶视觉位置，`Point.z` 为目标 ID |
| `/ar_pose_marker` | AR tag 位姿 |
| `audio_topic` | 触发录音识别 |
| `chinese_topic` | 语音识别文本 |
| `target_id_rotating` | 旋转靶编号 |
| `target_id_moving` | 移动靶编号 |
| `/voiceWords` | 播报文本 |
| `tts_service` | TTS 服务 |
| `/shoot` | 射击命令 topic，当前主流程未优先使用 |

语音链路不稳定时，可以手动发布靶号：

```bash
rostopic pub /target_id_rotating std_msgs/Int32 "data: 1" -1
rostopic pub /target_id_moving std_msgs/Int32 "data: 6" -1
```

## 调参指南

### 1. 场地与地图

相关文件：

```text
src/robot_slam/maps/shoot_lab.yaml
src/robot_slam/maps/shoot_lab.pgm
src/robot_slam/launch/navigation_shoot.launch
```

调参方法：

- 场地或围挡变化后，重新建图并保存为 `shoot_lab`。
- `shoot_lab.yaml` 中的 `image` 当前是 `/home/abot/8C8PE4/...` 绝对路径，换机器后必须同步修改。
- 在 RViz 中检查地图比例，规则场地应对应 `3.6m x 3.6m`。
- 射击场地有挡板和禁行线，地图中应保留会影响导航的实体障碍；不要把禁行线当作可穿越区域处理。

常用命令：

```bash
roslaunch robot_slam gmapping.launch
roslaunch robot_slam save_map.launch map_name:=shoot_lab
roslaunch robot_slam navigation_shoot.launch
roslaunch robot_slam view_nav.launch
```

### 2. 任务点与过渡点

相关文件：

```text
src/robot_slam/launch/multi_goal_shoot_2025.launch
```

当前参数：

```xml
<param name="goalListX" value="   1.06,   1.08,  1.19,  0.20"/>
<param name="goalListY" value="  -0.44,  -1.79, -3.00, -3.02"/>
<param name="goalListYaw" value=" 0.00,   0.00,  0.00,  0.00"/>
<param name="transX" value=" 0.12, 0.15, 0.20, 0.20 " />
<param name="transY" value="-0.42,-1.70,-1.62,-2.85" />
<param name="transYaw" value="0, 0  ,0,   0" />
```

含义：

- `goalList*`：任务点 1、任务点 2、任务点 3、终点。
- `trans*`：任务点之间的过渡点，用来绕开挡板、围挡。
- `Yaw` 单位是角度，脚本内会转换成四元数。

调参步骤：

1. 启动 `navigation_shoot.launch` 和 RViz。
2. 用 `2D Pose Estimate` 设置初始位姿。
3. 手动发送 `2D Nav Goal` 到任务点中心，观察是否能稳定进入任务点。
4. 在到达点用 `rostopic echo /amcl_pose` 记录当前 `x/y`。
5. 将记录值写入 `goalListX/Y`，再根据枪口朝向微调 `goalListYaw`。
6. 对挡板附近卡住的位置增加或移动 `transX/Y`。

现象与调整：

| 现象 | 优先调整 |
| --- | --- |
| 到点后车体没有完全进任务点 | 修正 `goalListX/Y`，减小 `xy_goal_tolerance` |
| 到点后枪口偏左/偏右很多 | 先改 `goalListYaw`，再改瞄准补偿 |
| 过挡板时擦边或绕路失败 | 移动 `transX/Y`，增大安全距离 |
| 导航到靶前过慢 | 稍增 `max_vel_trans`，但先保证不碰围挡 |
| 规划目标落入障碍膨胀区 | 调任务点坐标或 `default_tolerance`，不要盲目降膨胀半径 |

### 3. move_base / DWA

相关文件：

```text
src/robot_slam/params/carto/dwa_local_planner_params.yaml
src/robot_slam/params/carto/move_base_params.yaml
src/robot_slam/params/carto/costmap_common_params.yaml
src/robot_slam/params/carto/local_costmap_params.yaml
src/robot_slam/params/carto/global_costmap_params.yaml
```

重点参数：

| 参数 | 当前值 | 调参建议 |
| --- | --- | --- |
| `max_vel_x/y` | `0.60` | 上限速度，场地小，比赛前不要盲目拉高 |
| `max_vel_trans` | `0.4` | 整体平移速度上限，想压缩时间先小幅加到 `0.45` 左右试 |
| `min_vel_trans` | `0.2` | 太大容易到点冲过，太小容易爬不动 |
| `max_vel_theta` | `3` | 转向速度，过高会导致 AMCL 和相机画面抖 |
| `xy_goal_tolerance` | `0.15` | 任务点只有 `38cm x 32cm`，建议实测后压到 `0.06-0.10` |
| `yaw_goal_tolerance` | `0.15` | 枪口方向关键，建议压到 `0.05-0.10` |
| `sim_time` | `2.0` | 窄场地可适当降到 `1.0-1.5`，路径更灵活 |
| `path_distance_bias` | `32.0` | 越大越贴全局路径 |
| `goal_distance_bias` | `24.0` | 越大越急着冲目标点 |
| `inflation_radius` | `0.01` | 当前非常小，容易贴边；若擦围挡可逐步增大 |
| `footprint` | `0.38m x 0.34m` | 必须和实际车体/射击结构外廓一致 |

注意：当前 `holonomic_robot: true`，但 `vy_samples: 0`，意味着 DWA 不会主动采样横移速度；代码里的横移主要来自 `_move_until_obstacle(linear_y=...)` 这类开环控制。如果希望 `move_base` 也规划横移，需要把 `vy_samples` 调大并重新验证避障。

### 4. AMCL 定位

相关文件：

```text
src/robot_slam/launch/include/amcl.launch.xml
```

重点参数：

- `odom_model_type: omni`：适配全向底盘。
- `min_particles / max_particles`：当前 `500 / 2000`，小场地足够；定位跳变可加大。
- `update_min_d / update_min_a`：当前 `0.015 / 0.015`，更新较频繁。
- `laser_max_beams`：当前 `180`，增大可提升匹配但更耗 CPU。
- `laser_likelihood_max_dist`：当前 `4.0`，小场地可根据地图质量适当降低。

调参建议：

- 每次开始前都在 RViz 手动给准 `2D Pose Estimate`。
- 如果导航越跑越偏，先查 TF 和轮速里程计，再调 AMCL。
- 如果原地转向后定位发散，降低 `max_vel_theta` 或提高 IMU/里程计稳定性。
- 如果地图匹配不稳，检查 `/scan_filtered` 是否把围挡、挡板过滤掉了。

### 5. 圆形靶瞄准

相关文件：

```text
src/robot_slam/scripts/shoot_2025_new.py
```

重点常量：

```python
CIRCULAR_OBJECT_ID = 35
IMAGE_CENTER_X = 320
CIRCULAR_TURN_THRESHOLD = 6
CIRCULAR_FIRE_THRESHOLD = 6
CIRCULAR_STABLE_FRAMES = 4
CIRCULAR_FORCE_FIRE_TIMEOUT = 15.0
CIRCULAR_TURN_GAIN = 0.012
CIRCULAR_TURN_MAX = 1.20
CIRCULAR_TURN_MIN = 0.16
CIRCULAR_TURN_PULSE_TIME = 0.35
```

调参方法：

- 靶心总是偏左/偏右：先重新标定相机和枪口，再调 `IMAGE_CENTER_X`。
- 转向慢、迟迟对不准：略增 `CIRCULAR_TURN_GAIN` 或 `CIRCULAR_TURN_MIN`。
- 转向过冲、来回摆：减小 `CIRCULAR_TURN_GAIN`、`CIRCULAR_TURN_MAX` 或 `CIRCULAR_TURN_PULSE_TIME`。
- 偶发误射：增大 `CIRCULAR_STABLE_FRAMES` 或减小 `CIRCULAR_FIRE_THRESHOLD`。
- 一直不开火：放宽 `CIRCULAR_FIRE_THRESHOLD`，或检查 `/object_position` 的 ID 是否为 `35`。

### 6. 旋转靶与移动靶瞄准

相关输入：

```text
/ar_pose_marker
target_id_rotating
target_id_moving
```

重点常量：

```python
YAW_THRESHOLD_ROTATING = 0.03
YAW_THRESHOLD_MOVING = 0.03
TARGET_Y_RANGE = (-0.22, -0.10)
ROTATING_FIRE_X_OFFSET = 0.03
MOVING_FIRE_X_LEAD = 0.05
MOVING_FIRE_MIN_DELTA = 0.005
FIRE_SETTLE_TIME = 0.30
```

旋转靶：

- 总是打左偏：增大 `ROTATING_FIRE_X_OFFSET`。
- 总是打右偏：减小 `ROTATING_FIRE_X_OFFSET`。
- 对准后不打：检查 `TARGET_Y_RANGE` 是否匹配 AR tag 的实际 `y` 值。
- 还在晃就开火：增大 `FIRE_SETTLE_TIME` 或减小角速度。

移动靶：

- 总是打慢，弹着点落在靶后：增大 `MOVING_FIRE_X_LEAD`。
- 总是打早：减小 `MOVING_FIRE_X_LEAD`。
- 方向判断抖动：增大 `MOVING_FIRE_MIN_DELTA`。
- 对准过程太慢：略增移动靶角速度系数，但优先保证不开火过冲。

AR 识别相关：

- `src/track_tag/launch/usb_cam_with_calibration.launch` 设置摄像头和标定文件。
- `src/track_tag/launch/ar_track_camera.launch` 设置 `marker_size`、输入图像和 TF。
- 如果 AR 位置抖动大，先检查光照、焦距、标定文件和 `marker_size`，再调控制参数。

### 7. 开环后退与横移

当前主流程中存在硬编码开环移动：

```python
self._move_distance(linear_x=-0.14, duration=7.0, check_obstacle=False)
self._move_distance(linear_x=-0.15, duration=7.0, check_obstacle=False)
self._move_until_obstacle(linear_y=-0.14, timeout=2.1)
SAFE_STOP_DISTANCE = 0.16
```

这是比赛里最容易“省时间但出事故”的部分。规则中触碰围挡会结束比赛，因此调参顺序建议：

1. 空场低速测试，确认 `linear_x` 正负方向和 `linear_y` 正负方向。
2. 用秒表记录 1s、3s、5s 实际移动距离。
3. 调 `duration`，让后退后车体留有至少 `10-15cm` 安全余量。
4. 能用导航点替代时，优先使用 `transX/Y`，少用无避障开环。
5. 如果必须开环，尽量开启 `check_obstacle=True`，或把 `SAFE_STOP_DISTANCE` 调到更保守。

### 8. 语音识别与目标编号

相关文件：

```text
src/robot_slam/scripts/2026_shoot_demo.py
src/robot_slam/scripts/2026_shoot_target.py
src/robot_slam/scripts/shoot_2025_new.py
```

关键逻辑：

- `shoot_2025_new.py` 发布 `audio_topic` 触发录音。
- `2026_shoot_demo.py` 录音并发布 `chinese_topic`。
- `2026_shoot_target.py` 从识别文本中解析旋转靶/移动靶编号，发布 `target_id_rotating` 和 `target_id_moving`。
- `shoot_2025_new.py` 等待两个编号后进入任务流程。

调参点：

- 识别慢：增大 `speechResultTimeout`。
- 识别错数字：修改 `2026_shoot_target.py` 中的关键词映射。
- 现场噪声大：缩短或固定播报模板，录音前播放提示音，麦克风远离电机。
- 比赛前准备手动兜底命令，语音失败时快速发布两个 ID。

### 9. 射击机构

相关文件：

```text
src/robot_slam/scripts/shoot_2025_new.py
src/abot_base/abot_bringup/scripts/shoot.py
src/shoot_cmd/src/shoot_control.cpp
```

当前主流程使用串口：

```python
SERIAL_PORT = "/dev/shoot"
BAUD_RATE = 9600
```

开火指令为先启动再停止：

```text
55 01 12 00 00 00 01 69
55 01 11 00 00 00 01 68
```

调试建议：

- 先空载测试串口和电机响应。
- 确认每次只开火一次，避免重复进入同一阶段。
- 如果串口占用，检查是否同时启动了 `shoot.py` 和 `shoot_2025_new.py`。
- 如果命中离散大，优先检查机械固定、弹丸一致性和枪口高度，再改软件补偿。

## 开发建议

- 源码修改放在 `src/`。
- 比赛参数优先放在 launch 或脚本顶部常量，避免散落在流程中。
- 每轮调参后保留一份可回退配置。
- 正式前至少连续完成 5 次全流程模拟，再考虑提速。
