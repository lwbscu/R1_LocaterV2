# R1_LocaterV2：把 STM32G4 定位板做成 Lidar / H30 / 编码轮 / DT35 的 real2sim 调试闭环

> 建议分类：开发工具 / 开源项目 / 嵌入式<br>
> 建议标签：机器人、STM32、PySide6、传感器融合、real2sim<br>
> GitHub：https://github.com/lwbscu/R1_LocaterV2<br>
> 演示视频：https://raw.githack.com/lwbscu/R1_LocaterV2/main/docs/promo-video.html

![R1_LocaterV2 海报](https://cdn.jsdelivr.net/gh/lwbscu/R1_LocaterV2@main/docs/promotion/r1-locaterv2-poster.png)

R1_LocaterV2 是一块基于 STM32G4 的 R1 机器人定位板。它把 H30 MINI 惯导、双正交编码轮、Lidar 位姿、双 DT35 测距和 PySide6 上位机放到同一套调试闭环里。

这次重构的重点不是“读到传感器数据”，而是让每个传感器都能被验证、被记录、被回放，并且能用实车数据反向修正仿真模型。

## 01 定位语义先统一

<small>Coordinate frame first</small>

调试阶段最容易出错的是坐标原点。这个工程现在采用“红方启动区局部零点”作为主输出语义：

- 机器人在红方左上启动区几何中心上电时，`x=0, y=0, yaw=0`。
- 地图正东为 `+X`，地图正北为 `+Y`。
- 车向右移动时 `x` 增大，车向下移动时 `y` 减小。
- Lidar 的 `0,0,0` 是启动姿态下的局部位姿，不再把场地中心强行当成零点。

这个约定会同时影响上位机显示、底盘主控协议、日志回放和 DT35 反推位置。如果这里没有统一，后面融合算法只会越调越乱。

![启动局部零点与地图上位机](https://cdn.jsdelivr.net/gh/lwbscu/R1_LocaterV2@main/docs/promotion/r1-locaterv2-start-map.png)

图中机器人贴图固定按 83 cm x 83 cm 显示，中心点对齐定位输出的 `x/y`。

## 02 传感器不是孤立调试

<small>Sensor evidence in one loop</small>

固件侧使用 5 路串口/外设：

| 接口 | 波特率 | 用途 |
| --- | ---: | --- |
| USART1 | 115200 | 输出轻量 CSV 给上位机和日志 |
| USART2 | 1152000 | 发送 `PG + 11 float + checksum` 给底盘主控 |
| USART3 | 115200 | 接收 Lidar 数据 |
| UART4 | 460800 | 接收 H30 MINI yaw / 姿态 |
| UART5 | 115200 | 轮询两个 DT35 |

USART1 默认输出 12 列 `r1_csv_v3`：

```text
pos_x,pos_y,pos_yaw,lidar_x,lidar_y,lidar_yaw,encoder_x,encoder_y,h30_yaw,dt35_1_mm,dt35_2_mm,status_mask
```

字段少是刻意的。默认链路只发定位和关键传感器观测，调试诊断信息留在固件和上位机内部，不把无线串口和 UI 塞满。

## 03 上位机是高级串口助手，也是实验记录器

<small>Desktop telemetry and replay</small>

上位机使用 PySide6 + pyserial，不依赖 ROS、Foxglove、RViz、Web 或 Electron。它同时做三件事：

1. 串口助手：打开串口、显示 raw 行、发送命令、统计 FPS 和 RX bytes/s。
2. 实时地图：显示机器人、轨迹、Lidar、DT35 射线、场地墙体模型和传感器状态。
3. 数据采集：每 0.1 秒记录传感器帧，每 1 秒保存地图截图，后续用于离线分析。

![动态演示：上位机地图、传感器状态和 DT35 raycast](https://cdn.jsdelivr.net/gh/lwbscu/R1_LocaterV2@main/docs/promotion/r1-locaterv2-demo-teaser.gif)

上面这个动态预览对应完整视频页：<br>
https://raw.githack.com/lwbscu/R1_LocaterV2/main/docs/promo-video.html

## 04 DT35 的难点是“命中了哪面墙”

<small>Raycast model for distance sensors</small>

两个 DT35 都安装在机器人左右侧。它们测到的距离本身比较准，但机器人一旋转，激光方向就会跟着旋转；场地里还有梅林、坡道、长杆架、人和协作机器人遮挡。单独看一个距离值没有意义。

所以我们在上位机里建立了理想场地墙体模型：

- 红色墙体：规则墙体，可参与定位约束。
- 蓝色虚线区域：长杆架等强干扰区域，不参与修正。
- 绿色区域：坡道和特殊障碍，会挡光，但权重低于规则墙体。
- 实心障碍：梅林九宫格等会截断 DT35 光线，不能让射线穿透。

![DT35 墙体模型](https://cdn.jsdelivr.net/gh/lwbscu/R1_LocaterV2@main/docs/promotion/r1-locaterv2-field-model-overview.png)

DT35 处理流程是：用 H30 yaw 把传感器局部射线转到地图坐标系，再和墙体模型求交。实测距离如果和命中墙体残差太大，就降低置信度，而不是强行修正机器人位置。

## 05 real2sim 的价值在回放

<small>Replay, compare, then tune</small>

上位机采集一次数据后，会保存：

```text
locater_map/logs/RL_data/YYYYMMDD_HHMMSS_log/
  sensor_data/
    raw_serial.log
    raw_frames.csv
    display_frames.csv
    events.log
  png/
    frame_*.png
  metadata.json
```

这样每一帧传感器数据都能和地图截图对齐。后续可以直接离线回放，不需要每次改一点参数就重新推车。

当前离线基准中，代表路径平均 XY RMS 从约 `5.64 cm` 降到约 `1.97 cm`；混合场地巡航路径从约 `11.74 cm` 降到约 `7.36 cm`。这个结果不是最终比赛场地证明，但说明 Lidar、H30、编码轮和 DT35 已经能放到同一个可验证框架里。

## 如何运行上位机

```powershell
cd D:\STM32CubeMx\Project_File\STM32_Project\locater_lwb\R1_LocaterV2\locater_map
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt

python main.py --demo
python main.py --serial-port COM9 --baudrate 115200
```

打开真实串口后，点击“开始采集数据”，移动机器人，再次点击保存，就会生成一组可回放的数据。

## 当前状态

- 码盘定位板已迭代到 V2，主控芯片为 STM32G4。
- 初步实车全程 yaw 累计误差约 `0.04 deg`。
- 已集成 H30 yaw、双正交编码轮、Lidar 数据、双 DT35 测距和 PySide6 实时地图上位机。
- 当前重点是：以 Lidar 启动坐标为局部零点，用 H30 yaw 和编码轮做高频插值，用 DT35 与场地墙体模型做位置约束和异常筛选。

## 链接

- GitHub：https://github.com/lwbscu/R1_LocaterV2
- 宣传页：https://raw.githack.com/lwbscu/R1_LocaterV2/main/docs/index.html
- 演示视频：https://raw.githack.com/lwbscu/R1_LocaterV2/main/docs/promo-video.html

感谢李彦彦、王晨宇、李岳林、马克在硬件调试、传感器接入、定位算法和工程验证中的技术支持。
