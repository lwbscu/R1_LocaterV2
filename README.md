# R1_LocaterV2

R1_LocaterV2 是基于 STM32G4 的新一代码盘定位板工程，仓库地址：

https://github.com/lwbscu/R1_LocaterV2.git

## 更新文档

### 2026-06-12

- 码盘定位板已完成新一轮迭代，主控平台基于 STM32G4。
- 初步实车跑完全程后，yaw 累计误差约为 0.04 度。
- 已集成 H30 MINI yaw 数据、X/Y 编码计数、lidar 传感器数据。
- 当前定位输出以 lidar 为绝对基准，码盘与 H30 用于两帧 lidar 之间的高频插值。
- VOFA 调试输出保留纯码盘/H30 的 calib_x、calib_y、calib_yaw，便于独立观察本地里程计表现。

## 当前功能

- USART1：VOFA 实时调试输出。
- USART2：向主控发送定位数据。
- USART3：接收 lidar 数据。
- UART4：接收 H30 MINI IMU 数据。
- 支持本地 R 指令清零码盘/H30 插值状态，不清零 lidar 坐标。

## 相关文档

飞书文档：

https://zcnkmpjy7ukv.feishu.cn/wiki/RCPVw2yKLitHK3kWRelcJk8pnq0?from=from_copylink

## 致谢

感谢李彦彦、王晨宇、李岳林、马克在硬件调试、传感器接入、定位算法与工程验证中的技术支持。
