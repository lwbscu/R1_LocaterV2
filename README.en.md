# R1_LocaterV2

R1_LocaterV2 is an STM32G4-based localization board project for the R1 robot. It integrates H30 MINI yaw, two orthogonal encoder wheels, Lidar pose, dual DT35 distance sensors, and a Windows debugging station into one localization loop for real-time field localization, sensor diagnosis, data collection, and real2sim iteration.

<p align="center">
  <a href="https://lwbscu.github.io/R1_LocaterV2/">
    <img src="docs/promotion/r1-locaterv2-poster.png" alt="R1_LocaterV2 demo poster" width="1100">
  </a>
</p>

<p align="center">
  <a href="https://lwbscu.github.io/R1_LocaterV2/"><b>Open Project Promotion Page / Live Demo Page</b></a>
</p>

<p align="center">
  <a href="https://github.com/lwbscu/R1_LocaterV2"><img alt="GitHub Repo" src="https://img.shields.io/badge/GitHub-Repo-181717?style=flat&logo=github&logoColor=white"></a>
  <a href="https://lwbscu.github.io/R1_LocaterV2/"><img alt="Live Page" src="https://img.shields.io/badge/Live-Page-00a6d6?style=flat"></a>
  <a href="https://juejin.cn/spost/7654479136493420554"><img alt="Juejin Article" src="https://img.shields.io/badge/Juejin-Article-1e80ff?style=flat"></a>
  <a href="https://lwbscu.github.io/R1_LocaterV2/promo-video.html"><img alt="Demo Video" src="https://img.shields.io/badge/Demo-Video-ff4d4f?style=flat"></a>
  <img alt="STM32G4" src="https://img.shields.io/badge/MCU-STM32G4-2ea44f?style=flat">
  <img alt="Fusion" src="https://img.shields.io/badge/Multi--sensor-Fusion-ffc240?style=flat">
  <a href="README.md"><img alt="Chinese README" src="https://img.shields.io/badge/中文-README-64748b?style=flat"></a>
  <img alt="English README" src="https://img.shields.io/badge/English-README-f59e0b?style=flat">
</p>

## What's New

- [2026/06] Rebuilt the V2 localization board around STM32G4 firmware, a PySide6 desktop tool, log collection, and replay in one project.
- [2026/06] Integrated H30 yaw, dual orthogonal encoder wheels, Lidar pose, dual DT35 ranging, and a field wall model with a local startup origin for chassis output.
- [2026/06] Initial real-car full-run yaw accumulated error is about `0.04 deg`; the project now includes a real2sim / RLHF data loop for offline evaluation.

## Coordinate Convention

During race debugging, the main output uses a local coordinate system anchored at the red starting zone:

- When the robot is powered on at the geometric center of the red upper-left starting zone, `x=0, y=0, yaw=0`.
- Map east is `+X`; map north is `+Y`.
- Moving right increases `x`; moving downward decreases `y`.
- Lidar reports pose relative to its startup pose, instead of forcing the field center to `(0,0)`.
- The desktop tool can also show absolute map coordinates to verify textures, walls, DT35 ray hits, and replay alignment.

## Hardware Links

| Module | Peripheral | Role |
| --- | --- | --- |
| Desktop debugger | USART1 `115200` | Lightweight localization CSV for wireless serial and real-time map |
| Chassis controller | USART2 `1152000` | `PG + 11 float + checksum` binary frame |
| Lidar | USART3 `115200` | Receives Lidar localization data |
| H30 MINI | UART4 `460800` | Receives yaw / attitude data |
| DT35-1 / DT35-2 | UART5 `115200` | Polls two DT35 sensors; ID1 left-to-left, ID2 right-to-right |
| Orthogonal encoder 1/2 | TIM2 / TIM3 Encoder | Measures local displacement increments |

## Desktop Tool

The desktop tool lives in [`locater_map/`](locater_map/). It uses PySide6 and pyserial, with no ROS, Foxglove, RViz, Web, or Electron dependency.

Main capabilities:

- Real-time serial reception and raw serial assistant.
- Field map, robot texture, trajectories, DT35 rays, and wall-model visualization.
- Collection of `raw_serial.log`, structured CSV, and synchronized map PNG frames.
- Log replay, screenshots, real-car data analysis, and offline real2sim simulation.
- DT35 raycast is evaluated against the ideal field map. Missing real-world walls, uneven ground, human obstruction, and robot obstruction are treated as low-confidence or abnormal observations.

Install and run:

```powershell
cd R1_LocaterV2\locater_map
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt

python main.py --demo
python main.py --serial-port COM9 --baudrate 115200
```

Data collection:

```powershell
python main.py --serial-port COM9 --baudrate 115200
```

Click "Start Data Collection" in the GUI, move the robot, then click again to save. Data is stored under:

```text
locater_map/logs/RL_data/YYYYMMDD_HHMMSS_log/
```

Each session contains sensor CSV files, raw serial logs, event logs, map PNG sequences, and metadata.

## real2sim And DT35 Modeling

For DT35, the key question is not only the distance value, but which wall the ray actually hits. R1_LocaterV2 builds an ideal field wall model in the desktop tool:

- Red walls: usable localization constraints.
- Blue dashed regions: strong interference areas such as long-pole racks and gaps; avoided during correction.
- Green regions: ramps and Merlin/forest obstacles; they block rays but have lower correction weight than regular walls.

Algorithm flow:

1. Lidar provides the absolute anchor in the startup-local coordinate system.
2. H30 yaw provides high-frequency attitude reference.
3. Encoder wheels provide high-frequency displacement interpolation between Lidar frames.
4. DT35 rays are checked against the wall model using yaw-aware raycast to estimate hit target and confidence.
5. The desktop tool compares real logs with simulated paths to locate errors from wall models, DT35 mounting offsets, encoder scale, and missing field walls.

In the current offline benchmark, the fusion model reduces representative-path average XY RMS from about `5.64 cm` to about `1.97 cm`; a mixed cruise path is reduced from about `11.74 cm` to about `7.36 cm`. These are offline benchmark numbers and still require real-car verification on the final competition field.

## Directory Layout

```text
Core/Application/        STM32 application layer: localization, telemetry, DT35/H30/Lidar/encoder scheduling
Core/Src/                STM32Cube generated code and peripheral initialization
locater_map/             Windows desktop tool, map model, log collection, and replay
locater_map/assets/      Field map, chassis texture, real-car videos, and assets
docs/promotion/          README poster, demo videos, GitHub Pages pages, and generation scripts
```

## Acknowledgements

- Technical lead: [@lwbscu](https://github.com/lwbscu)
- Electrical control support: [@Thomaswang2005](https://github.com/Thomaswang2005), [@HIRAMHC111](https://github.com/HIRAMHC111)
- Lidar support: [@Getting05](https://github.com/Getting05), [@qyw23AI](https://github.com/qyw23AI)
- Hardware chip design: [@twenty-fourabc](https://github.com/twenty-fourabc), [@2718487561-a11y](https://github.com/2718487561-a11y), [@wancyu](https://github.com/wancyu)
- Mechanical structure design: Mark (GitHub username pending)
