# R1 Locater Map

Repository: https://github.com/lwbscu/R1_LocaterV2.git

Windows 2D realtime map and serial assistant for the R1 STM32G4 locater board. It receives STM32 USART1 data through the wireless serial link. It does not use ROS, network transport, Foxglove, RViz, Web, or Electron.

## Update Notes

- R1 locater board has been iterated to R1_LocaterV2 on STM32G4.
- Initial full-course yaw error was about 0.04 deg.
- Current firmware integrates H30 yaw, XY encoder odometry, and lidar pose.
- Thanks to Li Yanyan, Wang Chenyu, Li Yuelin, and Mark for technical support.
- Feishu doc: https://zcnkmpjy7ukv.feishu.cn/wiki/RCPVw2yKLitHK3kWRelcJk8pnq0?from=from_copylink

## Install

```powershell
cd D:\STM32CubeMx\Project_File\STM32_Project\locater_lwb\R1_LocaterV2\locater_map
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

## Run

```powershell
python main.py
python main.py --demo
python main.py --serial-port COM9 --baudrate 115200
python main.py --replay logs\YYYYMMDD_HHMMSS\parsed_frames.csv
```

Smoke-test commands:

```powershell
python main.py --demo --duration-s 3 --screenshot logs\demo_final.png
python main.py --serial-port COM9 --baudrate 115200 --duration-s 5 --screenshot logs\com9_after_flash.png
```

## USART1 Protocol

USART1 keeps VOFA FireWater-compatible numeric CSV. The current preferred protocol is compact `r1_csv_v3`:

```text
pos_x,pos_y,pos_yaw,lidar_x,lidar_y,lidar_yaw,encoder_x,encoder_y,h30_yaw,dt35_1_mm,dt35_2_mm,status_mask
```

Actual serial data is one line without the visual line breaks above.

Field meaning:

- `pos_*`: final lidar-first localization pose, in cm / deg.
- `lidar_*`: lidar absolute pose after firmware TF, in cm / deg.
- `encoder_x/y`: pure XY encoder odometry, in cm.
- `h30_yaw`: H30 yaw, in deg.
- `dt35_1_mm`: DT35-1 / ID 0x01 distance, facing robot local -X left, in mm.
- `dt35_2_mm`: DT35-2 / ID 0x02 distance, facing robot local +X right, in mm.
- `status_mask`: firmware status bits used by the UI lamps. Bits 10/11 mean orthogonal encoder 1/2 have received AB phase pulses; bit 0 is set only when both encoders have received pulses.
- CSV has no CRC, so `crc_state=no_crc` in logs is normal.

The app also keeps compatibility with old `r1_csv_v2` 25/41-column logs, legacy 5/6/9-column CSV, and the reserved `$R1M` ASCII frame with CRC16-CCITT-FALSE.

## UI

- Left panel: COM port, baudrate, protocol mode, reconnect, Chinese/English language toggle, red/blue start pose option, layer toggles, replay controls.
- Center: prior field map, fixed 83 cm x 83 cm robot texture, pos/calib/lidar trajectories, optional DT35 rays.
- Right panel: final pos, lidar pose, encoder XY, H30 yaw, DT35 distance state, serial FPS, RX bytes/s, and Chinese(English) sensor status lamps. Orthogonal encoder 1/2 are shown separately.
- Bottom panel: raw serial view and command sender. Realtime curves are intentionally not included; use VOFA for waveform viewing.

## Coordinate and Assets

- Field background: `assets/field_prior_map_clean_labeled_1215x1210cm.png`.
- Field size: 1215 cm x 1210 cm.
- World origin: field outer center, +X right, +Y up.
- Robot texture: `assets/r1_chassis_830mm_texture_1024.png`.
- Robot size: fixed 83 cm x 83 cm.
- Texture convention: image right is robot forward +Y, image down is robot right +X. With the default config, yaw=0 points to map +Y. DT35-1 faces robot local -X left; DT35-2 faces robot local +X right.

All runtime tuning is in `config/default_config.json`.

## Logs

Each run creates:

```text
logs/YYYYMMDD_HHMMSS/raw_serial.log
logs/YYYYMMDD_HHMMSS/parsed_frames.csv
logs/YYYYMMDD_HHMMSS/events.log
```

`logs/`, `.pytest_cache/`, and `__pycache__/` are local runtime outputs and should not be committed.

## Tests

```powershell
python -m json.tool config\default_config.json
python -m pytest -q
python -m compileall -q .
```

## Common Issues

- No COM port: check the wireless serial driver and click Refresh.
- No data: check USART1 baudrate 115200, TX/RX crossing, and common GND.
- VOFA works but map does not move: select `auto` or `r1_csv_v3`; current firmware outputs 12 numeric fields.
- Robot direction is off by 90/180 deg: tune `robot.texture_front_dir_deg_in_image`, `robot.yaw_offset_deg`, or `transform.data_yaw_offset_deg`.
- Coordinates are mirrored: tune `transform.data_x_sign` and `transform.data_y_sign`.
