# R1 Locater Map

Repository: https://github.com/lwbscu/R1_LocaterV2.git

Cross-platform 2D realtime map and serial assistant for the R1 STM32G4 locater board. It receives STM32 USART1 data through the wireless serial link. It does not use ROS, network transport, Foxglove, RViz, Web, or Electron.

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
python -m pip install -r requirements.txt
```

Ubuntu 22.04 / Linux local desktop:

```bash
cd R1_LocaterV2/locater_map
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run

```powershell
python main.py
python main.py --demo
python main.py --record --baudrate 115200 --duration-s 20
python main.py --analyze-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv
python main.py --simulate-fusion logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --lidar-stride 5
python main.py --simulate-fusion logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --fusion-strides 1,2,5,10,25,50
python main.py --validate-model-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --lidar-stride 25 --validate-model-output logs/model_validation.json --validate-model-fused-output logs/model_validation_fused.csv
python main.py --real-validation-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --lidar-stride 25 --real-validation-output-dir logs/real_validation_latest
python main.py --readiness-report --readiness-output-dir logs/readiness_latest
python main.py --readiness-report --readiness-real-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --readiness-output-dir logs/readiness_real_latest
python main.py --path-report-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --lidar-stride 25 --path-report-output logs/real_path_report.csv --path-report-summary logs/real_path_report.json
python main.py --dt35-hit-table
python main.py --dt35-hit-table --dt35-hit-poses="-360,520,90,top_ignore;-550,50,0,forest" --dt35-hit-output logs/dt35_hits.csv
python main.py --dt35-grid --dt35-grid-x=-580,580 --dt35-grid-y=-580,580 --dt35-grid-yaws=-180,-135,-90,-45,0,45,90,135 --dt35-grid-output logs/dt35_grid.csv --dt35-grid-summary logs/dt35_grid_summary.json
python main.py --dt35-role-map --dt35-grid-x=-580,580 --dt35-grid-y=-580,580 --dt35-grid-step 100 --dt35-grid-yaws=-180,-135,-90,-45,0,45,90,135 --dt35-role-output logs/dt35_roles.csv --dt35-role-summary logs/dt35_roles.json --dt35-role-svg logs/dt35_roles.svg
python main.py --dt35-role-map --dt35-role-poses="-360,520,90,top_ignore;-550,50,0,forest;-400,-420,0,ramp"
python main.py --dt35-validation-plan --dt35-grid-step 100 --dt35-validation-plan-output logs/dt35_validation_plan_latest.csv --dt35-validation-plan-summary logs/dt35_validation_plan_latest.json --dt35-validation-plan-md logs/dt35_validation_plan_latest.md
python main.py --dt35-field-sweep --dt35-grid-step 100 --dt35-field-sweep-output logs/dt35_field_sweep_latest.csv --dt35-field-sweep-summary logs/dt35_field_sweep_latest.json
python main.py --dt35-analyze-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --dt35-analyze-output logs/dt35_residuals.csv --dt35-analyze-summary logs/dt35_residuals.json --dt35-advice-output logs/dt35_advice.csv --dt35-advice-summary logs/dt35_advice.json --dt35-advice-md logs/dt35_advice.md
python main.py --field-model-svg logs/field_model_overlay_latest.svg
python main.py --field-model-audit logs/field_model_audit_latest.json
python main.py --synthetic-fusion --synthetic-path top_corridor --lidar-stride 10 --fusion-dt35-gain 1.0
python main.py --synthetic-suite --lidar-stride 25 --synthetic-encoder-x-scale 0.97 --synthetic-encoder-y-scale 1.02 --fusion-dt35-yaw-gain 0.0
python main.py --synthetic-suite --synthetic-suite-paths field_patrol --lidar-stride 25 --synthetic-encoder-x-scale 0.97 --synthetic-encoder-y-scale 1.02 --fusion-dt35-yaw-gain 0.0
python main.py --synthetic-benchmark --synthetic-samples 180 --synthetic-encoder-x-scale 0.97 --synthetic-encoder-y-scale 1.03 --synthetic-dt35-noise-mm 5 --lidar-stride 25 --fusion-dt35-gain 1.0 --synthetic-benchmark-output logs/synthetic_benchmark_latest.csv --synthetic-benchmark-summary logs/synthetic_benchmark_latest.json
python main.py --synthetic-obstacle-ablation --synthetic-samples 180 --synthetic-encoder-x-scale 0.97 --synthetic-encoder-y-scale 1.03 --synthetic-dt35-noise-mm 5 --lidar-stride 25 --fusion-dt35-gain 1.0 --synthetic-obstacle-ablation-output logs/obstacle_ablation_latest.csv --synthetic-obstacle-ablation-summary logs/obstacle_ablation_latest.json
python main.py --synthetic-path-report --synthetic-path field_patrol --synthetic-encoder-x-scale 0.97 --synthetic-encoder-y-scale 1.02 --lidar-stride 25 --synthetic-path-report-output logs/field_patrol_path_diag.csv --synthetic-path-report-summary logs/field_patrol_path_diag_summary.json
python main.py --synthetic-monte-carlo --synthetic-monte-carlo-runs 20 --lidar-stride 25 --synthetic-encoder-x-scale 0.97 --synthetic-encoder-y-scale 1.02 --synthetic-dt35-gains 0,0.4,0.6,0.85,1.0 --fusion-dt35-yaw-gain 0.0
python main.py --replay logs/YYYYMMDD_HHMMSS/parsed_frames.csv
```

The GUI lists the current serial ports and can auto-select the first likely USB serial adapter. Do not rely on a fixed port name after unplugging the adapter. On Linux, find the current device with:

```bash
python -m serial.tools.list_ports -v
ls -l /dev/ttyACM* /dev/ttyUSB*
```

Use the actual device path only when you need to force one adapter:

```bash
python main.py --serial-port <current-port> --baudrate 115200
```

If Linux reports permission denied, add the current user to the serial group and log in again:

```bash
sudo usermod -aG dialout $USER
```

If the PySide6 window fails to start on Ubuntu, install the common Qt platform dependencies:

```bash
sudo apt update
sudo apt install -y libxcb-cursor0 libxcb-xinerama0 libgl1
```

`--record` is the headless real-car calibration capture mode. It auto-selects a likely serial port when `--serial-port` is omitted; pass an explicit port if multiple adapters are present. Move the car during capture; each run creates
`logs/YYYYMMDD_HHMMSS_calib/` with `raw_serial.log`, `parsed_frames.csv`, `calibration_summary.json`, and
`calibration_notes.md`. The summary compares lidar deltas against encoder deltas and H30 yaw deltas for later
map/fusion tuning.

`--simulate-fusion` runs the upper-computer fusion model offline. It treats lidar as the absolute anchor,
uses encoder XY and H30 yaw for high-rate prediction between lidar anchor frames, and optionally applies
DT35 boundary residual correction. The output CSV is replay-compatible and the metrics JSON reports holdout
error against lidar.

The GUI uses the same model for live display when the `Live fusion` checkbox is enabled. Raw serial lines
and `parsed_frames.csv` still store the firmware values; only the displayed final `pos` and map trajectory
are replaced by the upper-computer fused pose. Disable the checkbox to inspect firmware `pos` directly.

## Real-Car Capture

Use the `开始采集数据 / Start capture` button when running the real robot, or run the GUI with
`--capture --duration-s N` for an automatic timed capture:

1. Open the serial port and confirm live frames are updating.
2. Press `开始采集数据`, then move the robot.
3. Press `停止并保存采集` after the run.

Each capture is saved under `logs/RL_data/YYYYMMDD_HHMMSS_log/` and contains:

- `sensor_data/raw_frames.csv`: parsed STM32 frames before display transform and live fusion.
- `sensor_data/display_frames.csv`: the exact frames drawn on the map after transform, start policy, and live fusion.
- `sensor_data/raw_serial.log`: original serial lines from the wireless USART1 stream.
- `sensor_data/events.log`: capture, serial, and save events.
- `png/`: synchronized map screenshots.
- `metadata.json`: capture counts, timing, fusion settings, and notes.

By default, sensor rows are sampled every `display.capture_data_interval_s = 0.1s`, and map screenshots are
saved every `display.capture_map_snapshot_interval_s = 1.0s`. File names include the capture sample index and
elapsed time in milliseconds so sensor CSV rows and PNG frames can be aligned for offline modeling.

Use the screenshots together with `sensor_data/display_frames.csv` to compare where the upper computer thought the robot
was against the physical field. The real practice field may not have all modeled outside walls outside Zone 1,
so DT35 residuals near missing walls should be treated as field-limit evidence before changing H30 or DT35
calibration.

`--validate-model-csv` is the real-car acceptance check. It replays a `parsed_frames.csv`, runs the same
live fusion model, and writes a JSON report comparing firmware `pos` and fused `pos` against lidar. The same
report includes DT35 residual statistics by target type, so a bad wall/forest/ramp model can be separated
from encoder drift or H30 yaw issues.
The report also has a `gates` section. A pass means: lidar reference exists, H30 yaw exists, DT35 has valid
measurements, at least one DT35 ray hits usable modeled geometry, fused XY is not worse than raw firmware
pose, fused XY RMS is within the configured limit, and usable DT35 residual RMS is below
`field_model.residual_warn_cm`. The `dt35_breakdown` section splits residuals by `sensor_1/sensor_2`,
target type, and exact target name, which is the first place to inspect when real-car validation fails.
The `dt35_quality` section promotes the most suspicious target edges into `bad_targets`. It does not treat
ordinary out-of-range walls as bad; a target is suspicious only when real valid DT35 measurements produce
large residuals, frequently hit ambiguous corners, or valid measurements rarely remain usable.

`--real-validation-csv` is the one-command real-car validation gate. It runs field-model audit,
model validation, DT35 residual analysis, DT35 target calibration advice, and path diagnostics into one output directory. The generated
`real_validation_suite.json` deliberately separates math regression from real completion: synthetic logs can
exercise the algorithm, but `real_validation_passed` is true only for non-synthetic logs with lidar, H30,
both DT35 sensors, both orthogonal encoder pulse flags, usable DT35 geometry, acceptable residuals, a fused
pose that is not worse than the raw firmware pose, and a path report proving DT35 actually changed the pose
without making RMS worse than the no-DT35 baseline. Because H30 yaw and DT35 distance are treated as accurate
hardware observations, the same gate also requires `dt35_advice.json` to have zero actionable target shifts;
if it fails, inspect `dt35_advice.md` and adjust the field model target geometry, DT35 mounting offset, or
lidar/world alignment instead of tuning H30 yaw offset or DT35 scale.

`--readiness-report` is the pre-real-car aggregation gate for the current objective. Without a real CSV, it
generates one directory containing `field_model_overlay.svg`, `field_model_audit.json`, `dt35_field_sweep.*`,
`dt35_observability.*`, `dt35_role_matrix.*`, `dt35_validation_plan.*`, `synthetic_benchmark.*`,
`obstacle_ablation.*`, `objective_coverage.*`, and
`readiness_report.*`. It proves the current field
model has usable wall, forest, and ramp constraints; the SVG overlay uses the same red/blue/green geometry
that the algorithm raycasts against; ignored blue interference is modeled but not used for
correction; H30 yaw remains the yaw authority; the observability matrix explicitly documents rank0/rank1/rank2
DT35 translation constraints; selected lidar/yaw poses explain what DT35-1 and DT35-2 are measuring;
forest/ramp solid-obstacle correction contributes in an ablation comparison; and synthetic
lidar/H30/encoder/DT35 fusion improves over raw encoder/H30 prediction under the default assumption that
H30 yaw and DT35 distance are accurate. This is not
final completion evidence because it has no real DT35 log. Add
`--readiness-real-csv logs/.../parsed_frames.csv` after capturing the real robot; the same report then embeds
`real_validation/` and only sets `completion_verified=true` when the real log passes.
`objective_coverage.md` maps the original modeling objective to evidence files and marks each item as
`passed_offline`, `passed_offline_needs_real_log`, `failed_real_log`, or `passed_real`, so remaining work is
explicit instead of inferred from raw JSON.

`--path-report-csv` is the real-log per-frame version of the synthetic path report. It reads a normal
`parsed_frames.csv`, writes a row for every frame, and also writes a replay-compatible fused CSV. Use it when
the summary gate fails or when the map looks wrong only in one route segment; sort the CSV by
`fused_xy_error_cm`, `no_dt35_xy_error_cm`, `dt35_improvement_cm`, `dt35_correction_mag_cm`,
`dt35_translation_rank`, `dt35_constraint_state`, or DT35 residual columns to locate the bad pose/yaw/target.
The summary JSON compares raw firmware pose, no-DT35 fusion, and DT35-enabled fusion. For real validation,
`dt35_active_frames` should be greater than zero and `fused_rms_xy_cm` should be no worse than
`no_dt35_rms_xy_cm`; otherwise DT35 is either being ignored by gating or hurting the model.

`--synthetic-fusion` generates known-truth field/world poses in the same center-origin coordinate frame as
lidar, turns them into synthetic lidar/H30/encoder/DT35 frames using the current field model, then runs the
same fusion code. The encoder synthetic path mirrors the firmware P/Q orthogonal-wheel algorithm in
`task_locater.c`: it generates integer TIM2/TIM3 delta counts, applies the 225 deg installation angle,
H30-yaw rotation compensation, and X/Y correction constants before accumulating encoder XY. This keeps
offline fusion tests aligned with the current STM32 implementation instead of using an ideal XY shortcut.
Use it to test yaw and DT35 behavior without moving the real car. Available paths include
`top_corridor`, `static_start`, `forest_side`, `ramp_side`, `center_divider`, and `yaw_sweep`.

Fusion assumption for the current hardware is: lidar gives absolute pose anchors, H30 yaw is the trusted
high-rate heading source, encoder XY predicts motion between anchors, and DT35 gives trusted side-distance
constraints. Therefore DT35 primarily corrects translation. The optional `--fusion-dt35-yaw-gain` defaults
to `0.0`; raise it only for experiments where DT35 geometry is intentionally allowed to correct yaw. In
normal tuning, keep H30 as the yaw authority and use DT35 to verify/correct lateral position. When yaw gain
is zero, the solver fixes yaw and solves only `x/y`, so a DT35 residual cannot be hidden inside an internal
yaw variable. When `h30_valid` is true, lidar yaw is kept as a diagnostic value and does not override the
fused yaw used for robot drawing or DT35 raycasting. The default DT35 translation gain is `1.0`, because current tuning treats H30 yaw and DT35
distance as accurate. This gain is still gated by field geometry, incidence angle, valid range, and ignored
target areas; lower the per-target field-model `correction_weight` only when a physical obstacle is known to
have less reliable geometry than a flat wall.
H30 and DT35 are treated as accurate sensors for normal tuning. Do not compensate a wrong field-model edge by
changing H30 yaw offset or DT35 scale/offset; first check whether the ray hit an ignored rack/gap area, a
corner, a ramp/forest edge, or an unmodeled object. Lidar remains the absolute world-coordinate reference for
recorded validation, while H30 yaw is the heading reference used for DT35 raycasting.
The fusion filter also learns a small display-side encoder X/Y scale from consecutive lidar anchors. This is
not a firmware encoder calibration and does not change the raw log. It only reduces interpolation drift
between lidar anchors when the encoder path is consistently a few percent short or long. DT35 then corrects
the remaining component that is observable from the current ray geometry.
The GUI default now allows DT35 to correct frames that also have a valid lidar anchor, because current tuning
treats H30 yaw and DT35 distance as high-confidence observations. The correction is still gated by ray target,
incidence, range, and residual; lidar remains the initial absolute pose source, while DT35 can clamp the
component that is geometrically observable from the current side ray. Offline CLI fusion commands use the same
`display.live_fusion_dt35_correct_lidar_frames` default from `config/default_config.json`; pass
`--fusion-dt35-correct-lidar-frames` only to force this behavior on when a custom config disables it.
For strict lidar-only anchor comparison, run with a config where `live_fusion_dt35_correct_lidar_frames` is false.
`--synthetic-suite` runs the same check across several representative field areas and writes
`logs/*_synthetic_suite/synthetic_suite.json` for comparison.
`field_patrol` is a mixed in-field route through the red start/top corridor, ignored long-pole area, forest
side, ramp side, lower wall, center divider, and rotated poses; use it as the default stress path before
real-car validation. Synthetic paths are kept inside the 1215 cm x 1210 cm field boundary; they do not apply
the UI red/blue start-pose display offset unless explicitly requested by code.
`--synthetic-path-report` writes a per-frame diagnostic CSV. Each row contains lidar truth, raw encoder/H30
prediction, fused pose, raw/fused errors, and both DT35 rays' target name, target type, measured distance,
expected distance, residual, residual gate, correction state, corner ambiguity flag, ray direction, and the dominant field
axis constrained by that ray. `dt35_*_allowed` means the ray hits trusted geometry; `dt35_*_fusion_allowed`
means it also passed the residual gate and can be used by the fusion solver. Use this when a route-level RMS number is not enough to identify which pose or
yaw range is causing an error. For example, with the current yaw convention a side-facing DT35 ray at
`ray_yaw_deg=-90` constrains field `x`; after the robot rotates, the same physical sensor may constrain
field `y` or diagonal `xy`. The CSV columns `dt35_*_ray_dx`, `dt35_*_ray_dy`,
`dt35_*_constraint_axis`, `dt35_*_correction_*_per_cm`, `dt35_translation_rank`, and
`dt35_constraint_state` make that explicit on every frame.
The same CSV also contains `no_dt35_*` and `dt35_correction_*` columns. These are generated by running the
same frames twice, once with DT35 disabled and once with DT35 enabled, so they show DT35's actual contribution
instead of only showing whether a ray was geometrically usable.
The summary JSON also groups error statistics by `dt35_translation_rank`, `dt35_constraint_state`, and
`dt35_principal_axis_label`. Use `dt35_constraint_state_error_stats` first: `rank0_no_dt35` means no usable
DT35 translation constraint, while `rank1_x`, `rank1_y`, and `rank1_xy` show which world direction the
trusted DT35 residual can clamp at that pose. With the current side-facing sensor layout, these groups are
more important than a single overall RMS number.
`--synthetic-monte-carlo` repeats randomized `random_patrol` paths across multiple seeds and prints average
and worst RMS by DT35 gain. This is the main offline stress test for the "lidar anchor + H30 yaw + encoder
prediction + DT35 side-distance correction" model. By default the synthetic runs treat DT35 as accurate.
Add `--synthetic-dt35-noise-mm 5` only when intentionally stress-testing noisy distance data.
`--synthetic-benchmark` is the quick regression gate for the current map model. It runs the representative
top corridor, forest, ramp, center divider, start-corner yaw sweep, and mixed field-patrol paths, then writes
one CSV/JSON report. Moving paths must improve against raw encoder/H30 prediction; static or pure-rotation
paths must stay within the configured stable error limit. Use this command after changing walls, ignored
zones, ramp/forest geometry, DT35 mounting, or fusion gains.
With the current checked-in model and the default accurate-DT35 assumption, the benchmark reduces average
holdout XY RMS from about 5.64 cm raw encoder/H30 prediction to about 1.97 cm fused pose. The mixed
`field_patrol` path drops from about 11.74 cm to about 7.36 cm; its remaining error is mainly from sections
where side-facing DT35 rays provide only a one-axis translation constraint before the next lidar anchor.
Add `--synthetic-dt35-noise-mm 5` when intentionally running a DT35 distance-noise stress test.
`--synthetic-obstacle-ablation` is the specific regression check for "forest/ramp are used by DT35
correction". It generates the same synthetic sensor frames with the full field model, then replays them twice:
once with normal `solid_obstacle` correction and once with forest/ramp correction weights forced to zero.
The output reports how many forest/ramp DT35 rays were fusion-usable and how much fused RMS changes when
those obstacle corrections are disabled.

## DT35 Field Model

DT35 raycasting uses field-center coordinates in cm. The current model classifies hits:

- `usable_wall`: rigid wall/partition. A valid DT35 residual can correct the pose.
- `solid_obstacle`: forest blocks and ramps. It blocks the laser and can correct the pose with lower weight
  than a wall, because the hit surface can be less regular.
- `blocker`: obstacle that blocks the laser but is not trusted for correction.
- `ignore`: noisy geometry, such as long-pole rack areas with gaps. If the closest hit is ignored, that
  DT35 frame is skipped for correction. Top long-pole racks are modeled as finite-thickness blue rectangles,
  not only center lines, so horizontal DT35 rays cannot look through them and accidentally use the back wall.

Robot local frame is `+Y` forward and `+X` right. DT35-1 is mounted at `(+40.4 cm, -3.3 cm)` and rays to
local `-X`; DT35-2 is mounted at `(-40.4 cm, -3.3 cm)` and rays to local `+X`. Yaw rotates these rays into
the world frame before raycasting, so rotating the car changes the expected wall intersection correctly.
Each hit also has an incidence angle. A perpendicular hit has `incidence_deg=0`; a grazing hit near parallel
to the wall is weak. `field_model.max_correction_incidence_deg` defaults to `75.0`, and
`field_model.incidence_weight_power` defaults to `1.0`, so oblique hits are down-weighted and extreme grazing
hits are filtered instead of being treated like strong wall measurements.
Corner hits are also filtered. If the same DT35 ray intersects two non-parallel modeled edges within
`field_model.corner_ambiguity_cm`, the hit is marked `corner_ambiguous` and skipped for correction. This
prevents forest/ramp/wall corners from being treated as a stable flat reference surface.

Ramp modeling separates top-view blocking from the lower-left side-view detail in `docs/地图尺寸原图.png`.
The top-view grey ramp footprint is modeled as about `150 cm x 150 cm`; the side-view `270 cm` dimension
describes the ramp/platform profile and is not used as a larger DT35 blocking rectangle. It is a
`solid_obstacle` with low correction weight because the sloped face can bias the measured distance compared
with a vertical wall.

`--dt35-hit-table` prints the expected target for both DT35 rays at selected field poses. Use this before
changing field geometry:

```powershell
python main.py --dt35-hit-table
python main.py --dt35-hit-table --dt35-hit-poses="-360,520,90,top_ignore;-550,50,0,forest;-400,-420,0,ramp"
python main.py --dt35-grid --dt35-grid-x=-580,580 --dt35-grid-y=-580,580 --dt35-grid-yaws=-180,-135,-90,-45,0,45,90,135 --dt35-grid-output logs/dt35_grid.csv --dt35-grid-summary logs/dt35_grid_summary.json
python main.py --dt35-role-map --dt35-grid-x=-580,580 --dt35-grid-y=-580,580 --dt35-grid-yaws=-180,-135,-90,-45,0,45,90,135 --dt35-role-output logs/dt35_roles.csv --dt35-role-summary logs/dt35_roles.json --dt35-role-svg logs/dt35_roles.svg
python main.py --dt35-validation-plan --dt35-grid-step 100 --dt35-validation-plan-output logs/dt35_validation_plan_latest.csv --dt35-validation-plan-summary logs/dt35_validation_plan_latest.json --dt35-validation-plan-md logs/dt35_validation_plan_latest.md
python main.py --dt35-field-sweep --dt35-grid-step 100 --dt35-field-sweep-output logs/dt35_field_sweep_latest.csv --dt35-field-sweep-summary logs/dt35_field_sweep_latest.json
python main.py --dt35-observability --dt35-grid-x=-580,580 --dt35-grid-y=-580,580 --dt35-grid-yaws=-180,-135,-90,-45,0,45,90,135 --dt35-observability-output logs/dt35_observability.csv --dt35-observability-summary logs/dt35_observability_summary.json
python main.py --dt35-yaw-matrix --dt35-yaw-matrix-output logs/dt35_yaw_matrix.csv --dt35-yaw-matrix-summary logs/dt35_yaw_matrix_summary.json
python main.py --field-model-svg logs/field_model_overlay_latest.svg
python main.py --field-model-audit logs/field_model_audit_latest.json
```

`--field-model-svg` exports the exact DT35 geometry used by the algorithm as an SVG overlay on top of
`field_prior_map_clean_labeled_1215x1210cm.png`. Red means usable correction wall, blue dashed geometry is
ignored interference, and green means solid obstacles such as forest and ramp zones. The default export also
draws representative robot poses and both DT35 rays, so it can be opened in a browser or VS Code and compared
directly against `docs/地图尺寸.png`, `docs/地图尺寸原图.png`, and the annotated red/blue/green map.

`--field-model-audit` writes the same model in machine-readable JSON: field size and 2 px/cm scale, DT35
mounting offsets, segment/rectangle target classes, default pose hit coverage, observability rank, fusion
assumptions, and the real-car evidence still missing before the model can be considered fully verified. Its
`model_self_check` gate only validates configuration and synthetic geometry: map size, 2 px/cm scale,
required wall/ignore/forest/ramp targets, DT35 mounting directions, and default-pose DT35 observability.
The audit also reports `default_pose_behavior`, including usable forest/ramp targets actually hit by DT35
rays. This prevents a false pass where forest or ramp rectangles exist in config but are never usable by the
ray model.
The audit now embeds `field_sweep.summary` as well, so the same JSON proves the full-grid model covers
usable X/Y wall constraints, forest constraints, ramp constraints, and ignored interference while keeping
ignored targets out of correction.
It intentionally keeps `completion_verified=false` until real DT35 logs pass residual validation.

The table tells whether each ray hits `usable_wall`, `solid_obstacle`, `ignore`, or nothing. Only
`usable_wall` and `solid_obstacle` are allowed to correct localization; ignored long-pole/gap areas are
kept in the model so the algorithm knows not to trust them. The grid mode also reports `within_range`, so
a geometrically valid wall farther than the DT35 range is not treated as usable in the current frame.
It also reports `grazing_filtered`, which counts hits on otherwise valid walls/obstacles that were rejected
because the ray was too close to parallel with the surface.
The grid CSV includes `ray_dx/ray_dy`, `constraint_axis`, and `correction_dx_per_cm/correction_dy_per_cm`.
The summary JSON includes `risk_counts`, `sensor_risk_counts`, `constraint_axis_counts`,
`sensor_axis_counts`, and `yaw_axis_counts`. These fields are the quickest way to check whether a planned
pose/yaw range is constrained by world `x`, world `y`, diagonal `xy`, ignored blue geometry, corner ambiguity,
or simple out-of-range distance before moving the real robot.
`--dt35-observability` groups the two DT35 rays by pose and reports translation rank. With the current
left/right side-facing layout, valid DT35 usually gives `rank1_x`, `rank1_y`, or `rank1_xy`: it can correct
position along one world direction but cannot independently recover full `x/y`. Full 2D localization still
comes from lidar anchors plus encoder/H30 prediction; DT35 is the trusted side-distance residual that clamps
the drift component visible from the current yaw.
`--dt35-role-map` is the human-readable version of the same check. For each lidar/world pose and H30 yaw it
reports DT35-1/2 mounting side, local ray direction, world ray yaw, world constraint axis, expected target,
target type, risk state, correction direction, and a short explanation. Use it when deciding whether a real
DT35 reading is measuring a wall, forest, ramp, ignored long-pole interference, a corner, or an out-of-range
surface.
Add `--dt35-role-svg` to export a yaw-sliced heatmap. Each panel is one H30 yaw; circles are DT35-1, squares
are DT35-2. Fill color shows usable/ignored/out-of-range/corner/grazing state, and stroke color shows target
class: red wall, green forest/ramp, blue ignored area.
`--dt35-validation-plan` selects representative real-car checkpoints from the same model. The CSV and
Markdown checklist include the target pose, sensor, expected target, expected distance, incidence angle,
risk state, and operator note.
Use it to physically place the robot at a small set of high-value poses before running a full route: it covers
world-X wall constraints, world-Y wall constraints, forest blocks, ramp blocks, and ignored long-pole
interference. The selected points assume lidar XY is the initial absolute coordinate and H30 yaw is accurate;
DT35 residuals at these points should match the predicted target within the configured residual threshold.
`--dt35-field-sweep` is the broader coverage check. It sweeps a grid of field XY poses and H30 yaw values,
then writes one row per pose/yaw explaining the two DT35 targets, risk state, expected distance, constraint
axis, and whether that pose uses a wall, forest, ramp, or ignored interference. The summary is the quickest
way to see the current hardware limitation: with both DT35 sensors facing left/right, most usable poses are
`rank1_x`, `rank1_y`, or diagonal `rank1_xy`, so DT35 clamps only one translation component at a time while
lidar anchors and encoder/H30 prediction supply the remaining component. A `rank0_no_dt35` pose is not a
software failure; it means both side rays are ignored, out of range, grazing, corner-ambiguous, or otherwise
unusable at that pose/yaw.
`--dt35-yaw-matrix` keeps several representative lidar/world XY positions fixed and sweeps H30 yaw. Use it
to answer what each physical DT35 is measuring at that pose: world `x`, world `y`, diagonal `xy`, ignored
blue geometry, out-of-range wall, forest, or ramp. For example, near the center divider, yaw=0 makes both
side DT35 sensors constrain world `x`; yaw=90 rotates the same physical sensors so they look toward the
upper/lower walls and constrain world `y`, but those hits may be out of the 250 cm valid range.

For real logs, analyze measured residuals with lidar position and H30 yaw:

```powershell
python main.py --dt35-analyze-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --dt35-pose-source lidar --dt35-yaw-source h30 --dt35-analyze-output logs/dt35_residuals.csv --dt35-analyze-summary logs/dt35_residuals.json --dt35-advice-output logs/dt35_advice.csv --dt35-advice-summary logs/dt35_advice.json --dt35-advice-md logs/dt35_advice.md
```

This command applies the same red/blue start-pose display policy as the UI, then computes each DT35 ray
against the field model. Large residuals on `usable_wall` or `solid_obstacle` usually mean the wall/obstacle
geometry, start-pose mapping, or DT35 mounting offset needs correction. Residuals on `ignore` targets should
not be used for localization.
The residual report separates three states: `usable_for_correction` means the ray hits trusted geometry,
`residual_within_gate` means the measured distance is close enough to the model, and `usable_for_fusion`
means both are true. If real logs show many `residual_gate_rejected_rays`, keep H30/DT35 calibration fixed
and inspect field geometry, lidar coordinate alignment, or whether the ray is hitting an unmodeled object.
The optional `--dt35-advice-*` outputs aggregate real residuals by modeled target. Under the normal
assumption that lidar XY, H30 yaw, and DT35 distance are trusted, it converts persistent hit-point offsets
into target-specific shift suggestions, for example "move `field_left` by -5 cm along world X". Use this to
adjust a wall/forest/ramp edge in the field model; do not use it to tune DT35 scale or H30 yaw offset.

Smoke-test commands:

```powershell
python main.py --demo --duration-s 3 --screenshot logs/demo_final.png
python main.py --serial-port <current-port> --baudrate 115200 --duration-s 5 --screenshot logs/serial_after_flash.png
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
- `dt35_1_mm`: DT35-1 / ID 0x01 distance, mounted at local +X 404 mm, local +Y -33 mm, facing local -X left, in mm.
- `dt35_2_mm`: DT35-2 / ID 0x02 distance, mounted at local +X -404 mm, local +Y -33 mm, facing local +X right, in mm.
- `status_mask`: firmware status bits used by the UI lamps. Bits 10/11 mean orthogonal encoder 1/2 have received AB phase pulses; bit 0 is set only when both encoders have received pulses.
- CSV has no CRC, so `crc_state=no_crc` in logs is normal.

The app also keeps compatibility with old `r1_csv_v2` 25/41-column logs, legacy 5/6/9-column CSV, and the reserved `$R1M` ASCII frame with CRC16-CCITT-FALSE.

## UI

- Left panel: serial port, baudrate, protocol mode, reconnect, Chinese/English language toggle, red/blue start pose option, layer toggles, replay controls.
- Center: prior field map, fixed 83 cm x 83 cm robot texture, live-fused pos/calib/lidar trajectories, optional DT35 rays.
- DT35 model overlay: red means usable wall/partition, blue means ignored noisy rack/gap area, green means solid obstacle such as forest and ramp that blocks the laser and can correct pose with lower weight.
- DT35 ray colors: green means the measured ray is valid for correction; purple means it hit an ignored area;
  red means a corner/edge ambiguity; yellow/orange means skipped or residual warning. Hover the ray to see
  target name, expected distance, residual, and incidence angle.
- Right panel: final pos, lidar pose, encoder XY, H30 yaw, DT35 distance state, DT35 expected target/residual, serial FPS, RX bytes/s, and Chinese(English) sensor status lamps. Orthogonal encoder 1/2 are shown separately.
- The right panel uses the same display coordinate frame as the map after the red/blue start-pose policy is applied. Raw serial logs and `parsed_frames.csv` keep the original firmware values.
- Bottom panel: raw serial view and command sender. Realtime curves are intentionally not included; use VOFA for waveform viewing.

## Coordinate and Assets

- Field background: `assets/field_prior_map_clean_labeled_1215x1210cm.png`.
- Field size: 1215 cm x 1210 cm.
- World origin: field outer center, +X right, +Y up.
- Robot texture: `assets/r1_chassis_830mm_texture_1024.png`.
- Robot size: fixed 83 cm x 83 cm.
- Texture convention: image right is robot forward +Y, image down is robot right +X. With the default config, yaw=0 points to map +Y. DT35-1 is on the robot right side and rays to local -X; DT35-2 is on the robot left side and rays to local +X.

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
python -m json.tool config/default_config.json
python -m pytest -q
python -m compileall -q .
```

## Common Issues

- No serial port: check the wireless serial driver and click Refresh.
- No data: check USART1 baudrate 115200, TX/RX crossing, and common GND.
- VOFA works but map does not move: select `auto` or `r1_csv_v3`; current firmware outputs 12 numeric fields.
- Robot direction is off by 90/180 deg: tune `robot.texture_front_dir_deg_in_image`, `robot.yaw_offset_deg`, or `transform.data_yaw_offset_deg`.
- Coordinates are mirrored: tune `transform.data_x_sign` and `transform.data_y_sign`.
