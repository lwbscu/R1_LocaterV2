from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _add_src_to_path() -> None:
    root = Path(__file__).resolve().parent
    src = root / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))


def _parse_float_pair(text: str, name: str) -> tuple[float, float]:
    parts = [item.strip() for item in str(text).split(",") if item.strip()]
    if len(parts) != 2:
        raise ValueError(f"{name} must be min,max")
    return float(parts[0]), float(parts[1])


def _dt35_correct_lidar_frames(args: argparse.Namespace, config: dict) -> bool:
    if bool(args.fusion_dt35_correct_lidar_frames):
        return True
    return bool(config.get("display", {}).get("live_fusion_dt35_correct_lidar_frames", False))


def main() -> int:
    _add_src_to_path()

    parser = argparse.ArgumentParser(description="R1 real-time locater map")
    parser.add_argument("--config", default=None, help="Path to config JSON")
    parser.add_argument("--demo", action="store_true", help="Start with mock robot data")
    parser.add_argument("--record", action="store_true", help="Record a headless calibration serial session")
    parser.add_argument("--analyze-csv", default=None, help="Analyze a parsed_frames.csv calibration log")
    parser.add_argument("--simulate-fusion", default=None, help="Simulate lidar/encoder/H30 fusion from parsed_frames.csv")
    parser.add_argument("--validate-model-csv", default=None, help="Validate live fusion and DT35 model against lidar from parsed_frames.csv")
    parser.add_argument("--validate-model-output", default=None, help="Output JSON path for --validate-model-csv")
    parser.add_argument("--validate-model-fused-output", default=None, help="Optional replay CSV path for live-fused frames")
    parser.add_argument("--real-validation-csv", default=None, help="Run full real-log validation suite from parsed_frames.csv")
    parser.add_argument("--real-validation-output-dir", default=None, help="Output directory for --real-validation-csv artifacts")
    parser.add_argument("--readiness-report", action="store_true", help="Run offline DT35/H30/lidar/encoder readiness gate and optional real-log gate")
    parser.add_argument("--readiness-output-dir", default=None, help="Output directory for --readiness-report artifacts")
    parser.add_argument("--readiness-real-csv", default=None, help="Optional parsed_frames.csv real log to include in --readiness-report")
    parser.add_argument("--readiness-samples", type=int, default=180, help="Synthetic samples per path for --readiness-report")
    parser.add_argument("--readiness-grid-step", type=float, default=100.0, help="DT35 field sweep grid step in cm for --readiness-report")
    parser.add_argument("--path-report-csv", default=None, help="Write per-frame DT35/fusion diagnostics from parsed_frames.csv")
    parser.add_argument("--path-report-output", default=None, help="CSV output path for --path-report-csv")
    parser.add_argument("--path-report-summary", default=None, help="JSON summary path for --path-report-csv")
    parser.add_argument("--dt35-hit-table", action="store_true", help="Print expected DT35 target hits for field poses")
    parser.add_argument("--dt35-hit-poses", default=None, help="Semicolon-separated x,y,yaw[,label] poses in field cm coordinates")
    parser.add_argument("--dt35-hit-output", default=None, help="Optional CSV path for --dt35-hit-table")
    parser.add_argument("--dt35-yaw-matrix", action="store_true", help="Sweep H30 yaw at selected lidar/world XY poses and print DT35 target/axis matrix")
    parser.add_argument("--dt35-yaw-matrix-poses", default=None, help="Semicolon-separated x,y[,label] base poses in field cm coordinates")
    parser.add_argument("--dt35-yaw-matrix-yaws", default="-180,-135,-90,-45,0,45,90,135,180", help="Comma-separated yaw list in deg for --dt35-yaw-matrix")
    parser.add_argument("--dt35-yaw-matrix-output", default=None, help="Optional CSV path for --dt35-yaw-matrix")
    parser.add_argument("--dt35-yaw-matrix-summary", default=None, help="Optional JSON summary path for --dt35-yaw-matrix")
    parser.add_argument("--dt35-role-map", action="store_true", help="Explain what each DT35 measures for poses/yaws")
    parser.add_argument("--dt35-role-poses", default=None, help="Optional semicolon-separated x,y,yaw[,label] poses for --dt35-role-map")
    parser.add_argument("--dt35-role-output", default=None, help="Optional CSV path for --dt35-role-map")
    parser.add_argument("--dt35-role-summary", default=None, help="Optional JSON summary path for --dt35-role-map")
    parser.add_argument("--dt35-role-svg", default=None, help="Optional SVG heatmap path for --dt35-role-map")
    parser.add_argument("--dt35-validation-plan", action="store_true", help="Pick representative real-car DT35 validation poses from the field model")
    parser.add_argument("--dt35-validation-plan-output", default=None, help="CSV output path for --dt35-validation-plan")
    parser.add_argument("--dt35-validation-plan-summary", default=None, help="JSON summary path for --dt35-validation-plan")
    parser.add_argument("--dt35-validation-plan-md", default=None, help="Markdown checklist output path for --dt35-validation-plan")
    parser.add_argument("--dt35-validation-plan-per-category", type=int, default=3, help="Cases to select per validation category")
    parser.add_argument("--dt35-field-sweep", action="store_true", help="Sweep field poses/yaws and summarize DT35 constraint/target coverage")
    parser.add_argument("--dt35-field-sweep-output", default=None, help="CSV output path for --dt35-field-sweep")
    parser.add_argument("--dt35-field-sweep-summary", default=None, help="JSON summary path for --dt35-field-sweep")
    parser.add_argument("--dt35-grid", action="store_true", help="Analyze DT35 target coverage over a field grid")
    parser.add_argument("--dt35-grid-x", default="-580,580", help="Grid X range min,max in cm")
    parser.add_argument("--dt35-grid-y", default="-580,580", help="Grid Y range min,max in cm")
    parser.add_argument("--dt35-grid-step", type=float, default=100.0, help="Grid step in cm")
    parser.add_argument("--dt35-grid-yaws", default="-180,-135,-90,-45,0,45,90,135", help="Comma-separated yaw list in deg")
    parser.add_argument("--dt35-grid-output", default=None, help="Optional CSV path for DT35 grid rows")
    parser.add_argument("--dt35-grid-summary", default=None, help="Optional JSON path for DT35 grid summary")
    parser.add_argument("--dt35-observability", action="store_true", help="Analyze DT35 translation observability over poses/grid")
    parser.add_argument("--dt35-observability-output", default=None, help="Optional CSV path for DT35 observability rows")
    parser.add_argument("--dt35-observability-summary", default=None, help="Optional JSON path for DT35 observability summary")
    parser.add_argument("--dt35-analyze-csv", default=None, help="Analyze DT35 residuals from parsed_frames.csv")
    parser.add_argument("--dt35-analyze-output", default=None, help="Optional CSV path for DT35 residual rows")
    parser.add_argument("--dt35-analyze-summary", default=None, help="Optional JSON path for DT35 residual summary")
    parser.add_argument("--dt35-advice-output", default=None, help="Optional CSV path for DT35 target calibration advice")
    parser.add_argument("--dt35-advice-summary", default=None, help="Optional JSON path for DT35 target calibration advice summary")
    parser.add_argument("--dt35-advice-md", default=None, help="Optional Markdown path for DT35 target calibration advice")
    parser.add_argument("--dt35-pose-source", default="lidar", choices=("lidar", "pos", "encoder", "calib"), help="XY pose source for DT35 residual analysis")
    parser.add_argument("--dt35-yaw-source", default="h30", choices=("h30", "lidar", "pos", "encoder", "calib"), help="Yaw source for DT35 residual analysis")
    parser.add_argument("--dt35-start-side", default=None, choices=("red", "blue", "none"), help="Override start side for DT35 analysis")
    parser.add_argument("--dt35-start-policy", default=None, choices=("auto_lidar_offline", "always_local_display", "off"), help="Override start pose policy for DT35 analysis")
    parser.add_argument("--field-model-svg", default=None, help="Export a SVG overlay of the current DT35 field model")
    parser.add_argument("--field-model-svg-poses", default=None, help="Optional semicolon-separated x,y,yaw[,label] poses for DT35 rays in the SVG")
    parser.add_argument("--field-model-audit", default=None, help="Export a JSON audit of map size, DT35 field geometry, hit coverage, and missing real evidence")
    parser.add_argument("--field-model-audit-poses", default=None, help="Optional semicolon-separated x,y,yaw[,label] poses for the field model audit")
    parser.add_argument("--synthetic-fusion", action="store_true", help="Generate synthetic sensor frames and run fusion")
    parser.add_argument("--synthetic-suite", action="store_true", help="Run a synthetic fusion sweep across paths and DT35 gains")
    parser.add_argument("--synthetic-benchmark", action="store_true", help="Run the default DT35/H30/lidar fusion benchmark over key field paths")
    parser.add_argument("--synthetic-benchmark-output", default=None, help="CSV output path for --synthetic-benchmark")
    parser.add_argument("--synthetic-benchmark-summary", default=None, help="JSON summary path for --synthetic-benchmark")
    parser.add_argument("--synthetic-obstacle-ablation", action="store_true", help="Compare fusion with/without forest/ramp correction")
    parser.add_argument("--synthetic-obstacle-paths", default=None, help="Comma-separated paths for --synthetic-obstacle-ablation")
    parser.add_argument("--synthetic-obstacle-ablation-output", default=None, help="CSV output path for --synthetic-obstacle-ablation")
    parser.add_argument("--synthetic-obstacle-ablation-summary", default=None, help="JSON summary path for --synthetic-obstacle-ablation")
    parser.add_argument("--synthetic-monte-carlo", action="store_true", help="Run random_patrol multi-seed fusion stress test")
    parser.add_argument("--synthetic-path-report", action="store_true", help="Write per-frame synthetic path DT35/fusion diagnostics")
    parser.add_argument("--synthetic-path-report-output", default=None, help="CSV path for --synthetic-path-report")
    parser.add_argument("--synthetic-path-report-summary", default=None, help="JSON summary path for --synthetic-path-report")
    parser.add_argument("--synthetic-monte-carlo-runs", type=int, default=20, help="Number of random_patrol seeds for --synthetic-monte-carlo")
    parser.add_argument("--synthetic-seed-base", type=int, default=1000, help="Base seed for --synthetic-monte-carlo")
    parser.add_argument("--synthetic-path", default="top_corridor", help="Synthetic path: top_corridor/static_start/forest_side/ramp_side/center_divider/yaw_sweep/field_patrol/random_patrol")
    parser.add_argument("--synthetic-suite-paths", default="top_corridor,forest_side,ramp_side,center_divider,start_corner_yaw_sweep,field_patrol", help="Comma-separated synthetic paths for --synthetic-suite")
    parser.add_argument("--synthetic-dt35-gains", default="0,0.4,0.6,0.85,1.0", help="Comma-separated DT35 gains for --synthetic-suite")
    parser.add_argument("--synthetic-samples", type=int, default=240, help="Synthetic frame count")
    parser.add_argument("--synthetic-encoder-x-scale", type=float, default=1.0, help="Synthetic encoder X scale")
    parser.add_argument("--synthetic-encoder-y-scale", type=float, default=1.0, help="Synthetic encoder Y scale")
    parser.add_argument("--synthetic-encoder-yaw-scale", type=float, default=1.0, help="Synthetic H30 yaw delta scale")
    parser.add_argument("--synthetic-h30-yaw-bias", type=float, default=0.0, help="Synthetic H30 yaw bias in deg")
    parser.add_argument("--synthetic-dt35-noise-mm", type=float, default=0.0, help="Synthetic DT35 distance noise amplitude")
    parser.add_argument("--lidar-stride", type=int, default=5, help="Use every Nth lidar frame in --simulate-fusion")
    parser.add_argument("--fusion-strides", default=None, help="Comma-separated lidar strides to sweep, e.g. 1,2,5,10,25")
    parser.add_argument("--fusion-lidar-gain", type=float, default=1.0, help="Lidar correction gain for --simulate-fusion")
    parser.add_argument("--fusion-dt35-gain", type=float, default=1.0, help="DT35 translation correction gain for --simulate-fusion")
    parser.add_argument("--fusion-dt35-yaw-gain", type=float, default=0.0, help="DT35 yaw correction gain; default 0 because H30 yaw is trusted")
    parser.add_argument("--fusion-dt35-correct-lidar-frames", action="store_true", help="Allow DT35 to correct frames that already used a lidar anchor")
    parser.add_argument("--fusion-output", default=None, help="Output CSV path for --simulate-fusion")
    parser.add_argument("--replay", default=None, help="Replay parsed_frames.csv")
    parser.add_argument("--serial-port", default=None, help="Open this COM port on startup")
    parser.add_argument("--baudrate", type=int, default=None, help="Override serial baudrate")
    parser.add_argument("--duration-s", type=float, default=None, help="Auto-close after N seconds")
    parser.add_argument("--screenshot", default=None, help="Save a screenshot before auto-close")
    parser.add_argument("--capture", action="store_true", help="Start GUI capture automatically and save it on timed close")
    args = parser.parse_args()

    if args.field_model_svg:
        from locater_map.config_loader import load_config
        from locater_map.dt35_analysis import parse_pose_specs
        from locater_map.field_model_export import write_field_model_svg

        config = load_config(args.config)
        poses = parse_pose_specs(args.field_model_svg_poses)
        write_field_model_svg(args.field_model_svg, config, poses)
        print(f"output={args.field_model_svg}")
        print(f"poses={len(poses)}")
        return 0

    if args.field_model_audit:
        from locater_map.config_loader import load_config
        from locater_map.dt35_analysis import parse_pose_specs
        from locater_map.field_model_audit import build_field_model_audit

        config = load_config(args.config)
        poses = parse_pose_specs(args.field_model_audit_poses)
        audit = build_field_model_audit(config, poses)
        output_path = Path(args.field_model_audit)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"output={args.field_model_audit}")
        print(f"poses={len(poses)}")
        failed_dimensions = [item["name"] for item in audit["manual_dimension_checks"] if not item["passed"]]
        behavior = audit["default_pose_behavior"]
        print(f"self_check={audit['model_self_check']['passed']} failed={audit['model_self_check']['failed_checks']}")
        print(f"dimension_failed={failed_dimensions}")
        print(
            f"usable_forest_rays={behavior['usable_forest_ray_count']} "
            f"usable_ramp_rays={behavior['usable_ramp_ray_count']}"
        )
        sweep = audit.get("field_sweep", {}).get("summary", {})
        print(
            f"field_sweep_passed={sweep.get('model_passed')} "
            f"forest_constraints={sweep.get('forest_constraint_poses')} "
            f"ramp_constraints={sweep.get('ramp_constraint_poses')} "
            f"ignored_interference={sweep.get('ignored_interference_poses')}"
        )
        return 0

    if args.record:
        from locater_map.calibration_recorder import record_to_session
        from locater_map.config_loader import load_config

        config = load_config(args.config)
        port = args.serial_port or str(config.get("serial", {}).get("default_port") or "")
        baudrate = int(args.baudrate or config.get("serial", {}).get("baudrate", 115200))
        duration_s = float(args.duration_s or 10.0)
        if not port:
            parser.error("--record requires --serial-port or serial.default_port in config")
        session_dir, summary = record_to_session(
            port=port,
            baudrate=baudrate,
            duration_s=duration_s,
            output_root=Path(config.get("_project_root", Path(__file__).resolve().parent)) / "logs",
            protocol_cfg=config.get("protocol", {}),
        )
        print(f"session_dir={session_dir}")
        print(f"frames={summary.stats.frames} raw_lines={summary.stats.raw_lines} fps={summary.stats.fps:.2f}")
        print(f"lidar_valid={summary.stats.lidar_valid_frames} h30_valid={summary.stats.h30_valid_frames}")
        print(f"encoder1_seen={summary.stats.encoder_1_seen_frames} encoder2_seen={summary.stats.encoder_2_seen_frames}")
        for note in summary.notes:
            print(f"- {note}")
        return 0

    if args.analyze_csv:
        from locater_map.calibration_recorder import CalibrationRecorder
        from locater_map.config_loader import load_config

        config = load_config(args.config)
        recorder = CalibrationRecorder(output_root=Path(config.get("_project_root", Path(__file__).resolve().parent)) / "logs")
        summary = recorder.load_csv(args.analyze_csv)
        print(f"session_dir={recorder.session_dir}")
        print(f"frames={summary.stats.frames}")
        print(f"lidar_valid={summary.stats.lidar_valid_frames} h30_valid={summary.stats.h30_valid_frames}")
        print(f"encoder1_seen={summary.stats.encoder_1_seen_frames} encoder2_seen={summary.stats.encoder_2_seen_frames}")
        for note in summary.notes:
            print(f"- {note}")
        return 0

    if (
        args.dt35_hit_table
        or args.dt35_yaw_matrix
        or args.dt35_role_map
        or args.dt35_validation_plan
        or args.dt35_field_sweep
        or args.dt35_grid
        or args.dt35_observability
    ):
        from locater_map.config_loader import load_config
        from locater_map.dt35_analysis import (
            analyze_observability,
            analyze_dt35_hits,
            generate_grid_poses,
            generate_yaw_matrix_poses,
            parse_pose_specs,
            parse_xy_pose_specs,
            print_observability_summary,
            print_coverage_summary,
            print_hit_rows,
            summarize_observability,
            summarize_coverage,
            write_coverage_summary_json,
            write_hit_rows_csv,
            write_observability_rows_csv,
            write_observability_summary_json,
        )

        config = load_config(args.config)
        if args.dt35_validation_plan:
            from locater_map.dt35_validation_plan import (
                generate_dt35_validation_plan,
                print_validation_plan_summary,
                write_validation_plan_csv,
                write_validation_plan_markdown,
                write_validation_plan_summary,
            )

            x_min, x_max = _parse_float_pair(args.dt35_grid_x, "--dt35-grid-x")
            y_min, y_max = _parse_float_pair(args.dt35_grid_y, "--dt35-grid-y")
            yaws = [float(item.strip()) for item in str(args.dt35_grid_yaws).split(",") if item.strip()]
            cases, summary = generate_dt35_validation_plan(
                config,
                x_min_cm=x_min,
                x_max_cm=x_max,
                y_min_cm=y_min,
                y_max_cm=y_max,
                step_cm=float(args.dt35_grid_step),
                yaws_deg=yaws,
                per_category=max(1, int(args.dt35_validation_plan_per_category)),
            )
            output = Path(args.dt35_validation_plan_output or "logs/dt35_validation_plan.csv")
            summary_output = Path(args.dt35_validation_plan_summary or "logs/dt35_validation_plan.json")
            markdown_output = Path(args.dt35_validation_plan_md or "logs/dt35_validation_plan.md")
            write_validation_plan_csv(output, cases)
            write_validation_plan_summary(summary_output, summary)
            write_validation_plan_markdown(markdown_output, cases, summary)
            print(f"output={output}")
            print(f"summary={summary_output}")
            print(f"markdown={markdown_output}")
            print_validation_plan_summary(summary)
            return 0

        if args.dt35_field_sweep:
            from locater_map.dt35_field_sweep import (
                print_field_sweep_summary,
                run_dt35_field_sweep,
                write_field_sweep_csv,
                write_field_sweep_summary,
            )

            x_min, x_max = _parse_float_pair(args.dt35_grid_x, "--dt35-grid-x")
            y_min, y_max = _parse_float_pair(args.dt35_grid_y, "--dt35-grid-y")
            yaws = [float(item.strip()) for item in str(args.dt35_grid_yaws).split(",") if item.strip()]
            rows, summary = run_dt35_field_sweep(
                config,
                x_min_cm=x_min,
                x_max_cm=x_max,
                y_min_cm=y_min,
                y_max_cm=y_max,
                step_cm=float(args.dt35_grid_step),
                yaws_deg=yaws,
            )
            output = Path(args.dt35_field_sweep_output or "logs/dt35_field_sweep.csv")
            summary_output = Path(args.dt35_field_sweep_summary or "logs/dt35_field_sweep.json")
            write_field_sweep_csv(output, rows)
            write_field_sweep_summary(summary_output, summary)
            print(f"output={output}")
            print(f"summary={summary_output}")
            print_field_sweep_summary(summary)
            return 0

        if args.dt35_role_map and args.dt35_role_poses:
            poses = parse_pose_specs(args.dt35_role_poses)
        elif args.dt35_yaw_matrix:
            base_poses = parse_xy_pose_specs(args.dt35_yaw_matrix_poses)
            yaws = [float(item.strip()) for item in str(args.dt35_yaw_matrix_yaws).split(",") if item.strip()]
            poses = generate_yaw_matrix_poses(base_poses, yaws)
        elif args.dt35_grid or args.dt35_observability or args.dt35_role_map:
            x_min, x_max = _parse_float_pair(args.dt35_grid_x, "--dt35-grid-x")
            y_min, y_max = _parse_float_pair(args.dt35_grid_y, "--dt35-grid-y")
            yaws = [float(item.strip()) for item in str(args.dt35_grid_yaws).split(",") if item.strip()]
            poses = generate_grid_poses(x_min, x_max, y_min, y_max, float(args.dt35_grid_step), yaws)
        else:
            poses = parse_pose_specs(args.dt35_hit_poses)
        rows = analyze_dt35_hits(config, poses)
        if args.dt35_role_map:
            from locater_map.dt35_role_report import (
                build_dt35_role_rows,
                print_dt35_role_rows,
                print_dt35_role_summary,
                summarize_dt35_roles,
                write_dt35_role_csv,
                write_dt35_role_summary,
            )
            from locater_map.dt35_role_svg import write_dt35_role_svg

            role_rows = build_dt35_role_rows(rows)
            summary = summarize_dt35_roles(role_rows)
            if args.dt35_role_output:
                write_dt35_role_csv(args.dt35_role_output, role_rows)
                print(f"output={args.dt35_role_output}")
            if args.dt35_role_summary:
                write_dt35_role_summary(args.dt35_role_summary, summary)
                print(f"summary={args.dt35_role_summary}")
            if args.dt35_role_svg:
                write_dt35_role_svg(args.dt35_role_svg, role_rows, config)
                print(f"svg={args.dt35_role_svg}")
            if args.dt35_role_output:
                print_dt35_role_summary(summary)
            else:
                print_dt35_role_rows(role_rows)
                print_dt35_role_summary(summary)
            return 0
        if args.dt35_observability:
            observability_rows = analyze_observability(rows)
            output = args.dt35_observability_output
            if output:
                write_observability_rows_csv(output, observability_rows)
                print(f"output={output}")
            summary = summarize_observability(observability_rows, rows)
            summary_output = args.dt35_observability_summary
            if summary_output:
                write_observability_summary_json(summary_output, summary)
                print(f"summary={summary_output}")
            print_observability_summary(summary)
            return 0
        output = args.dt35_yaw_matrix_output if args.dt35_yaw_matrix else args.dt35_grid_output if args.dt35_grid else args.dt35_hit_output
        if output:
            write_hit_rows_csv(output, rows)
            print(f"output={output}")
        summary = summarize_coverage(rows)
        summary_output = args.dt35_yaw_matrix_summary if args.dt35_yaw_matrix else args.dt35_grid_summary
        if summary_output:
            write_coverage_summary_json(summary_output, summary)
            print(f"summary={summary_output}")
        if args.dt35_grid or args.dt35_yaw_matrix:
            print_coverage_summary(summary)
        else:
            print_hit_rows(rows)
        return 0

    if args.dt35_analyze_csv:
        from locater_map.config_loader import load_config
        from locater_map.dt35_analysis import (
            analyze_dt35_frames,
            print_residual_summary,
            summarize_residuals,
            write_residual_rows_csv,
            write_residual_summary_json,
        )
        from locater_map.dt35_calibration_advisor import (
            build_calibration_advice,
            write_calibration_advice_csv,
            write_calibration_advice_markdown,
            write_calibration_advice_summary,
        )
        from locater_map.fusion_model import load_frames_csv

        config = load_config(args.config)
        frames = load_frames_csv(args.dt35_analyze_csv)
        start_side = None if args.dt35_start_side in (None, "none") else args.dt35_start_side
        rows = analyze_dt35_frames(
            config,
            frames,
            pose_source=str(args.dt35_pose_source),
            yaw_source=str(args.dt35_yaw_source),
            start_side=start_side,
            start_policy=args.dt35_start_policy,
        )
        summary = summarize_residuals(rows)
        output = args.dt35_analyze_output
        if output:
            write_residual_rows_csv(output, rows)
            print(f"output={output}")
        summary_output = args.dt35_analyze_summary
        if summary_output:
            write_residual_summary_json(summary_output, summary)
            print(f"summary={summary_output}")
        advice, advice_summary = build_calibration_advice(
            rows,
            source=str(args.dt35_analyze_csv),
        )
        advice_output = args.dt35_advice_output
        advice_summary_output = args.dt35_advice_summary
        advice_md_output = args.dt35_advice_md
        if advice_output:
            write_calibration_advice_csv(advice_output, advice)
            print(f"advice={advice_output}")
        if advice_summary_output:
            write_calibration_advice_summary(advice_summary_output, advice_summary)
            print(f"advice_summary={advice_summary_output}")
        if advice_md_output:
            write_calibration_advice_markdown(advice_md_output, advice, advice_summary)
            print(f"advice_markdown={advice_md_output}")
        print_residual_summary(summary)
        print(
            f"advice_targets={advice_summary.targets} actionable={advice_summary.actionable_targets} "
            f"worst_target={advice_summary.worst_target} worst_rms={advice_summary.worst_rms_residual_cm}"
        )
        return 0

    if args.validate_model_csv:
        from locater_map.config_loader import load_config
        from locater_map.fusion_model import FusionConfig, load_frames_csv, write_frames_csv
        from locater_map.model_validation import validate_model_log, write_validation_report

        config = load_config(args.config)
        source_path = Path(args.validate_model_csv)
        frames = load_frames_csv(source_path)
        start_side = None if args.dt35_start_side in (None, "none") else args.dt35_start_side
        fusion_cfg = FusionConfig(
            lidar_stride=max(1, int(args.lidar_stride)),
            lidar_gain=float(args.fusion_lidar_gain),
            dt35_gain=float(args.fusion_dt35_gain),
            dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
            dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
        )
        report, fused_frames = validate_model_log(
            config,
            frames,
            fusion_cfg,
            start_side=start_side,
            start_policy=args.dt35_start_policy,
        )
        output = Path(args.validate_model_output) if args.validate_model_output else source_path.with_name(source_path.stem + "_model_validation.json")
        write_validation_report(output, report)
        if args.validate_model_fused_output:
            write_frames_csv(args.validate_model_fused_output, fused_frames)
            print(f"fused_output={args.validate_model_fused_output}")
        pose = report.pose_error
        residuals = report.dt35_residuals
        print(f"output={output}")
        print(
            f"frames={pose.frames} lidar_reference={pose.lidar_reference_frames} "
            f"raw_rms_xy_cm={pose.raw_rms_xy_cm} fused_rms_xy_cm={pose.fused_rms_xy_cm} "
            f"raw_rms_yaw_deg={pose.raw_rms_yaw_deg} fused_rms_yaw_deg={pose.fused_rms_yaw_deg}"
        )
        print(
            f"dt35_valid_rays={residuals.get('valid_rays')} usable={residuals.get('usable_rays')} "
            f"dt35_rms_residual_cm={residuals.get('rms_residual_cm')} "
            f"target_type_counts={residuals.get('target_type_counts')}"
        )
        print(f"validation_passed={report.gates.passed} checks={report.gates.checks}")
        quality = report.dt35_quality
        print(
            f"dt35_quality_passed={quality.get('passed')} "
            f"bad_targets={len(quality.get('bad_targets', []))} "
            f"good_targets={quality.get('good_target_count')}"
        )
        for item in quality.get("bad_targets", [])[:5]:
            print(f"bad_target={item.get('target')} reasons={item.get('reasons')}")
        if report.gates.notes:
            print("validation_notes=" + " | ".join(report.gates.notes))
        return 0

    if args.real_validation_csv:
        from locater_map.config_loader import load_config
        from locater_map.fusion_model import FusionConfig
        from locater_map.real_validation_suite import run_real_validation_suite

        config = load_config(args.config)
        start_side = None if args.dt35_start_side in (None, "none") else args.dt35_start_side
        fusion_cfg = FusionConfig(
            lidar_stride=max(1, int(args.lidar_stride)),
            lidar_gain=float(args.fusion_lidar_gain),
            dt35_gain=float(args.fusion_dt35_gain),
            dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
            dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
        )
        result = run_real_validation_suite(
            config,
            args.real_validation_csv,
            args.real_validation_output_dir,
            fusion_cfg,
            start_side=start_side,
            start_policy=args.dt35_start_policy,
        )
        failed = [name for name, passed in result.checks.items() if not passed]
        print(f"output_dir={result.output_dir}")
        print(f"suite_report={result.artifacts.suite_report_json}")
        print(f"dt35_advice={result.artifacts.dt35_advice_md}")
        print(
            f"frames={result.frame_summary.get('frames')} synthetic={result.is_synthetic} "
            f"real_validation_passed={result.real_validation_passed}"
        )
        print(
            f"raw_rms_xy_cm={result.pose_error.get('raw_rms_xy_cm')} "
            f"fused_rms_xy_cm={result.pose_error.get('fused_rms_xy_cm')} "
            f"dt35_rms_residual_cm={result.dt35_residuals.get('rms_residual_cm')} "
            f"dt35_actionable_targets={result.dt35_advice.get('actionable_targets')}"
        )
        print(f"failed_checks={failed}")
        if result.notes:
            print("notes=" + " | ".join(result.notes))
        return 0

    if args.readiness_report:
        from locater_map.config_loader import load_config
        from locater_map.fusion_model import FusionConfig
        from locater_map.readiness_report import run_readiness_report

        config = load_config(args.config)
        fusion_cfg = FusionConfig(
            lidar_stride=max(1, int(args.lidar_stride)),
            lidar_gain=float(args.fusion_lidar_gain),
            dt35_gain=float(args.fusion_dt35_gain),
            dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
            dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
        )
        report = run_readiness_report(
            config,
            args.readiness_output_dir,
            fusion_cfg,
            real_csv=args.readiness_real_csv,
            samples=max(2, int(args.readiness_samples)),
            grid_step_cm=float(args.readiness_grid_step),
        )
        failed = [name for name, passed in report.checks.items() if not passed]
        print(f"output_dir={report.output_dir}")
        print(f"report={report.artifacts.report_json}")
        print(f"markdown={report.artifacts.report_md}")
        print(
            f"offline_readiness_passed={report.offline_readiness_passed} "
            f"completion_verified={report.completion_verified}"
        )
        print(
            f"synthetic_mean_raw_rms_xy_cm={report.synthetic_benchmark.get('mean_raw_rms_xy_cm')} "
            f"synthetic_mean_fused_rms_xy_cm={report.synthetic_benchmark.get('mean_fused_rms_xy_cm')} "
            f"dt35_fusion_frames={report.synthetic_benchmark.get('total_dt35_fusion_allowed_frames')}"
        )
        print(f"failed_checks={failed}")
        if report.next_actions:
            print("next_actions=" + " | ".join(report.next_actions))
        return 0

    if args.path_report_csv:
        from locater_map.config_loader import load_config
        from locater_map.fusion_model import FusionConfig, load_frames_csv, write_frames_csv
        from locater_map.path_diagnostics import (
            generate_path_diagnostic,
            write_path_diagnostic_csv,
            write_path_diagnostic_summary,
        )

        config = load_config(args.config)
        source_path = Path(args.path_report_csv)
        frames = load_frames_csv(source_path)
        start_side = None if args.dt35_start_side in (None, "none") else args.dt35_start_side
        fusion_cfg = FusionConfig(
            lidar_stride=max(1, int(args.lidar_stride)),
            lidar_gain=float(args.fusion_lidar_gain),
            dt35_gain=float(args.fusion_dt35_gain),
            dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
            dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
        )
        rows, summary, fused_frames = generate_path_diagnostic(
            config,
            frames,
            fusion_cfg,
            start_side=start_side,
            start_policy=args.dt35_start_policy,
        )
        csv_path = Path(args.path_report_output) if args.path_report_output else source_path.with_name(source_path.stem + "_path_report.csv")
        summary_path = Path(args.path_report_summary) if args.path_report_summary else source_path.with_name(source_path.stem + "_path_report.json")
        fused_path = csv_path.with_suffix(".fused.csv")
        write_path_diagnostic_csv(csv_path, rows)
        write_path_diagnostic_summary(summary_path, summary)
        write_frames_csv(fused_path, fused_frames)
        print(f"output={csv_path}")
        print(f"summary={summary_path}")
        print(f"fused_output={fused_path}")
        print(
            f"frames={summary.frames} raw_rms_xy_cm={summary.raw_rms_xy_cm} "
            f"no_dt35_rms_xy_cm={summary.no_dt35_rms_xy_cm} "
            f"fused_rms_xy_cm={summary.fused_rms_xy_cm} improved={summary.improved_frames} "
            f"worsened={summary.worsened_frames}"
        )
        print(
            f"dt35_active_frames={summary.dt35_active_frames} helped={summary.dt35_helped_frames} "
            f"dt35_worsened={summary.dt35_worsened_frames} "
            f"mean_correction_cm={summary.dt35_mean_correction_cm} "
            f"max_correction_cm={summary.dt35_max_correction_cm}"
        )
        print(
            f"dt35_valid_frames={summary.dt35_valid_frames} allowed_frames={summary.dt35_allowed_frames} "
            f"fusion_allowed_frames={summary.dt35_fusion_allowed_frames} "
            f"gate_rejected_frames={summary.dt35_residual_gate_rejected_frames} "
            f"corner_frames={summary.dt35_corner_frames} types={summary.dt35_type_counts}"
        )
        return 0

    if args.simulate_fusion:
        from locater_map.config_loader import load_config
        from locater_map.fusion_model import (
            FusionConfig,
            load_frames_csv,
            simulate_fusion,
            write_frames_csv,
            write_metrics_json,
        )

        config = load_config(args.config)
        source_path = Path(args.simulate_fusion)
        frames = load_frames_csv(source_path)
        if args.fusion_strides:
            strides = [max(1, int(item.strip())) for item in args.fusion_strides.split(",") if item.strip()]
            sweep = []
            for stride in strides:
                sim_cfg = FusionConfig(
                    lidar_stride=stride,
                    lidar_gain=float(args.fusion_lidar_gain),
                    dt35_gain=float(args.fusion_dt35_gain),
                    dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
                    dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
                )
                result = simulate_fusion(frames, sim_cfg, config)
                row = result.metrics.to_dict()
                row["lidar_stride"] = stride
                sweep.append(row)
            output = Path(args.fusion_output) if args.fusion_output else source_path.with_name(source_path.stem + "_fusion_sweep.json")
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(sweep, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"output={output}")
            print("stride,lidar_used,holdout,rms_xy_cm,rms_yaw_deg,max_xy_cm")
            for row in sweep:
                print(f"{row['lidar_stride']},{row['lidar_used_frames']},{row['lidar_holdout_frames']},"
                      f"{row['rms_xy_cm']},{row['rms_yaw_deg']},{row['max_xy_cm']}")
            return 0

        sim_cfg = FusionConfig(
            lidar_stride=max(1, int(args.lidar_stride)),
            lidar_gain=float(args.fusion_lidar_gain),
            dt35_gain=float(args.fusion_dt35_gain),
            dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
            dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
        )
        result = simulate_fusion(frames, sim_cfg, config)
        output = Path(args.fusion_output) if args.fusion_output else source_path.with_name(source_path.stem + "_fusion_sim.csv")
        write_frames_csv(output, result.frames)
        metrics_path = output.with_suffix(".metrics.json")
        write_metrics_json(metrics_path, result.metrics, sim_cfg)
        print(f"output={output}")
        print(f"metrics={metrics_path}")
        print(f"frames={result.metrics.frames} lidar_used={result.metrics.lidar_used_frames} holdout={result.metrics.lidar_holdout_frames}")
        print(f"rms_xy_cm={result.metrics.rms_xy_cm} rms_yaw_deg={result.metrics.rms_yaw_deg} max_xy_cm={result.metrics.max_xy_cm}")
        return 0

    if (
        args.synthetic_fusion
        or args.synthetic_suite
        or args.synthetic_benchmark
        or args.synthetic_obstacle_ablation
        or args.synthetic_monte_carlo
        or args.synthetic_path_report
    ):
        from datetime import datetime

        from locater_map.config_loader import load_config
        from locater_map.fusion_benchmark import (
            DEFAULT_BENCHMARK_PATHS,
            run_synthetic_benchmark,
            write_benchmark_csv,
            write_benchmark_summary,
        )
        from locater_map.fusion_model import FusionConfig, simulate_fusion, write_frames_csv, write_metrics_json
        from locater_map.obstacle_ablation import (
            DEFAULT_OBSTACLE_PATHS,
            run_obstacle_ablation,
            write_obstacle_ablation_csv,
            write_obstacle_ablation_summary,
        )
        from locater_map.path_diagnostics import (
            generate_synthetic_path_diagnostic,
            write_path_diagnostic_csv,
            write_path_diagnostic_summary,
        )
        from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames

        config = load_config(args.config)
        root = Path(config.get("_project_root", Path(__file__).resolve().parent))
        if args.synthetic_benchmark:
            paths = [item.strip() for item in str(args.synthetic_suite_paths).split(",") if item.strip()] or list(DEFAULT_BENCHMARK_PATHS)
            fusion_cfg = FusionConfig(
                lidar_stride=max(1, int(args.lidar_stride)),
                lidar_gain=float(args.fusion_lidar_gain),
                dt35_gain=float(args.fusion_dt35_gain),
                dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
                dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
            )
            rows, summary = run_synthetic_benchmark(
                config,
                paths,
                samples=max(2, int(args.synthetic_samples)),
                encoder_x_scale=float(args.synthetic_encoder_x_scale),
                encoder_y_scale=float(args.synthetic_encoder_y_scale),
                encoder_yaw_scale=float(args.synthetic_encoder_yaw_scale),
                h30_yaw_bias_deg=float(args.synthetic_h30_yaw_bias),
                dt35_noise_mm=float(args.synthetic_dt35_noise_mm),
                fusion_cfg=fusion_cfg,
            )
            output_dir = root / "logs" / f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_synthetic_benchmark"
            output_dir.mkdir(parents=True, exist_ok=True)
            csv_path = Path(args.synthetic_benchmark_output) if args.synthetic_benchmark_output else output_dir / "benchmark.csv"
            summary_path = Path(args.synthetic_benchmark_summary) if args.synthetic_benchmark_summary else output_dir / "benchmark_summary.json"
            write_benchmark_csv(csv_path, rows)
            write_benchmark_summary(summary_path, summary)
            print(f"output={csv_path}")
            print(f"summary={summary_path}")
            print(
                f"paths={summary.paths} passed={summary.passed_paths} failed={summary.failed_paths} "
                f"mean_raw_rms_xy_cm={summary.mean_raw_rms_xy_cm} "
                f"mean_fused_rms_xy_cm={summary.mean_fused_rms_xy_cm} "
                f"mean_improvement_rms_cm={summary.mean_improvement_rms_cm} "
                f"max_fused_rms_xy_cm={summary.max_fused_rms_xy_cm}"
            )
            for row in rows:
                print(
                    f"{row.path}: raw={row.raw_rms_xy_cm} fused={row.fused_rms_xy_cm} "
                    f"improve={row.improvement_rms_cm} benchmark_passed={row.benchmark_passed} "
                    f"reason={row.benchmark_reason} validation_passed={row.validation_passed} "
                    f"dt35_fusion_frames={row.dt35_fusion_allowed_frames} ranks={row.dt35_rank_counts_json}"
                )
            return 0

        if args.synthetic_obstacle_ablation:
            path_text = args.synthetic_obstacle_paths
            paths = [item.strip() for item in str(path_text).split(",") if item.strip()] if path_text else list(DEFAULT_OBSTACLE_PATHS)
            fusion_cfg = FusionConfig(
                lidar_stride=max(1, int(args.lidar_stride)),
                lidar_gain=float(args.fusion_lidar_gain),
                dt35_gain=float(args.fusion_dt35_gain),
                dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
                dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
            )
            rows, summary = run_obstacle_ablation(
                config,
                paths,
                samples=max(2, int(args.synthetic_samples)),
                encoder_x_scale=float(args.synthetic_encoder_x_scale),
                encoder_y_scale=float(args.synthetic_encoder_y_scale),
                encoder_yaw_scale=float(args.synthetic_encoder_yaw_scale),
                h30_yaw_bias_deg=float(args.synthetic_h30_yaw_bias),
                dt35_noise_mm=float(args.synthetic_dt35_noise_mm),
                fusion_cfg=fusion_cfg,
            )
            output_dir = root / "logs" / f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_obstacle_ablation"
            output_dir.mkdir(parents=True, exist_ok=True)
            csv_path = Path(args.synthetic_obstacle_ablation_output) if args.synthetic_obstacle_ablation_output else output_dir / "obstacle_ablation.csv"
            summary_path = Path(args.synthetic_obstacle_ablation_summary) if args.synthetic_obstacle_ablation_summary else output_dir / "obstacle_ablation_summary.json"
            write_obstacle_ablation_csv(csv_path, rows)
            write_obstacle_ablation_summary(summary_path, summary)
            print(f"output={csv_path}")
            print(f"summary={summary_path}")
            print(
                f"paths={summary.paths} passed={summary.passed_paths} failed={summary.failed_paths} "
                f"solid_rays={summary.total_full_solid_fusion_allowed_rays} "
                f"forest_rays={summary.total_full_forest_fusion_allowed_rays} "
                f"ramp_rays={summary.total_full_ramp_fusion_allowed_rays} "
                f"mean_full_fused={summary.mean_full_fused_rms_xy_cm} "
                f"mean_ablated_fused={summary.mean_ablated_fused_rms_xy_cm} "
                f"mean_penalty={summary.mean_ablation_penalty_cm}"
            )
            for row in rows:
                print(
                    f"{row.path}: full={row.full_fused_rms_xy_cm} ablated={row.ablated_fused_rms_xy_cm} "
                    f"penalty={row.ablation_penalty_cm} solid_rays={row.full_solid_fusion_allowed_rays} "
                    f"forest={row.full_forest_fusion_allowed_rays} ramp={row.full_ramp_fusion_allowed_rays} "
                    f"passed={row.ablation_passed}"
                )
            return 0

        if args.synthetic_path_report:
            synthetic_cfg = SyntheticConfig(
                samples=max(2, int(args.synthetic_samples)),
                path_name=str(args.synthetic_path),
                encoder_x_scale=float(args.synthetic_encoder_x_scale),
                encoder_y_scale=float(args.synthetic_encoder_y_scale),
                encoder_yaw_scale=float(args.synthetic_encoder_yaw_scale),
                h30_yaw_bias_deg=float(args.synthetic_h30_yaw_bias),
                dt35_noise_mm=float(args.synthetic_dt35_noise_mm),
            )
            fusion_cfg = FusionConfig(
                lidar_stride=max(1, int(args.lidar_stride)),
                lidar_gain=float(args.fusion_lidar_gain),
                dt35_gain=float(args.fusion_dt35_gain),
                dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
                dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
            )
            rows, summary, _fused_frames = generate_synthetic_path_diagnostic(config, synthetic_cfg, fusion_cfg)
            output_dir = root / "logs" / f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_path_diag_{synthetic_cfg.path_name}"
            output_dir.mkdir(parents=True, exist_ok=True)
            csv_path = Path(args.synthetic_path_report_output) if args.synthetic_path_report_output else output_dir / "path_diagnostic.csv"
            summary_path = Path(args.synthetic_path_report_summary) if args.synthetic_path_report_summary else output_dir / "path_diagnostic_summary.json"
            write_path_diagnostic_csv(csv_path, rows)
            write_path_diagnostic_summary(summary_path, summary)
            print(f"output={csv_path}")
            print(f"summary={summary_path}")
            print(
                f"frames={summary.frames} raw_rms_xy_cm={summary.raw_rms_xy_cm} "
                f"no_dt35_rms_xy_cm={summary.no_dt35_rms_xy_cm} "
                f"fused_rms_xy_cm={summary.fused_rms_xy_cm} improved={summary.improved_frames} "
                f"worsened={summary.worsened_frames}"
            )
            print(
                f"dt35_active_frames={summary.dt35_active_frames} helped={summary.dt35_helped_frames} "
                f"dt35_worsened={summary.dt35_worsened_frames} "
                f"mean_correction_cm={summary.dt35_mean_correction_cm} "
                f"max_correction_cm={summary.dt35_max_correction_cm}"
            )
            print(
                f"dt35_valid_frames={summary.dt35_valid_frames} allowed_frames={summary.dt35_allowed_frames} "
                f"fusion_allowed_frames={summary.dt35_fusion_allowed_frames} "
                f"gate_rejected_frames={summary.dt35_residual_gate_rejected_frames} "
                f"corner_frames={summary.dt35_corner_frames} types={summary.dt35_type_counts}"
            )
            return 0

        if args.synthetic_monte_carlo:
            output_dir = root / "logs" / f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_synthetic_monte_carlo"
            output_dir.mkdir(parents=True, exist_ok=True)
            gains = [float(item.strip()) for item in str(args.synthetic_dt35_gains).split(",") if item.strip()]
            rows = []
            for run_index in range(max(1, int(args.synthetic_monte_carlo_runs))):
                seed = int(args.synthetic_seed_base) + run_index
                frames = generate_synthetic_frames(
                    config,
                    SyntheticConfig(
                        samples=max(2, int(args.synthetic_samples)),
                        path_name="random_patrol",
                        encoder_x_scale=float(args.synthetic_encoder_x_scale),
                        encoder_y_scale=float(args.synthetic_encoder_y_scale),
                        encoder_yaw_scale=float(args.synthetic_encoder_yaw_scale),
                        h30_yaw_bias_deg=float(args.synthetic_h30_yaw_bias),
                        dt35_noise_mm=float(args.synthetic_dt35_noise_mm),
                        seed=seed,
                    ),
                )
                for gain in gains:
                    sim_cfg = FusionConfig(
                        lidar_stride=max(1, int(args.lidar_stride)),
                        lidar_gain=float(args.fusion_lidar_gain),
                        dt35_gain=gain,
                        dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
                        dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
                    )
                    result = simulate_fusion(frames, sim_cfg, config)
                    row = result.metrics.to_dict()
                    row.update({"seed": seed, "dt35_gain": gain, "lidar_stride": sim_cfg.lidar_stride})
                    rows.append(row)
            output = output_dir / "synthetic_monte_carlo.json"
            output.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"output={output}")
            print("dt35_gain,runs,avg_rms_xy_cm,max_rms_xy_cm,avg_rms_yaw_deg,avg_max_xy_cm")
            for gain in gains:
                gain_rows = [row for row in rows if row["dt35_gain"] == gain]
                rms_xy = [float(row["rms_xy_cm"]) for row in gain_rows if row["rms_xy_cm"] is not None]
                rms_yaw = [float(row["rms_yaw_deg"]) for row in gain_rows if row["rms_yaw_deg"] is not None]
                max_xy = [float(row["max_xy_cm"]) for row in gain_rows if row["max_xy_cm"] is not None]
                avg_rms = sum(rms_xy) / len(rms_xy) if rms_xy else None
                max_rms = max(rms_xy) if rms_xy else None
                avg_yaw = sum(rms_yaw) / len(rms_yaw) if rms_yaw else None
                avg_max_xy = sum(max_xy) / len(max_xy) if max_xy else None
                print(f"{gain},{len(gain_rows)},{avg_rms},{max_rms},{avg_yaw},{avg_max_xy}")
            return 0

        if args.synthetic_suite:
            output_dir = root / "logs" / f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_synthetic_suite"
            output_dir.mkdir(parents=True, exist_ok=True)
            paths = [item.strip() for item in str(args.synthetic_suite_paths).split(",") if item.strip()]
            gains = [float(item.strip()) for item in str(args.synthetic_dt35_gains).split(",") if item.strip()]
            rows = []
            for path_name in paths:
                frames = generate_synthetic_frames(
                    config,
                    SyntheticConfig(
                        samples=max(2, int(args.synthetic_samples)),
                        path_name=path_name,
                        encoder_x_scale=float(args.synthetic_encoder_x_scale),
                        encoder_y_scale=float(args.synthetic_encoder_y_scale),
                        encoder_yaw_scale=float(args.synthetic_encoder_yaw_scale),
                        h30_yaw_bias_deg=float(args.synthetic_h30_yaw_bias),
                        dt35_noise_mm=float(args.synthetic_dt35_noise_mm),
                    ),
                )
                for gain in gains:
                    sim_cfg = FusionConfig(
                        lidar_stride=max(1, int(args.lidar_stride)),
                        lidar_gain=float(args.fusion_lidar_gain),
                        dt35_gain=gain,
                        dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
                        dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
                    )
                    result = simulate_fusion(frames, sim_cfg, config)
                    row = result.metrics.to_dict()
                    row.update({"path": path_name, "dt35_gain": gain, "lidar_stride": sim_cfg.lidar_stride})
                    rows.append(row)
            output = output_dir / "synthetic_suite.json"
            output.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"output={output}")
            print("path,dt35_gain,lidar_stride,holdout,rms_xy_cm,rms_yaw_deg,max_xy_cm")
            for row in rows:
                print(f"{row['path']},{row['dt35_gain']},{row['lidar_stride']},{row['lidar_holdout_frames']},"
                      f"{row['rms_xy_cm']},{row['rms_yaw_deg']},{row['max_xy_cm']}")
            return 0

        synthetic_cfg = SyntheticConfig(
            samples=max(2, int(args.synthetic_samples)),
            path_name=str(args.synthetic_path),
            encoder_x_scale=float(args.synthetic_encoder_x_scale),
            encoder_y_scale=float(args.synthetic_encoder_y_scale),
            encoder_yaw_scale=float(args.synthetic_encoder_yaw_scale),
            h30_yaw_bias_deg=float(args.synthetic_h30_yaw_bias),
            dt35_noise_mm=float(args.synthetic_dt35_noise_mm),
        )
        frames = generate_synthetic_frames(config, synthetic_cfg)
        output_dir = root / "logs" / f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_synthetic_{synthetic_cfg.path_name}"
        output_dir.mkdir(parents=True, exist_ok=True)
        truth_path = output_dir / "synthetic_truth.csv"
        write_frames_csv(truth_path, frames)
        sim_cfg = FusionConfig(
            lidar_stride=max(1, int(args.lidar_stride)),
            lidar_gain=float(args.fusion_lidar_gain),
            dt35_gain=float(args.fusion_dt35_gain),
            dt35_yaw_gain=float(args.fusion_dt35_yaw_gain),
            dt35_correct_lidar_frames=_dt35_correct_lidar_frames(args, config),
        )
        result = simulate_fusion(frames, sim_cfg, config)
        fused_path = output_dir / "synthetic_fused.csv"
        write_frames_csv(fused_path, result.frames)
        metrics_path = output_dir / "synthetic_fused.metrics.json"
        write_metrics_json(metrics_path, result.metrics, sim_cfg)
        print(f"output_dir={output_dir}")
        print(f"truth={truth_path}")
        print(f"fused={fused_path}")
        print(f"metrics={metrics_path}")
        print(f"frames={result.metrics.frames} lidar_used={result.metrics.lidar_used_frames} holdout={result.metrics.lidar_holdout_frames}")
        print(f"rms_xy_cm={result.metrics.rms_xy_cm} rms_yaw_deg={result.metrics.rms_yaw_deg} max_xy_cm={result.metrics.max_xy_cm}")
        return 0

    from locater_map.app import run_app

    return run_app(
        config_path=args.config,
        demo=args.demo,
        replay_path=args.replay,
        serial_port=args.serial_port,
        baudrate=args.baudrate,
        duration_s=args.duration_s,
        screenshot_path=args.screenshot,
        capture_on_start=bool(args.capture),
    )


if __name__ == "__main__":
    raise SystemExit(main())
