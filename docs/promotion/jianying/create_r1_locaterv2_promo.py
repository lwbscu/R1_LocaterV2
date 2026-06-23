from __future__ import annotations

import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROMO_DIR = ROOT / "docs" / "promotion"
DEFAULT_SKILL_ROOT = Path.home() / ".codex" / "skills" / "jianying-editor"
SKILL_ROOT = Path(os.environ.get("JY_SKILL_ROOT", DEFAULT_SKILL_ROOT))
SCRIPTS_DIR = SKILL_ROOT / "scripts"

if not SCRIPTS_DIR.exists():
    raise SystemExit(f"Jianying skill scripts not found: {SCRIPTS_DIR}")

sys.path.insert(0, str(SCRIPTS_DIR))

from jy_wrapper import JyProject  # noqa: E402


def require(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(path)
    return str(path)


def main() -> None:
    project = JyProject(project_name="R1_LocaterV2_Promo", overwrite=True, width=1920, height=1080)

    timeline = [
        ("r1-locaterv2-poster.png", "0s", "5s", "项目封面"),
        ("r1-locaterv2-card-pipeline.png", "5s", "8s", "融合链路"),
        ("r1-locaterv2-card-real.png", "13s", "6s", "实车链路"),
        ("r1-locaterv2-real-car-h264.mp4", "19s", "18s", "实车视频"),
        ("r1-locaterv2-sim-preview.mp4", "37s", "8s", "仿真回放"),
        ("r1-locaterv2-card-outro.png", "45s", "7s", "结尾"),
    ]

    for filename, start, duration, label in timeline:
        project.add_media_safe(require(PROMO_DIR / filename), start_time=start, duration=duration, track_name="MainVideo")
        print(f"added {label}: {filename} @ {start} for {duration}")

    captions = [
        ("R1_LocaterV2: STM32G4 多传感器定位板", "0.4s", "2.8s"),
        ("Lidar 绝对位姿 + H30 yaw + 编码轮高频位移 + DT35 墙体约束", "5.8s", "4.2s"),
        ("上位机同时支持实时串口、日志采集、地图回放和截图序列", "13.6s", "3.8s"),
        ("实车视频：传感器链路与无线串口闭环验证", "20s", "5s"),
        ("real2sim: 用仿真筛选 DT35 置信度，再反推实车误差", "38s", "4.8s"),
        ("GitHub: lwbscu/R1_LocaterV2", "47s", "4s"),
    ]
    for text, start, duration in captions:
        project.add_text_simple(text, start_time=start, duration=duration, track_name="Captions")
        print(f"caption @ {start}: {text}")

    project.save()
    print("Jianying draft created: R1_LocaterV2_Promo")


if __name__ == "__main__":
    main()
