from __future__ import annotations

import json
import math
import shutil
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
PROMO_DIR = ROOT / "docs" / "promotion"
TMP_DIR = ROOT / ".skill-evals" / "product-promotion-run" / "media"
ASSETS_DIR = ROOT / "locater_map" / "assets"
SIM_LOG_DIR = ROOT / "locater_map" / "logs" / "RL_data" / "20260623_coord_fixed_four_stages" / "ideal_log" / "png"

W, H = 1920, 1080
BG = (9, 14, 22)
PANEL = (16, 24, 36)
PANEL_2 = (22, 35, 51)
TEXT = (232, 240, 248)
MUTED = (150, 163, 181)
CYAN = (39, 220, 255)
GREEN = (43, 226, 141)
RED = (255, 82, 96)
YELLOW = (255, 194, 64)


def run(cmd: list[str]) -> None:
    print(" ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        Path("C:/Windows/Fonts/msyhbd.ttc") if bold else Path("C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/simhei.ttf"),
        Path("C:/Windows/Fonts/arialbd.ttf") if bold else Path("C:/Windows/Fonts/arial.ttf"),
    ]
    for item in candidates:
        if item.exists():
            return ImageFont.truetype(str(item), size)
    return ImageFont.load_default()


F_TITLE = font(72, True)
F_SUBTITLE = font(36, True)
F_H2 = font(42, True)
F_BODY = font(28)
F_BODY_BOLD = font(28, True)
F_SMALL = font(22)


def round_rect(draw: ImageDraw.ImageDraw, xy: tuple[int, int, int, int], radius: int, fill, outline=None, width=1) -> None:
    draw.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline, width=width)


def fit_image(img: Image.Image, box: tuple[int, int], cover: bool = False) -> Image.Image:
    bw, bh = box
    iw, ih = img.size
    scale = max(bw / iw, bh / ih) if cover else min(bw / iw, bh / ih)
    nw, nh = max(1, int(iw * scale)), max(1, int(ih * scale))
    out = img.resize((nw, nh), Image.Resampling.LANCZOS)
    if cover:
        left = max(0, (nw - bw) // 2)
        top = max(0, (nh - bh) // 2)
        out = out.crop((left, top, left + bw, top + bh))
    return out


def paste_fit(canvas: Image.Image, img_path: Path, xy: tuple[int, int, int, int], cover: bool = False) -> None:
    img = Image.open(img_path).convert("RGB")
    x1, y1, x2, y2 = xy
    fitted = fit_image(img, (x2 - x1, y2 - y1), cover=cover)
    px = x1 + (x2 - x1 - fitted.width) // 2
    py = y1 + (y2 - y1 - fitted.height) // 2
    canvas.paste(fitted, (px, py))


def draw_wrapped(draw: ImageDraw.ImageDraw, text: str, xy: tuple[int, int], max_width: int, line_gap: int = 8, fnt=F_BODY, fill=TEXT) -> int:
    x, y = xy
    line = ""
    for ch in text:
        candidate = line + ch
        if draw.textlength(candidate, font=fnt) <= max_width or not line:
            line = candidate
            continue
        draw.text((x, y), line, font=fnt, fill=fill)
        y += fnt.size + line_gap
        line = ch
    if line:
        draw.text((x, y), line, font=fnt, fill=fill)
        y += fnt.size + line_gap
    return y


def draw_badge(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, color) -> None:
    x, y = xy
    pad_x = 18
    pad_y = 9
    tw = int(draw.textlength(text, font=F_SMALL))
    round_rect(draw, (x, y, x + tw + pad_x * 2, y + F_SMALL.size + pad_y * 2), 18, (*color[:3],), outline=None)
    draw.text((x + pad_x, y + pad_y - 2), text, font=F_SMALL, fill=(5, 9, 15))


def base_canvas() -> tuple[Image.Image, ImageDraw.ImageDraw]:
    canvas = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(canvas)
    for i in range(0, W, 80):
        alpha = int(14 + 18 * math.sin(i / 160))
        draw.line((i, 0, i - 360, H), fill=(10, 35, 55 + alpha), width=1)
    return canvas, draw


def make_poster() -> Path:
    out = PROMO_DIR / "r1-locaterv2-poster.png"
    ui = PROMO_DIR / "r1-locaterv2-ui-demo.png"
    field = ASSETS_DIR / "field_prior_map_clean_labeled_1215x1210cm.png"
    chassis = ASSETS_DIR / "r1_chassis_830mm_texture_1024.png"
    overlay = ROOT / "locater_map" / "logs" / "RL_data" / "20260623_coord_fixed_four_stages" / "ideal_log" / "png" / "overview.png"

    canvas, draw = base_canvas()
    round_rect(draw, (60, 70, 1210, 1010), 26, PANEL, outline=(45, 66, 91), width=2)
    if ui.exists():
        paste_fit(canvas, ui, (90, 105, 1180, 980), cover=True)
    elif field.exists():
        paste_fit(canvas, field, (110, 130, 1160, 960), cover=False)

    round_rect(draw, (1260, 70, 1860, 1010), 26, PANEL_2, outline=(45, 66, 91), width=2)
    draw.text((1310, 120), "R1_LocaterV2", font=F_TITLE, fill=TEXT)
    draw.text((1314, 208), "STM32G4 多传感器定位板", font=F_SUBTITLE, fill=CYAN)
    draw_wrapped(
        draw,
        "H30 yaw、双正交编码轮、Lidar 绝对位姿、双 DT35 测距与 PySide6 实时地图上位机对齐到同一套定位调试闭环。",
        (1314, 275),
        500,
        fnt=F_BODY,
        fill=TEXT,
    )
    draw_badge(draw, (1314, 415), "real2sim", GREEN)
    draw_badge(draw, (1454, 415), "UART/VOFA", CYAN)
    draw_badge(draw, (1618, 415), "DT35 raycast", YELLOW)
    y = 505
    bullets = [
        "CSV/二进制协议同时服务上位机与底盘主控",
        "地图坐标、底盘贴图、DT35 墙体模型按 cm 精确建模",
        "支持日志采集、回放、截图序列和离线融合评估",
    ]
    for item in bullets:
        draw.ellipse((1320, y + 8, 1332, y + 20), fill=GREEN)
        y = draw_wrapped(draw, item, (1350, y), 470, fnt=F_BODY, fill=TEXT) + 6
    draw.text((1320, 720), "实车链路 + 地图模型", font=F_BODY_BOLD, fill=YELLOW)
    if chassis.exists():
        paste_fit(canvas, chassis, (1320, 780, 1535, 980), cover=False)
    if overlay.exists():
        paste_fit(canvas, overlay, (1580, 780, 1820, 980), cover=False)
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out, quality=95)
    return out


def make_card(path: Path, title: str, subtitle: str, body: list[str], image: Path | None = None) -> Path:
    canvas, draw = base_canvas()
    draw.text((90, 95), title, font=F_TITLE, fill=TEXT)
    draw.text((94, 186), subtitle, font=F_SUBTITLE, fill=CYAN)
    y = 285
    for item in body:
        draw.rectangle((96, y + 10, 120, y + 34), fill=GREEN)
        y = draw_wrapped(draw, item, (145, y), 760, fnt=F_BODY, fill=TEXT) + 16
    if image and image.exists():
        round_rect(draw, (990, 120, 1830, 960), 24, PANEL, outline=(52, 77, 105), width=2)
        paste_fit(canvas, image, (1020, 155, 1800, 930), cover=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path, quality=95)
    return path


def media_probe_duration(path: Path) -> float:
    data = subprocess.check_output(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-of",
            "json",
            str(path),
        ],
        cwd=ROOT,
        text=True,
    )
    return float(json.loads(data)["format"]["duration"])


def encode_still(image: Path, duration: float, out: Path) -> None:
    run(
        [
            "ffmpeg",
            "-y",
            "-loop",
            "1",
            "-t",
            f"{duration:.2f}",
            "-i",
            str(image),
            "-vf",
            "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2,format=yuv420p",
            "-r",
            "30",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "20",
            str(out),
        ]
    )


def encode_real_clip(src: Path, out: Path) -> None:
    duration = min(18.0, media_probe_duration(src))
    run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(src),
            "-t",
            f"{duration:.2f}",
            "-vf",
            "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2,format=yuv420p",
            "-r",
            "30",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "21",
            str(out),
        ]
    )


def encode_sim_clip(out: Path) -> Path | None:
    if not SIM_LOG_DIR.exists():
        return None
    frames = sorted(SIM_LOG_DIR.glob("t_*.png"))
    if len(frames) < 8:
        return None
    tmp_frames = TMP_DIR / "sim_frames"
    if tmp_frames.exists():
        shutil.rmtree(tmp_frames)
    tmp_frames.mkdir(parents=True, exist_ok=True)
    selected = frames[:: max(1, len(frames) // 80)]
    for i, frame in enumerate(selected):
        shutil.copy2(frame, tmp_frames / f"{i:04d}.png")
    run(
        [
            "ffmpeg",
            "-y",
            "-framerate",
            "8",
            "-i",
            str(tmp_frames / "%04d.png"),
            "-vf",
            "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2,format=yuv420p",
            "-r",
            "30",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "20",
            str(out),
        ]
    )
    return out


def convert_real_video() -> tuple[Path, Path]:
    src = ASSETS_DIR / "实车.mp4"
    h264 = PROMO_DIR / "r1-locaterv2-real-car-h264.mp4"
    frame = PROMO_DIR / "r1-locaterv2-real-car-frame.png"
    if not src.exists():
        raise FileNotFoundError(src)
    run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(src),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-preset",
            "medium",
            "-crf",
            "21",
            "-c:a",
            "aac",
            "-b:a",
            "128k",
            str(h264),
        ]
    )
    run(["ffmpeg", "-y", "-ss", "00:00:08", "-i", str(src), "-frames:v", "1", str(frame)])
    return h264, frame


def make_demo_video() -> tuple[Path, Path]:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    poster = make_poster()
    h264, real_frame = convert_real_video()
    field_model_overview = ROOT / "locater_map" / "logs" / "RL_data" / "20260623_coord_fixed_four_stages" / "ideal_log" / "png" / "overview.png"
    if field_model_overview.exists():
        shutil.copy2(field_model_overview, PROMO_DIR / "r1-locaterv2-field-model-overview.png")
    card_pipeline = make_card(
        PROMO_DIR / "r1-locaterv2-card-pipeline.png",
        "传感器到坐标的闭环",
        "用实车数据校正仿真，用仿真约束实车误差",
        [
            "Lidar 给出启动姿态下的局部零点与相对位姿。",
            "H30 MINI 提供稳定 yaw，修正 DT35 射线方向和编码轮坐标变换。",
            "双正交编码轮负责高频位移，DT35 负责墙体距离约束与异常筛选。",
            "上位机记录 raw 串口、结构化帧和 2D 截图，支撑后续算法迭代。",
        ],
        PROMO_DIR / "r1-locaterv2-field-model-overview.png",
    )
    card_real = make_card(
        PROMO_DIR / "r1-locaterv2-card-real.png",
        "实车验证链路",
        "无线串口 + 上位机实时地图",
        [
            "USART1 输出 VOFA 兼容 CSV，同时供 PySide6 上位机解析。",
            "USART2 面向底盘主控发送定位数据帧。",
            "采集日志按时间命名，传感器数据与地图截图对齐保存。",
        ],
        real_frame,
    )
    card_outro = make_card(
        PROMO_DIR / "r1-locaterv2-card-outro.png",
        "开源定位调试工具链",
        "R1_LocaterV2 / STM32G4 / PySide6",
        [
            "仓库提供固件、上位机、地图模型、回放与数据分析工具。",
            "适合机器人定位调试、传感器融合验证和赛场 real2sim 复盘。",
            "感谢李彦彦、王晨宇、李岳林、马克提供技术支持。",
        ],
        ASSETS_DIR / "r1_chassis_830mm_texture_1024.png",
    )

    segments: list[Path] = []
    for idx, (image, duration) in enumerate([(poster, 5), (card_pipeline, 8), (card_real, 6)]):
        seg = TMP_DIR / f"seg_{idx:02d}.mp4"
        encode_still(image, float(duration), seg)
        segments.append(seg)
    real_seg = TMP_DIR / "seg_real_car.mp4"
    encode_real_clip(h264, real_seg)
    segments.append(real_seg)
    sim_seg = PROMO_DIR / "r1-locaterv2-sim-preview.mp4"
    if encode_sim_clip(sim_seg):
        segments.append(sim_seg)
    outro_seg = TMP_DIR / "seg_outro.mp4"
    encode_still(card_outro, 7.0, outro_seg)
    segments.append(outro_seg)

    concat_file = TMP_DIR / "concat.txt"
    concat_file.write_text("".join(f"file '{p.as_posix()}'\n" for p in segments), encoding="utf-8")
    demo = PROMO_DIR / "r1-locaterv2-demo.mp4"
    run(["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(concat_file), "-c", "copy", str(demo)])

    gif = PROMO_DIR / "r1-locaterv2-demo-teaser.gif"
    run(
        [
            "ffmpeg",
            "-y",
            "-ss",
            "00:00:05",
            "-t",
            "00:00:10",
            "-i",
            str(demo),
            "-vf",
            "fps=12,scale=960:-1:flags=lanczos",
            str(gif),
        ]
    )
    return demo, gif


def main() -> None:
    PROMO_DIR.mkdir(parents=True, exist_ok=True)
    demo, gif = make_demo_video()
    print(f"generated: {demo}")
    print(f"generated: {gif}")


if __name__ == "__main__":
    main()
