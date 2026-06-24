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
DOCS_DIR = ROOT / "docs"
PANGOLIN_LOGO = DOCS_DIR / "Pangolin.png"
R1_IP_IMAGE = DOCS_DIR / "R1.png"
SIM_STAGE_ROOT = ROOT / "locater_map" / "logs" / "RL_data" / "20260623_coord_fixed_four_stages"
SIM_LOG_DIR = SIM_STAGE_ROOT / "ideal_log" / "png"
SIM_STAGE_CLIPS = [
    ("ideal_log", "r1-locaterv2-sim-stage-ideal.mp4"),
    ("pid_log", "r1-locaterv2-sim-stage-pid.mp4"),
    ("async_occlusion_log", "r1-locaterv2-sim-stage-async-occlusion.mp4"),
    ("lidar_noise_log", "r1-locaterv2-sim-stage-lidar-noise.mp4"),
]
SIM_SOURCE_FPS = "1"
SIM_DISPLAY_FPS = "10"
SIM_FILTER = (
    "scale=1280:720:force_original_aspect_ratio=decrease,"
    "pad=1280:720:(ow-iw)/2:(oh-ih)/2,"
    f"framerate=fps={SIM_DISPLAY_FPS}:interp_start=0:interp_end=255:scene=100,"
    "format=yuv420p"
)

W, H = 1920, 1080
POSTER_W, POSTER_H = 2160, 1440
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
F_POSTER_TITLE = font(64, True)


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
    img = Image.open(img_path).convert("RGBA")
    x1, y1, x2, y2 = xy
    fitted = fit_image(img, (x2 - x1, y2 - y1), cover=cover)
    px = x1 + (x2 - x1 - fitted.width) // 2
    py = y1 + (y2 - y1 - fitted.height) // 2
    canvas.paste(fitted.convert("RGB"), (px, py), fitted)


def make_start_map_composite() -> Path:
    out = PROMO_DIR / "r1-locaterv2-start-map.png"
    field_path = ASSETS_DIR / "field_prior_map_clean_labeled_1215x1210cm.png"
    chassis_path = ASSETS_DIR / "r1_chassis_830mm_texture_1024.png"
    field = Image.open(field_path).convert("RGBA")
    if chassis_path.exists():
        chassis = Image.open(chassis_path).convert("RGBA")
        # Field assets are 2 px/cm. R1 chassis is 83 cm x 83 cm.
        chassis = fit_image(chassis, (166, 166), cover=False).convert("RGBA")
        # Red start area is at the upper-left field corner; place the robot center
        # in that start square instead of reusing a live screenshot with off-map pose.
        start_center_px = (108, 108)
        field.alpha_composite(chassis, (start_center_px[0] - chassis.width // 2, start_center_px[1] - chassis.height // 2))
        draw = ImageDraw.Draw(field)
        draw.ellipse(
            (
                start_center_px[0] - 8,
                start_center_px[1] - 8,
                start_center_px[0] + 8,
                start_center_px[1] + 8,
            ),
            outline=(39, 220, 255, 255),
            width=4,
        )
    out.parent.mkdir(parents=True, exist_ok=True)
    field.convert("RGB").save(out, quality=95)
    return out


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


def base_canvas(width: int = W, height: int = H) -> tuple[Image.Image, ImageDraw.ImageDraw]:
    canvas = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(canvas)
    for i in range(0, width, 80):
        alpha = int(14 + 18 * math.sin(i / 160))
        draw.line((i, 0, i - 360, height), fill=(10, 35, 55 + alpha), width=1)
    return canvas, draw


def make_poster() -> Path:
    out = PROMO_DIR / "r1-locaterv2-poster.png"
    field = ASSETS_DIR / "field_prior_map_clean_labeled_1215x1210cm.png"
    start_map = make_start_map_composite() if field.exists() else None
    def paste_media(img_path: Path, box: tuple[int, int, int, int], cover: bool = True, crop: tuple[int, int, int, int] | None = None) -> None:
        if not img_path.exists():
            return
        img = Image.open(img_path).convert("RGB")
        if crop is not None:
            img = img.crop(crop)
        x1, y1, x2, y2 = box
        fitted = fit_image(img, (x2 - x1, y2 - y1), cover=cover)
        px = x1 + (x2 - x1 - fitted.width) // 2
        py = y1 + (y2 - y1 - fitted.height) // 2
        canvas.paste(fitted, (px, py))

    canvas, draw = base_canvas(POSTER_W, POSTER_H)
    round_rect(draw, (70, 70, 2090, 1370), 32, PANEL_2, outline=(45, 66, 91), width=2)
    draw.text((130, 125), "R1_LocaterV2", font=F_TITLE, fill=TEXT)
    draw.text((135, 235), "STM32G4 多传感器定位板", font=F_SUBTITLE, fill=CYAN)
    draw_wrapped(
        draw,
        "H30 yaw、双正交编码轮、Lidar 启动局部位姿、双 DT35 测距与 PySide6 实时地图上位机对齐到同一套定位调试闭环。",
        (135, 335),
        760,
        fnt=F_BODY,
        fill=TEXT,
    )
    draw_badge(draw, (135, 495), "里程计", GREEN)
    draw_badge(draw, (270, 495), "多传感器融合", CYAN)
    draw_badge(draw, (520, 495), "RLHF 数据闭环", YELLOW)
    y = 600
    bullets = [
        "双正交编码轮里程计与 H30 yaw 提供高频局部位姿约束",
        "Lidar、DT35、里程计多源观测在同一地图坐标系下融合校验",
        "采集、回放、仿真和人工反馈形成 real2sim / RLHF 迭代闭环",
    ]
    for item in bullets:
        draw.ellipse((145, y + 8, 157, y + 20), fill=GREEN)
        y = draw_wrapped(draw, item, (177, y), 760, fnt=F_BODY, fill=TEXT) + 8

    brand_cards = [
        ((135, 960, 382, 1255), PANGOLIN_LOGO, "Pangolin 团队标志"),
        ((430, 960, 677, 1255), R1_IP_IMAGE, "R1 IP 形象"),
    ]
    for box, img_path, label in brand_cards:
        x1, y1, x2, y2 = box
        round_rect(draw, box, 22, (12, 18, 27), outline=(54, 79, 108), width=2)
        image_box = (x1 + 24, y1 + 24, x2 - 24, y2 - 72)
        if img_path.exists():
            paste_fit(canvas, img_path, image_box, cover=False)
        draw.text((x1 + 24, y2 - 48), label, font=F_SMALL, fill=TEXT)

    media_frame = (1010, 145, 2030, 1295)
    round_rect(draw, media_frame, 22, (8, 13, 20), outline=(55, 80, 110), width=1)
    if start_map and start_map.exists():
        paste_media(start_map, (1045, 180, 1995, 1235), cover=False)
    elif field.exists():
        paste_media(field, (1045, 180, 1995, 1235), cover=False)
    draw.text((1045, 1245), "起点 x=0, y=0, yaw=0 / +X 正东 / +Y 正北", font=F_SMALL, fill=YELLOW)
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


def make_ack_card(path: Path) -> Path:
    canvas, draw = base_canvas()
    draw.text((90, 80), "致谢", font=F_TITLE, fill=TEXT)

    body = [
        "技术总负责：@lwbscu",
        "电控技术支持：@Thomaswang2005 / @HIRAMHC111",
        "lidar 技术支持：@Getting05 / @qyw23AI",
        "硬件芯片设计：@twenty-fourabc / @2718487561-a11y / @wancyu",
        "机械结构设计：@mmkk-cn",
    ]
    y = 215
    for item in body:
        draw.rectangle((96, y + 12, 120, y + 36), fill=GREEN)
        draw.text((145, y), item, font=F_BODY, fill=TEXT)
        y += 72

    cards = [
        ((420, 640, 850, 990), PANGOLIN_LOGO, "Pangolin 团队标志"),
        ((1070, 640, 1500, 990), R1_IP_IMAGE, "R1 IP 形象"),
    ]
    for box, image, label in cards:
        x1, y1, x2, y2 = box
        round_rect(draw, box, 24, PANEL, outline=(52, 77, 105), width=2)
        if image.exists():
            paste_fit(canvas, image, (x1 + 42, y1 + 36, x2 - 42, y2 - 95), cover=False)
        label_width = int(draw.textlength(label, font=F_BODY))
        draw.text((x1 + (x2 - x1 - label_width) // 2, y2 - 66), label, font=F_BODY, fill=TEXT)
        draw.line((x1 + 42, y1 + 28, x1 + 165, y1 + 28), fill=CYAN if image == R1_IP_IMAGE else GREEN, width=8)

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
            "scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2,format=yuv420p",
            "-r",
            "24",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "29",
            "-movflags",
            "+faststart",
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
            "scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2,format=yuv420p",
            "-r",
            "24",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "29",
            "-movflags",
            "+faststart",
            str(out),
        ]
    )


def encode_video_excerpt(src: Path, out: Path, start_s: float, duration_s: float) -> None:
    run(
        [
            "ffmpeg",
            "-y",
            "-ss",
            f"{start_s:.2f}",
            "-i",
            str(src),
            "-t",
            f"{duration_s:.2f}",
            "-vf",
            "scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2,format=yuv420p",
            "-r",
            "24",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "29",
            "-movflags",
            "+faststart",
            str(out),
        ]
    )


def encode_sim_clip(out: Path, png_dir: Path | None = None) -> Path | None:
    source_dir = png_dir or SIM_LOG_DIR
    if not source_dir.exists():
        return None
    frames = sorted(source_dir.glob("t_*.png"))
    if len(frames) < 8:
        return None
    tmp_frames = TMP_DIR / f"sim_frames_{out.stem}"
    if tmp_frames.exists():
        shutil.rmtree(tmp_frames)
    tmp_frames.mkdir(parents=True, exist_ok=True)
    for i, frame in enumerate(frames):
        shutil.copy2(frame, tmp_frames / f"{i:04d}.png")
    run(
        [
            "ffmpeg",
            "-y",
            "-framerate",
            SIM_SOURCE_FPS,
            "-i",
            str(tmp_frames / "%04d.png"),
            "-vf",
            SIM_FILTER,
            "-r",
            SIM_DISPLAY_FPS,
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "28",
            "-movflags",
            "+faststart",
            str(out),
        ]
    )
    return out


def encode_sim_stage_clips() -> list[Path]:
    outputs: list[Path] = []
    for log_name, filename in SIM_STAGE_CLIPS:
        out = PROMO_DIR / filename
        clip = encode_sim_clip(out, SIM_STAGE_ROOT / log_name / "png")
        if clip is not None:
            outputs.append(clip)
    return outputs


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
            "28",
            "-vf",
            "scale=960:540:force_original_aspect_ratio=decrease,pad=960:540:(ow-iw)/2:(oh-ih)/2,format=yuv420p",
            "-r",
            "24",
            "-an",
            "-movflags",
            "+faststart",
            str(h264),
        ]
    )
    run(["ffmpeg", "-y", "-ss", "00:00:08", "-i", str(src), "-frames:v", "1", str(frame)])
    return h264, frame


def make_demo_video() -> tuple[Path, Path]:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    h264, real_frame = convert_real_video()
    poster = make_poster()
    encode_sim_stage_clips()
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
            "USART1 输出轻量定位 CSV，同时供 PySide6 上位机解析。",
            "USART2 面向底盘主控发送定位数据帧。",
            "采集日志按时间命名，传感器数据与地图截图对齐保存。",
        ],
        real_frame,
    )
    card_outro = make_ack_card(PROMO_DIR / "r1-locaterv2-card-outro.png")

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
        sim_demo_seg = TMP_DIR / "seg_sim_excerpt.mp4"
        encode_video_excerpt(sim_seg, sim_demo_seg, start_s=22.0, duration_s=16.0)
        segments.append(sim_demo_seg)
    outro_seg = TMP_DIR / "seg_outro.mp4"
    encode_still(card_outro, 7.0, outro_seg)
    segments.append(outro_seg)

    concat_file = TMP_DIR / "concat.txt"
    concat_file.write_text("".join(f"file '{p.as_posix()}'\n" for p in segments), encoding="utf-8")
    demo = PROMO_DIR / "r1-locaterv2-demo.mp4"
    run(["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(concat_file), "-c", "copy", "-movflags", "+faststart", str(demo)])

    gif = PROMO_DIR / "r1-locaterv2-demo-teaser.gif"
    run(
        [
            "ffmpeg",
            "-y",
            "-ss",
            "00:00:00",
            "-t",
            "00:00:12",
            "-i",
            str(demo),
            "-vf",
            "fps=10,scale=720:-1:flags=lanczos",
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
