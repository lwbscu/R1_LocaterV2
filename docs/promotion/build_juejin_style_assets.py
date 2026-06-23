from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageSequence


ROOT = Path(__file__).resolve().parent
OUT_DIR = ROOT / "juejin-style"
LEARNMAP_TEMPLATE = Path(
    r"D:\VS_Project\AI_x10_Learning_Projects\ai-10x-learning-coach"
    r"\docs\assets\juejin-style\section-01-demo.gif"
)
FONT_BOLD = Path(r"C:\Windows\Fonts\msyhbd.ttc")
FONT_REGULAR = Path(r"C:\Windows\Fonts\msyh.ttc")


SECTIONS = [
    ("01", "coordinate", "坐标先统一", "R1_LOCATER / COORDINATE"),
    ("02", "fusion", "多传感器闭环", "R1_LOCATER / SENSOR FUSION"),
    ("03", "replay", "上位机回放", "R1_LOCATER / DESKTOP REPLAY"),
    ("04", "raycast", "DT35 墙体模型", "R1_LOCATER / DT35 RAYCAST"),
    ("05", "real2sim", "real2sim 迭代", "R1_LOCATER / REAL2SIM"),
    ("06", "links", "代码与视频", "R1_LOCATER / LINKS"),
]


def load_font(path: Path, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(path), size=size)


def clean_title_area(draw: ImageDraw.ImageDraw) -> None:
    # Preserve the LearnMap icon and animated background; only replace the old text block.
    x0, y0, x1, y1 = 92, 9, 429, 76
    draw.rectangle((x0, y0, x1, y1), fill=(248, 251, 255, 246))
    for x in range(x0 + 8, x1, 26):
        draw.line((x, y0, x + 10, y1), fill=(221, 232, 255, 120), width=1)
    for y in range(y0 + 9, y1, 15):
        draw.line((x0, y, x1, y), fill=(224, 234, 255, 125), width=1)


def draw_title(frame: Image.Image, section: tuple[str, str, str, str]) -> Image.Image:
    number, _slug, title, subtitle = section
    img = frame.convert("RGBA")
    draw = ImageDraw.Draw(img)
    clean_title_area(draw)

    title_font = load_font(FONT_BOLD, 35)
    subtitle_font = load_font(FONT_BOLD, 9)
    number_font = load_font(FONT_BOLD, 11)

    # Blue-purple text hierarchy copied from LearnMap: Chinese is the main visual anchor,
    # English is a smaller technical subtitle.
    draw.text((96, 15), title, font=title_font, fill=(67, 59, 211, 255))
    draw.text((98, 57), subtitle, font=subtitle_font, fill=(90, 103, 205, 235))
    draw.rounded_rectangle((356, 50, 421, 68), radius=9, fill=(255, 255, 255, 225), outline=(128, 157, 255, 120), width=1)
    draw.text((371, 53), number, font=number_font, fill=(79, 75, 221, 245))
    return img.convert("RGB")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    template = Image.open(LEARNMAP_TEMPLATE)
    base_frames = [f.copy().convert("RGBA") for f in ImageSequence.Iterator(template)]
    duration = template.info.get("duration", 80)

    for section in SECTIONS:
        frames = [draw_title(frame, section) for frame in base_frames]
        out = OUT_DIR / f"section-{section[0]}-{section[1]}.gif"
        frames[0].save(
            out,
            save_all=True,
            append_images=frames[1:],
            duration=duration,
            loop=0,
            optimize=True,
            disposal=2,
        )
        print(out.relative_to(ROOT), out.stat().st_size)


if __name__ == "__main__":
    main()
