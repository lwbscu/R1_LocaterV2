from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PROMO_DIR = ROOT / "docs" / "promotion"
SECTIONS_FILE = PROMO_DIR / "juejin-style-sections.json"
OUT_DIR = PROMO_DIR / "juejin-style"


def main() -> int:
    skill_dir_value = os.environ.get("PRODUCT_PROMOTION_SKILL_DIR")
    if not skill_dir_value:
        raise RuntimeError("Set PRODUCT_PROMOTION_SKILL_DIR to the local product-promotion skill directory.")
    skill_dir = Path(skill_dir_value)
    generator = skill_dir / "scripts" / "generate_juejin_title_cards.py"
    if not generator.exists():
        raise FileNotFoundError(
            f"product-promotion title-card generator not found: {generator}. "
            "Set PRODUCT_PROMOTION_SKILL_DIR to the skill directory."
        )
    subprocess.run(
        [
            sys.executable,
            str(generator),
            "--sections",
            str(SECTIONS_FILE),
            "--out",
            str(OUT_DIR),
        ],
        cwd=ROOT,
        check=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
