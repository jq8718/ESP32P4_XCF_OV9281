"""Render actual C-decoder trace arrays and an annotated contact sheet."""

import csv
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "decode_stages"
FONT_PATH = Path("C:/Windows/Fonts/msyh.ttc")


def font(size):
    return ImageFont.truetype(str(FONT_PATH), size) if FONT_PATH.exists() else ImageFont.load_default()


def read_csv(name):
    with (OUT / name).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def load_pgm(name):
    return Image.open(OUT / name).convert("RGB")


def fit(image, size):
    scale = min(size[0] / image.width, size[1] / image.height)
    return image.resize((max(1, round(image.width * scale)), max(1, round(image.height * scale))), Image.Resampling.NEAREST)


def main():
    raw = Image.frombytes("L", (1280, 720), (OUT / "source.gray8").read_bytes()).convert("RGB")
    meta = json.loads((OUT / "source.json").read_text(encoding="utf-8"))
    source = raw.copy()
    draw = ImageDraw.Draw(source)
    for row in read_csv("04_marker_candidates.csv"):
        x, y, w, h = [int(row[key]) for key in ("x", "y", "w", "h")]
        draw.rectangle((x, y, x + w, y + h), outline="#ffbd31", width=3)
    for row in read_csv("04_selected_markers.csv"):
        x, y, w, h = [int(row[key]) for key in ("x", "y", "w", "h")]
        draw.rectangle((x, y, x + w, y + h), outline="#26f4bd", width=5)
    # The decoded ROI comes from the C trace result, not an image heuristic.
    result = json.loads((OUT / "trace_result.json").read_text(encoding="utf-8"))
    x0, y0, x1, y1 = result["marker_roi"]
    draw.rectangle((x0, y0, x1, y1), outline="#ff4d91", width=4)
    corners = result["corners"]
    draw.polygon([(corners[i], corners[i + 1]) for i in range(0, 8, 2)], outline="#3a9eff", width=4)
    source.save(OUT / "01_source_annotated.png")

    stage_files = [
        ("02_marker_sample_6x.pgm", "02_marker_sample_6x.png"),
        ("03_marker_binary.pgm", "03_marker_binary.png"),
        ("05_normalized_gray_128.pgm", "05_normalized_gray_128.png"),
        ("06_normalized_binary_128.pgm", "06_normalized_binary_128.png"),
        ("07_module_grid.pgm", "07_module_grid.png"),
        ("08_data_mapping.pgm", "08_data_mapping.png"),
    ]
    for pgm, png in stage_files:
        load_pgm(pgm).save(OUT / png)

    before = [int(row["decimal"]) for row in read_csv("09_codewords_before_ecc.csv")]
    after = [int(row["decimal"]) for row in read_csv("10_codewords_after_ecc.csv")]
    codewords = Image.new("RGB", (850, 460), "#f6f9fc")
    cd = ImageDraw.Draw(codewords)
    cd.text((24, 30), "ECC200 码字与载荷", fill="#14233b", font=font(34))
    cd.text((24, 105), "纠错前  " + "  ".join(f"{v:02X}" for v in before), fill="#213c62", font=font(25))
    cd.text((24, 165), "纠错后数据  " + "  ".join(f"{v:02X}" for v in after[:3]), fill="#213c62", font=font(25))
    cd.text((24, 235), "RS 校验通过 → ASCII/数字对解码", fill="#176a65", font=font(25))
    cd.text((24, 305), f"输出：{result['payload']}", fill="#12305d", font=font(35))
    cd.text((24, 382), f"帧 {meta['sequence']}  |  {result['grid'][0]}×{result['grid'][1]} 模块", fill="#52657b", font=font(22))
    codewords.save(OUT / "09_codewords_payload.png")

    cards = [
        ("01 原始帧与定位", "1280×720 Gray8；黄=候选，绿=选中，红=ROI，蓝=解码角点", source),
        ("02 六倍稀疏采样", "每 6 像素取样；暗点经过 3×3 灰度滤波", load_pgm("02_marker_sample_6x.pgm")),
        ("03 局部阈值", "13×13 积分图均值与全局阈值组成标记掩模", load_pgm("03_marker_binary.pgm")),
        ("04 三标记与 ROI", "连通域筛选、三点直角关系、边缘校验；框为讲解叠加", source.crop((170, 100, 530, 440))),
        ("05 归一化灰度", "用标记坐标轴映射到 128×128；此帧首次尝试成功", load_pgm("05_normalized_gray_128.pgm")),
        ("06 Otsu 二值化", "在归一化图上计算阈值；黑白图送入 DM 核心", load_pgm("06_normalized_binary_128.pgm")),
        ("07 透视模块采样", "识别 L 边与时钟边，估计尺寸，逐模块取样：10×10", load_pgm("07_module_grid.pgm")),
        ("08 去边框映射", "去掉实线与交替边框，保留 8×8 数据模块", load_pgm("08_data_mapping.pgm")),
        ("09 码字、纠错、文本", "Annex F 位放置 → RS 纠错 → 载荷解释", codewords),
    ]
    cell_w, cell_h = 900, 560
    sheet = Image.new("RGB", (cell_w * 3, cell_h * 3), "#dce5ed")
    painter = ImageDraw.Draw(sheet)
    for i, (title, subtitle, img) in enumerate(cards):
        col, row = i % 3, i // 3
        x, y = col * cell_w, row * cell_h
        painter.rounded_rectangle((x + 12, y + 12, x + cell_w - 12, y + cell_h - 12), 18, fill="white")
        painter.text((x + 34, y + 27), title, fill="#102742", font=font(34))
        painter.text((x + 34, y + 80), subtitle, fill="#52647a", font=font(19))
        shown = fit(img, (cell_w - 68, cell_h - 155))
        if i in (6, 7):
            shown = img.resize((380, 380), Image.Resampling.NEAREST)
        sheet.paste(shown, (x + (cell_w - shown.width) // 2, y + 130 + (cell_h - 155 - shown.height) // 2))
    sheet.save(OUT / "decode_pipeline.png", optimize=True)
    print(OUT / "decode_pipeline.png")


if __name__ == "__main__":
    main()
