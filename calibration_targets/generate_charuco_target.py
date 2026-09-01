#!/usr/bin/env python3
"""Generate a printable ChArUco calibration target (PDF + PNG) and the
matching industrial_calibration_ros CharucoGridTargetFinder detector YAML.

The PDF page is built as a single raster image saved with an explicit DPI,
so the physical page size in points is exactly (pixels / dpi * 72). Printed
at 100%% scale ("actual size", no "fit to page"), each square measures
exactly --square-mm millimetres. No vector re-flowing or "fit to page"
scaling is involved, so the physical size on paper is fully determined by
this file, not by the viewer/printer.

Usage (defaults match the 7x5 / 30mm / 23mm / DICT_4X4_250 board):
    python3 generate_charuco_target.py
"""
import argparse
from pathlib import Path

import cv2.aruco as aruco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

A4_MM = (210.0, 297.0)  # (short edge, long edge)
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def mm_to_px(mm: float, dpi: int) -> int:
    return round(mm / 25.4 * dpi)


def build_board(squares_x, squares_y, square_mm, marker_mm, dict_id, dpi):
    dictionary = aruco.getPredefinedDictionary(dict_id)
    board = aruco.CharucoBoard_create(
        squares_x, squares_y, square_mm / 1000.0, marker_mm / 1000.0, dictionary
    )
    w_px = mm_to_px(squares_x * square_mm, dpi)
    h_px = mm_to_px(squares_y * square_mm, dpi)
    # marginSize=0: the printed page margin is added separately below, not
    # baked into the board image, so the board's own pixel size maps exactly
    # to squares_x * square_mm x squares_y * square_mm.
    img = board.draw((w_px, h_px), marginSize=0, borderBits=1)
    return img


def wrap_text(draw, text, font, max_width_px):
    words = text.split()
    lines = []
    current = ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if draw.textbbox((0, 0), candidate, font=font)[2] <= max_width_px or not current:
            current = candidate
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


def compose_page(board_img, squares_x, squares_y, square_mm, marker_mm, dict_name, dpi):
    page_w_mm, page_h_mm = A4_MM[1], A4_MM[0]  # landscape
    page_w_px = mm_to_px(page_w_mm, dpi)
    page_h_px = mm_to_px(page_h_mm, dpi)

    page = Image.new("L", (page_w_px, page_h_px), color=255)
    board_pil = Image.fromarray(board_img)

    x_off = (page_w_px - board_pil.width) // 2
    y_off = (page_h_px - board_pil.height) // 2
    page.paste(board_pil, (x_off, y_off))

    draw = ImageDraw.Draw(page)
    font_size = mm_to_px(4.0, dpi)
    try:
        font = ImageFont.truetype(FONT_PATH, font_size)
    except OSError:
        font = ImageFont.load_default()

    raw_lines = [
        f"ChArUco {squares_x}x{squares_y}  square={square_mm:.1f}mm  "
        f"marker={marker_mm:.1f}mm  {dict_name}",
        "VERIFY BEFORE USE: measure one printed square edge-to-edge with a "
        f"ruler/calipers - it must be exactly {square_mm:.1f} mm. If not, "
        "your printer/viewer rescaled this page: reprint at 100% / \"actual "
        "size\", not \"fit to page\".",
    ]
    max_text_width = page_w_px - 2 * x_off
    lines = []
    for raw_line in raw_lines:
        lines.extend(wrap_text(draw, raw_line, font, max_text_width))

    text_y = y_off + board_pil.height + mm_to_px(6.0, dpi)
    for line in lines:
        draw.text((x_off, text_y), line, fill=0, font=font)
        bbox = draw.textbbox((x_off, text_y), line, font=font)
        text_y = bbox[3] + mm_to_px(2.0, dpi)

    return page


def write_detector_yaml(path, squares_x, squares_y, square_mm, marker_mm, dict_id, dict_name):
    path.write_text(
        "# ChArUco grid target finder\n"
        "# Generated to match calibration_targets/generate_charuco_target.py\n"
        "target_finder:\n"
        "  type: CharucoGridTargetFinder\n"
        f"  rows: {squares_y}\n"
        f"  cols: {squares_x}\n"
        f"  chessboard_dim: {square_mm / 1000.0:.3f}\n"
        f"  aruco_marker_dim: {marker_mm / 1000.0:.3f}\n"
        f"  dictionary: {dict_id}  # {dict_name}\n"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--squares-x", type=int, default=7, help="Board columns")
    parser.add_argument("--squares-y", type=int, default=5, help="Board rows")
    parser.add_argument("--square-mm", type=float, default=30.0, help="Chessboard square size (mm)")
    parser.add_argument("--marker-mm", type=float, default=23.0, help="ArUco marker size (mm)")
    parser.add_argument("--dict-name", default="DICT_4X4_250", help="cv2.aruco predefined dictionary name")
    parser.add_argument("--dpi", type=int, default=300, help="Raster resolution used for print-exact sizing")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).parent,
        help="Output directory",
    )
    args = parser.parse_args()

    if not hasattr(aruco, args.dict_name):
        raise SystemExit(f"Unknown dictionary name: {args.dict_name}")
    dict_id = getattr(aruco, args.dict_name)

    board_w_mm = args.squares_x * args.square_mm
    board_h_mm = args.squares_y * args.square_mm
    if board_w_mm > A4_MM[1] or board_h_mm > A4_MM[0]:
        raise SystemExit(
            f"Board ({board_w_mm:.0f}x{board_h_mm:.0f}mm) does not fit on "
            f"landscape A4 ({A4_MM[1]:.0f}x{A4_MM[0]:.0f}mm)."
        )

    board_img = build_board(
        args.squares_x, args.squares_y, args.square_mm, args.marker_mm, dict_id, args.dpi
    )

    stem = (
        f"charuco_{args.squares_x}x{args.squares_y}_"
        f"{args.square_mm:.0f}mm_{args.dict_name.lower()}"
    )
    args.out_dir.mkdir(parents=True, exist_ok=True)

    png_path = args.out_dir / f"{stem}.png"
    Image.fromarray(board_img).save(png_path, dpi=(args.dpi, args.dpi))

    page = compose_page(
        board_img, args.squares_x, args.squares_y, args.square_mm, args.marker_mm,
        args.dict_name, args.dpi,
    )
    pdf_path = args.out_dir / f"{stem}.pdf"
    page.save(pdf_path, "PDF", resolution=float(args.dpi))

    yaml_path = args.out_dir / f"{stem}_detector_config.yaml"
    write_detector_yaml(
        yaml_path, args.squares_x, args.squares_y, args.square_mm, args.marker_mm,
        dict_id, args.dict_name,
    )

    page_w_pt = page.width / args.dpi * 72.0
    page_h_pt = page.height / args.dpi * 72.0
    print(f"Board: {args.squares_x}x{args.squares_y} squares, "
          f"{board_w_mm:.0f}x{board_h_mm:.0f} mm, square={args.square_mm}mm, "
          f"marker={args.marker_mm}mm, {args.dict_name} (id={dict_id})")
    print(f"PDF page: A4 landscape, {page.width}x{page.height}px @ {args.dpi}dpi "
          f"= {page_w_pt:.1f}x{page_h_pt:.1f}pt (must equal 841.9x595.3pt for true A4)")
    print(f"Wrote: {png_path}")
    print(f"Wrote: {pdf_path}")
    print(f"Wrote: {yaml_path}")
    print()
    print(f"VERIFY: print {pdf_path.name} at 100% / \"actual size\" (not \"fit to "
          f"page\"), then measure one printed square edge-to-edge - it must be "
          f"exactly {args.square_mm:.1f} mm.")


if __name__ == "__main__":
    main()
