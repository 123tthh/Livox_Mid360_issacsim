#!/usr/bin/env python3
"""Render a deterministic angular-domain Petal/Rotary comparison figure."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from apply_mid360_petal_profile import (
    EXPECTED_TRAJECTORY_SHA256,
    POINTS_PER_STATE,
    STATE_COUNT,
    TRAJECTORY,
    load_trajectory,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "docs/validation/MID360_Petal_vs_Rotary_Angular_Pattern.png"
WIDTH = 1800
HEIGHT = 920
PLOT_TOP = 170
PLOT_HEIGHT = 610
PLOT_WIDTH = 760
LEFT_X = 95
RIGHT_X = 945
AZIMUTH_MIN = -180.0
AZIMUTH_MAX = 180.0
ELEVATION_MIN = -10.0
ELEVATION_MAX = 55.0


def _font(size: int, bold: bool = False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    path = Path("/usr/share/fonts/truetype/dejavu") / name
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def _coordinates(azimuth: np.ndarray, elevation: np.ndarray, plot_x: int):
    x = np.rint(
        plot_x + (azimuth - AZIMUTH_MIN) * (PLOT_WIDTH - 1) / (AZIMUTH_MAX - AZIMUTH_MIN)
    ).astype(np.int32)
    y = np.rint(
        PLOT_TOP
        + (ELEVATION_MAX - elevation)
        * (PLOT_HEIGHT - 1)
        / (ELEVATION_MAX - ELEVATION_MIN)
    ).astype(np.int32)
    return x, y


def _draw_axes(draw: ImageDraw.ImageDraw, plot_x: int) -> None:
    grid = (55, 68, 78)
    label = (190, 201, 208)
    draw.rectangle(
        (plot_x, PLOT_TOP, plot_x + PLOT_WIDTH, PLOT_TOP + PLOT_HEIGHT),
        fill=(17, 23, 28),
        outline=(120, 135, 145),
        width=2,
    )
    for azimuth in (-180, -120, -60, 0, 60, 120, 180):
        x, _ = _coordinates(np.array([azimuth]), np.array([0.0]), plot_x)
        draw.line((int(x[0]), PLOT_TOP, int(x[0]), PLOT_TOP + PLOT_HEIGHT), fill=grid, width=1)
        text = f"{azimuth}°"
        draw.text((int(x[0]), PLOT_TOP + PLOT_HEIGHT + 12), text, fill=label, font=_font(20), anchor="ma")
    for elevation in (-10, 0, 10, 20, 30, 40, 50):
        _, y = _coordinates(np.array([0.0]), np.array([elevation]), plot_x)
        draw.line((plot_x, int(y[0]), plot_x + PLOT_WIDTH, int(y[0])), fill=grid, width=1)
        draw.text((plot_x - 12, int(y[0])), f"{elevation}°", fill=label, font=_font(20), anchor="rm")
    draw.text(
        (plot_x + PLOT_WIDTH / 2, PLOT_TOP + PLOT_HEIGHT + 48),
        "Azimuth (sensor frame)",
        fill=label,
        font=_font(22),
        anchor="ma",
    )


def _petal_layer() -> Image.Image:
    trajectory = load_trajectory(TRAJECTORY)
    azimuth = np.asarray(trajectory.azimuth_deg, dtype=np.float32)
    azimuth = (azimuth + 180.0) % 360.0 - 180.0
    elevation = np.asarray(trajectory.elevation_deg, dtype=np.float32)
    x, y = _coordinates(azimuth, elevation, 0)
    valid = (x >= 0) & (x < PLOT_WIDTH) & (y >= PLOT_TOP) & (y < PLOT_TOP + PLOT_HEIGHT)
    density = np.zeros((PLOT_HEIGHT, PLOT_WIDTH), dtype=np.uint32)
    np.add.at(density, (y[valid] - PLOT_TOP, x[valid]), 1)

    strength = np.log1p(density.astype(np.float32))
    positive = strength[strength > 0]
    scale = float(np.percentile(positive, 99.5)) if positive.size else 1.0
    strength = np.clip(strength / max(scale, 1.0), 0.0, 1.0)
    rgba = np.zeros((PLOT_HEIGHT, PLOT_WIDTH, 4), dtype=np.uint8)
    rgba[..., 0] = (30 + 180 * strength).astype(np.uint8)
    rgba[..., 1] = (110 + 145 * strength).astype(np.uint8)
    rgba[..., 2] = (145 + 110 * strength).astype(np.uint8)
    rgba[..., 3] = (255 * np.power(strength, 0.55)).astype(np.uint8)
    return Image.fromarray(rgba, mode="RGBA")


def _draw_rotary(draw: ImageDraw.ImageDraw) -> None:
    elevations = np.linspace(-7.0, 52.0, 40)
    for index, elevation in enumerate(elevations):
        _, y = _coordinates(np.array([0.0]), np.array([elevation]), RIGHT_X)
        color = (45 + index * 3, 220, 155 + index * 2)
        draw.line((RIGHT_X, int(y[0]), RIGHT_X + PLOT_WIDTH, int(y[0])), fill=color, width=2)


def render(output: Path) -> None:
    image = Image.new("RGB", (WIDTH, HEIGHT), (10, 14, 18))
    draw = ImageDraw.Draw(image)
    draw.text(
        (WIDTH / 2, 42),
        "Livox MID-360 angular firing pattern comparison",
        fill=(238, 244, 247),
        font=_font(38, bold=True),
        anchor="ma",
    )
    draw.text(
        (WIDTH / 2, 96),
        "same 200,000 points/s and FOV; different temporal/angular organization",
        fill=(165, 181, 190),
        font=_font(23),
        anchor="ma",
    )
    _draw_axes(draw, LEFT_X)
    _draw_axes(draw, RIGHT_X)
    petal = _petal_layer()
    image.paste(petal, (LEFT_X, PLOT_TOP), petal)
    draw = ImageDraw.Draw(image)
    _draw_rotary(draw)
    draw.text(
        (LEFT_X + PLOT_WIDTH / 2, 138),
        "PETAL — official non-repetitive 4.0 s window",
        fill=(90, 230, 245),
        font=_font(26, bold=True),
        anchor="ma",
    )
    draw.text(
        (RIGHT_X + PLOT_WIDTH / 2, 138),
        "ROTARY — one repeatable 0.1 s revolution",
        fill=(80, 235, 165),
        font=_font(26, bold=True),
        anchor="ma",
    )
    footer = (
        f"Petal: {STATE_COUNT} unique states × 0.1 s × {POINTS_PER_STATE:,} rays  |  "
        "Rotary: 40 fixed elevation channels  |  "
        f"official CSV SHA-256 {EXPECTED_TRAJECTORY_SHA256[:16]}…"
    )
    draw.text((WIDTH / 2, 882), footer, fill=(150, 165, 174), font=_font(19), anchor="ma")
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, format="PNG", optimize=True)
    print(f"PATTERN_COMPARISON={output} SIZE={image.size[0]}x{image.size[1]}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    render(args.output.resolve())


if __name__ == "__main__":
    main()
