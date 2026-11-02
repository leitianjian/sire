"""Crop scene whitespace without rasterizing the saved PDF artwork."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

from PIL import Image, ImageChops
from pypdf import PdfReader, PdfWriter
from pypdf.generic import RectangleObject


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--margin", type=float, default=2.0, help="White border in PDF points")
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        raise ValueError("Preserve the source PDF; use a distinct output path")
    reader = PdfReader(args.input)
    if len(reader.pages) != 1 or reader.pages[0].rotation:
        raise ValueError("Expected a single unrotated scene page")
    page = reader.pages[0]
    original = list(map(float, page.mediabox))
    with tempfile.TemporaryDirectory(prefix="sire_scene_crop_") as directory:
        prefix = Path(directory) / "page"
        subprocess.run(["pdftoppm", "-f", "1", "-singlefile", "-r", "300", "-png",
                        str(args.input), str(prefix)], check=True, capture_output=True)
        with Image.open(prefix.with_suffix(".png")) as image:
            rgb = image.convert("RGB")
            delta = ImageChops.difference(rgb, Image.new("RGB", rgb.size, "white"))
            channels = delta.split()
            mask = ImageChops.lighter(ImageChops.lighter(channels[0], channels[1]), channels[2])
            bounds = mask.point(lambda value: 255 if value > 8 else 0).getbbox()
            if bounds is None:
                raise ValueError("No visible scene content")
            width, height = rgb.size
    x0, y0, x1, y1 = original
    left, top, right, bottom = bounds
    margin = args.margin
    crop = [max(x0, x0 + left / width * (x1 - x0) - margin),
            max(y0, y1 - bottom / height * (y1 - y0) - margin),
            min(x1, x0 + right / width * (x1 - x0) + margin),
            min(y1, y1 - top / height * (y1 - y0) + margin)]
    page.mediabox = RectangleObject(crop)
    page.cropbox = RectangleObject(crop)
    writer = PdfWriter()
    writer.add_page(page)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as stream:
        writer.write(stream)
    args.output.with_suffix(".crop.json").write_text(json.dumps({
        "source": str(args.input.resolve()),
        "source_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "original_box_pt": original, "cropped_box_pt": crop,
        "margin_pt": margin, "vector_artwork_preserved": True,
    }, indent=2), encoding="utf-8")
    print(f"Cropped scene from {x1-x0:.2f} x {y1-y0:.2f} pt "
          f"to {crop[2]-crop[0]:.2f} x {crop[3]-crop[1]:.2f} pt")


if __name__ == "__main__":
    main()
