#!/usr/bin/env python3
import argparse
import csv
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np
from PIL import Image


REQUIRED_FIELDS = ("id", "x_unit", "y_unit", "size_type")
SIZE_UNITS = {
    "small": 1.0,
    "big": 5.0,
}


@dataclass(frozen=True)
class MarkerLayout:
    marker_id: int
    x_unit: float
    y_unit: float
    size_type: str

    @property
    def size_unit(self):
        return SIZE_UNITS[self.size_type]

    @property
    def max_x(self):
        return self.x_unit + self.size_unit

    @property
    def max_y(self):
        return self.y_unit + self.size_unit

    @property
    def center_x(self):
        return self.x_unit + self.size_unit / 2.0

    @property
    def center_y(self):
        return self.y_unit + self.size_unit / 2.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate an ArUco board layout from a CSV file."
    )
    parser.add_argument("--layout", default="layout.csv", help="Input layout CSV path.")
    parser.add_argument(
        "--small-size",
        type=int,
        default=100,
        help="Small marker side length in pixels.",
    )
    parser.add_argument(
        "--margin",
        type=int,
        default=100,
        help="Output margin in pixels.",
    )
    parser.add_argument(
        "--output",
        default="out/layout",
        help="Output prefix without extension.",
    )
    parser.add_argument(
        "--debug-labels",
        action="store_true",
        help="Also write a labeled PNG for manual layout checks.",
    )
    return parser.parse_args()


def fail(message):
    raise ValueError(message)


def parse_int(value, field_name, row_number):
    try:
        return int(value)
    except (TypeError, ValueError):
        fail(f"row {row_number}: {field_name} must be an integer, got {value!r}")


def parse_float(value, field_name, row_number):
    try:
        return float(value)
    except (TypeError, ValueError):
        fail(f"row {row_number}: {field_name} must be a number, got {value!r}")


def read_layout(path):
    markers = []
    seen_ids = set()

    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            fail(f"{path}: CSV is empty")

        missing = [field for field in REQUIRED_FIELDS if field not in reader.fieldnames]
        if missing:
            fail(f"{path}: missing required fields: {', '.join(missing)}")

        for row_number, row in enumerate(reader, start=2):
            marker_id = parse_int(row.get("id"), "id", row_number)
            x_unit = parse_float(row.get("x_unit"), "x_unit", row_number)
            y_unit = parse_float(row.get("y_unit"), "y_unit", row_number)
            size_type = (row.get("size_type") or "").strip()

            if marker_id < 0 or marker_id >= 100:
                fail(
                    f"row {row_number}: id {marker_id} is outside "
                    "DICT_4X4_100 range [0, 99]"
                )
            if marker_id in seen_ids:
                fail(f"row {row_number}: duplicate marker id {marker_id}")
            if x_unit < 0 or y_unit < 0:
                fail(f"row {row_number}: x_unit and y_unit must be >= 0")
            if size_type not in SIZE_UNITS:
                fail(
                    f"row {row_number}: size_type must be one of "
                    f"{', '.join(SIZE_UNITS)}, got {size_type!r}"
                )

            seen_ids.add(marker_id)
            markers.append(MarkerLayout(marker_id, x_unit, y_unit, size_type))

    if not markers:
        fail(f"{path}: no markers found")

    validate_no_overlap(markers)
    return markers


def validate_no_overlap(markers):
    for index, first in enumerate(markers):
        for second in markers[index + 1 :]:
            x_overlap = min(first.max_x, second.max_x) - max(first.x_unit, second.x_unit)
            y_overlap = min(first.max_y, second.max_y) - max(first.y_unit, second.y_unit)
            if x_overlap > 0 and y_overlap > 0:
                fail(
                    "markers overlap: "
                    f"id {first.marker_id} [{first.x_unit}, {first.y_unit}, "
                    f"{first.max_x}, {first.max_y}] and "
                    f"id {second.marker_id} [{second.x_unit}, {second.y_unit}, "
                    f"{second.max_x}, {second.max_y}]"
                )


def get_aruco_dictionary():
    dictionary_id = cv2.aruco.DICT_4X4_100
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def generate_marker(dictionary, marker_id, side_pixels):
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side_pixels)
    else:
        marker = np.zeros((side_pixels, side_pixels), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, side_pixels, marker, 1)
    return marker


def layout_bounds(markers):
    min_x = min(marker.x_unit for marker in markers)
    min_y = min(marker.y_unit for marker in markers)
    max_x = max(marker.max_x for marker in markers)
    max_y = max(marker.max_y for marker in markers)
    return min_x, min_y, max_x, max_y


def unit_to_pixel(value, min_value, small_size, margin):
    return int(round((value - min_value) * small_size)) + margin


def render_layout(markers, small_size, margin, debug_labels):
    dictionary = get_aruco_dictionary()
    min_x, min_y, max_x, max_y = layout_bounds(markers)
    width = int(round((max_x - min_x) * small_size)) + margin * 2
    height = int(round((max_y - min_y) * small_size)) + margin * 2

    canvas = np.full((height, width), 255, dtype=np.uint8)

    for marker in markers:
        side_pixels = int(round(marker.size_unit * small_size))
        marker_image = generate_marker(dictionary, marker.marker_id, side_pixels)
        x_px = unit_to_pixel(marker.x_unit, min_x, small_size, margin)
        y_px = unit_to_pixel(marker.y_unit, min_y, small_size, margin)
        canvas[y_px : y_px + side_pixels, x_px : x_px + side_pixels] = marker_image

    labeled = None
    if debug_labels:
        labeled = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
        for marker in markers:
            side_pixels = int(round(marker.size_unit * small_size))
            x_px = unit_to_pixel(marker.x_unit, min_x, small_size, margin)
            y_px = unit_to_pixel(marker.y_unit, min_y, small_size, margin)
            cv2.rectangle(
                labeled,
                (x_px, y_px),
                (x_px + side_pixels, y_px + side_pixels),
                (0, 0, 255),
                2,
            )
            label = f"{marker.marker_id} ({marker.x_unit:g},{marker.y_unit:g})"
            label_y = max(18, y_px - 8)
            cv2.putText(
                labeled,
                label,
                (x_px, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

    return canvas, labeled


def write_offsets(markers, output_prefix):
    min_x, min_y, max_x, max_y = layout_bounds(markers)
    board_center_x = (min_x + max_x) / 2.0
    board_center_y = (min_y + max_y) / 2.0
    offsets_path = f"{output_prefix}_offsets.csv"

    with open(offsets_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "id",
                "size_type",
                "x_center_unit",
                "y_center_unit",
                "x_offset_unit",
                "y_offset_unit",
            ]
        )
        for marker in sorted(markers, key=lambda item: item.marker_id):
            writer.writerow(
                [
                    marker.marker_id,
                    marker.size_type,
                    f"{marker.center_x:.6g}",
                    f"{marker.center_y:.6g}",
                    f"{marker.center_x - board_center_x:.6g}",
                    f"{board_center_y - marker.center_y:.6g}",
                ]
            )

    return offsets_path


def write_images(canvas, labeled, output_prefix):
    png_path = f"{output_prefix}.png"
    pdf_path = f"{output_prefix}.pdf"

    if not cv2.imwrite(png_path, canvas):
        fail(f"failed to write {png_path}")

    Image.fromarray(canvas).convert("RGB").save(pdf_path)

    labeled_path = None
    if labeled is not None:
        labeled_path = f"{output_prefix}_labeled.png"
        if not cv2.imwrite(labeled_path, labeled):
            fail(f"failed to write {labeled_path}")

    return png_path, pdf_path, labeled_path


def main():
    args = parse_args()

    try:
        if args.small_size <= 0:
            fail("--small-size must be > 0")
        if args.margin < 0:
            fail("--margin must be >= 0")

        output_dir = os.path.dirname(args.output)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        markers = read_layout(args.layout)
        canvas, labeled = render_layout(
            markers, args.small_size, args.margin, args.debug_labels
        )
        png_path, pdf_path, labeled_path = write_images(canvas, labeled, args.output)
        offsets_path = write_offsets(markers, args.output)

        print(f"wrote {png_path}")
        print(f"wrote {pdf_path}")
        print(f"wrote {offsets_path}")
        if labeled_path:
            print(f"wrote {labeled_path}")
        print(f"markers: {len(markers)}")
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
