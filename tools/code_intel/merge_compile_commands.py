#!/usr/bin/env python3
"""合并多个 compile_commands.json 文件。

Usage:
  1) Auto-scan build subdirectories (default):
     ./tools/code_intel/merge_compile_commands.py

  2) Merge specific build subdirectories or json files:
     ./tools/code_intel/merge_compile_commands.py uav_control px4_bridge
     ./tools/code_intel/merge_compile_commands.py build/uav_control/compile_commands.json

Options:
  -o, --output <path>    Output file path (default: ./build/compile_commands.json)
  -b, --build-dir <path> Build directory root for subdir auto-discovery
                         (default: ./build)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, List


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="合并 compile_commands.json 文件")
    parser.add_argument(
        "inputs",
        nargs="*",
        help=(
            "构建子目录名（如 uav_control）、目录路径或 compile_commands.json 路径。"
            "若为空，则自动扫描 build 目录。"
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        default="build/compile_commands.json",
        help="合并后输出路径（默认：./build/compile_commands.json）",
    )
    parser.add_argument(
        "-b",
        "--build-dir",
        default="build",
        help="构建根目录（默认：./build）",
    )
    parser.add_argument(
        "--verbose-missing",
        action="store_true",
        help="（兼容保留）自动扫描时输出缺失 compile_commands.json 的一级目录。",
    )
    return parser.parse_args()


def _discover_default(build_dir: Path) -> List[Path]:
    if not build_dir.exists():
        return []
    # Recursively discover compile databases under build/, not only one level.
    found = [p for p in build_dir.rglob("compile_commands.json") if p.is_file()]
    # Stable de-dup after resolve (handles symlink/redundant paths).
    unique: List[Path] = []
    seen = set()
    for p in sorted(found):
        rp = p.resolve()
        if rp not in seen:
            unique.append(rp)
            seen.add(rp)
    return unique


def _discover_missing_first_level(build_dir: Path) -> List[Path]:
    if not build_dir.exists():
        return []
    missing: List[Path] = []
    for child in sorted(p for p in build_dir.iterdir() if p.is_dir()):
        if not (child / "compile_commands.json").is_file():
            missing.append(child.resolve())
    return missing


def _resolve_inputs(raw_inputs: Iterable[str], build_dir: Path) -> List[Path]:
    resolved: List[Path] = []
    for item in raw_inputs:
        p = Path(item)
        candidates: List[Path] = []

        if p.is_file():
            candidates.append(p)
        elif p.is_dir():
            candidates.append(p / "compile_commands.json")
        else:
            # Treat as build subdir name under build/
            candidates.append(build_dir / item / "compile_commands.json")
            # Also support explicit path-like input that may not exist yet in cwd
            candidates.append(p / "compile_commands.json")

        found = next((c for c in candidates if c.is_file()), None)
        if found is None:
            raise FileNotFoundError(
                f"无法从输入 '{item}' 解析到 compile_commands.json"
            )
        resolved.append(found)

    # Stable de-dup while preserving order
    unique: List[Path] = []
    seen = set()
    for p in resolved:
        rp = p.resolve()
        if rp not in seen:
            unique.append(rp)
            seen.add(rp)
    return unique


def _load_entries(path: Path) -> List[dict]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"'{path}' 不是合法 JSON：{exc}") from exc
    if not isinstance(data, list):
        raise ValueError(f"'{path}' 顶层必须是 JSON 数组")
    out = []
    for idx, entry in enumerate(data):
        if not isinstance(entry, dict):
            raise ValueError(f"'{path}' 中第 {idx} 项不是 JSON 对象")
        out.append(entry)
    return out


def main() -> int:
    args = _parse_args()
    build_dir = Path(args.build_dir).resolve()
    output = Path(args.output).resolve()

    if args.inputs:
        inputs = _resolve_inputs(args.inputs, build_dir)
    else:
        inputs = _discover_default(build_dir)
        # 默认输出缺失目录；--verbose-missing 仅作向后兼容开关保留。
        missing = _discover_missing_first_level(build_dir)
        if missing:
            print("以下一级构建目录缺少 compile_commands.json：")
            for p in missing:
                print(f"  - {p}")
        else:
            print("一级构建目录均已包含 compile_commands.json。")

    if not inputs:
        print(
            f"未找到可合并的 compile_commands.json，已检查目录：{build_dir}",
            file=sys.stderr,
        )
        return 1

    merged: List[dict] = []
    for path in inputs:
        merged.extend(_load_entries(path))

    # De-dup by (file, directory) so the latest source wins if repeated.
    deduped = {}
    for entry in merged:
        key = (entry.get("file"), entry.get("directory"))
        deduped[key] = entry
    result = list(deduped.values())

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"已合并 {len(inputs)} 个文件 -> {output}")
    for p in inputs:
        print(f"  - {p}")
    print(f"合并后条目数：{len(result)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
