#!/usr/bin/env python3
from pathlib import Path
import json
import re

ROOT = Path(__file__).resolve().parents[1]


def title_from_markdown(markdown, fallback):
    title_match = re.search(r"^##\s+(.+?)\s*$", markdown, re.MULTILINE)
    if title_match:
        return title_match.group(1).strip(" #")
    title_match = re.search(r"^#\s+(.+?)\s*$", markdown, re.MULTILINE)
    if title_match:
        return title_match.group(1).strip(" #")
    return fallback


def title_from_path(file_path):
    name = Path(file_path).stem
    if name == "index":
        return "概览"
    return name


def collect_tree_entries(nodes, entries=None):
    if entries is None:
        entries = []
    if not isinstance(nodes, list):
        return entries
    for node in nodes:
        if "file" in node:
            entries.append(
                {
                    "file": node["file"],
                    "title": node.get("title") or title_from_path(node["file"]),
                }
            )
        collect_tree_entries(node.get("children"), entries)
    return entries


def chapter_entries(chapter):
    if chapter.get("children"):
        entries = collect_tree_entries(chapter["children"])
    else:
        entries = []
        files = chapter.get("files", [])
        sections = chapter.get("sections", [])
        for index, file_path in enumerate(files):
            section = sections[index] if index < len(sections) else {}
            title = chapter["title"]
            if len(files) > 1:
                title = section.get("title") or title_from_path(file_path)
            entries.append({"file": file_path, "title": title})

    unique_entries = []
    seen = set()
    for entry in entries:
        file_path = entry["file"]
        if file_path in seen:
            continue
        seen.add(file_path)
        unique_entries.append(entry)
    return unique_entries


def main():
    chapters = json.loads((ROOT / "content" / "chapters.json").read_text(encoding="utf-8"))
    data = []
    for chapter in chapters:
        for entry in chapter_entries(chapter):
            file_path = entry["file"]
            markdown = (ROOT / "content" / file_path).read_text(encoding="utf-8")
            title = entry.get("title") or title_from_markdown(markdown, chapter["title"])
            data.append(
                {
                    "file": "content/" + file_path,
                    "title": title,
                    "chapterTitle": chapter["title"],
                    "group": chapter["group"],
                    "content": markdown,
                }
            )

    output = "window.SUNRAY_CHAPTERS = "
    output += json.dumps(chapters, ensure_ascii=False, indent=2)
    output += ";\nwindow.SUNRAY_DOCS = "
    output += json.dumps(data, ensure_ascii=False, indent=2)
    output += ";\n"
    (ROOT / "assets" / "doc-data.js").write_text(output, encoding="utf-8")


if __name__ == "__main__":
    main()
