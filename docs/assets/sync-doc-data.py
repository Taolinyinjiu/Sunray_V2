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


def main():
    chapters = json.loads((ROOT / "content" / "chapters.json").read_text(encoding="utf-8"))
    data = []
    for chapter in chapters:
        section_by_file = {}
        for index, file_path in enumerate(chapter["files"]):
            if index < len(chapter.get("sections", [])):
                section_by_file[file_path] = chapter["sections"][index]

        for file_path in chapter["files"]:
            markdown = (ROOT / "content" / file_path).read_text(encoding="utf-8")
            section = section_by_file.get(file_path, {})
            title = chapter["title"]
            if len(chapter["files"]) > 1:
                title = section.get("title") or title_from_markdown(markdown, chapter["title"])
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
