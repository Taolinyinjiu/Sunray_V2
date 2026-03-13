#!/usr/bin/env python3

import argparse
import difflib
import re
import sys
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


@dataclass(frozen=True)
class Transition:
    source: str
    target: str
    event: str
    guard: str = ""
    synthetic: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate Sunray FSM Mermaid diagram from C++ source."
    )
    repo_root = Path(__file__).resolve().parents[3]
    parser.add_argument(
        "--header",
        default=str(repo_root / "control/uav_control/include/sunray_statemachine/sunray_statemachine.h"),
        help="Path to sunray_statemachine.h",
    )
    parser.add_argument(
        "--source",
        default=str(repo_root / "control/uav_control/src/sunray_statemachine/sunray_statemachine.cpp"),
        help="Path to sunray_statemachine.cpp",
    )
    parser.add_argument(
        "--output",
        default=str(repo_root / "control/uav_control/state_machine_diagram.md"),
        help="Output Markdown file path",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Write generated Markdown to --output",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail if --output is not up to date with generated content",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Treat validation warnings as errors",
    )
    return parser.parse_args()


def strip_cpp_comments(text: str) -> str:
    text = re.sub(r"//.*", "", text)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return text


def extract_braced_block(text: str, start_idx: int) -> Tuple[str, int]:
    if start_idx < 0 or start_idx >= len(text) or text[start_idx] != "{":
        raise ValueError("extract_braced_block: start_idx must point to '{'")
    depth = 0
    for idx in range(start_idx, len(text)):
        ch = text[idx]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[start_idx + 1 : idx], idx
    raise ValueError("Unbalanced braces while extracting block")


def parse_enum_entries(header_text: str, enum_name: str) -> List[str]:
    pattern = re.compile(rf"enum\s+class\s+{re.escape(enum_name)}\s*\{{", re.M)
    match = pattern.search(header_text)
    if not match:
        raise ValueError(f"Cannot find enum class {enum_name}")
    open_brace = header_text.find("{", match.start())
    block, _ = extract_braced_block(header_text, open_brace)

    entries: List[str] = []
    for raw_line in block.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if "," in line:
            line = line.split(",", 1)[0].strip()
        if "=" in line:
            line = line.split("=", 1)[0].strip()
        if not line:
            continue
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", line):
            entries.append(line)
    if not entries:
        raise ValueError(f"Enum {enum_name} has no entries")
    return entries


def extract_function_body(source_text: str, signature_regex: str) -> str:
    sig = re.compile(signature_regex, re.M)
    match = sig.search(source_text)
    if not match:
        raise ValueError(f"Cannot find function with pattern: {signature_regex}")
    open_brace = source_text.find("{", match.end())
    if open_brace < 0:
        raise ValueError("Cannot find function opening brace")
    body, _ = extract_braced_block(source_text, open_brace)
    return body


def split_switch_case_blocks(switch_body: str) -> List[Tuple[List[str], str]]:
    blocks: List[Tuple[List[str], str]] = []
    active_states: List[str] = []
    active_lines: List[str] = []
    depth = 0

    for line in switch_body.splitlines():
        stripped = line.strip()

        case_match = None
        if depth == 0:
            case_match = re.match(r"case\s+SunrayState::([A-Za-z_][A-Za-z0-9_]*)\s*:\s*$", stripped)

        if depth == 0 and case_match:
            if active_states and any(l.strip() for l in active_lines):
                blocks.append((list(active_states), "\n".join(active_lines)))
                active_lines = []
                active_states = []
            active_states.append(case_match.group(1))
        elif depth == 0 and stripped.startswith("default:"):
            if active_states and any(l.strip() for l in active_lines):
                blocks.append((list(active_states), "\n".join(active_lines)))
            active_states = []
            active_lines = []
        elif active_states:
            active_lines.append(line)

        depth += line.count("{") - line.count("}")

    if active_states and any(l.strip() for l in active_lines):
        blocks.append((list(active_states), "\n".join(active_lines)))

    return blocks


def extract_guards(condition: str) -> str:
    def trim_outer_parens(expr: str) -> str:
        s = expr.strip()
        while s.startswith("(") and s.endswith(")"):
            depth = 0
            valid = True
            for idx, ch in enumerate(s):
                if ch == "(":
                    depth += 1
                elif ch == ")":
                    depth -= 1
                    if depth < 0:
                        valid = False
                        break
                    if depth == 0 and idx != len(s) - 1:
                        valid = False
                        break
            if not valid or depth != 0:
                break
            s = s[1:-1].strip()
        return s

    parts = re.split(r"&&|\|\|", condition)
    guard_parts: List[str] = []
    for raw in parts:
        p = raw.strip()
        if not p:
            continue
        if "SunrayEvent::" in p:
            continue
        p = trim_outer_parens(p)
        if p:
            guard_parts.append(p)

    deduped: List[str] = []
    seen = set()
    for p in guard_parts:
        if p not in seen:
            deduped.append(p)
            seen.add(p)

    return " && ".join(deduped)


def extract_transitions_from_case_block(source_states: Sequence[str], block_text: str) -> List[Transition]:
    transitions: List[Transition] = []
    if_pattern = re.compile(
        r"if\s*\((?P<cond>.*?)\)\s*\{\s*return\s+transition_to\s*\(\s*SunrayState::(?P<target>[A-Za-z_][A-Za-z0-9_]*)\s*\)\s*;\s*\}",
        re.S,
    )
    for match in if_pattern.finditer(block_text):
        condition = " ".join(match.group("cond").split())
        target = match.group("target")
        events = re.findall(r"SunrayEvent::([A-Za-z_][A-Za-z0-9_]*)", condition)
        guard = extract_guards(condition)
        if not events:
            events = ["INTERNAL"]
        for source in source_states:
            for event in events:
                transitions.append(
                    Transition(source=source, target=target, event=event, guard=guard)
                )
    return transitions


def parse_dispatch_transitions(source_text: str) -> List[Transition]:
    dispatch_body = extract_function_body(
        source_text,
        r"bool\s+Sunray_StateMachine::dispatch\s*\(\s*SunrayEvent\s+event\s*\)",
    )
    switch_match = re.search(r"switch\s*\(\s*current_state_\s*\)\s*\{", dispatch_body)
    if not switch_match:
        raise ValueError("Cannot find switch(current_state_) in dispatch()")
    switch_open = dispatch_body.find("{", switch_match.start())
    switch_body, _ = extract_braced_block(dispatch_body, switch_open)

    transitions: List[Transition] = []
    for states, case_block in split_switch_case_blocks(switch_body):
        transitions.extend(extract_transitions_from_case_block(states, case_block))
    return transitions


def inject_synthetic_transitions(transitions: List[Transition], states: Sequence[str]) -> List[Transition]:
    all_transitions = list(transitions)
    if "EMERGENCY_LAND" in states:
        flight_states = [s for s in states if s != "OFF" and s != "EMERGENCY_LAND"]
        for s in flight_states:
            all_transitions.append(
                Transition(
                    source=s,
                    target="EMERGENCY_LAND",
                    event="HEALTH_CHECK_FAILED",
                    guard="update() fallback",
                    synthetic=True,
                )
            )
    return all_transitions


def dedupe_transitions(transitions: Sequence[Transition]) -> List[Transition]:
    seen = set()
    result: List[Transition] = []
    for t in transitions:
        key = (t.source, t.target, t.event, t.guard, t.synthetic)
        if key not in seen:
            seen.add(key)
            result.append(t)
    return result


def validate_graph(
    states: Sequence[str],
    events: Sequence[str],
    transitions: Sequence[Transition],
) -> Tuple[List[str], List[str]]:
    errors: List[str] = []
    warnings: List[str] = []

    state_set = set(states)
    event_set = set(events)

    used_events = {t.event for t in transitions if t.event != "INTERNAL" and not t.synthetic}
    referenced_states = set()
    for t in transitions:
        referenced_states.add(t.source)
        referenced_states.add(t.target)
        if t.source not in state_set:
            errors.append(f"Unknown source state in transition: {t.source}")
        if t.target not in state_set:
            errors.append(f"Unknown target state in transition: {t.target}")
        if t.event not in {"INTERNAL", "HEALTH_CHECK_FAILED"} and t.event not in event_set:
            errors.append(f"Unknown event in transition: {t.event}")

    unused_events = [e for e in events if e not in used_events]
    if unused_events:
        warnings.append("Unused events in dispatch(): " + ", ".join(unused_events))

    incoming = defaultdict(int)
    outgoing = defaultdict(int)
    for t in transitions:
        incoming[t.target] += 1
        outgoing[t.source] += 1

    for s in states:
        if s != "OFF" and incoming[s] == 0:
            warnings.append(f"State has no incoming transitions: {s}")
        if s != "OFF" and outgoing[s] == 0:
            warnings.append(f"State has no outgoing transitions: {s}")

    reachable = set()
    graph = defaultdict(list)
    for t in transitions:
        graph[t.source].append(t.target)

    queue = deque(["OFF"])
    reachable.add("OFF")
    while queue:
        cur = queue.popleft()
        for nxt in graph[cur]:
            if nxt not in reachable:
                reachable.add(nxt)
                queue.append(nxt)

    unreachable = [s for s in states if s not in reachable]
    if unreachable:
        warnings.append("Unreachable states from OFF: " + ", ".join(unreachable))

    if not transitions:
        errors.append("No transitions extracted from dispatch().")

    if not referenced_states:
        errors.append("No states referenced in parsed transitions.")

    return errors, warnings


def render_mermaid(states: Sequence[str], transitions: Sequence[Transition]) -> str:
    state_order = {name: idx for idx, name in enumerate(states)}

    grouped: Dict[Tuple[str, str], List[Tuple[str, str, bool]]] = defaultdict(list)
    for t in transitions:
        grouped[(t.source, t.target)].append((t.event, t.guard, t.synthetic))

    lines = ["stateDiagram-v2", "    [*] --> OFF", ""]

    sorted_edges = sorted(
        grouped.items(), key=lambda kv: (state_order.get(kv[0][0], 999), state_order.get(kv[0][1], 999), kv[0][0], kv[0][1])
    )

    for (source, target), labels in sorted_edges:
        labels_sorted = sorted(labels, key=lambda x: (x[2], x[0], x[1]))
        label_chunks = []
        for event, guard, synthetic in labels_sorted:
            label = event
            if guard:
                label += f" [{guard}]"
            if synthetic:
                label += " (synthetic)"
            label_chunks.append(label)
        label_text = r"\n".join(label_chunks)
        lines.append(f"    {source} --> {target}: {label_text}")

    lines.extend(
        [
            "",
            "    note right of OFF",
            "      can_takeoff() =",
            "      state_available &&",
            "      takeoff_callback_ready &&",
            "      controller_registered &&",
            "      odometry_source_valid",
            "    end note",
        ]
    )
    return "\n".join(lines)


def render_markdown(
    mermaid_text: str,
    header_path: Path,
    source_path: Path,
    errors: Sequence[str],
    warnings: Sequence[str],
) -> str:
    lines = [
        "# Sunray FSM State Diagram (Auto-Generated)",
        "",
        "Do not edit this file manually. Regenerate with:",
        "",
        "```bash",
        "python3 control/uav_control/scripts/gen_fsm_diagram.py --write",
        "```",
        "",
        "Parsed from:",
        f"- `{header_path.as_posix()}`",
        f"- `{source_path.as_posix()}`",
        "",
        "```mermaid",
        mermaid_text,
        "```",
        "",
        "## Validation",
    ]

    if errors:
        lines.append("- Errors:")
        for err in errors:
            lines.append(f"  - {err}")
    else:
        lines.append("- Errors: none")

    if warnings:
        lines.append("- Warnings:")
        for warning in warnings:
            lines.append(f"  - {warning}")
    else:
        lines.append("- Warnings: none")

    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    header_path = Path(args.header).resolve()
    source_path = Path(args.source).resolve()
    output_path = Path(args.output).resolve()

    if not header_path.exists():
        print(f"[gen_fsm_diagram] header file not found: {header_path}", file=sys.stderr)
        return 1
    if not source_path.exists():
        print(f"[gen_fsm_diagram] source file not found: {source_path}", file=sys.stderr)
        return 1

    header_text = strip_cpp_comments(header_path.read_text(encoding="utf-8"))
    source_text = strip_cpp_comments(source_path.read_text(encoding="utf-8"))

    states = parse_enum_entries(header_text, "SunrayState")
    events = parse_enum_entries(header_text, "SunrayEvent")

    transitions = parse_dispatch_transitions(source_text)
    transitions = inject_synthetic_transitions(transitions, states)
    transitions = dedupe_transitions(transitions)

    errors, warnings = validate_graph(states, events, transitions)

    mermaid_text = render_mermaid(states, transitions)
    generated = render_markdown(mermaid_text, header_path, source_path, errors, warnings)

    if args.write:
        output_path.write_text(generated, encoding="utf-8")
        print(f"[gen_fsm_diagram] wrote: {output_path}")

    if args.check:
        if not output_path.exists():
            print(f"[gen_fsm_diagram] check failed: missing output file {output_path}", file=sys.stderr)
            return 2
        existing = output_path.read_text(encoding="utf-8")
        if existing != generated:
            print(f"[gen_fsm_diagram] check failed: {output_path} is out of date", file=sys.stderr)
            diff = "".join(
                difflib.unified_diff(
                    existing.splitlines(keepends=True),
                    generated.splitlines(keepends=True),
                    fromfile="existing",
                    tofile="generated",
                )
            )
            sys.stderr.write(diff)
            return 2
        print(f"[gen_fsm_diagram] check passed: {output_path} is up to date")

    if errors:
        for err in errors:
            print(f"[gen_fsm_diagram] ERROR: {err}", file=sys.stderr)
        return 3

    if warnings:
        for warning in warnings:
            print(f"[gen_fsm_diagram] WARN: {warning}")
        if args.strict:
            return 4

    if not args.write and not args.check:
        print(generated)

    return 0


if __name__ == "__main__":
    sys.exit(main())
