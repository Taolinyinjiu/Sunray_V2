#!/usr/bin/env python3

import curses
import locale
import textwrap
import threading
import time

import rospy

from sunray_msgs.msg import SystemInfo
from sunray_msgs.srv import GetFeatures
from sunray_msgs.srv import ListFeatures
from sunray_msgs.srv import StartFeature
from sunray_msgs.srv import StopFeature


locale.setlocale(locale.LC_ALL, "")


class SunraySystemTUI:
    def __init__(self):
        rospy.init_node("sunray_system_tui", anonymous=False, disable_signals=True)

        self.list_features = rospy.ServiceProxy("/sunray_system/list_features", ListFeatures)
        self.get_features = rospy.ServiceProxy("/sunray_system/get_features", GetFeatures)
        self.start_feature = rospy.ServiceProxy("/sunray_system/start_feature", StartFeature)
        self.stop_feature = rospy.ServiceProxy("/sunray_system/stop_feature", StopFeature)

        self.features = []
        self.grouped_rows = []
        self.feature_details = {}
        self.selected_index = 0
        self.status_message = "正在初始化..."
        self.last_refresh_time = 0.0
        self.system_info = None
        self.system_info_time = 0.0
        self.lock = threading.RLock()
        self.start_mode_terminal = False

        self.system_info_sub = rospy.Subscriber("/sunray/system_info", SystemInfo, self.on_system_info, queue_size=1)

    def on_system_info(self, message):
        with self.lock:
            self.system_info = message
            self.system_info_time = time.time()

    def refresh(self):
        with self.lock:
            try:
                response = self.list_features()
                self.features = []
                for name in response.feature_names:
                    detail = self.get_features(name)
                    self.features.append(
                        {
                            "name": name,
                            "group": detail.group or "未分组",
                            "running": detail.running,
                            "description": detail.description,
                        }
                    )

                self.features = sorted(self.features, key=lambda item: (item["group"], item["name"]))
                self.rebuild_grouped_rows()

                if self.features:
                    self.selected_index = max(0, min(self.selected_index, len(self.features) - 1))
                    current_name = self.features[self.selected_index]["name"]
                    detail = self.get_features(current_name)
                    self.feature_details[current_name] = {
                        "name": detail.name,
                        "group": detail.group or "未分组",
                        "running": detail.running,
                        "description": detail.description,
                        "auto_start": detail.auto_start,
                        "depends_on": list(detail.depends_on),
                        "stop_timeout_sec": detail.stop_timeout_sec,
                        "start_preview_units": list(detail.start_preview_units),
                        "start_preview_commands": list(detail.start_preview_commands),
                        "message": detail.message,
                    }

                self.status_message = "刷新成功"
                self.last_refresh_time = time.time()
            except Exception as exc:
                self.status_message = "刷新失败: %s" % exc

    def run(self, stdscr):
        self.init_colors()
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(200)

        self.refresh()

        while not rospy.is_shutdown():
            self.draw(stdscr)
            key = stdscr.getch()

            if key == -1:
                if time.time() - self.last_refresh_time > 1.5:
                    self.refresh()
                continue

            if key == 27:
                break
            if key == curses.KEY_UP:
                if self.features:
                    self.selected_index = (self.selected_index - 1) % len(self.features)
                    self.refresh_selected_only()
            elif key == curses.KEY_DOWN:
                if self.features:
                    self.selected_index = (self.selected_index + 1) % len(self.features)
                    self.refresh_selected_only()
            elif key in (curses.KEY_F1, curses.KEY_F0 + 1):
                self.start_mode_terminal = not self.start_mode_terminal
                self.status_message = "启动模式: %s" % ("终端启动" if self.start_mode_terminal else "后台启动")
            elif key in (curses.KEY_F2, curses.KEY_F0 + 2):
                self.start_selected()
            elif key in (curses.KEY_F3, curses.KEY_F0 + 3):
                self.stop_selected()
            elif key in (curses.KEY_F5, curses.KEY_F0 + 5):
                self.refresh()

        stdscr.nodelay(False)

    def refresh_selected_only(self):
        if not self.features:
            return
        with self.lock:
            try:
                current_name = self.features[self.selected_index]["name"]
                detail = self.get_features(current_name)
                self.feature_details[current_name] = {
                    "name": detail.name,
                    "group": detail.group or "未分组",
                    "running": detail.running,
                    "description": detail.description,
                    "auto_start": detail.auto_start,
                    "depends_on": list(detail.depends_on),
                    "stop_timeout_sec": detail.stop_timeout_sec,
                    "start_preview_units": list(detail.start_preview_units),
                    "start_preview_commands": list(detail.start_preview_commands),
                    "message": detail.message,
                }
                self.last_refresh_time = time.time()
            except Exception as exc:
                self.status_message = "状态查询失败: %s" % exc

    def rebuild_grouped_rows(self):
        grouped = {}
        for feature in self.features:
            grouped.setdefault(feature["group"], []).append(feature)

        rows = []
        for group_name in sorted(grouped.keys()):
            group_features = sorted(grouped[group_name], key=lambda item: item["name"])
            running_count = sum(1 for feature in group_features if feature["running"])
            rows.append(
                {
                    "type": "group",
                    "label": group_name,
                    "running_count": running_count,
                    "total_count": len(group_features),
                }
            )
            for feature in group_features:
                rows.append({"type": "feature", "feature": feature})
        self.grouped_rows = rows

    def draw_inline_field(self, stdscr, y, left, width, title, value, title_pair=6, value_pair=2):
        if width < 8:
            return
        text = "%s: %s" % (title, value or "-")
        if len(text) > width:
            text = text[: max(3, width - 1)] + "…"
        self.safe_addnstr(stdscr, y, left, text, width, self.attr(value_pair))
        title_text = "%s:" % title
        self.safe_addnstr(stdscr, y, left, title_text, min(len(title_text), width), self.attr(title_pair, curses.A_BOLD))

    def draw_preview_panel(self, stdscr, top, left, width, height, preview_units, preview_commands):
        if height < 3 or width < 10:
            return
        self.safe_addnstr(stdscr, top, left, "启动预览", width, self.attr(6, curses.A_BOLD))
        y = top + 1
        bottom = top + height - 1
        if not preview_units:
            self.safe_addnstr(stdscr, y, left, "- (无)", width, self.attr(5))
            return
        for index, preview_unit in enumerate(preview_units):
            if y > bottom:
                return
            self.safe_addnstr(stdscr, y, left, "- " + preview_unit, width, self.attr(5, curses.A_BOLD))
            y += 1
            command_text = ""
            if index < len(preview_commands):
                command_text = preview_commands[index]
            for wrapped in textwrap.wrap("  " + command_text, width=width) or [""]:
                if y > bottom:
                    return
                self.safe_addnstr(stdscr, y, left, wrapped, width, self.attr(10))
                y += 1

    def draw_feature_row(self, stdscr, y, left, width, feature, selected):
        if width < 12:
            return
        running = feature["running"]
        marker = "●"
        status_badge = "ON" if running else "OFF"
        badge_attr = self.attr(7 if running else 8, curses.A_BOLD)
        text_attr = self.attr(6, curses.A_BOLD) if selected else self.attr(2)
        icon_attr = self.attr(3 if running else 4, curses.A_BOLD)

        if selected:
            self.safe_addnstr(stdscr, y, left, " " * width, width, self.attr(10))

        badge_width = len(status_badge) + 2
        name_width = max(6, width - badge_width - 5)
        feature_name = feature["name"]
        if len(feature_name) > name_width:
            feature_name = feature_name[: max(3, name_width - 1)] + "…"

        self.safe_addnstr(stdscr, y, left + 1, marker, 2, icon_attr)
        self.safe_addnstr(stdscr, y, left + 3, feature_name, name_width, text_attr)
        self.safe_addnstr(stdscr, y, left + width - badge_width - 1, (" %s " % status_badge), badge_width, badge_attr)

    def start_selected(self):
        if not self.features:
            return
        feature_name = self.features[self.selected_index]["name"]
        try:
            response = self.start_feature(
                feature_name=feature_name,
                override_args=[],
                restart_if_running=False,
                start_with_terminal=self.start_mode_terminal,
            )
            self.status_message = response.message
            self.refresh()
        except Exception as exc:
            self.status_message = "启动失败: %s" % exc

    def stop_selected(self):
        if not self.features:
            return
        feature_name = self.features[self.selected_index]["name"]
        try:
            response = self.stop_feature(feature_name=feature_name, force=False)
            self.status_message = response.message
            self.refresh()
        except Exception as exc:
            self.status_message = "停止失败: %s" % exc

    def init_colors(self):
        self.colors_enabled = curses.has_colors()
        if not self.colors_enabled:
            return

        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)
        curses.init_pair(2, curses.COLOR_WHITE, -1)
        curses.init_pair(3, curses.COLOR_GREEN, -1)
        curses.init_pair(4, curses.COLOR_RED, -1)
        curses.init_pair(5, curses.COLOR_YELLOW, -1)
        curses.init_pair(6, curses.COLOR_CYAN, -1)
        curses.init_pair(7, curses.COLOR_BLACK, curses.COLOR_GREEN)
        curses.init_pair(8, curses.COLOR_BLACK, curses.COLOR_RED)
        curses.init_pair(9, curses.COLOR_BLACK, curses.COLOR_YELLOW)
        curses.init_pair(10, curses.COLOR_BLUE, -1)
        curses.init_pair(11, curses.COLOR_MAGENTA, -1)

    def attr(self, pair_id=0, extra=0):
        if not getattr(self, "colors_enabled", False) or pair_id <= 0:
            return extra
        return curses.color_pair(pair_id) | extra

    def safe_addnstr(self, stdscr, y, x, text, max_chars, attr=0):
        max_y, max_x = stdscr.getmaxyx()
        if y < 0 or x < 0 or y >= max_y or x >= max_x:
            return
        available = max_x - x
        if available <= 0:
            return
        safe_width = min(max(0, max_chars), max(0, available - 1))
        if safe_width <= 0:
            return
        try:
            stdscr.addnstr(y, x, text, safe_width, attr)
        except curses.error:
            pass

    def safe_hline(self, stdscr, y, x, ch, count):
        max_y, max_x = stdscr.getmaxyx()
        if y < 0 or x < 0 or y >= max_y or x >= max_x:
            return
        safe_count = min(count, max_x - x)
        if safe_count <= 0:
            return
        try:
            stdscr.hline(y, x, ch, safe_count)
        except curses.error:
            pass

    def safe_vline(self, stdscr, y, x, ch, count):
        max_y, max_x = stdscr.getmaxyx()
        if y < 0 or x < 0 or y >= max_y or x >= max_x:
            return
        safe_count = min(count, max_y - y)
        if safe_count <= 0:
            return
        try:
            stdscr.vline(y, x, ch, safe_count)
        except curses.error:
            pass

    def safe_addch(self, stdscr, y, x, ch, attr=0):
        max_y, max_x = stdscr.getmaxyx()
        if y < 0 or x < 0 or y >= max_y or x >= max_x:
            return
        if x >= max_x - 1:
            return
        try:
            stdscr.addch(y, x, ch, attr)
        except curses.error:
            pass

    def draw_box(self, stdscr, top, left, height, width, title, title_attr=0, body_attr=0):
        max_y, max_x = stdscr.getmaxyx()
        if top < 0 or left < 0 or top >= max_y or left >= max_x:
            return
        height = min(height, max_y - top)
        width = min(width, max_x - left)
        if height < 3 or width < 4:
            return
        stdscr.attrset(body_attr)
        for row in range(top, top + height):
            self.safe_addnstr(stdscr, row, left, " " * max(0, width), width, body_attr)
        stdscr.attrset(title_attr)
        self.safe_addnstr(stdscr, top, left + 2, " %s " % title, max(0, width - 4), title_attr)
        stdscr.attrset(body_attr)
        self.safe_hline(stdscr, top, left, curses.ACS_HLINE, width)
        self.safe_hline(stdscr, top + height - 1, left, curses.ACS_HLINE, width)
        self.safe_vline(stdscr, top, left, curses.ACS_VLINE, height)
        self.safe_vline(stdscr, top, left + width - 1, curses.ACS_VLINE, height)
        self.safe_addch(stdscr, top, left, curses.ACS_ULCORNER)
        self.safe_addch(stdscr, top, left + width - 1, curses.ACS_URCORNER)
        self.safe_addch(stdscr, top + height - 1, left, curses.ACS_LLCORNER)
        self.safe_addch(stdscr, top + height - 1, left + width - 1, curses.ACS_LRCORNER)

    def draw_meter(self, stdscr, y, x, width, label, value):
        label_text = "%s %5.1f%%" % (label, value)
        self.safe_addnstr(stdscr, y, x, label_text, max(1, width), self.attr(6, curses.A_BOLD))
        bar_width = max(10, width - 12)
        fill_width = int(max(0.0, min(100.0, value)) * bar_width / 100.0)
        bar_y = y + 1
        empty_width = max(0, bar_width - fill_width)
        color_pair = 3 if value < 60.0 else (9 if value < 85.0 else 8)
        self.safe_addnstr(stdscr, bar_y, x, "[", 1, self.attr(10))
        if fill_width > 0:
            self.safe_addnstr(stdscr, bar_y, x + 1, "█" * fill_width, fill_width, self.attr(color_pair, curses.A_BOLD))
        if empty_width > 0:
            self.safe_addnstr(stdscr, bar_y, x + 1 + fill_width, "·" * empty_width, empty_width, self.attr(10))
        self.safe_addnstr(stdscr, bar_y, x + 1 + bar_width, "]", 1, self.attr(10))

    def draw(self, stdscr):
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        stdscr.bkgd(" ", self.attr(0))

        if height < 16 or width < 72:
            self.safe_addnstr(stdscr, 0, 0, "Sunray System TUI", width, self.attr(1, curses.A_BOLD))
            self.safe_addnstr(stdscr, 2, 0, "终端窗口太小，当前至少需要 72x16。", width, self.attr(8, curses.A_BOLD))
            self.safe_addnstr(stdscr, 4, 0, "请放大终端后重试。", width, self.attr(2))
            footer_text = "Esc 退出"
            footer_x = max(0, (width - len(footer_text)) // 2)
            self.safe_addnstr(stdscr, height - 1, footer_x, footer_text, len(footer_text), self.attr(2))
            stdscr.refresh()
            return

        header_attr = self.attr(1, curses.A_BOLD)
        self.safe_addnstr(stdscr, 0, 0, " Sunray System TUI ".ljust(width), width, header_attr)
        status_text = "状态: %s" % self.status_message
        self.safe_addnstr(stdscr, 1, 0, status_text.ljust(width), width, self.attr(2))
        mode_text = "启动模式: %s" % ("终端启动" if self.start_mode_terminal else "后台启动")
        self.safe_addnstr(stdscr, 2, 0, mode_text.ljust(width), width, self.attr(11, curses.A_BOLD))

        content_top = 4
        footer_height = 2
        content_height = max(10, height - content_top - footer_height)
        left_width = max(30, min(42, width // 3))
        right_width = width - left_width
        if right_width < 36:
            right_width = 36
            left_width = width - right_width
        left_width = max(30, left_width)
        right_width = max(36, width - left_width)
        detail_height = max(9, content_height // 2)
        system_height = content_height - detail_height
        if system_height < 8:
            system_height = 8
            detail_height = content_height - system_height

        self.draw_box(stdscr, content_top, 0, content_height, left_width, "功能列表", self.attr(6, curses.A_BOLD))
        self.draw_box(stdscr, content_top, left_width, detail_height, right_width, "当前功能详情", self.attr(6, curses.A_BOLD))
        self.draw_box(stdscr, content_top + detail_height, left_width, system_height, right_width, "系统信息 /sunray/system_info", self.attr(6, curses.A_BOLD))

        list_inner_top = content_top + 1
        list_inner_height = max(1, content_height - 2)
        selected_row_index = self.find_selected_row_index()
        scroll_offset = 0
        if selected_row_index >= list_inner_height:
            scroll_offset = selected_row_index - list_inner_height + 1

        for row_idx in range(list_inner_height):
            row_index = scroll_offset + row_idx
            if row_index >= len(self.grouped_rows):
                break

            row = self.grouped_rows[row_index]
            if row["type"] == "group":
                group_text = "[ %s  %d/%d ]" % (row["label"], row["running_count"], row["total_count"])
                self.safe_addnstr(
                    stdscr,
                    list_inner_top + row_idx,
                    1,
                    group_text.ljust(left_width - 3),
                    left_width - 3,
                    self.attr(6, curses.A_BOLD),
                )
                continue

            feature = row["feature"]
            selected = feature["name"] == self.features[self.selected_index]["name"]
            self.draw_feature_row(stdscr, list_inner_top + row_idx, 1, left_width - 3, feature, selected)

        if self.features:
            current_name = self.features[self.selected_index]["name"]
            detail = self.feature_details.get(current_name, {})
            panel_left = left_width + 2
            panel_width = max(20, right_width - 4)
            column_gap = 3
            column_width = max(12, (panel_width - column_gap) // 2)
            left_col = panel_left
            right_col = panel_left + column_width + column_gap
            info_top = content_top + 1

            self.draw_inline_field(stdscr, info_top, left_col, column_width, "名称", detail.get("name", current_name), 6, 2)
            self.draw_inline_field(stdscr, info_top, right_col, column_width, "分组", detail.get("group", "未分组"), 11, 2)
            self.draw_inline_field(
                stdscr,
                info_top + 1,
                left_col,
                column_width,
                "状态",
                "运行中" if detail.get("running", False) else "未运行",
                3 if detail.get("running", False) else 4,
                2,
            )
            self.draw_inline_field(
                stdscr,
                info_top + 1,
                right_col,
                column_width,
                "自动启动",
                "是" if detail.get("auto_start", False) else "否",
                5,
                2,
            )
            self.draw_inline_field(
                stdscr,
                info_top + 2,
                left_col,
                column_width,
                "停止超时",
                "%.1fs" % float(detail.get("stop_timeout_sec", 0.0)),
                10,
                2,
            )
            self.draw_inline_field(
                stdscr,
                info_top + 2,
                right_col,
                column_width,
                "依赖功能",
                ", ".join(detail.get("depends_on", [])) or "(无)",
                6,
                2,
            )

            desc_top = info_top + 4
            desc_width = panel_width
            desc_text = "描述: %s" % (detail.get("description", "") or "-")
            desc_lines = textwrap.wrap(desc_text, width=desc_width) or ["-"]
            for idx, line in enumerate(desc_lines[:2]):
                self.safe_addnstr(stdscr, desc_top + idx, panel_left, line, desc_width, self.attr(2))

            preview_top = desc_top + len(desc_lines) + 1
            preview_height = content_top + detail_height - preview_top - 1
            self.draw_preview_panel(
                stdscr,
                preview_top,
                panel_left,
                panel_width,
                preview_height,
                detail.get("start_preview_units", []),
                detail.get("start_preview_commands", []),
            )

        system_top = content_top + detail_height + 1
        system_max_width = max(20, right_width - 3)
        info = self.system_info
        if info is not None:
            self.draw_meter(stdscr, system_top, left_width + 1, system_max_width, "CPU", info.cpu_percent)
            self.draw_meter(stdscr, system_top + 3, left_width + 1, system_max_width, "内存", info.memory_percent)

            nodes_title = "活跃 ROS 节点 (%d)" % len(info.active_ros_nodes)
            self.safe_addnstr(stdscr, system_top + 6, left_width + 1, nodes_title, system_max_width, self.attr(6, curses.A_BOLD))

            node_lines = max(1, content_top + content_height - 2 - (system_top + 7))
            visible_nodes = info.active_ros_nodes[:node_lines]
            for idx, node_name in enumerate(visible_nodes):
                self.safe_addnstr(stdscr, system_top + 7 + idx, left_width + 1, "- " + node_name, system_max_width, self.attr(2))
            if len(info.active_ros_nodes) > len(visible_nodes) and system_top + 7 + len(visible_nodes) < height - 2:
                remaining = len(info.active_ros_nodes) - len(visible_nodes)
                self.safe_addnstr(stdscr, system_top + 7 + len(visible_nodes), left_width + 1, "... 还有 %d 个节点" % remaining, system_max_width, self.attr(5))
        else:
            self.safe_addnstr(stdscr, system_top, left_width + 1, "等待 /sunray/system_info ...", system_max_width, self.attr(5, curses.A_BOLD))

        footer_text = "方向键选择   F1 切换启动模式   F2 启动   F3 停止   F5 刷新   Esc 退出"
        footer_x = max(0, (width - len(footer_text)) // 2)
        self.safe_addnstr(stdscr, height - 1, footer_x, footer_text, len(footer_text), self.attr(1))
        stdscr.refresh()

    def find_selected_row_index(self):
        if not self.features:
            return 0
        selected_name = self.features[self.selected_index]["name"]
        for index, row in enumerate(self.grouped_rows):
            if row["type"] == "feature" and row["feature"]["name"] == selected_name:
                return index
        return 0


def main():
    app = SunraySystemTUI()
    curses.wrapper(app.run)


if __name__ == "__main__":
    main()
