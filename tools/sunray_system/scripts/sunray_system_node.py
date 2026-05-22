#!/usr/bin/env python3

import glob
import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
import uuid

import rosgraph
import rosnode
import rospkg
import rospy
import yaml

from sunray_msgs.msg import SystemInfo
from sunray_msgs.srv import GetFeatures, GetFeaturesResponse
from sunray_msgs.srv import ListFeatures, ListFeaturesResponse
from sunray_msgs.srv import StartFeature, StartFeatureResponse
from sunray_msgs.srv import StopFeature, StopFeatureResponse


class LaunchUnitRuntime:
    def __init__(
        self,
        unit_name,
        command,
        process,
        pid_file="",
        shell_pid_file="",
        script_path="",
        terminal_mode=False,
        startup_grace_sec=3.0,
    ):
        self.unit_name = unit_name
        self.command = command
        self.process = process
        self.started_at = time.time()
        self.pid_file = pid_file
        self.shell_pid_file = shell_pid_file
        self.script_path = script_path
        self.terminal_mode = terminal_mode
        self.startup_grace_sec = startup_grace_sec


class FeatureRuntime:
    def __init__(self):
        self.units = []
        self.last_error = ""

    @property
    def running(self):
        return any(self._is_runtime_unit_running(unit) for unit in self.units)

    def _is_runtime_unit_running(self, unit):
        target_pid = 0
        if getattr(unit, "pid_file", ""):
            try:
                with open(unit.pid_file, "r") as handle:
                    target_pid = int(handle.read().strip())
            except Exception:
                target_pid = 0
        if target_pid > 0:
            try:
                os.kill(target_pid, 0)
                return True
            except OSError:
                pass
        if getattr(unit, "terminal_mode", False):
            shell_pid = 0
            if getattr(unit, "shell_pid_file", ""):
                try:
                    with open(unit.shell_pid_file, "r") as handle:
                        shell_pid = int(handle.read().strip())
                except Exception:
                    shell_pid = 0
            if shell_pid > 0:
                try:
                    os.kill(shell_pid, 0)
                    return True
                except OSError:
                    pass
            return False
        return unit.process.poll() is None


class SunraySystemSupervisor:
    def __init__(self):
        self._lock = threading.RLock()
        self._rospack = rospkg.RosPack()
        self._features = {}
        self._runtimes = {}
        self._uuid = str(uuid.uuid4())
        self._last_cpu_sample = self._read_cpu_sample()
        self._start_time = time.time()
        self._runtime_dir = os.path.join("/tmp", "sunray_system_runtime")
        os.makedirs(self._runtime_dir, exist_ok=True)

        self._config_file = rospy.get_param("~config_file")
        self._load_config(self._config_file)

        self._system_info_pub = rospy.Publisher("/sunray/system_info", SystemInfo, queue_size=1)

        self._list_srv = rospy.Service("~list_features", ListFeatures, self._handle_list_features)
        self._start_srv = rospy.Service("~start_feature", StartFeature, self._handle_start_feature)
        self._stop_srv = rospy.Service("~stop_feature", StopFeature, self._handle_stop_feature)
        self._status_srv = rospy.Service("~get_features", GetFeatures, self._handle_get_features)

        self._monitor_timer = rospy.Timer(rospy.Duration(1.0), self._monitor_processes)
        self._system_info_timer = rospy.Timer(rospy.Duration(1.0), self._publish_system_info)
        self._autostart_features()
        rospy.loginfo("sunray_system ready, loaded %d features from %s", len(self._features), self._config_file)

    def _load_config(self, path):
        with open(path, "r") as handle:
            data = yaml.safe_load(handle) or {}

        features = data.get("features", [])
        loaded = {}
        for feature in features:
            name = feature.get("name", "").strip()
            if not name:
                raise ValueError("Feature entry missing non-empty name")
            if name in loaded:
                raise ValueError("Duplicate feature name: %s" % name)
            loaded[name] = feature
            self._runtimes[name] = FeatureRuntime()

        self._features = loaded

    def _autostart_features(self):
        for name, feature in self._features.items():
            if feature.get("auto_start", False):
                success, message, _ = self._start_feature(name, [], False, False, set())
                if not success:
                    rospy.logerr("autostart feature %s failed: %s", name, message)

    def _handle_list_features(self, _request):
        with self._lock:
            names = []
            for name in sorted(self._features.keys()):
                names.append(name)
            return ListFeaturesResponse(feature_names=names)

    def _handle_start_feature(self, request):
        with self._lock:
            success, message, _ = self._start_feature(
                request.feature_name.strip(),
                list(request.override_args),
                request.restart_if_running,
                request.start_with_terminal,
                set(),
            )
            return StartFeatureResponse(success=success, message=message)

    def _handle_stop_feature(self, request):
        with self._lock:
            success, message, _ = self._stop_feature(request.feature_name.strip(), request.force)
            return StopFeatureResponse(success=success, message=message)

    def _handle_get_features(self, request):
        with self._lock:
            feature_name = request.feature_name.strip()
            if feature_name not in self._features:
                return GetFeaturesResponse(
                    success=False,
                    message="unknown feature: %s" % feature_name,
                    name="",
                    group="",
                    running=False,
                    description="",
                    auto_start=False,
                    depends_on=[],
                    stop_timeout_sec=0.0,
                    start_preview_units=[],
                    start_preview_commands=[],
                )

            feature = self._features[feature_name]
            runtime = self._runtimes[feature_name]
            preview_units = []
            preview_commands = []
            message = "ok"
            try:
                preview_units, preview_commands = self._build_start_preview_units(feature_name)
            except Exception as exc:
                message = "preview unavailable: %s" % exc
                rospy.logwarn("feature=%s preview unavailable: %s", feature_name, exc)

            return GetFeaturesResponse(
                success=True,
                message=message,
                name=feature_name,
                group=feature.get("group", "未分组"),
                running=runtime.running,
                description=feature.get("description", ""),
                auto_start=bool(feature.get("auto_start", False)),
                depends_on=list(feature.get("depends_on", [])),
                stop_timeout_sec=float(feature.get("stop_timeout_sec", 8.0)),
                start_preview_units=preview_units,
                start_preview_commands=preview_commands,
            )

    def _start_feature(self, feature_name, override_args, restart_if_running, start_with_terminal, visiting):
        if start_with_terminal:
            return self._start_feature_with_terminal(feature_name, override_args, restart_if_running)

        if feature_name not in self._features:
            return False, "unknown feature: %s" % feature_name, []

        if feature_name in visiting:
            return False, "cyclic dependency detected at %s" % feature_name, []

        runtime = self._runtimes[feature_name]
        if runtime.running:
            if not restart_if_running:
                active_units = [unit.unit_name for unit in runtime.units if unit.process.poll() is None]
                return True, "feature already running", active_units
            self._stop_feature(feature_name, False)

        feature = self._features[feature_name]
        visiting.add(feature_name)

        started_units = []
        for dependency in feature.get("depends_on", []):
            dep_success, dep_message, dep_units = self._start_feature(
                dependency, [], False, start_with_terminal, visiting
            )
            if not dep_success:
                runtime.last_error = dep_message
                visiting.remove(feature_name)
                return False, "dependency %s failed: %s" % (dependency, dep_message), started_units + dep_units

        launches = feature.get("launches", [])
        if not launches:
            visiting.remove(feature_name)
            return False, "feature has no launches: %s" % feature_name, []

        runtime.units = []
        runtime.last_error = ""

        try:
            if start_with_terminal:
                prepared_units = []
                accumulated_delay_sec = 0.0
                for launch in launches:
                    unit_name = launch.get("name") or launch.get("file") or "launch"
                    command = self._build_command(launch, override_args)
                    pid_file = self._make_runtime_file(feature_name, unit_name, ".pid")
                    script_path = self._create_terminal_launch_script(
                        feature_name,
                        unit_name,
                        command,
                        pid_file,
                        accumulated_delay_sec,
                    )
                    prepared_units.append(
                        {
                            "unit_name": unit_name,
                            "command": command,
                            "pid_file": pid_file,
                            "script_path": script_path,
                            "startup_grace_sec": accumulated_delay_sec + 5.0,
                        }
                    )
                    started_units.append(unit_name)
                    accumulated_delay_sec += max(0.0, float(launch.get("delay_sec", 0.0)))

                terminal_command = self._build_feature_terminal_command(feature_name, prepared_units)
                if not terminal_command:
                    raise RuntimeError("gnome-terminal is required for tabbed terminal launch")

                terminal_process = subprocess.Popen(
                    terminal_command,
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

                for unit in prepared_units:
                    runtime.units.append(
                        LaunchUnitRuntime(
                            unit["unit_name"],
                            unit["command"],
                            terminal_process,
                            pid_file=unit["pid_file"],
                            script_path=unit["script_path"],
                            terminal_mode=True,
                            startup_grace_sec=unit["startup_grace_sec"],
                        )
                    )
                    rospy.loginfo(
                        "started feature=%s unit=%s terminal_pid=%d",
                        feature_name,
                        unit["unit_name"],
                        terminal_process.pid,
                    )
            else:
                for launch in launches:
                    unit_name = launch.get("name") or launch.get("file") or "launch"
                    command = self._build_command(launch, override_args)
                    pid_file = self._make_runtime_file(feature_name, unit_name, ".pid")
                    process = subprocess.Popen(
                        command,
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    self._write_pid_file(pid_file, process.pid)
                    runtime.units.append(
                        LaunchUnitRuntime(
                            unit_name,
                            command,
                            process,
                            pid_file=pid_file,
                            startup_grace_sec=3.0,
                        )
                    )

                    started_units.append(unit_name)
                    rospy.loginfo("started feature=%s unit=%s pid=%d", feature_name, unit_name, process.pid)

                    delay_sec = float(launch.get("delay_sec", 0.0))
                    if delay_sec > 0.0:
                        time.sleep(delay_sec)
        except Exception as exc:
            runtime.last_error = str(exc)
            self._stop_feature(feature_name, True)
            visiting.remove(feature_name)
            return False, "start feature failed: %s" % exc, started_units

        visiting.remove(feature_name)
        return True, "started feature %s" % feature_name, started_units

    def _start_feature_with_terminal(self, feature_name, override_args, restart_if_running):
        if feature_name not in self._features:
            return False, "unknown feature: %s" % feature_name, []

        runtime = self._runtimes[feature_name]
        if runtime.running:
            if not restart_if_running:
                active_units = [unit.unit_name for unit in runtime.units if self._is_unit_running(unit)]
                return True, "feature already running", active_units
            self._stop_feature(feature_name, False)

        ordered_features = self._resolve_feature_start_order(feature_name)
        features_to_launch = []
        for ordered_feature_name in ordered_features:
            ordered_runtime = self._runtimes[ordered_feature_name]
            if ordered_runtime.running:
                continue

            launches = self._features[ordered_feature_name].get("launches", [])
            if not launches:
                return False, "feature has no launches: %s" % ordered_feature_name, []

            features_to_launch.append(
                {
                    "feature_name": ordered_feature_name,
                    "launches": launches,
                    "override_args": override_args if ordered_feature_name == feature_name else [],
                }
            )

        if not features_to_launch:
            return True, "feature already running", []

        prepared_units = []
        started_units = []
        accumulated_delay_sec = 0.0
        try:
            for feature_plan in features_to_launch:
                current_feature_name = feature_plan["feature_name"]
                current_runtime = self._runtimes[current_feature_name]
                current_runtime.units = []
                current_runtime.last_error = ""

                for launch in feature_plan["launches"]:
                    unit_name = launch.get("name") or launch.get("file") or "launch"
                    tab_title = self._format_launch_display_name(current_feature_name, unit_name)

                    command = self._build_command(launch, feature_plan["override_args"])
                    pid_file = self._make_runtime_file(current_feature_name, unit_name, ".pid")
                    shell_pid_file = self._make_runtime_file(current_feature_name, unit_name, ".tabpid")
                    script_path = self._create_terminal_launch_script(
                        current_feature_name,
                        tab_title,
                        command,
                        pid_file,
                        shell_pid_file,
                        accumulated_delay_sec,
                    )
                    prepared_units.append(
                        {
                            "feature_name": current_feature_name,
                            "unit_name": unit_name,
                            "tab_title": tab_title,
                            "command": command,
                            "pid_file": pid_file,
                            "shell_pid_file": shell_pid_file,
                            "script_path": script_path,
                            "startup_grace_sec": accumulated_delay_sec + 5.0,
                        }
                    )
                    started_units.append("%s/%s" % (current_feature_name, unit_name))
                    accumulated_delay_sec += max(0.0, float(launch.get("delay_sec", 0.0)))

            terminal_command = self._build_feature_terminal_command(feature_name, prepared_units)
            if not terminal_command:
                raise RuntimeError("gnome-terminal is required for tabbed terminal launch")

            terminal_process = subprocess.Popen(
                terminal_command,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            for unit in prepared_units:
                self._runtimes[unit["feature_name"]].units.append(
                    LaunchUnitRuntime(
                        unit["unit_name"],
                        unit["command"],
                        terminal_process,
                        pid_file=unit["pid_file"],
                        shell_pid_file=unit["shell_pid_file"],
                        script_path=unit["script_path"],
                        terminal_mode=True,
                        startup_grace_sec=unit["startup_grace_sec"],
                    )
                )
                rospy.loginfo(
                    "started feature=%s unit=%s terminal_pid=%d",
                    unit["feature_name"],
                    unit["unit_name"],
                    terminal_process.pid,
                )
        except Exception as exc:
            for feature_plan in features_to_launch:
                current_feature_name = feature_plan["feature_name"]
                self._runtimes[current_feature_name].last_error = str(exc)
                self._stop_feature(current_feature_name, True)
            return False, "start feature failed: %s" % exc, started_units

        return True, "started feature %s" % feature_name, started_units

    def _resolve_feature_start_order(self, feature_name):
        ordered = []
        visited = set()
        visiting = set()

        def visit(current_feature_name):
            if current_feature_name not in self._features:
                raise ValueError("unknown feature: %s" % current_feature_name)
            if current_feature_name in visited:
                return
            if current_feature_name in visiting:
                raise ValueError("cyclic dependency detected at %s" % current_feature_name)

            visiting.add(current_feature_name)
            for dependency in self._features[current_feature_name].get("depends_on", []):
                visit(dependency)
            visiting.remove(current_feature_name)
            visited.add(current_feature_name)
            ordered.append(current_feature_name)

        visit(feature_name)
        return ordered

    def _build_start_preview_units(self, feature_name):
        ordered_features = self._resolve_feature_start_order(feature_name)
        preview_units = []
        preview_commands = []
        for ordered_feature_name in ordered_features:
            launches = self._features[ordered_feature_name].get("launches", [])
            for launch in launches:
                unit_name = launch.get("name") or launch.get("file") or "launch"
                preview_units.append(self._format_launch_display_name(ordered_feature_name, unit_name))
                preview_commands.append(self._format_launch_command_preview(launch))
        return preview_units, preview_commands

    def _format_launch_display_name(self, feature_name, unit_name):
        feature = self._features.get(feature_name, {})
        group_name = feature.get("group", "未分组")
        return "%s | %s | %s" % (group_name, feature_name, unit_name)

    def _format_launch_command_preview(self, launch):
        package_name = launch.get("package", "").strip()
        launch_file = launch.get("file", "").strip()
        command = ["roslaunch", package_name, launch_file]
        command.extend(list(launch.get("args", [])))
        return " ".join(shlex.quote(token) for token in command if token)

    def _stop_feature(self, feature_name, force):
        if feature_name not in self._features:
            return False, "unknown feature: %s" % feature_name, []

        runtime = self._runtimes[feature_name]
        if not runtime.units:
            return True, "feature already stopped", []

        stopped_units = [unit.unit_name for unit in runtime.units]
        timeout_sec = float(self._features[feature_name].get("stop_timeout_sec", 8.0))
        sig = signal.SIGKILL if force else signal.SIGINT

        for unit in runtime.units:
            target_pid = self._read_pid_file(unit.pid_file)
            if target_pid > 0:
                self._signal_process_tree(target_pid, sig)
            if unit.terminal_mode:
                if force:
                    shell_pid = self._read_pid_file(unit.shell_pid_file)
                    if shell_pid > 0:
                        self._signal_process_tree(shell_pid, signal.SIGKILL)
                continue
            if unit.process.poll() is None:
                self._signal_process_tree(unit.process.pid, sig)

        if not force:
            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                if all(not self._is_unit_running(unit) for unit in runtime.units):
                    break
                time.sleep(0.2)

            for unit in runtime.units:
                target_pid = self._read_pid_file(unit.pid_file)
                if self._is_process_alive(target_pid):
                    self._signal_process_tree(target_pid, signal.SIGKILL)
                if unit.terminal_mode:
                    shell_pid = self._read_pid_file(unit.shell_pid_file)
                    if shell_pid > 0:
                        self._signal_process_tree(shell_pid, signal.SIGTERM)
                        time.sleep(0.05)
                        if self._is_process_alive(shell_pid):
                            self._signal_process_tree(shell_pid, signal.SIGKILL)
                    continue
                if unit.process.poll() is None:
                    self._signal_process_tree(unit.process.pid, signal.SIGKILL)

        for unit in runtime.units:
            self._cleanup_unit_artifacts(unit)
        runtime.units = []
        return True, "stopped feature %s" % feature_name, stopped_units

    def _build_command(self, launch, override_args):
        package_name = launch.get("package", "").strip()
        launch_file = launch.get("file", "").strip()
        if not package_name or not launch_file:
            raise ValueError("launch entry requires package and file")

        package_path = self._rospack.get_path(package_name)
        launch_path = os.path.join(package_path, "launch", launch_file)
        if not os.path.isfile(launch_path):
            raise ValueError("launch file not found: %s" % launch_path)

        command = ["roslaunch", package_name, launch_file]
        command.extend(list(launch.get("args", [])))
        command.extend(override_args)
        return command

    def _monitor_processes(self, _event):
        with self._lock:
            for feature_name, runtime in self._runtimes.items():
                active_units = []
                for unit in runtime.units:
                    if self._is_unit_running(unit):
                        active_units.append(unit)
                        continue
                    return_code = unit.process.poll()
                    if unit.terminal_mode:
                        if return_code not in (None, 0):
                            runtime.last_error = "terminal unit %s exited with code %d" % (
                                unit.unit_name,
                                return_code,
                            )
                            rospy.logwarn(
                                "feature=%s unit=%s terminal exited with code=%d",
                                feature_name,
                                unit.unit_name,
                                return_code,
                            )
                        else:
                            rospy.loginfo("feature=%s unit=%s exited normally", feature_name, unit.unit_name)
                        self._cleanup_unit_artifacts(unit)
                        continue
                    if return_code != 0:
                        runtime.last_error = "unit %s exited with code %d" % (unit.unit_name, return_code)
                        rospy.logwarn("feature=%s unit=%s exited with code=%d", feature_name, unit.unit_name, return_code)
                    else:
                        rospy.loginfo("feature=%s unit=%s exited normally", feature_name, unit.unit_name)
                    self._cleanup_unit_artifacts(unit)
                runtime.units = active_units

    def _make_runtime_file(self, feature_name, unit_name, suffix):
        safe_feature = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in feature_name)
        safe_unit = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in unit_name)
        token = uuid.uuid4().hex[:8]
        return os.path.join(self._runtime_dir, "%s__%s__%s%s" % (safe_feature, safe_unit, token, suffix))

    def _create_terminal_launch_script(
        self,
        feature_name,
        unit_name,
        command,
        pid_file,
        shell_pid_file,
        start_delay_sec,
    ):
        script_path = self._make_runtime_file(feature_name, unit_name, ".sh")
        with open(script_path, "w") as handle:
            handle.write("#!/usr/bin/env bash\n")
            handle.write("set +e\n")
            handle.write("sunray_terminal_title=%s\n" % shlex.quote(unit_name))
            handle.write("sunray_title_keeper_pid=\"\"\n")
            handle.write("set_sunray_terminal_title() {\n")
            handle.write("  printf '\\033]0;%s\\007' \"$sunray_terminal_title\"\n")
            handle.write("}\n")
            handle.write("start_sunray_title_keeper() {\n")
            handle.write("  while true; do\n")
            handle.write("    set_sunray_terminal_title\n")
            handle.write("    sleep 1\n")
            handle.write("  done &\n")
            handle.write("  sunray_title_keeper_pid=$!\n")
            handle.write("}\n")
            handle.write("stop_sunray_title_keeper() {\n")
            handle.write("  if [ -n \"${sunray_title_keeper_pid:-}\" ]; then\n")
            handle.write("    kill \"$sunray_title_keeper_pid\" 2>/dev/null\n")
            handle.write("    wait \"$sunray_title_keeper_pid\" 2>/dev/null\n")
            handle.write("    sunray_title_keeper_pid=\"\"\n")
            handle.write("  fi\n")
            handle.write("}\n")
            handle.write("pid_file=%s\n" % shlex.quote(pid_file))
            handle.write("shell_pid_file=%s\n" % shlex.quote(shell_pid_file))
            handle.write("rm -f \"$pid_file\"\n")
            handle.write("rm -f \"$shell_pid_file\"\n")
            handle.write("echo \"$$\" > \"$shell_pid_file\"\n")
            handle.write("set_sunray_terminal_title\n")
            handle.write("start_sunray_title_keeper\n")
            if start_delay_sec > 0.0:
                handle.write("sleep %s\n" % str(start_delay_sec))
            handle.write("echo \"[Sunray] Launch: %s\"\n" % unit_name)
            handle.write("echo \"[Sunray] Command: %s\"\n" % " ".join(shlex.quote(token) for token in command))
            handle.write("echo\n")
            handle.write("%s &\n" % " ".join(shlex.quote(token) for token in command))
            handle.write("roslaunch_pid=$!\n")
            handle.write("echo \"$roslaunch_pid\" > \"$pid_file\"\n")
            handle.write("wait \"$roslaunch_pid\"\n")
            handle.write("exit_code=$?\n")
            handle.write("rm -f \"$pid_file\"\n")
            handle.write("stop_sunray_title_keeper\n")
            handle.write("set_sunray_terminal_title\n")
            handle.write("echo \"[Sunray] roslaunch exited with code ${exit_code}. Press Enter to keep/close this tab.\"\n")
            handle.write("read\n")
            handle.write("rm -f \"$shell_pid_file\"\n")
        os.chmod(script_path, 0o755)
        return script_path

    def _build_feature_terminal_command(self, feature_name, prepared_units):
        if not shutil.which("gnome-terminal"):
            return []

        command = ["gnome-terminal"]
        for idx, unit in enumerate(prepared_units):
            title = unit["tab_title"]
            command.extend(
                [
                    "--window" if idx == 0 else "--tab",
                    "--title",
                    title,
                    "--command",
                    "bash -lc %s" % shlex.quote(unit["script_path"]),
                ]
            )
        return command

    def _write_pid_file(self, pid_file, pid):
        with open(pid_file, "w") as handle:
            handle.write(str(int(pid)))

    def _read_pid_file(self, pid_file):
        if not pid_file or not os.path.isfile(pid_file):
            return 0
        try:
            with open(pid_file, "r") as handle:
                return int(handle.read().strip())
        except Exception:
            return 0

    def _is_process_alive(self, pid):
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    def _signal_process_tree(self, pid, sig):
        if pid <= 0:
            return
        try:
            os.killpg(pid, sig)
        except OSError:
            pass
        try:
            os.kill(pid, sig)
        except OSError:
            pass

    def _is_unit_running(self, unit):
        target_pid = self._read_pid_file(unit.pid_file)
        if self._is_process_alive(target_pid):
            return True
        if unit.terminal_mode:
            shell_pid = self._read_pid_file(unit.shell_pid_file)
            if time.time() - unit.started_at < unit.startup_grace_sec:
                return self._is_process_alive(shell_pid) or unit.process.poll() is None
            return False
        if time.time() - unit.started_at < unit.startup_grace_sec:
            return True
        if unit.process.poll() is None:
            return True
        return False

    def _cleanup_unit_artifacts(self, unit):
        for path in (unit.pid_file, unit.shell_pid_file, unit.script_path):
            if path and os.path.exists(path):
                try:
                    os.remove(path)
                except OSError:
                    pass

    def _publish_system_info(self, _event):
        msg = SystemInfo()
        msg.header.stamp = rospy.Time.now()
        msg.cpu_percent = self._read_cpu_percent()
        msg.memory_percent = self._read_memory_percent()
        msg.active_ros_nodes = self._get_active_ros_nodes()
        self._system_info_pub.publish(msg)

    def _read_cpu_sample(self):
        try:
            with open("/proc/stat", "r") as handle:
                fields = handle.readline().split()
            if len(fields) < 5 or fields[0] != "cpu":
                return None

            values = [int(value) for value in fields[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)
            return idle, total
        except Exception:
            return None

    def _read_cpu_percent(self):
        current = self._read_cpu_sample()
        if current is None or self._last_cpu_sample is None:
            self._last_cpu_sample = current
            return 0.0

        prev_idle, prev_total = self._last_cpu_sample
        idle, total = current
        total_delta = max(0, total - prev_total)
        idle_delta = max(0, idle - prev_idle)
        self._last_cpu_sample = current

        if total_delta == 0:
            return 0.0

        busy = 1.0 - float(idle_delta) / float(total_delta)
        return max(0.0, min(100.0, busy * 100.0))

    def _read_memory_percent(self):
        mem_total_kb = 0.0
        mem_available_kb = 0.0
        try:
            with open("/proc/meminfo", "r") as handle:
                for line in handle:
                    if line.startswith("MemTotal:"):
                        mem_total_kb = float(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        mem_available_kb = float(line.split()[1])
                    if mem_total_kb > 0.0 and mem_available_kb > 0.0:
                        break
        except Exception:
            return 0.0

        if mem_total_kb <= 0.0:
            return 0.0

        mem_used_kb = max(0.0, mem_total_kb - mem_available_kb)
        return max(0.0, min(100.0, mem_used_kb * 100.0 / mem_total_kb))

    def _get_active_ros_nodes(self):
        try:
            return sorted(rosnode.get_node_names())
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "failed to query ROS nodes: %s", exc)
            return []


def main():
    if not rosgraph.is_master_online():
        rospy.logwarn("ROS master is not online when sunray_system starts")

    rospy.init_node("sunray_system", anonymous=False)

    try:
        SunraySystemSupervisor()
    except Exception as exc:
        rospy.logfatal("sunray_system init failed: %s", exc)
        raise

    rospy.spin()


if __name__ == "__main__":
    main()
