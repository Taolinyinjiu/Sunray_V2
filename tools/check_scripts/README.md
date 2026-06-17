# Sunray Check Scripts

This directory is the active dependency check and dependency resolve system used
by the repository root entrypoints:

```bash
./check.sh
./install_sunray_dependency.sh
```

## Entry Points

- `./check.sh` starts dependency checking. With no arguments it opens the check
  TUI; with module or group arguments it runs CLI checks.
- `./check.sh --resolve <module-or-group>` runs resolver scripts for the selected
  modules. Use `--dry-run` to preview the plan.
- `./install_sunray_dependency.sh` provides a fishros-style numeric menu, preheats
  sudo for real install flows, then delegates to `./check.sh --resolve`.
- Use `--skip-apt-update` when unrelated third-party apt sources are temporarily
  broken and the local package index is already usable. Use
  `--strict-apt-update` when diagnosing apt source health and you want update
  failures to stop the resolver immediately.

## Configuration Boundary

- `modules.yaml` describes Sunray modules, groups, module dependencies, and
  conflicts.
- `module_config.yaml` describes extra external dependencies to check after a
  base ROS Noetic environment is already installed.
- `install_menu.yaml` maps stable numeric menu ids to module or group targets.

## Resolver Scripts

Resolver scripts live under:

```text
tools/check_scripts/check/reslove/<module>/*.sh
```

The `reslove` spelling is historical and kept for compatibility. The resolver
also supports a future `check/resolve` directory if one is introduced.

Apt helpers intentionally do not edit `/etc/apt` sources. The resolver should
avoid being blocked by unrelated third-party sources where possible, but fixing
or disabling those sources remains an explicit user/system maintenance action.

## Relationship With `tools/build_scripts`

`tools/build_scripts` remains the build-system implementation. Do not assume
edits under `tools/build_scripts/check` affect the root `./check.sh`; the active
root check path is this `tools/check_scripts` tree.
