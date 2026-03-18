# PX4 参数元数据缓存目录

建议把不同固件版本的参数文件放在这里，例如：

- `parameters_v1.14.0.json.xz`
- `parameters_v1.15.4.json.xz`
- `parameters_main_2026-03-18.json.xz`

`test/px4_param_tui/scripts/px4_param_meta_server.py` 支持直接读取：

- `.json.xz`（推荐）
- `.json`
- `.xml`（兼容）

