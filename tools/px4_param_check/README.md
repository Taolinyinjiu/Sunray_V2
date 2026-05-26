# PX4 参数检查与设置工具
本工具通过 MAVROS 参数服务检查或写入 Sunray 关心的 PX4 参数。

## 前提
先启动 MAVROS，并确认参数服务存在：
```bash
roslaunch sunray_mavros mavros.launch
```
默认命名空间来自 `config/*.yaml`：
```yaml
agent_name: uav
agent_id: 1
mavros_node: mavros
```
默认访问 `/uav1/mavros/param/get` 和 `/uav1/mavros/param/set`。

## 配置文件
配置文件放在 `config/` 目录，默认使用：
```text
config/px4_params_default.yaml
```
可按机型或任务新增配置，例如 `px4_params_s150.yaml`、`px4_params_s300.yaml`。
配置中的 `parameter_group` 用来分组管理参数，`--group` 需要填写完整分组名。

## 检查PX4参数
```bash
## 使用默认配置文件
./check_px4_param.sh
## 使用指定配置文件
./check_px4_param.sh --config px4_params_default.yaml
```
输出会显示当前值、推荐值、状态和描述。

## 设置PX4参数
```bash
## 使用默认配置文件
./set_px4_param.sh
## 使用指定配置文件
./set_px4_param.sh --config px4_params_default.yaml
```
`set` 会先读取当前值：一致则跳过，不一致则写入推荐值，并显示成功或失败。
设置成功后默认询问是否重启，20 秒未选择则跳过；`--reboot` 会直接重启飞控。

## 常用启动参数
- `--config FILE`：指定配置文件名或完整路径。
- `--agent-id ID`：切换飞机编号，例如 `--agent-id 2` 对应 `/uav2/mavros`。
- `--mavros-ns NS`：直接指定 MAVROS 命名空间，优先级最高。
- `--timeout SEC`：设置每次 MAVROS 服务调用超时时间，默认 `3` 秒。
