# PX4 参数检查与设置工具
本工具通过 MAVROS 参数服务检查或写入 Sunray 关心的 PX4 参数。

## 前提
`check_px4_param.sh` 和 `set_px4_param.sh` 会先自动 source ROS 和当前工作空间：

```text
/opt/ros/${ROS_DISTRO:-noetic}/setup.bash
<workspace>/devel/setup.bash
```

其中 `<workspace>` 由脚本位置自动推导，不需要写死成某台电脑上的绝对路径。

如果 MAVROS 参数服务不存在，脚本会默认尝试自动启动：

```bash
roslaunch sunray_mavros mavros.launch agent_name:=uav agent_id:=1
```

MAVROS 启动后，脚本会先检查参数服务是否出现，再等待 `${mavros_ns}/state` 中 `connected: True`。如果 `roslaunch` 进程提前退出，会打印 MAVROS 日志末尾；如果节点已启动但串口、权限、波特率或飞控供电异常，脚本会在连接检查阶段超时并提示检查这些项。

也可以手动提前启动 MAVROS：

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
## 不自动启动 MAVROS
./check_px4_param.sh --no-start-mavros
## 自动启动 MAVROS 时指定飞控串口
./check_px4_param.sh --fcu-url /dev/ttyACM0:921600
## 只检查参数服务，不等待 MAVROS state connected=true
./check_px4_param.sh --no-wait-connected
## 手动指定 ROS/workspace setup 文件
./check_px4_param.sh --ros-setup /opt/ros/noetic/setup.bash --workspace-setup ../../devel/setup.bash
```
输出会显示当前值、推荐值、状态和描述。

## 设置PX4参数
```bash
## 使用默认配置文件
./set_px4_param.sh
## 使用指定配置文件
./set_px4_param.sh --config px4_params_default.yaml
## 不自动启动 MAVROS
./set_px4_param.sh --no-start-mavros
## 自动启动 MAVROS 时指定飞控串口
./set_px4_param.sh --fcu-url /dev/ttyACM0:921600
## 只检查参数服务，不等待 MAVROS state connected=true
./set_px4_param.sh --no-wait-connected
```
`set` 会先读取当前值：一致则跳过，不一致则写入推荐值，并显示成功或失败。
设置成功后默认询问是否重启，20 秒未选择则跳过；`--reboot` 会直接重启飞控。

## 常用启动参数
- `--config FILE`：指定配置文件名或完整路径。
- `--agent-id ID`：切换飞机编号，例如 `--agent-id 2` 对应 `/uav2/mavros`。
- `--mavros-ns NS`：直接指定 MAVROS 命名空间，优先级最高。
- `--timeout SEC`：设置每次 MAVROS 服务调用超时时间，默认 `3` 秒。
- `--no-start-mavros`：检查脚本不自动启动 MAVROS，仅检查已有服务。
- `--fcu-url URL`：检查脚本自动启动 MAVROS 时传给 `mavros.launch` 的 `fcu_url`。
- `--ros-setup FILE`：指定需要 source 的 ROS `setup.bash`。
- `--workspace-setup FILE`：指定需要 source 的工作空间 `setup.bash`。
- `--mavros-start-timeout SEC`：自动启动 MAVROS 后等待参数服务的时间，默认 `15` 秒。
- `--mavros-connect-timeout SEC`：等待 MAVROS `state.connected=true` 的时间，默认 `20` 秒。
- `--no-wait-connected`：不等待 MAVROS 与飞控连接成功；仅建议 fake/sim MAVROS 或特殊调试时使用。
