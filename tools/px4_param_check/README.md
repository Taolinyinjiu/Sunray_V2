# PX4 参数检查工具

本目录用于维护 Sunray 关心的 PX4 参数，并通过 MAVROS 参数服务查询或写入飞控参数。

## 命名空间配置

`drivers/sunray_mavros/launch/mavros.launch` 中 MAVROS 节点被放在下面的 group 中：

```xml
<group ns="$(arg agent_name)$(arg agent_id)">
```

所以默认 `agent_name=uav`、`agent_id=1` 时，MAVROS 参数服务路径是：

```text
/uav1/mavros/param/get
/uav1/mavros/param/set
```

本目录通过 `config/px4_params_default.yaml` 维护默认目标和参数建议值：

```yaml
agent_name: uav
agent_id: 1
mavros_node: mavros
```

脚本会自动拼出 `/uav1/mavros`。如果要检查其他飞机，可以通过命令行覆盖：

```bash
./check_px4_param.sh --agent-id 2
./set_px4_param.sh --agent-name uav --agent-id 2
```

也可以直接指定完整 MAVROS namespace：

```bash
./check_px4_param.sh --mavros-ns /uav2/mavros
```

## 配置文件

配置文件统一放在 `config/` 目录下，方便后续按不同机型、不同任务或不同调参方案维护多套配置。

当前默认配置：

- `config/px4_params_default.yaml`：默认目标飞机、MAVROS 节点配置和 PX4 参数建议值列表。
- `config/px4_params_s150.yaml`：S150 机型参数配置占位文件。
- `config/px4_params_s300.yaml`：S300 机型参数配置占位文件。

脚本参数支持传配置文件名，也支持传完整路径。只传文件名时，脚本会自动到 `config/` 目录下查找。

示例：

```bash
./check_px4_param.sh --config px4_params_default.yaml
./set_px4_param.sh --config uav2_params.yaml
./check_px4_param.sh --config /tmp/test_params.yaml
```

浮点参数比较默认使用 `0.001` 的容差，避免 PX4/MAVROS 返回 `0.949999988` 而配置写 `0.95` 时被误判为不同。可以通过参数覆盖：

```bash
./check_px4_param.sh --tolerance 0.0001
./set_px4_param.sh --tolerance 0.0001
```

## MAVROS 参数服务说明

下面以 `/uav1/mavros` 命名空间为例。

### `/uav1/mavros/param/get`

查询飞控中某一个参数的当前值。

典型用途：

- 检查单个 PX4 参数是否存在。
- 读取单个参数当前值，用来和建议值对比。
- `check_px4_param.sh` 主要使用这个服务。

示例：

```bash
rosservice call /uav1/mavros/param/get "{param_id: 'MPC_XY_P'}"
```

### `/uav1/mavros/param/set`

设置飞控中某一个参数的值。

典型用途：

- 修改单个 PX4 参数。
- 按 YAML 中的建议值逐个写入参数。
- `set_px4_param.sh` 主要使用这个服务。

示例：

```bash
rosservice call /uav1/mavros/param/set "{param_id: 'MPC_XY_P', value: {integer: 0, real: 0.95}}"
```

注意：

- 浮点参数写到 `real` 字段，`integer` 填 `0`。
- 整型参数写到 `integer` 字段，`real` 填 `0.0`。
- 有些参数修改后需要重启飞控才会完全生效。

### `/uav1/mavros/param/pull`

从飞控拉取整套参数到 MAVROS 本地参数缓存。

典型用途：

- 刚连接飞控后，同步飞控端参数到 MAVROS。
- 批量查询前，确保 MAVROS 缓存和飞控当前参数一致。
- 参数在 QGC、NSH 或其他工具中被修改后，可以先执行 `pull` 再查询。

示例：

```bash
rosservice call /uav1/mavros/param/pull "{}"
```

注意：

- `pull` 的方向是：飞控 -> MAVROS。
- 它不会修改飞控参数，只会更新 MAVROS 侧缓存。

### `/uav1/mavros/param/push`

将 MAVROS 本地参数缓存推送到飞控。

典型用途：

- 当 MAVROS 本地已有一批参数缓存，并希望整体写回飞控时使用。
- 一般调参脚本不优先使用它，因为作用范围比 `set` 更大。

示例：

```bash
rosservice call /uav1/mavros/param/push "{}"
```

注意：

- `push` 的方向是：MAVROS -> 飞控。
- 使用前要确认 MAVROS 本地缓存内容是你想写入飞控的内容。
- 相比逐个调用 `set`，`push` 的误操作影响范围更大。

## 本目录脚本

查询参数：

```bash
./check_px4_param.sh
```

按 `px4_params_default.yaml` 写入建议值：

```bash
./set_px4_param.sh
```

只处理某一类参数：

```bash
./check_px4_param.sh --group Commander
./set_px4_param.sh --group "Multicopter Position Control"
```

`--group` 需要传入配置文件中 `parameter_group` 的完整 `name`。
