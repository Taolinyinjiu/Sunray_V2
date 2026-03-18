# px4_param_tui 元数据工具

## 1. 用 CLI 查询

```bash
python3 test/px4_param_tui/scripts/px4_param_meta_server.py \
  --metadata /home/taolin/Documents/GitHub/PX4-Autopilot/build/cuav_7-nano_default/parameters.json.xz \
  --query count
```

```bash
python3 test/px4_param_tui/scripts/px4_param_meta_server.py \
  --metadata /home/taolin/Documents/GitHub/PX4-Autopilot/build/cuav_7-nano_default/parameters.json.xz \
  --query name:EKF2_HGT_REF
```

```bash
python3 test/px4_param_tui/scripts/px4_param_meta_server.py \
  --metadata /home/taolin/Documents/GitHub/PX4-Autopilot/build/cuav_7-nano_default/parameters.json.xz \
  --query search:EKF2_ \
  --limit 20
```

## 2. 作为 ROS 节点启动

```bash
rosrun px4_param_tui px4_param_meta_server.py \
  --metadata /home/taolin/Documents/GitHub/PX4-Autopilot/build/cuav_7-nano_default/parameters.json.xz \
  --ros
```

节点接口：

- 订阅：`~query`（`std_msgs/String`）
- 发布：`~result`（`std_msgs/String`，JSON 格式）
- 服务：`~reload`（`std_srvs/Trigger`）

查询协议：

- `count`
- `name:PARAM_NAME`
- `search:KEYWORD`
- `search:KEYWORD|50`（限制返回数量）

## 3. 元数据驱动 TUI（新节点，外观与原 TUI 一致）

```bash
roslaunch px4_param_tui px4_param_tui_meta.launch \
  metadata:=/home/taolin/Documents/GitHub/PX4-Autopilot/build/cuav_7-nano_default/parameters.json.xz
```

也可以直接运行二进制：

```bash
devel/lib/px4_param_tui/px4_param_tui_meta_node \
  /home/taolin/Documents/GitHub/PX4-Autopilot/build/cuav_7-nano_default/parameters.json.xz
```
