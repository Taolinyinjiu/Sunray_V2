<img src="https://pic1.imgdb.cn/item/67ceabf3066befcec6e26c76.png" alt="yundrone logo" align="right" height="90" />

## Sunray - 开源无人机仿真与实践平台

**Sunray**是云纵科技“**Sunray 系列科研无人机**”的配套开源代码，为无人机智能化和自主化飞行提供**全套解决方案**。

- 支持对无人车、无人机的控制，提供完善的二次开发接口
- 内置丰富的接口使用例程，提供二次开发使用教程
- 接入多个无人机开源项目，如 FAST-LIO2、EGO-planner 等
- 支持 RVIZ 仿真、Gazebo 仿真及真实无人机平台实验，满足不同科研需求

### 快速安装



### 仿真使用

- 启动仿真一键启动面板：

```bash
roslaunch sunray_launcher_panel sunray_launcher_panel.launch
```
  - 可以启动仿真简易地面站
  - 可以启动任何已经写好的仿真脚本

- 配置文件拆成两类：

  ```text
  tools/sunray_launcher_panel/config/sunray_launch_groups.yaml
  tools/sunray_launcher_panel/config/sunray_quick_launch_groups.yaml
  ```


### 真机测试


- Sunray系统后台任务（建议设置为开机自启）
  ```bash
  cd ~/Sunray_v2
  source devel/setup.bash
  roslaunch sunray_system sunray_system.launch
  ```
  - 如果需要修改启动任务，请查看`tools/sunray_system/config/features.yaml`
- Sunray脚本启动器（TUI界面，通过SSH启动）
  ```bash
  roslaunch scripts_manage scripts_manage_tui.launch
  ```

### 版权声明

- 本开源项目仅限个人使用，请勿用于商业用途。
- 如利用本项目进行营利活动，云纵科技将追究侵权行为。
