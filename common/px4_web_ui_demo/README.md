# px4_web_ui_demo

Simple HTML dashboard demo for `px4_web_api_demo`.

## What it does

- Displays a web page at `http://<host>:<port>/`
- Polls PX4 params every 1s via API endpoints
- Provides a small form to set param values

## Run

Start API backend first:

```bash
roslaunch px4_web_api_demo px4_web_api_demo.launch uav_id:=1 uav_name:=uav port:=8080
```

Start UI demo:

```bash
roslaunch px4_web_ui_demo px4_web_ui_demo.launch port:=8090 api_base_url:=http://127.0.0.1:8080
```

Open browser:

```text
http://127.0.0.1:8090
```

