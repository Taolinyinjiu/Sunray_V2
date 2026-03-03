# px4_web_api_demo

Minimal demo package: expose HTTP endpoints and bridge to PX4 parameter services via MAVROS.

## Endpoints

- `GET /health`
- `GET /param?name=EKF2_EV_CTRL`
- `POST /param` with JSON body:
  - `{"name":"EKF2_EV_CTRL","type":"int","value":9}`
  - `{"name":"EKF2_EV_DELAY","type":"float","value":10.5}`

## Run

```bash
cmake -S common/px4_web_api_demo -B build/px4_web_api_demo
cmake --build build/px4_web_api_demo -j4
source devel/setup.bash
roslaunch px4_web_api_demo px4_web_api_demo.launch uav_id:=1 uav_name:=uav port:=8080
```

## Quick test

```bash
curl "http://127.0.0.1:8080/health"
curl "http://127.0.0.1:8080/param?name=EKF2_EV_CTRL"
curl -X POST "http://127.0.0.1:8080/param" \
  -H "Content-Type: application/json" \
  -d '{"name":"EKF2_EV_CTRL","type":"int","value":9}'
```

