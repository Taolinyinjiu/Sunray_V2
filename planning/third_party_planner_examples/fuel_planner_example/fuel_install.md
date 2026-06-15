1. 缺失Nlopt.cmake
```bash
你这台 Ubuntu 20.04 上，先装这两个包最合适：

sudo apt update
sudo apt install -y libnlopt-dev libnlopt-cxx-dev

装完先验证：

dpkg -L libnlopt-cxx-dev | rg 'nlopt.hpp|NLoptConfig.cmake|nlopt-config.cmake'
dpkg -L libnlopt-dev | rg 'NLoptConfig.cmake|nlopt-config.cmake'

如果已经装了，但 build.sh 还是报找不到 NLoptConfig.cmake，就把 CMake 路径显式补上：

export NLopt_DIR="$(dirname "$(dpkg -L libnlopt-cxx-dev | rg 'NLoptConfig\.cmake|nlopt-config\.cmake' | head -n1)")"
export CMAKE_PREFIX_PATH="$NLopt_DIR:$CMAKE_PREFIX_PATH"
./build.sh fuel_planner

```
2. 