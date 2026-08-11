# FR3 Pinocchio C++ 仿真

本工程使用 Franka 官方 `franka_description` 模型，实现 FR3 的正运动学、雅可比、逆运动学和最小加加速度关节轨迹。

- Pinocchio 计算、IK 和轨迹生成全部在 C++ 中完成。
- Python 只通过 pybind11 调用 C++，并负责命令行和 MeshCat 显示。
- 机械臂对外为 7 自由度，末端坐标系为 `fr3_hand_tcp`。
- `pip install -e .` 会自动调用 CMake 编译 C++ 扩展，不需要 `build.sh` 或 `run_sim.sh`。

默认 URDF 为：

```text
models/fr3_franka_hand.urdf
```

它由官方文件生成：

```text
/home/xense/fastiter/franka_description/robots/fr3/fr3.urdf.xacro
```

## 工程结构

```text
environment.yml                          mamba 环境定义
CMakeLists.txt                           C++/pybind11 构建配置
cpp/include/fr3_control_sim/robot_model.hpp
cpp/src/robot_model.cpp                  Pinocchio FK/IK/轨迹实现
cpp/src/bindings.cpp                     pybind11 绑定
python/fr3_control_sim/                  Python 包和 MeshCat 显示
examples/fr3_sim.py                      仿真入口
models/fr3_franka_hand.urdf              FR3 + Franka Hand 模型
cpp/tests/test_kinematics.cpp            C++ 测试
tests/smoke_test.py                      Python/pybind 测试
```

## 安装

原生 C++ 依赖由 mamba 安装，当前工程由 pip 编译和安装。

```text
mamba：Pinocchio、编译器、Eigen、urdfdom、tinyxml2 等
pip：自动运行 CMake，编译并安装 fr3-control-sim
```

### 方案一：使用 environment.yml（推荐）

在工程根目录执行：

```bash
cd fastiter-control-sim

mamba env create -f environment.yml
mamba activate fr3sim
python -m pip install -e .
```

`environment.yml` 已包含全部 mamba 依赖，因此不需要再单独执行 `mamba install`。

如果 `fr3sim` 已经存在，可以更新环境：

```bash
mamba env update -n fr3sim -f environment.yml
mamba deactivate
mamba activate fr3sim
python -m pip install -e .
```

### 方案二：手动创建环境

下面的命令与 `environment.yml` 等价：

```bash
cd /home/xense/fastiter/fastiter-control-sim

# 1. 创建基础环境
mamba create -n fr3sim -c conda-forge python=3.12

# 2. 安装 C++ 工具链和第三方库
mamba install -n fr3sim -c conda-forge \
  numpy pinocchio=3.9 pybind11 cmake pkg-config \
  cxx-compiler make eigen urdfdom console_bridge tinyxml2 \
  pip setuptools wheel

# 3. 激活环境
mamba activate fr3sim

# 4. 自动运行 CMake、编译 C++ 并 editable 安装
python -m pip install -e .
```

需要查看完整 CMake 编译输出时使用：

```bash
python -m pip install -v -e .
```

如果不能访问 PyPI，但 mamba 环境中已经安装了 setuptools、wheel 和 pybind11，可以关闭 pip 构建隔离：

```bash
python -m pip install -v -e . --no-build-isolation
```

## editable 安装说明

`pip install -e .` 会执行以下操作：

1. setuptools 调用工程的 CMake 配置。
2. CMake 使用当前 mamba 环境中的 Python、Pinocchio 和编译器。
3. 编译 C++ 核心和 `_fr3_sim` pybind11 模块。
4. 将 `python/fr3_control_sim` 作为 editable Python 包安装。

修改 Python 文件后会立即生效。修改 C++ 文件后，需要重新编译：

```bash
python -m pip install -e .
```

非 editable 安装使用：

```bash
python -m pip install .
```

## 安装验证

```bash
python --version
PKG_CONFIG_PATH="$CONDA_PREFIX/lib/pkgconfig" \
  pkg-config --modversion pinocchio
PKG_CONFIG_PATH="$CONDA_PREFIX/lib/pkgconfig" \
  pkg-config --variable=prefix pinocchio
python tests/smoke_test.py
python -c "import fr3_control_sim; print(fr3_control_sim.__file__)"
python -c "import fr3_control_sim._fr3_sim as m; print(m.__file__)"
```

正常情况下应看到：

- Python 为当前 `fr3sim` 环境中的解释器。
- Pinocchio 版本为 `3.9.0`。
- Pinocchio prefix 为 `$CONDA_PREFIX`，而不是 `/usr/local`。
- Python 3.12 对应的扩展名为 `_fr3_sim.cpython-312-*.so`。

检查动态库是否缺失：

```bash
ldd "$(python -c 'import fr3_control_sim._fr3_sim as m; print(m.__file__)')" \
  | grep "not found"
```

没有输出表示依赖完整。

## 运行仿真

### Headless 测试

```bash
mamba activate fr3sim
python examples/fr3_sim.py --headless --mode demo
```

### MeshCat 仿真

```bash
python examples/fr3_sim.py --mode demo
```

浏览器会自动打开 MeshCat。若只想启动服务并手动打开终端中的 URL：

```bash
python examples/fr3_sim.py --mode demo --no-open-browser
```

### FK

输入 7 个关节角，单位为度：

```bash
python examples/fr3_sim.py --mode fk \
  --q 0 -45 0 -135 0 90 45
```

### IK

目标位置格式为 `x y z`，单位为米：

```bash
python examples/fr3_sim.py --mode ik \
  --target 0.35 0.10 0.45
```

完整目标位姿格式为 `x y z roll pitch yaw`，位置单位为米，RPY 单位为弧度：

```bash
python examples/fr3_sim.py --mode ik \
  --target 0.35 0.10 0.45 3.1415926 0.0 0.2
```

IK 成功后，C++ 会生成 50 Hz 最小加加速度关节轨迹，并由 MeshCat 播放。

## Python 接口示例

```python
import numpy as np
from fr3_control_sim import IKOptions, RobotModel, pose_from_xyz_rpy

model = RobotModel("models/fr3_franka_hand.urdf")
q0 = model.home_configuration()

T = model.forward_kinematics(q0)
J = model.jacobian(q0)

target = pose_from_xyz_rpy(
    np.array([0.35, 0.10, 0.45]),
    np.array([3.1415926, 0.0, 0.2]),
)
result = model.inverse_kinematics(target, q0, IKOptions())
trajectory = model.minimum_jerk_trajectory(q0, result.q, 2.0, 0.02)
```

## 可选：手动运行 C++ 测试

正常安装不需要手动执行 CMake。需要单独运行 CTest 时：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython3_FIND_VIRTUALENV=ONLY \
  -DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=ON \
  -Dpybind11_DIR="$(python -m pybind11 --cmakedir)" \
  -DFR3_SIM_BUILD_TESTS=ON

cmake --build build --parallel "$(nproc)"
ctest --test-dir build --output-on-failure
```

## 重新生成官方 URDF

工程已包含可用 URDF，通常不需要重新生成。官方描述更新后可执行：

```bash
python /opt/ros/humble/bin/xacro \
  -o models/fr3_franka_hand.urdf \
  /home/xense/fastiter/franka_description/robots/fr3/fr3.urdf.xacro \
  hand:=true ee_id:=franka_hand with_sc:=false

check_urdf models/fr3_franka_hand.urdf
```

最初使用的 `/home/xense/fastiter/franka_description_URDF/urdfs/fr3_franka_hand.urdf` 存在手爪父链接命名问题，因此本工程使用官方 `franka_description` 生成的模型。

## 常见问题

### CMake 找到 Pinocchio 3.4.0

说明当前 mamba 环境没有安装 Pinocchio，pkg-config 回退到了 `/usr/local`。重新执行：

```bash
mamba install -n fr3sim -c conda-forge \
  numpy pinocchio=3.9 pybind11 cmake pkg-config \
  cxx-compiler make eigen urdfdom console_bridge tinyxml2 \
  pip setuptools wheel
```

### 找不到 tinyxml2

说明当前环境缺少原生 C++ 依赖。检查：

```bash
mamba list -n fr3sim | grep -E "pinocchio|tinyxml2|urdfdom|console_bridge"
```

### pip 为什么不能单独安装所有依赖

pip 会安装 Python 包并编译当前工程，但不会管理 conda-forge 的 C++ 编译器、Pinocchio、urdfdom 和 tinyxml2。它们必须先通过 `environment.yml` 或 `mamba install` 安装。

## 许可证

本项目源代码采用 [MIT License](LICENSE)。由 Franka 官方 `franka_description` 生成的模型仍遵循其原始 Apache-2.0 许可证。
