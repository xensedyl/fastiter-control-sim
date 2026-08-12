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

它由官方文件生成：[franka-research3.urdf.xacro](https://github.com/frankarobotics/franka_description/blob/main/robots/fr3/fr3.urdf.xacro)

## 工程结构

```text
environment.yml                          mamba 环境定义
CMakeLists.txt                           C++/pybind11 构建配置
cpp/include/fr3_control_sim/robot_model.hpp
cpp/src/robot_model.cpp                  Pinocchio FK/IK/轨迹实现
cpp/src/bindings.cpp                     pybind11 绑定
python/fr3_control_sim/                  Python 包和 MeshCat 显示
examples/fr3_sim.py                      仿真入口
examples/fr3_sim_qt.py                   Qt FK/IK 滑条控制界面
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
git clone https://github.com/xensedyl/fastiter-control-sim.git
```

or

```bash
git clone git@github.com:xensedyl/fastiter-control-sim.git
```

```bash
cd fastiter-control-sim

mamba env create -f environment.yml
mamba activate fr3sim
pip install -e .
```

`environment.yml` 已包含全部 mamba 依赖，因此不需要再单独执行 `mamba install`。它还会在激活环境时清除继承自系统 ROS 的 `PYTHONPATH` 和 `AMENT_PREFIX_PATH`，防止 pip 错把 `/opt/ros` 中属于另一个 Python 版本的包当成当前环境的包。

如果 `fr3sim` 已经存在，可以更新环境：

```bash
mamba env update -n fr3sim -f environment.yml
mamba deactivate
mamba activate fr3sim
pip install -e .
```

### 方案二：手动创建环境

下面的命令与 `environment.yml` 等价：

```bash
git clone https://github.com/xensedyl/fastiter-control-sim.git
```

or

```bash
git clone git@github.com:xensedyl/fastiter-control-sim.git
```

1. 创建基础环境
```bash
cd fastiter-control-sim
mamba create -n fr3sim -c conda-forge python=3.12
```

2. 安装 C++ 工具链和第三方库
```bash
mamba install -n fr3sim -c conda-forge \
  numpy pyside6 pinocchio=3.9 pybind11 cmake pkg-config \
  cxx-compiler make eigen urdfdom console_bridge tinyxml2 \
  pip setuptools wheel
```

3. 激活环境
```bash
mamba activate fr3sim
```

4. 隔离系统中可能已经 source 的 ROS Python 路径
```bash
unset PYTHONPATH AMENT_PREFIX_PATH
export PYTHONNOUSERSITE=1
```

5. 自动运行 CMake、编译 C++ 并 editable 安装
```bash
pip install -e .
```

需要查看完整 CMake 编译输出时使用：

```bash
pip install -v -e .
```

如果不能访问 PyPI，但 mamba 环境中已经安装了 setuptools、wheel 和 pybind11，可以关闭 pip 构建隔离：

```bash
pip install -v -e . --no-build-isolation
```

## editable 安装说明

`pip install -e .` 会执行以下操作：

1. setuptools 调用工程的 CMake 配置。
2. CMake 使用当前 mamba 环境中的 Python、Pinocchio 和编译器。
3. 编译 C++ 核心和 `_fr3_sim` pybind11 模块。
4. 将 `python/fr3_control_sim` 作为 editable Python 包安装。

修改 Python 文件后会立即生效。修改 C++ 文件后，需要重新编译：

```bash
pip install -e .
```

非 editable 安装使用：

```bash
pip install .
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

### 交互式 FK

不提供 `--q` 时，会持续在终端等待输入。关节角单位为度，输入 `q`、`quit`、`exit` 或空行退出：

```bash
python examples/fr3_sim.py --mode fk
```

终端输入示例：

```text
joint angles > 0 -45 0 -135 0 90 45
  ee position: [0.30689 0.      0.48688] m
  ee rpy:      [-180.    0.    0.] deg
```

每次输入都会调用 C++ 正运动学，并立即刷新 MeshCat。

### 单次 FK

输入 7 个关节角，单位为度：

```bash
python examples/fr3_sim.py --mode fk \
  --q 0 -45 0 -135 0 90 45
```

### 交互式 IK

不提供 `--target` 时，会持续在终端等待目标位姿：

```bash
python examples/fr3_sim.py --mode ik
```

终端支持两种输入格式：

```text
target pose [...] > 0.35 0.10 0.45
target pose [...] > 0.35 0.10 0.45 3.1415926 0.0 0.2
```

- 前 3 个值是 `x y z`，单位为米；只输入 3 个值时保持当前末端方向。
- 后 3 个值是 `roll pitch yaw`，单位为弧度。
- 每次 IK 都以上一次成功结果为初值。
- IK 成功后，C++ 生成 50 Hz 最小加加速度轨迹，MeshCat 播放后继续等待下一次输入。
- 输入 `q`、`quit`、`exit` 或空行退出。

### 单次 IK

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

可用 `--posture-gain` 修改零空间 home 姿态约束强度；程序启动或求解时会打印当前值：

```bash
python examples/fr3_sim.py --mode ik \
  --target 0.35 0.10 0.45 \
  --posture-gain 0.1
```

可以直接输入或拖动数值，范围为 `0 ~ 10` 。修改后会自动重新求解 IK：

- `0` ：关闭零空间约束
- `0.1` ：默认值
- 增大：更强地趋向 home 姿态，但可能增加迭代次数

IK 成功后，C++ 会生成 50 Hz 最小加加速度关节轨迹，并由 MeshCat 播放。

FR3 是 7 自由度机械臂，末端位姿任务只有 6 个约束。C++ IK 默认在 Jacobian 零空间中加入 home 姿态约束，使冗余肘部姿态趋向：

```text
[0, -45, 0, -135, 0, 90, 45] deg
```

`IKOptions.posture_gain` 控制约束强度，默认值为 `0.1`；设置为 `0.0` 可关闭零空间姿态约束。求解器会先收敛末端任务，进入位姿容差后再尽量优化零空间姿态，因此不会降低远距离目标的主任务优先级。较大的增益可能增加迭代次数；通常使用默认值即可。

### Qt 滑条控制

启动 Qt 控制面板和 MeshCat：

```bash
python examples/fr3_sim_qt.py
```

窗口包含两个页签：

- `FK - Joint sliders`：拖动 `joint1` 到 `joint7`，单位为度；范围直接来自 C++ 模型中的关节限位。
- `IK - XYZ / RPY sliders`：拖动 `x/y/z`（米）和 `roll/pitch/yaw`（弧度）；Qt 默认以最高 30 Hz 读取最新目标调用 C++ IK，并以 60 Hz 对关节显示做最小加加速度插值，因此连续拖动时也会持续跟随。
- IK 页签中的 `posture_gain (null-space)` 可直接编辑零空间 home 姿态约束强度；改动后会自动重新求解，设置为 `0` 可关闭约束。

IK 页签以上一次成功解作为下一次求解初值。不可达目标会显示红色错误信息，并保持机器人上一次成功姿态不变。IK 成功后，FK 页签的关节滑条显示最新求解目标；MeshCat 中的机器人则显示正在插值的姿态。

Qt 层的跟随速度可以单独调节，不会修改 IK 算法：

```bash
python examples/fr3_sim_qt.py --mode ik \
  --ik-update-hz 30 --render-hz 60 --smooth-time 0.12
```

`--smooth-time` 越小响应越快，越大视觉越柔和；设置为 `0` 可恢复求解后直接跳到新姿态。

调节建议：

- 更灵敏：--smooth-time `0.08`
- 更柔和：--smooth-time `0.18`
- 禁用平滑：--smooth-time `0`

默认打开 FK 页签；直接打开 IK 页签：

```bash
python examples/fr3_sim_qt.py --mode ik
```

也可以在命令行指定初始零空间约束增益，之后仍可在 IK 页签中继续修改：

```bash
python examples/fr3_sim_qt.py --mode ik --posture-gain 0.1
```

只运行 Qt 控制面板、不启动 MeshCat：

```bash
python examples/fr3_sim_qt.py --no-meshcat
```

如果现有 `fr3sim` 环境还没有 Qt，更新环境后重新激活：

```bash
mamba env update -n fr3sim -f environment.yml
mamba deactivate
mamba activate fr3sim
pip install -e .
```

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

## xacro 转换为 URDF

工程已经包含可用的 `models/fr3_franka_hand.urdf`。安装、编译和运行仿真都不需要 ROS，通常也不需要重新生成 URDF。`scripts/generate_official_urdf.sh` 是一个通用的 xacro 到 URDF 转换脚本。

### 安装 xacro

本工程不依赖 ROS。`xacro==2.1.1` 和 `PyYAML>=6.0` 已写入 `pyproject.toml`，因此 `python -m pip install -e .` 会把它们安装到当前 mamba 环境：

```bash
pip install -e .
```

如果当前 shell 曾经执行过 ROS 的 `setup.bash`，ROS 的 `PYTHONPATH` 可能让 pip 错以为 `xacro` 已经安装。通过本工程最新的 `environment.yml` 创建或更新环境后，重新激活即可自动隔离这些路径：

```bash
mamba env update -n fr3sim -f environment.yml
mamba deactivate
mamba activate fr3sim
pip install -e .
```

如果只想单独安装生成工具：

```bash
PYTHONNOUSERSITE=1 PYTHONPATH= AMENT_PREFIX_PATH= \
  python -m pip install "xacro==2.1.1" "PyYAML>=6.0"
```

安装后检查当前 Python 能否导入：

```bash
command -v python
PYTHONNOUSERSITE=1 PYTHONPATH= AMENT_PREFIX_PATH= \
  python -c "import xacro, yaml; print(xacro.__file__); print(yaml.__version__)"
```

脚本的第一个参数是 xacro 文件的绝对路径，第二个参数是输出 URDF。使用官方 FR3 xacro 重新生成模型：

```bash
./scripts/generate_official_urdf.sh \
  /home/xense/fastiter/franka_description/robots/fr3/fr3.urdf.xacro \
  models/fr3_franka_hand.urdf
```

脚本只接收这两个参数，不接收或修改 xacro 内部参数。输入 xacro 将使用它自身声明的默认值；需要不同配置时，直接修改或准备对应的 xacro 文件，再指定新的输出 URDF 文件名。

脚本支持普通 xacro、相对路径 include，以及 xacro 所在软件包自身的 `$(find package_name)`。它会从输入文件向上查找最近的 `package.xml` 并建立本地软件包映射，因此不需要 ROS，也不需要 `ament-index-python`，原始 xacro 文件不会被修改。

如果 xacro 还引用了其他软件包，把这些软件包的根目录或它们共同的父目录放入 `XACRO_PACKAGE_PATH`：

```bash
XACRO_PACKAGE_PATH=/absolute/path/to/workspace/src \
  ./scripts/generate_official_urdf.sh \
    /absolute/path/to/robot_description/urdf/robot.urdf.xacro \
    models/robot.urdf
```

因此，它可以转换任何语法有效、并且所有 include、YAML 和软件包依赖都能在本地解析的 xacro。

转换会先写入输出目录中的临时文件，只有 xacro 展开、XML `<robot>` 根节点检查以及可选的 `check_urdf` 全部成功后，才会替换目标 URDF。转换失败不会破坏已有的输出文件。

脚本只使用当前激活 mamba 环境中的 Python：

```bash
python -c 'import xacro; xacro.main()'
```

脚本不会查找任何外部 xacro 可执行文件，只会从当前 Python 环境导入 xacro。运行时会清除外部 `PYTHONPATH` 和 `AMENT_PREFIX_PATH`，避免意外加载 ROS 中的 Python 包。如果导入失败，脚本会提示先执行 `python -m pip install -e .`。

`check_urdf` 只用于生成后的额外校验；没有该命令时仍会检查 XML 格式和 `<robot>` 根节点。

## 常见问题

### CMake 找到 Pinocchio 3.4.0

说明当前 mamba 环境没有安装 Pinocchio，pkg-config 回退到了 `/usr/local`。重新执行：

```bash
mamba install -n fr3sim -c conda-forge \
  numpy pyside6 pinocchio=3.9 pybind11 cmake pkg-config \
  cxx-compiler make eigen urdfdom console_bridge tinyxml2 \
  pip setuptools wheel
```

### 找不到 tinyxml2

说明当前环境缺少原生 C++ 依赖。检查：

```bash
mamba list -n fr3sim | grep -E "pinocchio|tinyxml2|urdfdom|console_bridge"
```

### 生成 URDF 时提示无法导入 xacro

这通常不是依赖没有声明，而是当前 shell 的 ROS `PYTHONPATH` 让 pip 看到了 `/opt/ros` 中的 xacro，却没有把它安装到 `fr3sim`。更新并重新激活环境后重装：

```bash
mamba env update -n fr3sim -f environment.yml
mamba deactivate
mamba activate fr3sim
python -m pip install -e .
python -c "import xacro, yaml; print(xacro.__file__); print(yaml.__file__)"
```

然后使用正确的 `.urdf` 后缀生成：

```bash
./scripts/generate_official_urdf.sh \
  /home/xense/fastiter/franka_description/robots/fr3/fr3.urdf.xacro \
  models/fr3_franka_hand.urdf
```

### pip 为什么不能单独安装所有依赖

pip 会安装 Python 包并编译当前工程，但不会管理 conda-forge 的 C++ 编译器、Pinocchio、urdfdom 和 tinyxml2。它们必须先通过 `environment.yml` 或 `mamba install` 安装。

## 许可证

本项目源代码采用 [MIT License](LICENSE)。由 Franka 官方 `franka_description` 生成的模型仍遵循其原始 Apache-2.0 许可证。
