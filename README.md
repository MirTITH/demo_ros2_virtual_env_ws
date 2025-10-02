# 在 ROS2 中使用虚拟环境的方法

这个工作空间展示了如何在 ROS2 中使用 Python 虚拟环境（venv）。这样就可以方便地在 ROS2 中使用 pytorch 等库。

本项目使用 [uv](https://docs.astral.sh/uv/) 来管理虚拟环境和依赖包。

## 测试体验

### 克隆这个工作空间：

```bash
git clone <repository_url>
cd demo_ros2_virtual_env_ws
```

### 安装 uv：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

> 更多安装方法：https://docs.astral.sh/uv/getting-started/installation/#standalone-installer

### 给 uv 换源（可选）：

编辑 `~/.config/uv/uv.toml`，添加以下内容：

```toml
[[index]]
url = "https://mirrors.ustc.edu.cn/pypi/simple"
default = true
```

一些国内镜像源：
- 哈深校内源：https://mirrors.osa.moe/pypi/web/simple
- 清华大学：https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
- 中国科学技术大学：https://mirrors.ustc.edu.cn/pypi/simple
- 阿里云： http://mirrors.aliyun.com/pypi/simple
- 腾讯云镜像源：https://mirrors.cloud.tencent.com/pypi/simple

### 创建并激活虚拟环境：

```bash
uv sync
```

### 构建工作空间：

```bash
# 加上 --symlink-install 会以符号链接的方式安装包，这样改动代码后不需要重新构建。（添加新文件后仍然需要重新构建）
colcon build --symlink-install
```

### 运行 demo 节点：

source ros2 工作空间：

```bash
# 如果使用 bash：
. install/setup.bash 

# 如果使用 zsh：
. install/setup.zsh
```

方法一：启动launch文件：

```bash
ros2 launch demo_uv_pkg launch_demo_node.launch.py
```

方法二：直接运行节点：

```bash
source .venv/bin/activate
python src/demo_uv_pkg/demo_uv_pkg/demo_py_node.py
```
很遗憾，不能使用 ros2 run demo_uv_pkg demo_py_node，因为 ros2 命令会强制使用系统的 python 解释器，而不是虚拟环境中的解释器。

## 如何在自己的工作空间中使用

### 创建 uv 项目：

在你的 ROS2 工作空间根目录下，执行：

```bash
uv init
```

### 删除不需要的文件：

uv 会创建一些示例文件，请按照下面的说明删除不需要的文件：

需要删除的文件：
- `main.py`：uv 示例文件，不需要

必须保留的文件：
- `pyproject.toml`：uv 项目的配置文件

可以选择删除或保留的文件：
- `.python-version`：指定 Python 版本
- `uv.lock`：锁定依赖包版本
- `.gitignore`：git 忽略文件
- `README.md`：项目说明文件

注意检查是否出现了2个大小写不同的 `README.md` 文件

`.python-version` 文件中指定了与当前系统 Python 版本一致的版本号。如果你的工作空间需要给使用其他版本系统的用户使用，请删除这个文件和 `uv.lock` 文件。

### 添加所需的依赖包

```bash
uv add <package_name>

# 或者
uv add -r requirements.txt
```

> 使用 uv add 命令会自动创建虚拟环境到 `.venv` 目录下，并安装依赖包

### 参考本项目的 `launch_demo_node.launch.py`，创建自己的 launch 文件

### 大功告成

你已经完成了所有步骤。现在可以愉快地在 ROS2 中使用虚拟环境了！

而且，别人在克隆你的工作空间后，只需要执行 `uv sync` 就可以创建虚拟环境并安装依赖包。

## 原理说明

ROS2 要求 Python 版本必须与系统 Python 版本一致（比如 Ubuntu 22.04 是 Python 3.10），所以创建虚拟环境时也要使用相同版本的 Python。

uv 是一个 Python 包和工程管理工具，可以方便地创建和管理虚拟环境，并安装依赖包。它主要读取 `pyproject.toml` 文件中的依赖项，并将它们安装到虚拟环境中。

你可以使用 `uv add` 命令添加新的依赖包，比如：

```bash
uv add requests

# Specify a version constraint
uv add 'requests==2.31.0'

# Add a git dependency
uv add git+https://github.com/psf/requests
```

更多用法请参考官方文档：https://docs.astral.sh/uv/guides/

你也可以不使用 uv，直接使用 venv 创建虚拟环境，然后使用 pip 安装依赖包：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

在 `launch_demo_node.launch.py` 中，我们使用 `ExecuteProcess`，直接调用虚拟环境中的 Python 解释器来运行节点，而不是通常的 `Node`，这样就可以确保节点在虚拟环境中运行。

## 常见问题

### VSCode 智能提示找不到包的解决办法

确保已安装以下扩展：
- Python
- Robotics Developer Environment

确保 VSCode 打开的是工作空间的根目录，并编译工作空间。

1. 解决找不到 ROS2 包的问题

    按 `Ctrl+Shift+P`，选择 `ROS2: Update Python Path`，检查 `.vscode/settings.json` 中是否类似下面的内容（应该包含以`/opt/ros/humble`和当前工作空间路径开头的路径）：
    ```json
    {
        "python.autoComplete.extraPaths": [
            "/home/ubuntu/Documents/demo_ros2_virtual_env_ws/build/demo_uv_pkg",
            "/home/ubuntu/Documents/demo_ros2_virtual_env_ws/install/demo_uv_pkg/lib/python3.10/site-packages",
            "/opt/ros/humble/lib/python3.10/site-packages",
            "/opt/ros/humble/local/lib/python3.10/dist-packages"
        ],
        "python.analysis.extraPaths": [
            "/home/ubuntu/Documents/demo_ros2_virtual_env_ws/build/demo_uv_pkg",
            "/home/ubuntu/Documents/demo_ros2_virtual_env_ws/install/demo_uv_pkg/lib/python3.10/site-packages",
            "/opt/ros/humble/lib/python3.10/site-packages",
            "/opt/ros/humble/local/lib/python3.10/dist-packages"
        ]
    }
    ```

    如果没有，请重启 VSCode，然后再试一次。

2. 解决找不到虚拟环境包的问题

    随便打开一个 .py 文件，点击右下角的 Python 版本号，选择 `.venv` 目录下的 Python 解释器即可。

此时 VSCode 应该能同时找到 ROS2 的包和虚拟环境中的包了。