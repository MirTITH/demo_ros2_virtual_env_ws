from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
import os


def launch_setup(context, *args, **kwargs):
    launch_entities = []

    pkg_path = "src/demo_uv_pkg"
    python_executable = os.path.join(".venv", "bin", "python")

    launch_entities.append(
        ExecuteProcess(
            cmd=[python_executable, os.path.join(pkg_path, "demo_uv_pkg", "demo_py_node.py")],
            output="screen",
            shell=True,
        ),
    )

    return launch_entities


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(declared_arguments)
