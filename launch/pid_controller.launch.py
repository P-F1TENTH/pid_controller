import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

package_name = "pid_controller"


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package=package_name,
                executable="pid_controller",
                name="pid_controller",
                emulate_tty=True,
            )
        ]
    )
