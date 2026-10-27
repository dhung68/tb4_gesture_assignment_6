"""Two independent processes. All tunable defaults are launch arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from tb4_gesture_control.gesture_recognition_node import GESTURE_DEFAULTS
from tb4_gesture_control.robot_behavior_node import BEHAVIOR_DEFAULTS


def generate_launch_description():
    arguments = [DeclareLaunchArgument('use_sim_time', default_value='true',
                                       description='Use Gazebo /clock for behavior phase timing')]
    nodes = []
    for executable, defaults in (
        ('robot_behavior_node', BEHAVIOR_DEFAULTS),
        ('gesture_recognition_node', GESTURE_DEFAULTS),
    ):
        parameters = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                    value_type=bool)}
        for name, value in defaults.items():
            default_string = str(value).lower() if isinstance(value, bool) else str(value)
            arguments.append(DeclareLaunchArgument(name, default_value=default_string))
            parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=type(value))
        nodes.append(Node(package='tb4_gesture_control', executable=executable,
                          name=executable, output='screen', parameters=[parameters]))
    # Closing webcam/Q or any process failure shuts down the pair. The control
    # watchdog independently covers a hung or abruptly killed perception process.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason=f'{node.name} exited'))]))
        for node in nodes]
    return LaunchDescription(arguments + handlers + nodes)
