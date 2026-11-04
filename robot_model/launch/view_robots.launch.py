"""Shows several Baxters in one RViz 2 picture, each where it stands in the room.

    ros2 launch baxter_description view_robots.launch.py                    # all robots found
    ros2 launch baxter_description view_robots.launch.py robots:="one two"  # these
    ros2 launch baxter_description view_robots.launch.py rviz:=false        # without the window

For robots that have a name (docs/two-robots.md). Per robot one robot_state_publisher is
started, which reads that robot's joint angles (/one/robot/joint_states) and works out
where its parts are. So that the robots can be told apart in one picture, every
coordinate frame of a robot has the robot's name in front: one/base, one/left_gripper,
two/base, ... That is the usual ROS 2 form for several robots.

Where a robot stands in the room is not known to the robot. Its bridge says it: the
ROBOT_POSITION line of the robot's settings file becomes the transform from the frame
"world" to the robot's base (docker/bridge-jazzy/robot_place.py). Without that line all
robots stand on the same spot.

All transforms go to /baxter_model/tf and /baxter_model/tf_static (not the usual /tf,
see view_robot.launch.py for why). A program of your own that wants the transforms of
all robots listens there.

Without `robots:=...` the launch file looks for four seconds which robots' bridges are
running. Robots that are started later are not added.
"""

import os
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Every bridge offers this topic for its robot. It is how robots are found.
SIGN_OF_A_ROBOT = '/baxter_bridge/robot_info'
LOOK_FOR = 4.0   # seconds

TRANSFORMS = [
    ('/tf', '/baxter_model/tf'),
    ('/tf_static', '/baxter_model/tf_static'),
]


def robots_found():
    """Return the names of the robots whose bridges are running ('' for one without a name)."""
    import rclpy

    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        f'baxter_view_robots_lookup_{os.getpid()}',
        context=context,
        start_parameter_services=False
    )
    names = set()
    end = time.monotonic() + LOOK_FOR
    try:
        while time.monotonic() < end:
            for topic, _ in node.get_topic_names_and_types():
                if topic.endswith(SIGN_OF_A_ROBOT) and node.count_publishers(topic) > 0:
                    name = topic[:-len(SIGN_OF_A_ROBOT)].lstrip('/')
                    if '/' not in name:
                        names.add(name)
            time.sleep(0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
    return sorted(names)


def picture_for(robots, share):
    """Write an RViz configuration that shows these robots; return the file's path.

    It is the single robot's configuration (rviz/baxter.rviz) with one "RobotModel" entry
    per robot instead of one, and the room ("world") as the frame everything is drawn in.
    """
    with open(os.path.join(share, 'rviz', 'baxter.rviz'), encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    manager = config['Visualization Manager']
    displays = []
    for display in manager['Displays']:
        if display['Class'] != 'rviz_default_plugins/RobotModel':
            displays.append(display)
            continue
        for robot in robots:
            entry = yaml.safe_load(yaml.safe_dump(display))   # a copy of its own
            entry['Name'] = f'Robot {robot}' if robot else 'Robot'
            entry['Description Topic']['Value'] = (f'/{robot}' if robot else '') + '/robot_description'
            entry['TF Prefix'] = robot
            displays.append(entry)
    manager['Displays'] = displays
    manager['Global Options']['Fixed Frame'] = 'world'
    manager['Views']['Current']['Distance'] = 5.0
    config['Panels'][0]['Property Tree Widget']['Expanded'] = []

    handle, path = tempfile.mkstemp(prefix='baxter_robots_', suffix='.rviz')
    with os.fdopen(handle, 'w', encoding='utf-8') as out:
        yaml.safe_dump(config, out)
    return path


def start(context):
    """Make one robot_state_publisher per robot, and RViz with a picture of all of them."""
    share = get_package_share_directory('baxter_description')
    with open(os.path.join(share, 'urdf', 'baxter.urdf'), encoding='utf-8') as model_file:
        model = model_file.read()

    asked_for = LaunchConfiguration('robots').perform(context).split()
    robots = [name.strip('/') for name in asked_for] if asked_for else robots_found()
    if not robots:
        return [LogInfo(msg='No robot found: no bridge is running in this ROS 2 domain. '
                            'Start the bridges first, or name the robots: robots:="one two"')]

    described = ', '.join(name if name else '(one without a name)' for name in robots)
    actions = [LogInfo(msg=f'Showing: {described}')]
    for robot in robots:
        namespace = '/' + robot if robot else ''
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{
                'robot_description': model,
                'frame_prefix': robot + '/' if robot else '',
            }],
            remappings=[('joint_states', namespace + '/robot/joint_states')] + TRANSFORMS
        ))
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', picture_for(robots, share)],
        remappings=TRANSFORMS,
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open the RViz 2 window (false: only publish the transforms)'
        ),
        DeclareLaunchArgument(
            'robots',
            default_value='',
            description='Names of the robots to show, separated by spaces (empty: all found)'
        ),
        OpaqueFunction(function=start),
    ])
