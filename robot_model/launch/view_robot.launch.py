"""Shows Baxter's current posture in RViz 2.

    ros2 launch baxter_description view_robot.launch.py
    ros2 launch baxter_description view_robot.launch.py rviz:=false    # without the window
    ros2 launch baxter_description view_robot.launch.py robot:=one     # the robot named "one"

Two programs are started:

  robot_state_publisher  reads the joint angles the robot reports (/robot/joint_states,
                         through the bridge) and works out from the robot model where
                         every part of the robot is.
  rviz2                  draws the robot from that.

The positions of the parts ("transforms") go to /baxter_model/tf instead of the usual
/tf. The robot publishes a /tf of its own; the bridge would carry ours to the robot and
mix the two. A program of your own that wants the transforms listens to
/baxter_model/tf and /baxter_model/tf_static.

A robot with a name (docs/two-robots.md): `robot:=one`, or the variable BAXTER_ROBOT=one,
shows that robot. Everything then has the name in front, as the robot's own topics do:
the joint angles are read from /one/robot/joint_states and the transforms go to
/one/baxter_model/tf. For all robots in one picture see view_robots.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    """Make the two programs, in the namespace of the robot that is to be shown."""
    share = get_package_share_directory('baxter_description')
    with open(os.path.join(share, 'urdf', 'baxter.urdf'), encoding='utf-8') as model_file:
        model = model_file.read()

    robot = LaunchConfiguration('robot').perform(context).strip().strip('/')
    namespace = '/' + robot if robot else ''
    transforms = [
        ('/tf', namespace + '/baxter_model/tf'),
        ('/tf_static', namespace + '/baxter_model/tf_static'),
    ]
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': model}],
            remappings=[('joint_states', namespace + '/robot/joint_states')] + transforms
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            arguments=['-d', os.path.join(share, 'rviz', 'baxter.rviz')],
            # The configuration asks for /robot_description; a named robot's is under its name.
            remappings=[('/robot_description', namespace + '/robot_description')] + transforms,
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open the RViz 2 window (false: only publish the transforms)'
        ),
        DeclareLaunchArgument(
            'robot',
            default_value=os.environ.get('BAXTER_ROBOT', ''),
            description='Name of the robot to show (empty: the robot without a name)'
        ),
        OpaqueFunction(function=start),
    ])
