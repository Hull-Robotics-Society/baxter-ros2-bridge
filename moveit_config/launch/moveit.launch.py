"""MoveIt 2 for Baxter: plan a motion that avoids collisions, look at it, run it.

    ros2 launch baxter_moveit_config moveit.launch.py
    ros2 launch baxter_moveit_config moveit.launch.py rviz:=false     # without the window
    ros2 launch baxter_moveit_config moveit.launch.py server:=false   # the action server runs elsewhere
    ros2 launch baxter_moveit_config moveit.launch.py robot:=one      # the robot named "one"
    ros2 launch baxter_moveit_config moveit.launch.py acceleration_limit:=8.0
                                    # another top acceleration (rad/s^2) than config/joint_limits.yaml
    ros2 launch baxter_moveit_config moveit.launch.py left_electric_gripper:=true right_electric_gripper:=true
                                    # Rethink's electric grippers are mounted
    ros2 launch baxter_moveit_config moveit.launch.py planner:=chomp  # CHOMP instead of OMPL
    ros2 launch baxter_moveit_config moveit.launch.py robot_model:=generic
                                    # baxter_common's model instead of the robot's own

Five programs are started:

  joint_states_for_moveit passes the joint angles the robot reports (/robot/joint_states,
                          through the bridge) on to the others, stamped with this
                          computer's time (scripts/joint_states_for_moveit.py says why)
  robot_state_publisher   works out from the joint angles where the robot's parts are
  joint trajectory        baxter_interface's action server: takes a timed path for an arm
  action server           and makes the arm follow it (docs/trajectory-action.md)
  move_group              MoveIt itself: plans on request, and sends the plan to the
                          action server
  rviz2                   the window: drag a hand to where it should go, "Plan", "Execute"

The positions of the robot's parts go to /baxter_model/tf, as with RViz alone
(robot_model/launch/view_robot.launch.py says why).

The robot model: by default the robot's own (robot_model:=robot), as in the original
configuration. Every Baxter keeps it in its ROS 1 parameter /robot_description, with the
arms' mounting points measured for that robot; the bridge passes it on as the topic
robot/robot_description (docker/bridge-jazzy/robot_model.py), and this file waits up to
30 s for it.
robot_model:=generic takes baxter_common's general model instead (robot_model/), the
same for every robot; robot_model:=<path> a model file of your own.

Electric grippers: with `left_electric_gripper:=true` and/or `right_electric_gripper:=true`
(or LEFT_ELECTRIC_GRIPPER=1 / RIGHT_ELECTRIC_GRIPPER=1 in the settings file) MoveIt uses
the matching config/baxter_*gripper*.srdf: the pairs of the gripper's parts that are never
checked against each other, as in the original. The gripper's shape itself comes from the
robot model: the robot's own has it if the robot reports the gripper in it (a warning
says when the setting and the model disagree); with robot_model:=generic it is Rethink's
model of the gripper (baxter_description's xacro/baxter_grippers.urdf.xacro), with
Rethink's default fingers (extended narrow, slot 2, paddle tips) unless finger:=,
finger_slot:=, finger_tip:= name others (urdf/rethink_ee/electric_gripper/fingers in
baxter_description).

Two planners, as in the original: OMPL (the default) and CHOMP (planner:=chomp;
config/chomp_planning.yaml). Both are loaded; RViz's panel can switch between them.

A robot with a name (docs/two-robots.md): `robot:=one`, or the variable BAXTER_ROBOT=one,
runs all of this for that robot, in its namespace: MoveIt's action is then
/one/move_action, and baxter_interface.planning finds it when the program is started for
the same robot. One MoveIt per robot; planning for two robots together is not part of this.
"""

import os
import tempfile
import time
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Where the action server offers an arm's action, and the name MoveIt asks for
# (config/moveit_controllers.yaml). An action is five topics and services under its name.
ACTION_PARTS = (
    '_action/send_goal',
    '_action/cancel_goal',
    '_action/get_result',
    '_action/feedback',
    '_action/status',
)


def action_remappings():
    remappings = []
    for side in ('left', 'right'):
        asked = f'{side}_arm_controller/follow_joint_trajectory'
        offered = f'robot/limb/{side}/follow_joint_trajectory'
        remappings += [(f'{asked}/{part}', f'{offered}/{part}') for part in ACTION_PARTS]
    return remappings


def picture_for(namespace, here):
    """Return the RViz configuration to use: the package's, or a copy for a named robot.

    The motion planning panel has to be told in which namespace MoveIt runs.
    """
    original = os.path.join(here, 'rviz', 'moveit.rviz')
    if not namespace:
        return original
    with open(original, encoding='utf-8') as config_file:
        config = config_file.read()
    config = config.replace('Move Group Namespace: ""', f'Move Group Namespace: {namespace}')
    handle, path = tempfile.mkstemp(prefix='baxter_moveit_', suffix='.rviz')
    with os.fdopen(handle, 'w', encoding='utf-8') as out:
        out.write(config)
    return path


def switched_on(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes')


# How long to wait for the robot's own model from the bridge.
MODEL_WAIT = 30.0   # s


def wait_for_robot_model(topic, seconds=MODEL_WAIT):
    """Return the robot's own model (URDF text) from the bridge's topic, or None."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    # A ROS context of its own, so that nothing of it is left for the programs started below.
    context = rclpy.Context()
    rclpy.init(context=context)
    received = []
    try:
        node = rclpy.create_node('moveit_robot_model_reader', context=context,
                                 start_parameter_services=False)
        node.create_subscription(
            String, topic, lambda message: received.append(message.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        end = time.monotonic() + seconds
        while not received and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.2)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
    return received[0] if received else None


def grippers_in(model):
    """Return (left, right): whether a robot model (URDF text) has the electric gripper's fingers."""
    links = {link.get('name') for link in ElementTree.fromstring(model).findall('link')}
    return 'l_gripper_l_finger' in links, 'r_gripper_l_finger' in links


def robot_model_for(context, namespace):
    """Return the robot model (file, xacro arguments) and the SRDF for the grippers mounted."""
    description = get_package_share_directory('baxter_description')
    left = switched_on(context, 'left_electric_gripper')
    right = switched_on(context, 'right_electric_gripper')
    semantic = {
        (False, False): 'config/baxter.srdf',
        (True, False): 'config/baxter_left_gripper.srdf',
        (False, True): 'config/baxter_right_gripper.srdf',
        (True, True): 'config/baxter_both_grippers.srdf',
    }[(left, right)]
    choice = LaunchConfiguration('robot_model').perform(context).strip()

    if choice == 'generic':
        if not (left or right):
            return os.path.join(description, 'urdf', 'baxter.urdf'), None, semantic
        mappings = {
            'left_electric_gripper': str(left).lower(),
            'right_electric_gripper': str(right).lower(),
        }
        for name in ('finger', 'finger_slot', 'finger_tip'):
            mappings[name] = LaunchConfiguration(name).perform(context).strip()
        return os.path.join(description, 'xacro', 'baxter_grippers.urdf.xacro'), mappings, semantic

    if choice != 'robot':
        if not os.path.isfile(choice):
            raise RuntimeError(f"robot_model:={choice}: no such file. Use robot (the robot's own "
                               "model, the default), generic, or the path of a model file.")
        return os.path.abspath(choice), None, semantic

    topic = namespace + '/robot/robot_description'
    print(f"Waiting for the robot's own robot model on {topic} (from the bridge) ...", flush=True)
    model = wait_for_robot_model(topic)
    if model is None:
        raise RuntimeError(
            f"The robot's own robot model did not arrive on {topic} within {MODEL_WAIT:g} s. "
            'Is the bridge running, for this robot? To plan with the general model instead: '
            'robot_model:=generic')
    # Rethink's gripper model refers to its own ROS 1 package; here its files are part of
    # baxter_description (robot_model/fetch_model.sh).
    model = model.replace('package://rethink_ee_description/meshes/',
                          'package://baxter_description/meshes/rethink_ee/')
    for side, said, has in (('left', left, grippers_in(model)[0]), ('right', right, grippers_in(model)[1])):
        if said and not has:
            print(f"WARNING: {side.upper()}_ELECTRIC_GRIPPER is set, but the robot's model has no "
                  f'electric gripper on the {side} arm. MoveIt does not know its shape.', flush=True)
        if has and not said:
            print(f"WARNING: the robot's model has an electric gripper on the {side} arm, but "
                  f'{side.upper()}_ELECTRIC_GRIPPER is not set: start with {side}_electric_gripper:=true '
                  f"(or {side.upper()}_ELECTRIC_GRIPPER=1 in .env), or MoveIt takes the gripper's own parts for a collision.",
                  flush=True)
    handle, path = tempfile.mkstemp(prefix='baxter_robot_model_', suffix='.urdf')
    with os.fdopen(handle, 'w', encoding='utf-8') as out:
        out.write(model)
    print(f"Using the robot's own robot model ({len(model)} characters).", flush=True)
    return path, None, semantic


def start(context):
    """Make the five programs, in the namespace of the robot they are for."""
    robot = LaunchConfiguration('robot').perform(context).strip().strip('/')
    namespace = '/' + robot if robot else ''
    here = get_package_share_directory('baxter_moveit_config')
    model, mappings, semantic = robot_model_for(context, namespace)
    planner = LaunchConfiguration('planner').perform(context).strip()
    if planner not in ('ompl', 'chomp'):
        raise RuntimeError(f'planner:={planner}: use ompl or chomp')

    moveit = (
        MoveItConfigsBuilder('baxter', package_name='baxter_moveit_config')
        .robot_description(file_path=model, mappings=mappings)
        .robot_description_semantic(file_path=semantic)
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline=planner, pipelines=['ompl', 'chomp'])
        .to_moveit_configs()
    )

    # The top acceleration for every joint, if one was given at the start.
    limits = moveit.joint_limits
    asked = LaunchConfiguration('acceleration_limit').perform(context).strip()
    if asked:
        for joint in limits['robot_description_planning']['joint_limits'].values():
            joint['max_acceleration'] = float(asked)
    settings = moveit.to_dict()
    settings.update(limits)

    robot_state = [
        ('joint_states', 'moveit/joint_states'),
        ('/tf', namespace + '/baxter_model/tf'),
        ('/tf_static', namespace + '/baxter_model/tf_static'),
    ]
    execution = {
        # How far the arm may be from where a plan starts when the plan is run (rad). The
        # original configuration's value (trajectory_execution.launch in
        # reference/moveit_robots_baxter). With 0.02, a plan run some seconds after it was
        # made was refused on the robot: the arm had settled by a little more since.
        'trajectory_execution.allowed_start_tolerance': 0.1,
        # MoveIt gives up on a motion that takes much longer than planned. The margins
        # are those of the original configuration (baxter_controllers.yaml in
        # reference/moveit_robots_baxter): 50 times the planned time, plus 20 s.
        'trajectory_execution.allowed_execution_duration_scaling': 50.0,
        'trajectory_execution.allowed_goal_duration_margin': 20.0,
        # The original's resolution of the 3D map of the room (baxter_moveit_sensor_manager.launch
        # in reference/moveit_robots_baxter), used only once a depth camera feeds one.
        'octomap_resolution': 0.02,
        # Tell RViz and other programs about the scene MoveIt plans in.
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    return [
        Node(
            package='baxter_moveit_config',
            executable='joint_states_for_moveit.py',
            namespace=namespace,
            parameters=[moveit.robot_description]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[moveit.robot_description],
            remappings=robot_state
        ),
        Node(
            package='baxter_interface',
            executable='joint_trajectory_action_server',
            namespace=namespace,
            output='screen',
            condition=IfCondition(LaunchConfiguration('server'))
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            namespace=namespace,
            output='screen',
            parameters=[settings, execution],
            remappings=robot_state + action_remappings()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            arguments=['-d', picture_for(namespace, here)],
            parameters=[
                moveit.robot_description,
                moveit.robot_description_semantic,
                moveit.robot_description_kinematics,
                moveit.planning_pipelines,
                limits,
            ],
            remappings=robot_state,
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open the RViz 2 window with the motion planning panel'
        ),
        DeclareLaunchArgument(
            'server',
            default_value='true',
            description="Start baxter_interface's joint trajectory action server "
                        '(false if it already runs)'
        ),
        DeclareLaunchArgument(
            'acceleration_limit',
            default_value='',
            description='Top acceleration for every joint in rad/s^2 '
                        '(empty: the values of config/joint_limits.yaml)'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='robot',
            description="robot: the robot's own model (from the bridge); generic: baxter_common's; "
                        'or the path of a model file'
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='ompl',
            description='The planner used unless a request names one: ompl or chomp'
        ),
        DeclareLaunchArgument(
            'left_electric_gripper',
            default_value=os.environ.get('LEFT_ELECTRIC_GRIPPER', '0'),
            description="Rethink's electric gripper is mounted on the left arm"
        ),
        DeclareLaunchArgument(
            'right_electric_gripper',
            default_value=os.environ.get('RIGHT_ELECTRIC_GRIPPER', '0'),
            description="Rethink's electric gripper is mounted on the right arm"
        ),
        DeclareLaunchArgument(
            'finger',
            default_value='extended_narrow',
            description="The electric grippers' fingers (both grippers, both fingers)"
        ),
        DeclareLaunchArgument(
            'finger_slot',
            default_value='2',
            description="The slot the fingers are mounted in, as numbered in Rethink's model"
        ),
        DeclareLaunchArgument(
            'finger_tip',
            default_value='paddle_tip',
            description='The tips on the fingers'
        ),
        DeclareLaunchArgument(
            'robot',
            default_value=os.environ.get('BAXTER_ROBOT', ''),
            description='Name of the robot to plan for (empty: the robot without a name)'
        ),
        OpaqueFunction(function=start),
    ])
