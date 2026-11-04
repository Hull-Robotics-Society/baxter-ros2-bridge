from setuptools import find_packages, setup

package_name = 'baxter_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    # The two images that the smoke test and show_image put on Baxter's screen, and the
    # marker file that tells type checkers (Pylance in VS Code) that the library carries
    # type annotations.
    package_data={package_name: ['images/*.png', 'py.typed']},
    data_files=[
        # Lets ROS 2 tools (ros2 run, ros2 pkg) find the package.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # Tells colcon to run the tests with pytest (without it, colcon falls back to unittest,
    # which finds no tests in test/).
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Samuel Galla',
    maintainer_email='Techniksam@users.noreply.github.com',
    description='Python interface to the Baxter robot for ROS 2, through the bridge.',
    license='BSD-3-Clause',
    entry_points={
        # Command-line tools: ros2 run baxter_interface <name>
        'console_scripts': [
            'enable_robot = baxter_interface.tools.enable_robot:main',
            'move_to_neutral = baxter_interface.tools.move_to_neutral:main',
            'tuck_arms = baxter_interface.tools.tuck_arms:main',
            'camera_control = baxter_interface.tools.camera_control:main',
            'calibrate_arm = baxter_interface.tools.calibrate_arm:main',
            'tare = baxter_interface.tools.tare:main',
            'smoke_test = baxter_interface.tools.smoke_test:main',
            'robot_info = baxter_interface.tools.robot_info:main',
            'show_image = baxter_interface.tools.show_image:main',
            'base_status = baxter_interface.tools.base_status:main',
            'joint_trajectory_action_server = '
            'baxter_interface.tools.joint_trajectory_action_server:main',
        ],
    },
)
