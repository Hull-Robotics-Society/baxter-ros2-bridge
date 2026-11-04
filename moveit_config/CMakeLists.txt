cmake_minimum_required(VERSION 3.10)
project(baxter_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

# The helper the launch file starts (see its text), and the script that makes baxter.srdf.
install(PROGRAMS scripts/joint_states_for_moveit.py scripts/make_srdf.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
