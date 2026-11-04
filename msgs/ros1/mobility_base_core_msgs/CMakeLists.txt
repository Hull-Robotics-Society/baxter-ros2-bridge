cmake_minimum_required(VERSION 2.8.3)
project(mobility_base_core_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  BatteryState.msg
  BumperState.msg
  JoystickRaw.msg
  Mode.msg
)

add_service_files(DIRECTORY srv FILES
  GetMaxSpeed.srv
  SetMaxSpeed.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
