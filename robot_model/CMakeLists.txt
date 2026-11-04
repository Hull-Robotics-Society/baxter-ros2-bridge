cmake_minimum_required(VERSION 3.10)
project(baxter_description)

find_package(ament_cmake REQUIRED)

# urdf/ and meshes/ are downloaded by fetch_model.sh; without them there is nothing to show.
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/baxter.urdf)
  message(FATAL_ERROR "The robot model is missing. Run first:  bash robot_model/fetch_model.sh")
endif()

install(DIRECTORY urdf meshes xacro launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
