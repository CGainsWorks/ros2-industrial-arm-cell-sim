cmake_minimum_required(VERSION 3.8)
project(cell_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# The Gazebo world is generated into simulation/ with everything else, and
# installed here so launch files can find it by package.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../simulation/cell.world
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
