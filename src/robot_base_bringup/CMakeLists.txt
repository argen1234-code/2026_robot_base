cmake_minimum_required(VERSION 3.8)
project(robot_base_bringup)

find_package(ament_cmake REQUIRED)

# 纯资源包：把 launch / config 装到 share/
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
