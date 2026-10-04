cmake_minimum_required(VERSION 3.8)
project(robot_base_description)

find_package(ament_cmake REQUIRED)

# 纯资源包：把 urdf / rviz / launch 装到 share/
install(DIRECTORY urdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

# xacro 文件需要被安装后仍能互相 include（用 $(find ...) 定位），
# 上面的 install(DIRECTORY) 已满足该要求。

ament_package()
