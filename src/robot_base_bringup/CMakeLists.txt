cmake_minimum_required(VERSION 3.8)
project(robot_base_bringup)

find_package(ament_cmake REQUIRED)

# 纯资源包：把 launch / config 装到 share/
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/chassis_bridge.py scripts/wechat_bridge.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/chassis_protocol.py scripts/wechat_protocol.py scripts/wechat_map.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_chassis_protocol test/test_chassis_protocol.py)
  ament_add_pytest_test(test_wechat_protocol test/test_wechat_protocol.py)
endif()

ament_package()
