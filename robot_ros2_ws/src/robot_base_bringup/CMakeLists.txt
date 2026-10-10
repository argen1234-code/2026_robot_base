cmake_minimum_required(VERSION 3.8)
project(robot_base_bringup)

find_package(ament_cmake REQUIRED)

# 纯资源包：把 launch / config 装到 share/
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY udev
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/chassis_bridge.py scripts/wechat_bridge.py DESTINATION lib/${PROJECT_NAME})
# chassis_telemetry.py 是遥测帧解析的【唯一一份】实现，chassis_bridge 与
# tools/twist_sign_probe.py 都依赖它 —— 漏装它会导致 chassis_bridge 起不来。
install(FILES scripts/chassis_protocol.py scripts/chassis_telemetry.py
                scripts/wechat_protocol.py scripts/wechat_map.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_chassis_protocol test/test_chassis_protocol.py)
  ament_add_pytest_test(test_chassis_telemetry test/test_chassis_telemetry.py)
  ament_add_pytest_test(test_wechat_protocol test/test_wechat_protocol.py)
endif()

ament_package()
