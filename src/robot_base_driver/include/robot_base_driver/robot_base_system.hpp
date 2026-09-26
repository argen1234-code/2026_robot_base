#ifndef ROBOT_BASE_DRIVER__ROBOT_BASE_SYSTEM_HPP_
#define ROBOT_BASE_DRIVER__ROBOT_BASE_SYSTEM_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_base_driver
{

/// 差速底盘硬件接口。
///
/// 通过 pluginlib 注册为 "robot_base_driver/RobotBaseSystem"。
///
/// 关节顺序约定：index 0 = 左轮，index 1 = 右轮，
/// 由 URDF 中 <ros2_control> 块内 <joint> 的书写顺序决定，
/// 必须与 controllers.yaml 里 left_wheel_names/right_wheel_names 对应。
///
/// TODO(hardware): 当前 read/write 是桩实现（命令回声为状态），
///   让整套链路在没有真实硬件时也能跑通。接真机时替换这两个函数即可。
class RobotBaseSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 左右两个驱动轮
  static constexpr std::size_t kNumWheels = 2;

  // 关节名，索引与 hw_* 数组一一对应（0=左, 1=右）
  std::vector<std::string> wheel_joint_names_;

  // 硬件状态与命令。用双精度数组是因为硬件接口只持有裸指针，
  // 这些值必须在本对象生命周期内保持地址稳定。
  std::vector<double> hw_positions_{0.0, 0.0};
  std::vector<double> hw_velocities_{0.0, 0.0};
  std::vector<double> hw_commands_{0.0, 0.0};

  // 来自 URDF <hardware><param> 的串口配置
  std::string serial_port_{"/dev/ttyUSB0"};
  int baud_rate_{115200};

  bool motor_enabled_{false};

  // TODO(hardware): 串口句柄/协议状态在接真机时加在这里
  // int serial_fd_{-1};
};

}  // namespace robot_base_driver

#endif  // ROBOT_BASE_DRIVER__ROBOT_BASE_SYSTEM_HPP_
