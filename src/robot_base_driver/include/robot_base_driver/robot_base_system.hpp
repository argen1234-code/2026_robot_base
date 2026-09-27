#ifndef ROBOT_BASE_DRIVER__ROBOT_BASE_SYSTEM_HPP_
#define ROBOT_BASE_DRIVER__ROBOT_BASE_SYSTEM_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
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
/// 接口导出方式（ROS 2 Jazzy / hardware_interface >= 4.4）：
///   状态与命令接口由框架按 URDF 中的声明自动创建，
///   本类不再重写 export_state_interfaces()/export_command_interfaces()
///   （这两个在 Jazzy 已废弃），而是通过 get_state_interface_handle() /
///   get_command_interface_handle() 取句柄，再用 set_state()/get_command() 读写。
///   句柄在 on_configure()/on_activate() 中一次性解析并缓存，
///   控制循环里不再做 map 查找，也不走 wait_until_* 的阻塞路径。
///
/// TODO(hardware): 当前 read/write 是桩实现（命令回声为状态），
///   让整套链路在没有真实硬件时也能跑通。接真机时替换这两个函数即可。
class RobotBaseSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 左右两个驱动轮
  static constexpr std::size_t kNumWheels = 2;

  /// 解析并缓存接口句柄。状态句柄在 on_configure() 里解析，
  /// 命令句柄在 on_activate() 里解析（此时框架保证命令接口已可用）。
  /// \returns 全部解析成功返回 true。
  bool resolve_state_handles();
  bool resolve_command_handles();

  /// 拼出框架使用的接口键名，形如 "left_wheel_joint/velocity"。
  static std::string interface_key(
    const std::string & joint_name, const std::string & interface_name);

  // 关节名，索引与两个轮子一一对应（0=左, 1=右）
  std::vector<std::string> wheel_joint_names_;

  // 硬件自有的状态缓冲，索引 0=左轮 1=右轮。
  //
  // 这里刻意【不】回读框架的 state 存储：框架刚创建接口时值是 NaN，
  // 首次 get_state() 会把它读进来，之后所有积分结果都变成 NaN。
  // 真机上这两组值本来就该由编码器填充，所以由驱动自己持有才是对的。
  std::vector<double> hw_positions_{0.0, 0.0};
  std::vector<double> hw_velocities_{0.0, 0.0};

  // 缓存的接口句柄。地址在硬件组件生命周期内稳定，控制循环里可直接解引用。
  std::vector<hardware_interface::StateInterface::SharedPtr> position_states_;
  std::vector<hardware_interface::StateInterface::SharedPtr> velocity_states_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> velocity_commands_;

  // 来自 URDF <hardware><param> 的串口配置
  std::string serial_port_{"/dev/ttyUSB0"};
  int baud_rate_{115200};

  bool motor_enabled_{false};

  // TODO(hardware): 串口句柄/协议状态在接真机时加在这里
  // int serial_fd_{-1};
};

}  // namespace robot_base_driver

#endif  // ROBOT_BASE_DRIVER__ROBOT_BASE_SYSTEM_HPP_
