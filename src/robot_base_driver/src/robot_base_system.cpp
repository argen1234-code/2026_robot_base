#include "robot_base_driver/robot_base_system.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_base_driver
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("RobotBaseSystem");
}
}  // namespace

hardware_interface::CallbackReturn RobotBaseSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 先让基类校验 URDF 里 <ros2_control> 块的基本结构
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 读取 <hardware><param name="..."> 里的串口配置
  const auto it_port = info_.hardware_parameters.find("serial_port");
  if (it_port != info_.hardware_parameters.end()) {
    serial_port_ = it_port->second;
  }

  const auto it_baud = info_.hardware_parameters.find("baud_rate");
  if (it_baud != info_.hardware_parameters.end()) {
    try {
      baud_rate_ = std::stoi(it_baud->second);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger(), "baud_rate 参数无法解析为整数: '%s' (%s)",
        it_baud->second.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (info_.joints.size() != kNumWheels) {
    RCLCPP_ERROR(
      logger(), "URDF 中声明的关节数为 %zu，但本硬件接口要求恰好 %zu 个（左右驱动轮）",
      info_.joints.size(), kNumWheels);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 记录关节名。同时校验每个关节都提供了必需的接口。
  wheel_joint_names_.clear();
  wheel_joint_names_.reserve(kNumWheels);

  for (const auto & joint : info_.joints) {
    bool has_velocity_command = false;
    bool has_position_state = false;
    bool has_velocity_state = false;

    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_command = true;
      }
    }
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        has_position_state = true;
      }
      if (si.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state = true;
      }
    }

    if (!has_velocity_command || !has_position_state || !has_velocity_state) {
      RCLCPP_ERROR(
        logger(),
        "关节 '%s' 的接口不完整：需要 velocity 命令接口，以及 position+velocity 状态接口",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    wheel_joint_names_.push_back(joint.name);
    RCLCPP_INFO(logger(), "已注册驱动轮关节: %s", joint.name.c_str());
  }

  // 状态与命令清零，位置地址必须稳定（export_*_interfaces 会把地址交出去）
  hw_positions_.assign(kNumWheels, 0.0);
  hw_velocities_.assign(kNumWheels, 0.0);
  hw_commands_.assign(kNumWheels, 0.0);

  RCLCPP_INFO(
    logger(), "初始化完成，串口配置: %s @ %d",
    serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotBaseSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(hardware): 在此打开串口并完成握手/校验，例如
  //   serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  //   若失败则 return CallbackReturn::ERROR;
  RCLCPP_INFO(
    logger(), "[桩实现] 跳过串口打开 (%s @ %d)",
    serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotBaseSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 命令清零，避免激活瞬间电机收到上电前的残留值而突然窜动
  hw_commands_.assign(kNumWheels, 0.0);
  hw_velocities_.assign(kNumWheels, 0.0);
  motor_enabled_ = true;

  // TODO(hardware): 在此使能电机驱动器
  RCLCPP_INFO(logger(), "电机已使能（桩实现，未下发真实指令）");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotBaseSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motor_enabled_ = false;
  hw_commands_.assign(kNumWheels, 0.0);

  // TODO(hardware): 在此失能电机驱动器
  RCLCPP_INFO(logger(), "电机已失能（桩实现，未下发真实指令）");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotBaseSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kNumWheels * 2);

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    // StateInterface 的构造函数签名来自基类 ReadOnlyHandle：
    //   (prefix_name, interface_name, double * value_ptr)
    state_interfaces.emplace_back(
      wheel_joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      wheel_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotBaseSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kNumWheels);

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    // 注意：CommandInterface 的拷贝构造被显式 delete，只能就地构造，
    // 因此这里必须用 emplace_back 而不能用 push_back。
    command_interfaces.emplace_back(
      wheel_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type RobotBaseSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(hardware): 接真机时在此读串口，把编码器数据写入
  //   hw_positions_[i] / hw_velocities_[i]
  //
  // 桩实现：把速度命令回声为实际速度，并按周期积分出位置。
  // 这样 diff_drive_controller 在没有硬件时也能算出连贯的 odom，
  // 便于验证整条链路。
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    hw_velocities_[i] = motor_enabled_ ? hw_commands_[i] : 0.0;
    hw_positions_[i] += hw_velocities_[i] * period.seconds();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotBaseSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(hardware): 接真机时在此把 hw_commands_（rad/s）经串口下发给电机驱动器
  return hardware_interface::return_type::OK;
}

}  // namespace robot_base_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_base_driver::RobotBaseSystem, hardware_interface::SystemInterface)
