#include "robot_base_driver/robot_base_system.hpp"

#include <memory>
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

std::string RobotBaseSystem::interface_key(
  const std::string & joint_name, const std::string & interface_name)
{
  // 框架内部用 "<关节名>/<接口名>" 作为接口键，例如 "left_wheel_joint/velocity"
  return joint_name + "/" + interface_name;
}

hardware_interface::CallbackReturn RobotBaseSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // 让基类做框架侧的初始化与结构校验
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const hardware_interface::HardwareInfo & info = params.hardware_info;

  // 读取 <hardware><param name="..."> 里的串口配置
  const auto it_port = info.hardware_parameters.find("serial_port");
  if (it_port != info.hardware_parameters.end()) {
    serial_port_ = it_port->second;
  }

  const auto it_baud = info.hardware_parameters.find("baud_rate");
  if (it_baud != info.hardware_parameters.end()) {
    try {
      baud_rate_ = std::stoi(it_baud->second);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger(), "baud_rate 参数无法解析为整数: '%s' (%s)",
        it_baud->second.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (info.joints.size() != kNumWheels) {
    RCLCPP_ERROR(
      logger(), "URDF 中声明的关节数为 %zu，但本硬件接口要求恰好 %zu 个（左右驱动轮）",
      info.joints.size(), kNumWheels);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 记录关节名。同时校验每个关节都提供了必需的接口。
  wheel_joint_names_.clear();
  wheel_joint_names_.reserve(kNumWheels);

  for (const auto & joint : info.joints) {
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

  // 状态缓冲清零。真机时这里应该是"读一次编码器取初值"。
  hw_positions_.assign(kNumWheels, 0.0);
  hw_velocities_.assign(kNumWheels, 0.0);

  RCLCPP_INFO(
    logger(), "初始化完成，串口配置: %s @ %d",
    serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RobotBaseSystem::resolve_state_handles()
{
  position_states_.clear();
  velocity_states_.clear();
  position_states_.reserve(kNumWheels);
  velocity_states_.reserve(kNumWheels);

  for (const auto & joint : wheel_joint_names_) {
    const std::string position_key = interface_key(joint, hardware_interface::HW_IF_POSITION);
    const std::string velocity_key = interface_key(joint, hardware_interface::HW_IF_VELOCITY);

    if (!has_state(position_key) || !has_state(velocity_key)) {
      RCLCPP_ERROR(
        logger(), "状态接口缺失: '%s' 或 '%s'",
        position_key.c_str(), velocity_key.c_str());
      return false;
    }

    position_states_.push_back(get_state_interface_handle(position_key));
    velocity_states_.push_back(get_state_interface_handle(velocity_key));
  }

  return true;
}

bool RobotBaseSystem::resolve_command_handles()
{
  velocity_commands_.clear();
  velocity_commands_.reserve(kNumWheels);

  for (const auto & joint : wheel_joint_names_) {
    const std::string velocity_key = interface_key(joint, hardware_interface::HW_IF_VELOCITY);

    if (!has_command(velocity_key)) {
      RCLCPP_ERROR(logger(), "命令接口缺失: '%s'", velocity_key.c_str());
      return false;
    }

    velocity_commands_.push_back(get_command_interface_handle(velocity_key));
  }

  return true;
}

hardware_interface::CallbackReturn RobotBaseSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 进入 INACTIVE：状态接口此时已由框架建好，可以解析并缓存
  if (!resolve_state_handles()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  // 进入 ACTIVE：命令接口此刻才保证可用，所以在这里解析命令句柄
  if (!resolve_command_handles()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 命令清零，避免激活瞬间电机收到上电前的残留值而突然窜动
  for (const auto & handle : velocity_commands_) {
    if (handle) {
      set_command(handle, 0.0, false);
    }
  }
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

  for (const auto & handle : velocity_commands_) {
    if (handle) {
      set_command(handle, 0.0, false);
    }
  }

  // TODO(hardware): 在此失能电机驱动器
  RCLCPP_INFO(logger(), "电机已失能（桩实现，未下发真实指令）");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotBaseSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(hardware): 接真机时在此读串口，把编码器数据写入
  //   set_state(velocity_states_[i], ...) / set_state(position_states_[i], ...)
  //
  // 桩实现：使能后把速度命令回声为实际速度，并按周期积分出位置。
  // 这样 diff_drive_controller 在没有硬件时也能算出连贯的 odom，
  // 便于验证整条链路。
  //
  // 注意 1：所有句柄在 on_configure()/on_activate() 已解析并缓存，
  //   这里只做指针解引用，不做 map 查找，也不走 wait_until_* 阻塞路径。
  // 注意 2：积分用的是本类自有的 hw_positions_/hw_velocities_，
  //   不是回读框架的 state 存储 —— 后者初值是 NaN，回读会被污染成 NaN。
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    double velocity = 0.0;
    if (motor_enabled_ && i < velocity_commands_.size() && velocity_commands_[i]) {
      get_command<double>(velocity_commands_[i], velocity, false);
    }

    hw_velocities_[i] = velocity;
    hw_positions_[i] += velocity * period.seconds();

    set_state(velocity_states_[i], hw_velocities_[i], false);
    set_state(position_states_[i], hw_positions_[i], false);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotBaseSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(hardware): 接真机时在此把速度命令（rad/s）经串口下发给电机驱动器，
  //   命令值从 velocity_commands_ 句柄读：
  //     double cmd = 0.0;
  //     get_command<double>(*(velocity_commands_[i]), cmd, false);
  return hardware_interface::return_type::OK;
}

}  // namespace robot_base_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_base_driver::RobotBaseSystem, hardware_interface::SystemInterface)
