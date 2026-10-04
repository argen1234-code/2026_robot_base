# 左右驱动轮速度（单位 rad/s），用于调试或直接控制硬件。
# 注意：正常运行时底盘速度由 diff_drive_controller 下发，
#       本消息主要用于绕过控制器直接驱动电机的调试场景。
float64 left
float64 right
