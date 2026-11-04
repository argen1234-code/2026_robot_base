#include "app_debug.h"
#include "app_config.h"

volatile AppDebug g_debug={
 .cmd2={
  .speed_limit_rad_s=APP_DEFAULT_SPEED_CAP,
  .current_limit_a=APP_DEFAULT_CURRENT_A,
  .accel_rad_s2=APP_DEFAULT_ACCEL
 },
 .cmd={
  .speed_limit_rad_s=APP_DEFAULT_SPEED_CAP,
  .current_limit_a=APP_DEFAULT_CURRENT_A,
  .accel_rad_s2=APP_DEFAULT_ACCEL
 }
};
