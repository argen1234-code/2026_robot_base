#ifndef APP_DEBUG_H
#define APP_DEBUG_H
#include "bsp_can.h"
#include "app_chassis.h"

/* Watch/Live Watch: add g_debug and expand its four members.
 * WRITE cmd only. motor/oled/system are firmware-published observations.
 * CPU RUNNING: cmd.seq odd -> edit fields -> new nonzero even seq LAST.
 * Two identical command snapshots on distinct service cycles are required.
 * volatile alone does not provide consistency; follow the seq protocol. */
typedef struct {
 uint32_t seq;                /* Odd: editing; new nonzero even: submit last. */
 uint32_t command;            /* 1 prepare, 2 enable, 3 run, 4 stop, 5 disable, 6 clear. */
 uint32_t confirm_key;        /* 0x52533035 required for prepare/enable/run. */
 float target_rad_s;          /* Signed rad/s; abs <= speed_limit_rad_s. */
 float speed_limit_rad_s;     /* (0,33] rad/s; default 33; prepare only. */
 float current_limit_a;       /* (0,11] A phase current; default rated 2.4. */
 float accel_rad_s2;          /* (0,150] rad/s^2; default 20 per manual. */
} MotorCommand;

typedef struct {
 uint32_t snapshot_seq;       /* Publication seq: odd while updating, even when stable. */
 uint32_t accepted_seq;       /* Most recently accepted asynchronous command seq. */
 uint32_t executed_seq;       /* Most recently finished/rejected command seq. */
 uint32_t result;             /* RESULT_*: 0 success, 16 pending. */
 uint32_t reject_reason;      /* RESULT_* rejection/fault reason. */
 uint32_t state;              /* MOTOR_* application state, see app_motor.h. */
 uint32_t mode;               /* Configured motor control mode: 2 = speed. */
 uint32_t online;             /* 1 = recent valid feedback, otherwise 0. */
 uint32_t enabled;            /* Fresh feedback reports Motor state; not physical stop proof. */
 uint32_t enable_requested;   /* MCU's current enable authorization. */
 uint32_t motor_state;        /* Protocol: 0 Reset, 1 Cali, 2 Motor. */
 uint32_t feedback_age_ms;    /* ms; UINT32_MAX = no feedback. */
 uint32_t fault_bits;         /* Protocol fault summary bits. */
 uint32_t fault;              /* Full fault bitmap from type 21. */
 uint32_t warning;            /* Warning bitmap from type 21. */
 float target_rad_s;          /* Accepted target, rad/s. */
 float applied_rad_s;         /* Rate-limited transmitted target, rad/s. */
 float position_rad;          /* Position feedback, rad; check online/age. */
 float velocity_rad_s;        /* Velocity feedback, rad/s; check online/age. */
 float torque_nm;             /* Torque feedback, N.m; not phase current. */
 float temperature_c;         /* Motor temperature, degrees Celsius. */
 float current_a;             /* Filtered Iq read from 0x701A, A. */
 float bus_voltage_v;         /* Bus voltage read from 0x701C, V. */
 float current_limit_a;       /* Active phase-current limit, A. */
 float speed_limit_rad_s;     /* Active speed limit, rad/s. */
 float accel_rad_s2;          /* Active acceleration limit, rad/s^2. */
 uint32_t current_age_ms;     /* ms; UINT32_MAX = no Iq reading. */
 uint32_t voltage_age_ms;     /* ms; UINT32_MAX = no voltage reading. */
 uint32_t timeout_readback;   /* 0x7028 units: 20000 = 1 s; expected 5000. */
 uint32_t version;            /* Reported motor firmware version. */
 uint32_t version_valid;      /* Version reply received. */
 uint32_t motor_heartbeat;    /* Main-loop motor service count. */
 uint32_t health_fault;       /* Bit0 DCache enabled; bit1 TIM2 debug-frozen. */
 uint32_t cpu_dcache_enabled; /* Must remain 0 in this project. */
 uint32_t job_command;        /* Current asynchronous command, 0 = idle. */
 uint32_t job_step;           /* Current bounded transaction step. */
 uint32_t published_tick_ms;  /* HAL tick covered by this stable telemetry snapshot. */
 CanStats can;               /* RX/TX/error/recovery counters; expand as needed. */
 uint32_t configured_id;     /* Actual command destination, default 0x7F as in official sample. */
 uint32_t identity_valid;    /* Matching type-0 identity reply received. */
 uint32_t identity_id;       /* ID returned by motor, not inferred from an ACK. */
 uint8_t identity_uid[8];    /* Motor MCU unique ID returned by type 0. */
 uint32_t feedback_count,parameter_count,unmatched_frames;
 uint32_t readback_index,readback_expected,readback_actual,readback_status;
 /* status: 0 none, 1 matched, 2 motor rejected, 3 mismatch, 4 timeout. */
 uint32_t accel_motor_supported; /* 0: motor rejects 7022; MCU still ramps speed. */
 uint32_t accel_parameter,timeout_parameter; /* Actual supported parameter aliases. */
 uint32_t timeout_motor_supported; /* 0: no verified motor-side CAN timeout. */
} MotorStatus;

typedef struct {
 uint32_t heartbeat;          /* Display service count, including backoff periods. */
 uint32_t ready;              /* Last full frame transferred successfully. */
 uint32_t refresh_ok;         /* Successful full-frame transfers. */
 uint32_t refresh_fail;       /* Failed initialization/refresh/recovery cycles. */
 uint32_t consecutive_fail;   /* Consecutive failed cycles. */
 uint32_t recovery_attempts;  /* Bounded bus recovery starts. */
 uint32_t recovery_ok;        /* Successful bus recovery completions. */
 uint32_t last_error;         /* HAL I2C error plus BSP timeout/busy flags. */
 uint32_t last_success_ms;    /* HAL tick of last successful full frame. */
 uint32_t next_retry_ms;      /* HAL tick when next retry may start. */
} OledStatus;

typedef struct {
 uint32_t loop_heartbeat;     /* Cooperative scheduler service count. */
 uint32_t health_heartbeat;   /* Health check count, about 10 Hz. */
 uint32_t max_service_us;     /* Maximum observed service execution time, us. */
 uint32_t last_service_us;    /* Most recent service execution time, us. */
 uint32_t fatal;              /* 0 normal; nonzero fatal startup/runtime reason. */
 uint32_t boot_stage;         /* 1 entry,2 HAL,3 clock,4 GPIO,5 CAN,6 I2C,7 timer,8 motor,9 ready. */
 uint32_t tick_ms;            /* HAL tick at last service/fatal capture. */
 uint32_t vtor;               /* Must be 0x08000000 for this Flash image. */
 uint32_t ccr;                /* IC(bit17) and DC(bit16) must both be 0. */
 uint32_t cfsr;               /* Fault status captured on fatal error. */
 uint32_t hfsr;               /* HardFault status captured on fatal error. */
 uint32_t exception;          /* IPSR at fatal capture: 3 = HardFault. */
} AppStatus;

/* setup.command: 1 = authorize the discovered pair, 2 = isolated ID change.
 * seq/key/command/old_id/new_id/isolated_key are HOST writes; all other fields
 * are firmware observations. Never auto-authorize after boot or ID change. */
typedef struct {
 uint32_t seq,key,command,old_id,new_id,isolated_key;
 uint32_t executed_seq,result,busy,dual_ready;
} MotorSetup;

/* SWD pair commit. 1 acquire and zero targets, 2 paired targets, 3 release.
 * Uses the prepared per-motor limits/ramps. No additional speed limit.
 * Host renews command 2; 600 ms without a valid renewal stops both targets.
 * Status is firmware-owned. Existing AppDebug member offsets stay intact. */
typedef struct {
 uint32_t seq,key,command;
 float motor1_rad_s,motor2_rad_s;
} DriveCommand;
typedef struct {uint32_t executed_seq,result,active;} DriveStatus;

typedef struct {
 MotorCommand cmd;           /* WRITE: edit parameters, then commit seq. */
 MotorStatus motor;          /* OBSERVE ONLY: motor and CAN status. */
 OledStatus oled;            /* OBSERVE ONLY: display and recovery status. */
 AppStatus system;           /* OBSERVE ONLY: scheduler timing and health. */
 MotorCommand cmd2;
 MotorStatus motor2;
 MotorSetup setup;
 uint32_t stop_all_seq; /* HOST: new nonzero even value requests BOTH disabled. */
 DriveCommand drive_cmd;
 DriveStatus drive;
 ChassisCommand chassis_cmd;
 ChassisStatus chassis;
} AppDebug;

extern volatile AppDebug g_debug; /* Single Watch root; stored in internal RAM. */
#endif
