"""Exercise the production readback state machine with a simulated CAN peer."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = next((base / name
             for base in Path(__file__).resolve().parents
             for name in ('RobStride05_H743', 'RobStride05_H743_HAL')
             if (base / name / 'App' / 'app_motor.c').exists()), None)
# Exercise the RTOS dual motor production implementation.
assert ROOT is not None, 'firmware project not found above the console folder'


class FirmwareReadbackTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('gcc'), 'gcc required for firmware simulation')
    def test_real_transaction_engine_rejections_aliases_mismatch_and_timeout(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)
            (p/'main.h').write_text('''#include <stdint.h>
typedef struct {uint32_t CCR;} MockSCB;
extern MockSCB mock_scb;
#define SCB (&mock_scb)
#define SCB_CCR_DC_Msk (1U<<16)
#define __DMB() ((void)0)
uint32_t HAL_GetTick(void);
''')
            (p/'check.c').write_text('''#include <assert.h>
#include <stdio.h>
#include "app_motor.c"
volatile AppDebug g_debug;
MockSCB mock_scb;
/* This test isolates parameter readback; transport integration is exercised
 * by Tools/test_dual_motor.c with the real Bluetooth parser. */
void AppBluetooth_Init(void){}
void AppBluetooth_Service(uint32_t now){(void)now;}
void AppBluetooth_Cancel(void){}
uint32_t AppBluetooth_OwnsMotors(void){return 0;}
void BspJy901s_Init(void){}
bool BspJy901s_Read(uint8_t *b){(void)b;return false;}
static uint32_t tick;
uint32_t HAL_GetTick(void){return tick;}
uint32_t BspTime_Us(void){return tick*1000;}
bool BspTime_DebugFrozen(void){return false;}
bool BspCan_Start(void){return true;}
bool BspCan_SendBegin(const CanFrame *f){(void)f;return true;}
uint32_t BspCan_SendPoll(void){return CAN_TX_OK;}
void BspCan_Cancel(void){}
void BspCan_DiscardRx(void){}
bool BspCan_Receive(CanFrame *f){(void)f;return false;}
bool BspCan_IsBusOff(void){return false;}
bool BspCan_Recover(void){return true;}
void BspCan_GetStats(CanStats *out){memset(out,0,sizeof(*out));}
static void reply(uint16_t index,uint32_t expected,uint32_t actual,bool ok){
 memset(&ctx->s,0,sizeof(ctx->s));memset(&ctx->fb,0,sizeof(ctx->fb));ctx->op_count=ctx->op_at=0;
 write_verify(index,expected);ctx->op_at=1;ctx->op_phase=2;ctx->job=CMD_PREPARE;ctx->job_seq=42;
 ctx->op_counter=0;ctx->op_start=tick=0;
 ctx->fb.parameter_count=1;ctx->fb.param_index=index;ctx->fb.param_raw=actual;ctx->fb.param_ok=ok;
}
int main(void){
 AppMotor_Init();ctx=&motors[0];g_debug.setup.dual_ready=1;
 reply(RS_ACCEL,0x3e4ccccd,0,false);service_job();
 assert(ctx->job==CMD_PREPARE && ctx->op_at==0 && ctx->op_phase==0);
 assert(ctx->ops[0].frame.data[0]==0x1e && ctx->ops[0].frame.data[1]==0x20);
 assert(ctx->ops[1].frame.data[0]==0x1e);
 reply(RS_ACCEL_LEGACY,0x3e4ccccd,0,false);service_job();
 assert(ctx->job && ctx->op_at==2 && !ctx->s.accel_motor_supported);
 reply(RS_TIMEOUT,5000,0,false);service_job();
 assert(ctx->job && ctx->op_at==0 && ctx->ops[1].frame.data[0]==0x0c);
 reply(RS_TIMEOUT_LEGACY,5000,0,false);service_job();
 assert(ctx->job && ctx->op_at==2 && !ctx->s.timeout_motor_supported);
 reply(RS_TIMEOUT,5000,5000,true);service_job();
 assert(ctx->job && ctx->op_at==2 && ctx->s.timeout_motor_supported);
 reply(RS_CURRENT_LIMIT,0x3e99999a,0,false);service_job();
 assert(!ctx->job && ctx->s.state==MOTOR_FAULT && ctx->s.result==RESULT_READBACK);
 assert(ctx->s.readback_status==2 && ctx->s.readback_index==RS_CURRENT_LIMIT);
 reply(RS_ACCEL,0x3e4ccccd,0x3e99999a,true);service_job();
 assert(!ctx->job && ctx->s.readback_status==3 && ctx->s.readback_actual==0x3e99999a);
 reply(RS_TIMEOUT,5000,0,true);ctx->fb.parameter_count=0;tick=100;service_job();
 assert(!ctx->job && ctx->s.readback_status==4 && ctx->s.readback_index==RS_TIMEOUT);
 reply(RS_TIMEOUT,5000,5000,true);ctx->fb.param_index=RS_CURRENT;tick=100;service_job();
 assert(!ctx->job && ctx->s.readback_status==4);
 /* A RUN latches its speed; it never bypasses explicit enable. */
 memset(&ctx->s,0,sizeof(ctx->s));memset(&ctx->fb,0,sizeof(ctx->fb));
 MotorCommand c={.command=CMD_RUN,.confirm_key=APP_ENABLE_KEY,
 .target_rad_s=0.05f,.speed_limit_rad_s=0.2f,.current_limit_a=0.3f,.accel_rad_s2=0.2f};
 ctx->active=c;ctx->prepared=1;ctx->armed=0;ctx->s.mode=2;ctx->wall_fault=ctx->health_fault=0;ctx->job=0;
 /* Public RS05 speed-mode range, including both exact endpoints. */
 MotorCommand full=c;full.speed_limit_rad_s=33;full.current_limit_a=11;full.accel_rad_s2=150;
 full.target_rad_s=33;assert(limits_ok(&full));
 full.target_rad_s=-33;assert(limits_ok(&full));
 full.target_rad_s=33.01f;assert(!limits_ok(&full));
 full.target_rad_s=0;full.current_limit_a=11.01f;assert(!limits_ok(&full));
 full.current_limit_a=11;full.accel_rad_s2=150.01f;assert(!limits_ok(&full));
 tick=100;ctx->fb.feedback_count=1;ctx->fb.feedback_ms=tick;ctx->fb.motor_state=2;
 assert(execute(&c)==RESULT_STATE);
 ctx->armed=1;ctx->s.state=MOTOR_ENABLED;
 tick=6100;ctx->fb.feedback_ms=tick;ctx->previous_drop=0;
 check_safety();assert(ctx->armed && ctx->s.state==MOTOR_ENABLED);
 assert(execute(&c)==RESULT_OK && ctx->s.state==MOTOR_RUNNING);
 for(tick=7100;tick<=67100;tick+=1000){
  ctx->fb.feedback_ms=tick;check_safety();
  assert(ctx->armed && ctx->s.state==MOTOR_RUNNING && ctx->s.target_rad_s==0.05f);
 }
 c.command=CMD_STOP;assert(execute(&c)==RESULT_OK && ctx->s.target_rad_s==0);
 c.command=CMD_RUN;c.target_rad_s=-0.05f;
 ctx->fb.feedback_ms=tick;assert(execute(&c)==RESULT_OK && ctx->s.target_rad_s==-0.05f);
 tick+=150;check_safety();
 assert(!ctx->armed && ctx->s.state==MOTOR_FAULT && ctx->s.result==RESULT_OFFLINE && ctx->s.target_rad_s==0);
 /* Observation throttling must retain a consistent timestamp and cannot
  * slow the production safety engine above. */
 ctx->publish_valid=0;tick=7000;ctx->s.target_rad_s=0;publish();
 uint32_t seq=g_debug.motor.snapshot_seq;
 tick=7019;ctx->s.target_rad_s=0.1f;publish();
 assert(g_debug.motor.snapshot_seq==seq && g_debug.motor.target_rad_s==0);
 tick=7020;publish();
 assert(g_debug.motor.snapshot_seq==seq+2 && g_debug.motor.published_tick_ms==7020);
 assert(g_debug.motor.target_rad_s==0.1f);
 puts("Production readback engine: aliases, rejection, mismatch, timeout passed");
 return 0;
}
''')
            binary = p/'check.exe'
            subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(p/'check.c'),
                            str(ROOT/'Devices/robstride05.c'),str(ROOT/'Devices/jy901s.c'),
                            str(ROOT/'App/app_chassis.c'),str(ROOT/'App/app_pid.c'),'-I'+str(p),
                            '-I'+str(ROOT/'App'),'-I'+str(ROOT/'Devices'),'-I'+str(ROOT/'BSP'),
                            '-lm','-o',str(binary)], check=True)
            result = subprocess.run([str(binary)],check=True,capture_output=True,text=True)
            self.assertIn('passed',result.stdout)
