#include "app_motor.h"
#include "app_config.h"
#include "app_bluetooth.h"
#include "app_usb.h"
#include "robstride05.h"
#include "bsp_time.h"
#include "main.h"
#include <string.h>
#include <math.h>

enum {OP_SEND=0,OP_READ=1,OP_STATE=2};
typedef struct {CanFrame frame;uint32_t kind,expected;} Operation;
/* Both contexts are serviced by the ONE motor task. ctx is never used by
 * other tasks/ISRs; only immutable seqlock publications cross task boundaries.
 * The shared CAN TX slot has an owner, so a job cannot cancel/poll its peer. */
typedef struct {
 uint8_t id;
 volatile MotorCommand *mailbox;
 volatile MotorStatus *published;
MotorStatus s;
Rs05Feedback fb;
MotorCommand candidate,active,pending;
uint32_t candidate_valid,last_seen,prepared,armed,health_fault;
uint32_t last_tx,last_poll,last_stop,last_recovery,poll_index;
uint32_t last_probe,probe_step;
uint32_t iq_tick,vbus_tick,iq_valid,vbus_valid,previous_drop;
uint32_t wall_last,wall_valid,wall_fault,urgent_stop,service_tx,service_type;
uint32_t publish_tick,publish_valid;
Operation ops[20];
uint32_t op_count,op_at,op_phase,op_start,op_counter,job,job_seq;

} MotorContext;
static MotorContext motors[APP_MOTOR_COUNT],*ctx,*tx_owner;
static uint32_t recovery_tick,stop_seen,setup_seen,setup_candidate_seq;
static MotorSetup setup_candidate;
static uint32_t setup_phase,setup_tick;
static Rs05Feedback setup_fb;
static DriveCommand drive_candidate;
static uint32_t drive_seen,drive_candidate_valid,drive_tick;

static void drive_cancel(void)
{
 uint32_t i;
 AppBluetooth_Cancel();
 AppChassis_Cancel();
 if(g_debug.drive.active) {
  for(i=0;i<APP_MOTOR_COUNT;i++) {
   motors[i].s.target_rad_s=0;
   if(motors[i].armed && !motors[i].job)motors[i].s.state=MOTOR_STOPPING;
  }
 }
 g_debug.drive.active=0;drive_candidate_valid=0;
 drive_seen=g_debug.drive_cmd.seq;
}

static bool tx_begin(const CanFrame *f)
{
 if(tx_owner && tx_owner!=ctx)return false;
 if(!BspCan_SendBegin(f))return false;
 tx_owner=ctx;return true;
}
static uint32_t tx_poll(void)
{
 uint32_t r;
 if(tx_owner!=ctx)return CAN_TX_IDLE;
 r=BspCan_SendPoll();if(r!=CAN_TX_BUSY)tx_owner=NULL;
 return r;
}
static void tx_cancel(void)
{if(tx_owner==ctx){BspCan_Cancel();tx_owner=NULL;}}

/* ============================================================================
 * app_motor.c —— RS05 电机速度环：命令状态机 + CAN 事务 + 安全保护
 *
 * RTOS 双电机版：一个电机任务每 2 ms 服务两个独立上下文，CAN 统一收帧
 * 并按来源 ID 分流；发送通道按上下文归属仲裁。各步骤有界、不阻塞。
 *
 * 一、状态机（ctx->s.state）
 *      DISABLED ──准备(PREPARE)──> CONFIGURING ──全部读回核对通过──> PREPARED
 *      PREPARED ──使能(ENABLE)───> ENABLING ──看到电机进入 Motor 态──> ENABLED
 *      ENABLED  ──运行(RUN)─────> RUNNING（目标速度在 MCU 内保持，直到停止/失能/故障）
 *      RUNNING  ──停止(STOP)────> STOPPING ──斜坡降到 0 且实测接近 0──> ENABLED
 *      任意状态 ──失能(DISABLE)/任一保护触发──> DISABLED / FAULT
 *    "准备/使能"不是瞬间完成的：它们要跟电机做一串"写-读回-核对"的 CAN 事务，
 *    这期间状态是 CONFIGURING/ENABLING，命令还在被异步执行。
 *
 * 二、事务引擎（ctx->ops[] + ctx->job/ctx->op_at/ctx->op_phase）
 *    一次准备/使能最多 20 个有界步骤，每步只有三种：
 *      OP_SEND  发一帧（要等它真正上总线，看 TXBTO，而不是"进了 FIFO 就算成功"）
 *      OP_READ  发读请求并等回应，核对数值（不匹配就整笔失败，绝不"差不多就过"）
 *      OP_STATE 发类型 3/4，等反馈里电机状态变成期望值
 *    ctx->op_phase：0=准备发送，1=等发送完成，2=等回应/等状态。任何一步超时
 *    （APP_PARAM_TIMEOUT_MS=100 ms）或数值不符，立刻 trip()，不会半途而废继续。
 *    整套东西只用静态数组，不 malloc、不排队、不加锁，天然可重入分析。
 *
 * 三、安全保护（check_safety，每个服务周期都跑）
 *    · 反馈超过 150 ms 没更新 → 离线，trip
 *    · 电机报故障位/详细故障、或处于 Cali 态 → trip
 *    · 实测速度超过"准备时设定的上限 + 0.15 rad/ctx->s" → 超速，trip
 *    · 使能态下电机状态不是 Motor(2) → trip
 *    · CAN Bus-off → trip，并且每秒最多尝试恢复一次（恢复后保持失能）
 *    · RX 环形队列丢帧（rx_drop 变化）→ trip
 *    · 两次服务间隔 > 100 ms（调试暂停后恢复）→ trip SERVICE_GAP
 *    trip 的动作：撤销目标、清准备标志、状态置 FAULT，并（尽力）发一帧停止。
 *    故障是锁存的：只有重新走"准备→使能"才会解除，绝不自动恢复运动。
 *
 * 四、与外部的关系
 *    · 上行：(*ctx->published) 每 20 ms 发布一次稳定快照（seqlock：snapshot_seq
 *      奇数表示正在写、偶数稳定），上位机的 SWD 读取靠它判断一致性。
 *    · 下行：上位机写 (*ctx->mailbox)（奇数序号=编辑中，新偶数序号=提交），
 *      本文件要求"连续两个服务周期读到完全相同的一份命令"才执行，
 *      并把结果写回 executed_seq/result。
 *    · 本文件只写 (*ctx->published) 与 cmd 相关字段，不碰 system/oled 段。
 * ==========================================================================*/

static bool finite_value(float f){return isfinite(f)!=0;}
static bool online(void){return ctx->fb.feedback_count && HAL_GetTick()-ctx->fb.feedback_ms<APP_FEEDBACK_TIMEOUT_MS;}
static bool motor_ok(void){return !ctx->fb.fault_bits && !ctx->fb.fault && ctx->fb.motor_state!=1;}
static bool limits_ok(const MotorCommand *c)
{
 return finite_value(c->target_rad_s) && finite_value(c->speed_limit_rad_s) &&
 finite_value(c->current_limit_a) && finite_value(c->accel_rad_s2) &&
 c->speed_limit_rad_s>0 && c->speed_limit_rad_s<=APP_SPEED_MAX_RAD_S &&
 c->current_limit_a>0 && c->current_limit_a<=APP_CURRENT_MAX_A &&
 c->accel_rad_s2>0 && c->accel_rad_s2<=APP_ACCEL_MAX_RAD_S2 &&
 fabsf(c->target_rad_s)<=c->speed_limit_rad_s;
}
/* 撤销"已准备"的全部痕迹：清准备/武装标志、清超时读回、目标速度归零。
 * 它不发任何 CAN 帧、也不改 ctx->s.state —— 只负责让"运动授权"失效。 */
static void invalidate(void)
{
 ctx->prepared=ctx->armed=0;ctx->s.timeout_readback=0;
 ctx->s.target_rad_s=ctx->s.applied_rad_s=0;
}
/* 放弃当前事务（准备/使能做到一半）：清空 ctx->ops 队列、取消未发完的 CAN 发送、
 * 复位相位。它不改变电机状态，由调用者决定接下来置什么态。 */
static void cancel_job(void)
{ctx->job=0;ctx->op_count=ctx->op_at=ctx->op_phase=0;tx_cancel();ctx->service_tx=0;}
/* ----------------------------------------------------------------------------
 * trip —— 触发保护（整个文件唯一的"紧急处置"入口）
 *
 * reason 会被写进 result/reject_reason，供上位机显示具体原因（离线/电机故障/
 * 超速/读回不符/Bus-off/服务间断…）。流程：
 *   1. 若正有事务在执行，先把它标记成"已执行到 ctx->job_seq"（上位机据此结束等待）；
 *   2. cancel_job 放弃剩余步骤，invalidate 撤销运动授权；
 *   3. 状态置 MOTOR_FAULT，并置 ctx->urgent_stop —— 让 service_normal 下个周期尽快
 *      补发一帧类型 4（全零停止）。
 * 为什么用标志而不是直接发：trip 可能发生在"已有帧占用 TX 通道"时，不能阻塞等，
 * 只能置标志、下一拍再发。
 * --------------------------------------------------------------------------*/
static void trip(uint32_t reason)
{
 if(ctx->job){ctx->s.executed_seq=ctx->job_seq;}
 cancel_job();invalidate();ctx->s.state=MOTOR_FAULT;ctx->s.result=ctx->s.reject_reason=reason;ctx->urgent_stop=1;
}
/* 把 RX 环形队列里的帧全部取出来解析（一次最多 32 帧）。
 * 解析在任务里做、不在中断里做：ISR 只负责把 8 字节搬进队列，重活留给这里，
 * 这样 ISR 永远很短，不会拖慢其它中断。
 * 顺带把 0x701A(电流) / 0x701C(母线电压) 的回复转成浮点存进快照。 */
static void drain(void)
{
 CanFrame f;uint32_t n,i;
 for(n=0;n<32U && BspCan_Receive(&f);n++) {
  if(g_debug.setup.busy && Rs05_Parse((uint8_t)setup_candidate.new_id,&f,&setup_fb))continue;
  for(i=0;i<APP_MOTOR_COUNT;i++) {
   uint32_t before;
   ctx=&motors[i];before=ctx->fb.parameter_count;
   if(!Rs05_Parse(ctx->id,&f,&ctx->fb))continue;
   if(before!=ctx->fb.parameter_count && ctx->fb.param_ok) {
    float v=Rs05_RawFloat(ctx->fb.param_raw);
    if(finite_value(v) && ctx->fb.param_index==RS_CURRENT){ctx->s.current_a=v;ctx->iq_tick=f.tick_ms;ctx->iq_valid=1;}
    if(finite_value(v) && ctx->fb.param_index==RS_VBUS){ctx->s.bus_voltage_v=v;ctx->vbus_tick=f.tick_ms;ctx->vbus_valid=1;}
   }
   break;
  }
  if(i==APP_MOTOR_COUNT)motors[0].s.unmatched_frames++;
 }
}
/* 往当前事务里追加一步。ctx->ops[] 是固定 20 项，所有准备/使能流程都在这之内，
 * 所以不存在动态分配失败的问题。 */
static void add(uint32_t kind,CanFrame frame,uint32_t expected)
{ctx->ops[ctx->op_count].kind=kind;ctx->ops[ctx->op_count].frame=frame;ctx->ops[ctx->op_count++].expected=expected;}
static void read_verify(uint16_t index,uint32_t raw){add(OP_READ,Rs05_Read(ctx->id,index),raw);}
/* 写参数 + 读回核对：先发一帧 18（写），再发一帧 17（读），回读值必须逐位相等。
 * 这是"不信任电机侧"的核心手段 —— 只有读回一致，才认为参数真的写进去了。 */
static void write_verify(uint16_t index,uint32_t raw)
{add(OP_SEND,Rs05_WriteRaw(ctx->id,index,raw),0);read_verify(index,raw);}
static void begin_job(uint32_t command,uint32_t seq)
{cancel_job();ctx->job=command;ctx->job_seq=seq;ctx->s.result=RESULT_PENDING;ctx->s.accepted_seq=seq;}
/* 使能序列（发完这些步骤后，电机应进入 Motor 态）：
 *   读回 运行模式=2 / 电流限幅 / 加速度 / 通信超时（确认参数没被别人改过）
 *   → 写 0x700A 目标速度 = 0（确保使能瞬间不会动）
 *   → 发类型 3 使能 → 等反馈里的状态变成 2。
 * 任何一步读回不符都会让整笔事务失败（RESULT_READBACK），不会继续使能。 */
static void enable_steps(void)
{
 read_verify(RS_RUN_MODE,ctx->s.mode);
 read_verify(RS_CURRENT_LIMIT,Rs05_FloatRaw(ctx->active.current_limit_a));
 read_verify((uint16_t)ctx->s.accel_parameter,Rs05_FloatRaw(ctx->active.accel_rad_s2));
 read_verify((uint16_t)ctx->s.timeout_parameter,APP_MOTOR_TIMEOUT_UNITS);
 write_verify(RS_SPEED_REF,Rs05_FloatRaw(0));
 add(OP_STATE,Rs05_Command(ctx->id,3,0),2);
}
/* 事务成功收尾：按 ctx->job 类型更新应用状态
 *   PREPARE → 把候选参数生效（ctx->active=ctx->pending）、ctx->prepared=1、状态 PREPARED
 *   ENABLE  → ctx->armed=1、状态 ENABLED、目标/实际速度清零（使能后不会自己动）
 *   其它（DISABLE/CLEAR）→ ctx->armed=0、状态 DISABLED
 * 注意：只有"全部步骤都通过"才会走到这里，任何失败都已经被 trip 拦下。 */
static void finish_job(void)
{
 uint32_t done=ctx->job;ctx->s.executed_seq=ctx->job_seq;ctx->s.result=ctx->s.reject_reason=RESULT_OK;ctx->job=0;
 if(done==CMD_PREPARE) {
  ctx->active=ctx->pending;ctx->prepared=1;ctx->s.mode=2;ctx->s.state=MOTOR_PREPARED;
  ctx->s.timeout_readback=ctx->s.timeout_motor_supported?APP_MOTOR_TIMEOUT_UNITS:0;
  ctx->s.speed_limit_rad_s=ctx->active.speed_limit_rad_s;ctx->s.current_limit_a=ctx->active.current_limit_a;ctx->s.accel_rad_s2=ctx->active.accel_rad_s2;
 }else if(done==CMD_ENABLE) {
  ctx->armed=1;ctx->s.state=MOTOR_ENABLED;ctx->s.target_rad_s=ctx->s.applied_rad_s=0;
  ctx->last_tx=HAL_GetTick();
 }else {ctx->armed=0;ctx->s.state=MOTOR_DISABLED;}
}
/* ----------------------------------------------------------------------------
 * service_job —— 事务引擎：每调用一次，只把当前这一步往前推一点
 *
 * 三个阶段（ctx->op_phase）：
 *   0 发起：记下"基线计数"（读参数看 parameter_count、等状态看 feedback_count），
 *           这样能识别"这个回复是新来的"而不是上一条的残留；
 *           然后交给 BspCan_SendBegin 发出去。
 *   1 等发送完成：tx_poll() 返回 BUSY 就下个周期再来；失败 → trip(TX)。
 *           若这一步是 OP_SEND，到这里就算完成，进入下一步。
 *   2 等回应：
 *       OP_READ  —— 参数计数变了且索引对得上：核对数值；特别处理"电机明确拒绝
 *                   0x7022/0x7028"的老固件，改用手册旧地址再试一次；仍不符就 trip;
 *       OP_STATE —— 反馈计数变了且电机状态等于期望值：完成；期间若电机报故障，
 *                   立刻 trip(MOTOR_FAULT)。
 *   任一阶段总耗时超过 APP_PARAM_TIMEOUT_MS(100 ms) → 判超时并 trip。
 * 整个函数没有循环等待，全靠"下次调用再检查"，所以不会卡住其它任务。
 * --------------------------------------------------------------------------*/
static void service_job(void)
{
 Operation *o;uint32_t r,now=HAL_GetTick();uint16_t index;
 if(!ctx->job)return;
 if(ctx->op_at>=ctx->op_count){finish_job();return;}
 o=&ctx->ops[ctx->op_at];index=(uint16_t)(o->frame.data[0]|((uint16_t)o->frame.data[1]<<8));
 if(ctx->op_phase==0) {
  if(tx_owner && tx_owner!=ctx)return;
  ctx->op_counter=o->kind==OP_READ?ctx->fb.parameter_count:ctx->fb.feedback_count;ctx->op_start=now;
  if(!tx_begin(&o->frame)){trip(RESULT_TX);return;}
  if(((o->frame.id>>24)&31U)==3U)ctx->armed=1;
  ctx->op_phase=1;return;
 }
 if(ctx->op_phase==1) {
  r=tx_poll();if(r==CAN_TX_BUSY)return;
  if(r!=CAN_TX_OK){trip(RESULT_TX);return;}
  if(o->kind==OP_SEND){ctx->op_at++;ctx->op_phase=0;return;}
  ctx->op_phase=2;
 }
 if(o->kind==OP_READ && ctx->fb.parameter_count!=ctx->op_counter && ctx->fb.param_index==index) {
  uint32_t mask=index==RS_RUN_MODE?0xFFU:UINT32_MAX;
  ctx->s.readback_index=index;ctx->s.readback_expected=o->expected;ctx->s.readback_actual=ctx->fb.param_raw;
  ctx->s.readback_status=!ctx->fb.param_ok?2U:(ctx->fb.param_raw&mask)!=(o->expected&mask)?3U:1U;
  /* Older RS05 firmware exposes these parameters under the documented 20xx
   * table only. Retry that exact alias after an explicit rejection, not after
   * a timeout/mismatch. Both writes are volatile (never send save/type 22). */
  if(ctx->job==CMD_PREPARE && !ctx->fb.param_ok && (index==RS_ACCEL || index==RS_TIMEOUT)) {
   uint16_t alias=index==RS_ACCEL?RS_ACCEL_LEGACY:RS_TIMEOUT_LEGACY;
   ctx->ops[ctx->op_at-1].frame=Rs05_WriteRaw(ctx->id,alias,o->expected);o->frame=Rs05_Read(ctx->id,alias);
   ctx->op_at--;ctx->op_phase=0;return;
  }
  /* MCU always ramps the target; tolerate only an explicitly unsupported
   * acceleration alias. Current/mode/zero speed remain mandatory. */
  if(index==RS_ACCEL_LEGACY && !ctx->fb.param_ok) {
   ctx->s.accel_parameter=index;
   ctx->s.accel_motor_supported=0;ctx->op_at++;ctx->op_phase=0;return;
  }
  if(index==RS_TIMEOUT_LEGACY && !ctx->fb.param_ok) {
   ctx->s.timeout_motor_supported=0;ctx->s.timeout_parameter=index;
   ctx->op_at++;ctx->op_phase=0;return;
  }
  if(ctx->s.readback_status!=1U){trip(RESULT_READBACK);return;}
  if(index==RS_ACCEL || index==RS_ACCEL_LEGACY){ctx->s.accel_motor_supported=1;ctx->s.accel_parameter=index;}
  if(index==RS_TIMEOUT || index==RS_TIMEOUT_LEGACY){ctx->s.timeout_motor_supported=1;ctx->s.timeout_parameter=index;}
  ctx->op_at++;ctx->op_phase=0;return;
 }
 if(o->kind==OP_STATE && ctx->fb.feedback_count!=ctx->op_counter && online() && ctx->fb.motor_state==o->expected) {
  if(o->expected==2 && !motor_ok()){trip(RESULT_MOTOR_FAULT);return;}
  ctx->op_at++;ctx->op_phase=0;return;
 }
 if(now-ctx->op_start>=APP_PARAM_TIMEOUT_MS) {
  if(o->kind==OP_READ){ctx->s.readback_index=index;ctx->s.readback_expected=o->expected;ctx->s.readback_actual=0;ctx->s.readback_status=4;}
  trip(o->kind==OP_READ?RESULT_READBACK:RESULT_TIMEOUT);
 }
}
/* ----------------------------------------------------------------------------
 * execute —— 命令准入：把一份 MotorCommand 变成"接受/拒绝/开始执行"
 *
 * 顺序刻意如此（先安全、后业务）：
 *   1. 失能/清错是"紧急命令"，即使参数非法、即使正在执行别的命令也允许，
 *      因为这类命令只会让系统更安全；
 *   2. 停止只要求"已使能"，不发帧，只是把目标速度置 0，让斜坡自己降下来；
 *   3. 其余命令依次检查：命令号 → 数值范围 → 确认键 → 是否忙 → 健康位 →
 *      总线状态 → 状态机是否允许 → 参数是否与"准备时"一致。
 * 失败时返回 RESULT_*（上位机原样显示），不会改动当前的运动目标 ——
 * 也就是"拒绝一条命令"不会让电机停下来，只有安全类原因（离线/故障/超速）才 trip。
 * --------------------------------------------------------------------------*/
static uint32_t execute(const MotorCommand *c)
{
 if(c->command==CMD_DISABLE || c->command==CMD_CLEAR_FAULT) {
  /* Urgent command overrides a ctx->pending ctx->job/target, even with bad limit fields. */
  if(ctx->job){ctx->s.executed_seq=ctx->job_seq;}
  begin_job(c->command,c->seq);invalidate();ctx->urgent_stop=0;ctx->s.state=MOTOR_DISABLED;
  add(OP_STATE,Rs05_Command(ctx->id,4,c->command==CMD_CLEAR_FAULT?1:0),0);
  return RESULT_PENDING;
 }
 if(c->command==CMD_STOP) {
  if(ctx->job)return RESULT_BUSY;
  if(!ctx->armed)return RESULT_STATE;
  ctx->s.target_rad_s=0;ctx->s.state=MOTOR_STOPPING;return RESULT_OK;
 }
 if(!g_debug.setup.dual_ready)return RESULT_CONFIRM;
 if(c->command<1 || c->command>6)return RESULT_BAD_COMMAND;
 if(!limits_ok(c))return RESULT_RANGE;
 if(c->confirm_key!=APP_ENABLE_KEY)return RESULT_CONFIRM;
 if(ctx->job)return RESULT_BUSY;
 if(ctx->health_fault)return RESULT_HEALTH;
 if(BspCan_IsBusOff())return RESULT_BUSOFF;
 if(c->command==CMD_PREPARE) {
  if(ctx->armed)return RESULT_STATE;
  if(c->target_rad_s!=0)return RESULT_RANGE;
  /* Only a new explicit PREPARE can release a resume latch. */
  ctx->wall_fault=0;invalidate();ctx->pending=*c;begin_job(c->command,c->seq);ctx->urgent_stop=0;ctx->s.state=MOTOR_CONFIGURING;
  add(OP_STATE,Rs05_Command(ctx->id,4,0),0);
  write_verify(RS_RUN_MODE,2U);
  write_verify(RS_SPEED_REF,Rs05_FloatRaw(0));
  write_verify(RS_CURRENT_LIMIT,Rs05_FloatRaw(c->current_limit_a));
  write_verify(RS_ACCEL,Rs05_FloatRaw(c->accel_rad_s2));write_verify(RS_TIMEOUT,APP_MOTOR_TIMEOUT_UNITS);
  add(OP_SEND,Rs05_Reporting(ctx->id,true),0);add(OP_SEND,Rs05_VersionRequest(ctx->id),0);
  return RESULT_PENDING;
 }
 if(ctx->wall_fault)return RESULT_SERVICE_GAP;
 if(!online())return RESULT_OFFLINE;
 if(!motor_ok())return RESULT_MOTOR_FAULT;
 if(!ctx->prepared || c->speed_limit_rad_s!=ctx->active.speed_limit_rad_s ||
    c->current_limit_a!=ctx->active.current_limit_a || c->accel_rad_s2!=ctx->active.accel_rad_s2)return RESULT_STATE;
 if(c->command==CMD_ENABLE) {
  if(ctx->armed || ctx->s.state!=MOTOR_PREPARED || c->target_rad_s!=0)return RESULT_STATE;
  begin_job(c->command,c->seq);ctx->s.state=MOTOR_ENABLING;enable_steps();
  return RESULT_PENDING;
 }
 if(c->command==CMD_RUN) {
  if(!ctx->armed || ctx->s.mode!=2)return RESULT_STATE;
  /* A RUN latches the requested speed until another command or a fault. */
  ctx->s.target_rad_s=c->target_rad_s;ctx->s.state=MOTOR_RUNNING;
  return RESULT_OK;
 }
 return RESULT_BAD_COMMAND;
}
/* ----------------------------------------------------------------------------
 * poll_command —— 用 seqlock 读上位机/调试器写的命令区
 *
 * (*ctx->mailbox) 是"单写者（调试器）+ 单读者（本任务）"共享内存，没有锁。
 * 规则：写者先把 seq 改成奇数表示"正在改"，改完所有字段再把 seq 写成新的偶数。
 * 这里要求：
 *   · seq 非 0、偶数（不在编辑中）、与上次执行的不同；
 *   · 读前读后的 seq 一致（没被改到一半）；
 *   · 连续两个服务周期读到完全相同的一份命令（防止半写状态被立刻执行）。
 * 全部满足才 execute()，然后记下 executed_seq 并把结果写回 (*ctx->published)。
 * --------------------------------------------------------------------------*/
static void poll_command(void)
{
 MotorCommand c;uint32_t a,b,r;
 a=(*ctx->mailbox).seq;__DMB();
 if(!a || (a&1U) || a==ctx->last_seen){ctx->candidate_valid=0;return;}
 c=(*ctx->mailbox);__DMB();b=(*ctx->mailbox).seq;
 if(a!=b || (b&1U)){ctx->candidate_valid=0;return;}
 if(!ctx->candidate_valid || memcmp(&c,&ctx->candidate,sizeof(c))){ctx->candidate=c;ctx->candidate_valid=1;return;}
 ctx->last_seen=b;ctx->candidate_valid=0;
 if((g_debug.drive.active || AppChassis_Active() || AppBluetooth_OwnsMotors() ||
     AppUsb_OwnsMotors()) && c.command<=CMD_RUN)r=RESULT_STATE;
 else {
  if(g_debug.drive.active || AppChassis_Active() || AppBluetooth_OwnsMotors() ||
     AppUsb_OwnsMotors())drive_cancel();
  r=execute(&c);
 }
 if(r!=RESULT_PENDING){ctx->s.executed_seq=b;ctx->s.result=ctx->s.reject_reason=r;}
 /* Invalid input is rejected without changing the ctx->active target. DISABLE
  * remains available independent of key, NaN or parameter editing. */
}
/* ----------------------------------------------------------------------------
 * check_safety —— 每个服务周期（2 ms）都跑的安全检查
 *
 * 只看"已经观测到的事实"，不做任何补救性发送（除了 Bus-off 的自动恢复）：
 *   · rx_drop 变化 → 说明 RX 队列溢出，可能丢了反馈/回复 → 使能中直接 trip
 *   · Bus-off → trip，并且最多每秒尝试恢复一次（恢复后仍保持失能，需重新准备）
 *   · 已准备/武装/执行中，但电机报故障 → trip（清除故障的命令除外）
 *   · 已武装时还要看：是否离线、健康位、是否超速、电机是否被外部改变状态
 *   · 已准备但未武装时，如果离线也要 trip（此时无法确认电机状态）
 * 这些条件都是"防呆"：即使上位机一直发"运行"，一旦反馈异常也会立刻停。
 * --------------------------------------------------------------------------*/
static void check_safety(void)
{
 CanStats cs;BspCan_GetStats(&cs);
 if(cs.rx_drop!=ctx->previous_drop && (ctx->armed||ctx->job))trip(RESULT_OFFLINE);
 ctx->previous_drop=cs.rx_drop;
 if(BspCan_IsBusOff()) {
  if(ctx->s.state!=MOTOR_FAULT || ctx->job || ctx->armed)trip(RESULT_BUSOFF);

 }
 if((ctx->prepared||ctx->armed||ctx->job) && !motor_ok() && ctx->job!=CMD_CLEAR_FAULT)trip(RESULT_MOTOR_FAULT);
 if(ctx->armed) {
  if(!online())trip(RESULT_OFFLINE);
  else if(ctx->health_fault)trip(RESULT_HEALTH);
  else if(fabsf(ctx->fb.velocity_rad_s)>ctx->active.speed_limit_rad_s+0.15f)trip(RESULT_OVERSPEED);
  else if(!ctx->job && ctx->fb.motor_state!=2)trip(RESULT_STATE);
 }
 if(ctx->prepared && !ctx->armed && !ctx->job && !online())trip(RESULT_OFFLINE);
}
/* ----------------------------------------------------------------------------
 * service_normal —— 没有事务在跑时的"日常节奏"（每 2 ms 调一次）
 *
 * 优先级从高到低：
 *   1. 还有一帧在发（ctx->service_tx）→ 先把它等完；失败时只有"已武装"或
 *      "这是个停止帧且当前不是故障态"才 trip（停止帧失败不该把正常状态打成故障）。
 *   2. ctx->urgent_stop（trip 之后）→ 立刻补发一帧类型 4 停止。
 *   3. 已武装 → 每 20 ms 发一次 0x700A 速度，按 accel*0.02 的步长逼近目标
 *      （这就是"20 ms 速度斜坡"：加速度由准备时的 accel_rad_s2 决定）。
 *      若处于 STOPPING 且斜坡到 0、实测速度也接近 0 → 回到 ENABLED。
 *   4. 未武装 → 每 500 ms 发一帧停止（兜底，防止电机被外部使能后一直转）。
 *   5. 未使能也未准备 → 每 100 ms 做一次"只读探询"：身份/版本/电流/电压，
 *      轮换着问，既不使能也不写运动参数，所以上电后能自动认出电机。
 *   6. 已准备未使能 → 每 100 ms 轮询电流/母线电压（供屏幕和上位机显示）。
 * 注意：每一步最多发一帧，发不出去就下个周期再说，绝不在这里阻塞等 ACK。
 * --------------------------------------------------------------------------*/
static void service_normal(uint32_t now)
{
 CanFrame f;uint32_t r;
 if(ctx->service_tx) {
  r=tx_poll();if(r==CAN_TX_BUSY)return;
  ctx->service_tx=0;
  if(r!=CAN_TX_OK && (ctx->armed || (ctx->service_type==4U && ctx->s.state!=MOTOR_FAULT))) {
   trip(RESULT_TX);return;
  }
 }
 if(BspCan_IsBusOff())return;
 if(tx_owner && tx_owner!=ctx)return;
 if(ctx->urgent_stop){f=Rs05_Command(ctx->id,4,0);ctx->urgent_stop=0;ctx->last_stop=now;}
 else if(ctx->armed && now-ctx->last_tx>=20U) {
  float step=ctx->active.accel_rad_s2*0.02f,diff=ctx->s.target_rad_s-ctx->s.applied_rad_s;
  if(diff>step)diff=step;
  if(diff < -step)diff=-step;
  ctx->s.applied_rad_s+=diff;
  f=Rs05_WriteFloat(ctx->id,RS_SPEED_REF,ctx->s.applied_rad_s);
  ctx->last_tx=now;
  if(ctx->s.state==MOTOR_STOPPING && ctx->s.applied_rad_s==0 && fabsf(ctx->fb.velocity_rad_s)<0.05f)ctx->s.state=MOTOR_ENABLED;
 }else if(!ctx->armed && now-ctx->last_stop>=500U) {f=Rs05_Command(ctx->id,4,0);ctx->last_stop=now;}
 else if(!ctx->armed && !ctx->prepared && now-ctx->last_probe>=100U) {
  /* Commissioning/telemetry only: never enable or write motion parameters.
   * Retry reporting while offline so powering the motor after MCU also works. */
  ctx->last_probe=now;
  switch(ctx->probe_step++%5U) {
   case 0: f=Rs05_IdentityRequest(ctx->id);break;
   case 1: f=online()?Rs05_Read(ctx->id,RS_RUN_MODE):Rs05_Reporting(ctx->id,true);break;
   case 2: f=ctx->fb.version_valid?Rs05_Read(ctx->id,RS_RUN_MODE):Rs05_VersionRequest(ctx->id);break;
   case 3: f=Rs05_Read(ctx->id,RS_CURRENT);break;
   default:f=Rs05_Read(ctx->id,RS_VBUS);break;
  }
 }else if(ctx->prepared && now-ctx->last_poll>=100U) {
  const uint16_t indexes[]={RS_CURRENT,RS_VBUS};
  f=Rs05_Read(ctx->id,indexes[ctx->poll_index++%2U]);ctx->last_poll=now;
 }else return;
 ctx->service_type=(f.id>>24)&31U;
 if(tx_begin(&f))ctx->service_tx=1;
 else if(ctx->armed || (ctx->service_type==4U && ctx->s.state!=MOTOR_FAULT))trip(RESULT_TX);
}
/* ----------------------------------------------------------------------------
 * publish —— 周期发布一份"稳定"的电机快照（每 20 ms 一次）
 *
 * 为什么是 20 ms：上位机通过 SWD 读 148 字节的控制/遥测区，读一次要几毫秒，
 * 如果每 1 ms 就更新一次，读到一半就被改，序号校验会一直失败。所以固件把
 * 快照"保持" 20 ms，给读取方一个稳定的窗口。
 *
 * seqlock 写法：
 *   snapshot_seq += 2  → 变成奇数（读取方看到奇数就知道正在写）
 *   拷贝整个结构体（约 280 字节）
 *   最后把 snapshot_seq 写成偶数（写入完成）
 * 读取方要求"读前读后序号相同且为偶数"，否则重读。
 *
 * 控制与安全逻辑不受这 20 ms 限制：它们每个服务周期都在跑，这里只影响"对外可见的
 * 数据新鲜度"。反馈年龄/电流电压年龄也在这里换算成毫秒。
 * --------------------------------------------------------------------------*/
static void publish(void)
{
 uint32_t now=HAL_GetTick();MotorStatus out;
 /* Keep observations stable long enough for a running-target USB/SWD read.
  * Control, command acceptance and safety still execute every service cycle. */
 if(ctx->publish_valid && now-ctx->publish_tick<20U)return;
 ctx->publish_valid=1;ctx->publish_tick=now;ctx->s.published_tick_ms=now;
 ctx->s.online=online();ctx->s.feedback_age_ms=ctx->fb.feedback_count?now-ctx->fb.feedback_ms:UINT32_MAX;
 ctx->s.position_rad=ctx->fb.position_rad;ctx->s.velocity_rad_s=ctx->fb.velocity_rad_s;ctx->s.torque_nm=ctx->fb.torque_nm;ctx->s.temperature_c=ctx->fb.temperature_c;
 ctx->s.fault_bits=ctx->fb.fault_bits;ctx->s.fault=ctx->fb.fault;ctx->s.warning=ctx->fb.warning;ctx->s.motor_state=ctx->fb.motor_state;
 ctx->s.version=ctx->fb.version;ctx->s.version_valid=ctx->fb.version_valid;
 ctx->s.current_age_ms=ctx->iq_valid?now-ctx->iq_tick:UINT32_MAX;ctx->s.voltage_age_ms=ctx->vbus_valid?now-ctx->vbus_tick:UINT32_MAX;
 ctx->s.enable_requested=ctx->armed;ctx->s.enabled=online() && ctx->fb.motor_state==2;
 ctx->s.health_fault=ctx->health_fault;ctx->s.cpu_dcache_enabled=(SCB->CCR&SCB_CCR_DC_Msk)?1U:0U;
 ctx->s.job_command=ctx->job;ctx->s.job_step=ctx->op_at;BspCan_GetStats(&ctx->s.can);
 ctx->s.configured_id=ctx->id;ctx->s.identity_valid=ctx->fb.identity_count!=0;ctx->s.identity_id=ctx->fb.identity_id;
 memcpy(ctx->s.identity_uid,ctx->fb.uid,8);ctx->s.feedback_count=ctx->fb.feedback_count;ctx->s.parameter_count=ctx->fb.parameter_count;
 ctx->s.snapshot_seq+=2;out=ctx->s;out.snapshot_seq=ctx->s.snapshot_seq-1;
 (*ctx->published).snapshot_seq=out.snapshot_seq;__DMB();(*ctx->published)=out;
 __DMB();(*ctx->published).snapshot_seq=ctx->s.snapshot_seq;
}
/* Commissioning is exclusive: no normal CAN traffic until it completes.
 * ID writes require a physical-isolation declaration; software cannot count
 * two devices with identical IDs. No enable, zeroing, save or protocol switch. */
static bool all_idle(void)
{
 uint32_t i;
 for(i=0;i<APP_MOTOR_COUNT;i++)
  if(motors[i].armed || motors[i].job || motors[i].prepared)return false;
 return true;
}
static void stop_all(void)
{
 uint32_t i;
 drive_cancel();
 /* Revoke the USB bridge's ownership too, but never touch Bluetooth's manual
  * hold: a global stop must not let ROS auto-recover past a Bluetooth STOP. */
 AppUsb_Cancel();
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  ctx=&motors[i];
  if(ctx->job){ctx->s.executed_seq=ctx->job_seq;ctx->s.result=ctx->s.reject_reason=RESULT_STATE;}
  cancel_job();invalidate();ctx->s.state=MOTOR_DISABLED;ctx->urgent_stop=1;
  ctx->last_seen=ctx->mailbox->seq;ctx->candidate_valid=0;
 }
}
/* Validate BOTH channels before changing either target; CAN frames still use
 * the original arbitrated transmitter and each motor's prepared ramp. */
static uint32_t drive_execute(const DriveCommand *c,uint32_t now)
{
 uint32_t i;float targets[2];
 targets[0]=c->motor1_rad_s;targets[1]=c->motor2_rad_s;
 /* Acquire always takes over at zero target. Existing feedback, targets and
  * ramp values do not gate a mode switch; the normal ramp handles takeover. */
 if(c->command==1U)targets[0]=targets[1]=0;
 if(c->command==3U){drive_cancel();return RESULT_OK;}
 if(AppBluetooth_OwnsMotors())return RESULT_BUSY;
 if(AppChassis_Active())return RESULT_STATE;
 if(c->key!=APP_ENABLE_KEY)return RESULT_CONFIRM;
 if(c->command!=1U && c->command!=2U)return RESULT_BAD_COMMAND;
 if(c->command==1U && g_debug.drive.active)return RESULT_STATE;
 if(c->command==2U && !g_debug.drive.active)return RESULT_STATE;
 if(!g_debug.setup.dual_ready || g_debug.setup.busy)return RESULT_CONFIRM;
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  ctx=&motors[i];
  if(ctx->s.mode!=motors[0].s.mode)return RESULT_STATE;
  if(!finite_value(targets[i]) || fabsf(targets[i])>ctx->active.speed_limit_rad_s)return RESULT_RANGE;
  if(ctx->job)return RESULT_BUSY;
  if(ctx->health_fault || ctx->wall_fault)return RESULT_HEALTH;
  if(BspCan_IsBusOff())return RESULT_BUSOFF;
  if(!online())return RESULT_OFFLINE;
  if(!motor_ok())return RESULT_MOTOR_FAULT;
  if(!ctx->armed || !ctx->prepared || (ctx->s.state!=MOTOR_ENABLED &&
     ctx->s.state!=MOTOR_RUNNING && ctx->s.state!=MOTOR_STOPPING))return RESULT_STATE;
 }
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  motors[i].s.target_rad_s=targets[i];
  motors[i].s.state=targets[i]==0?MOTOR_STOPPING:MOTOR_RUNNING;
 }
 g_debug.drive.active=1;drive_tick=now;return RESULT_OK;
}
static void drive_service(uint32_t now)
{
 DriveCommand c;uint32_t seq,r,i;bool healthy=true;
 if(g_debug.drive.active) {
  for(i=0;i<APP_MOTOR_COUNT;i++) {
   ctx=&motors[i];
   if(!ctx->armed || !ctx->prepared || ctx->job || !online() ||
      !motor_ok() || ctx->health_fault || ctx->wall_fault)healthy=false;
  }
  if(!healthy || now-drive_tick>=600U) {
   drive_cancel();g_debug.drive.result=healthy?RESULT_TIMEOUT:RESULT_PROTECTION;
  }
 }
 seq=g_debug.drive_cmd.seq;__DMB();
 if(!seq || (seq&1U) || seq==drive_seen){drive_candidate_valid=0;return;}
 c=g_debug.drive_cmd;__DMB();
 if(seq!=g_debug.drive_cmd.seq)return;
 if(!drive_candidate_valid || memcmp(&c,&drive_candidate,sizeof(c))) {
  drive_candidate=c;drive_candidate_valid=1;return;
 }
 drive_seen=seq;drive_candidate_valid=0;r=drive_execute(&c,now);
 if(r!=RESULT_OK && g_debug.drive.active)drive_cancel();
 g_debug.drive.result=r;__DMB();g_debug.drive.executed_seq=seq;
}
static void setup_done(uint32_t result)
{
 if(g_debug.setup.busy){BspCan_Cancel();tx_owner=NULL;}
 g_debug.setup.busy=0;
 g_debug.setup.result=result;__DMB();g_debug.setup.executed_seq=setup_seen;
 setup_phase=0;
}
static void setup_service(uint32_t now)
{
 MotorSetup c;uint32_t seq=g_debug.setup.seq,r,i;CanFrame f;
 if(!g_debug.setup.busy && seq && !(seq&1U) && seq!=setup_seen) {
  __DMB();c=g_debug.setup;__DMB();
  if(seq!=g_debug.setup.seq)return;
  if(setup_candidate_seq!=seq || memcmp(&c,&setup_candidate,24)) {
   setup_candidate=c;setup_candidate_seq=seq;return;
  }
  setup_seen=seq;
  if(AppBluetooth_OwnsMotors()){setup_done(RESULT_BUSY);return;}
  if(c.key!=APP_ENABLE_KEY){setup_done(RESULT_CONFIRM);return;}
  if(!all_idle()){setup_done(RESULT_STATE);return;}
  if(c.command==1U) {
   /* Both identities must be recent, at distinct configured IDs and UIDs. */
   for(i=0;i<APP_MOTOR_COUNT;i++) {
    if(!motors[i].fb.identity_count || now-motors[i].fb.identity_ms>1500U ||
       !motors[i].fb.feedback_count || now-motors[i].fb.feedback_ms>=APP_FEEDBACK_TIMEOUT_MS) {
     setup_done(RESULT_OFFLINE);return;
    }
   }
   if(!memcmp(motors[0].fb.uid,motors[1].fb.uid,8)){setup_done(RESULT_CONFIRM);return;}
   g_debug.setup.dual_ready=1;setup_done(RESULT_OK);return;
  }
  if(c.command!=2U || c.old_id<1U || c.old_id>127U || c.new_id<1U || c.new_id>127U || c.old_id==c.new_id) {
   setup_done(RESULT_RANGE);return;
  }
  if(c.isolated_key!=APP_ISOLATED_KEY){setup_done(RESULT_CONFIRM);return;}
  g_debug.setup.dual_ready=0;stop_all();BspCan_Cancel();tx_owner=NULL;
  memset(&setup_fb,0,sizeof(setup_fb));g_debug.setup.busy=1;
  g_debug.setup.result=RESULT_PENDING;setup_phase=0;setup_tick=now;
 }
 if(!g_debug.setup.busy)return;
 if(BspCan_IsBusOff() || now-setup_tick>1500U){setup_done(RESULT_TIMEOUT);return;}
 /* stop old ID -> await reset feedback -> change ID -> query new identity */
 if(setup_phase==0U) {
  f=Rs05_Command((uint8_t)setup_candidate.old_id,4,0);
  if(!BspCan_SendBegin(&f)){setup_done(RESULT_TX);return;}setup_phase=1;
 }else if(setup_phase==1U || setup_phase==3U || setup_phase==5U) {
  r=BspCan_SendPoll();if(r==CAN_TX_BUSY)return;
  if(r!=CAN_TX_OK){setup_done(RESULT_TX);return;}
  setup_phase++;
 }else if(setup_phase==2U) {
  /* Stop transmission alone is insufficient: require fresh reset feedback. */
  for(i=0;i<APP_MOTOR_COUNT;i++)if(motors[i].id==setup_candidate.old_id &&
     motors[i].fb.feedback_count && (int32_t)(motors[i].fb.feedback_ms-setup_tick)>=0 &&
     motors[i].fb.motor_state==0U)break;
  if(i==APP_MOTOR_COUNT)return;
  f=Rs05_Command((uint8_t)setup_candidate.old_id,7,0);
  f.id|=setup_candidate.new_id<<16;
  if(!BspCan_SendBegin(&f)){setup_done(RESULT_TX);return;}setup_phase=3;
 }else if(setup_phase==4U) {
  f=Rs05_IdentityRequest((uint8_t)setup_candidate.new_id);
  if(!BspCan_SendBegin(&f)){setup_done(RESULT_TX);return;}setup_phase=5;
 }else if(setup_fb.identity_count) {
  /* Drop old observations so authorization requires fresh pair discovery. */
  for(i=0;i<APP_MOTOR_COUNT;i++) {
   memset(&motors[i].fb,0,sizeof(motors[i].fb));
   motors[i].iq_valid=motors[i].vbus_valid=0;
  }
  setup_done(RESULT_OK);
 }
}
void AppMotor_Init(void)
{
 uint32_t i;bool started=BspCan_Start();
 memset(motors,0,sizeof(motors));tx_owner=NULL;
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  ctx=&motors[i];ctx->id=(uint8_t)(i?APP_MOTOR2_ID:APP_MOTOR_ID);
  ctx->mailbox=i?&g_debug.cmd2:&g_debug.cmd;
  ctx->published=i?&g_debug.motor2:&g_debug.motor;
  ctx->wall_last=BspTime_Us();ctx->wall_valid=1;
  if(!started)trip(RESULT_TX);else ctx->urgent_stop=1;
  publish();
 }
 AppChassis_Init();
 AppBluetooth_Init();
 AppUsb_Init();
}
void AppMotor_Service(void)
{
 uint32_t us=BspTime_Us(),now=HAL_GetTick(),i;
 bool gap=BspTime_DebugFrozen();
 for(i=0;i<APP_MOTOR_COUNT;i++)
  if(motors[i].wall_valid && us-motors[i].wall_last>APP_SERVICE_GAP_US)gap=true;
 if(gap) {
  drive_cancel();
  g_debug.setup.dual_ready=0;
  if(g_debug.setup.busy)setup_done(RESULT_SERVICE_GAP);
  BspCan_DiscardRx();
  for(i=0;i<APP_MOTOR_COUNT;i++) {
   ctx=&motors[i];ctx->wall_fault=1;ctx->last_seen=ctx->mailbox->seq;
   ctx->candidate_valid=0;ctx->fb.feedback_count=0;ctx->iq_valid=ctx->vbus_valid=0;
   trip(RESULT_SERVICE_GAP);
  }
 }
 drain();
 if(g_debug.stop_all_seq && !(g_debug.stop_all_seq&1U) && g_debug.stop_all_seq!=stop_seen) {
  stop_seen=g_debug.stop_all_seq;
  if(g_debug.setup.busy)setup_done(RESULT_STATE);
  stop_all();
 }
 setup_service(now);
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  ctx=&motors[i];
  if(!g_debug.setup.busy){check_safety();poll_command();}
 }
 drive_service(now);
 AppChassis_Service(now);
 AppBluetooth_Service(now);
 /* USB 桥放在最后：本拍 IMU/CAN 快照已更新，且它可以用本地电机接口。 */
 AppUsb_Service(now);
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  ctx=&motors[i];ctx->wall_last=us;ctx->wall_valid=1;ctx->s.motor_heartbeat++;
  if(!g_debug.setup.busy) {
   if(ctx->job)service_job();else service_normal(now);
  }
  publish();
 }
 if(BspCan_IsBusOff() && now-recovery_tick>=1000U) {
  recovery_tick=now;g_debug.setup.dual_ready=0;
  stop_all();(void)BspCan_Recover();tx_owner=NULL;
 }
}
void AppMotor_GetSnapshot(MotorStatus *out){*out=g_debug.motor;}
void AppMotor_StopAll(void){stop_all();}
void AppMotor_RemoteStop(void)
{
 if(g_debug.drive.active || AppChassis_Active())drive_cancel();
 AppMotor_PairTargets(0,0);
}
uint32_t AppMotor_AuthorizePair(void)
{
 uint32_t i,now=HAL_GetTick();
 if(g_debug.setup.busy)return RESULT_BUSY;
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  MotorContext *m=&motors[i];
  if(m->job || m->armed)return RESULT_BUSY;
  if(!m->fb.identity_count || now-m->fb.identity_ms>1500U ||
     !m->fb.feedback_count || now-m->fb.feedback_ms>=APP_FEEDBACK_TIMEOUT_MS)return RESULT_OFFLINE;
 }
 if(!memcmp(motors[0].fb.uid,motors[1].fb.uid,8))return RESULT_CONFIRM;
 g_debug.setup.dual_ready=1;return RESULT_OK;
}
uint32_t AppMotor_LocalCommand(uint32_t slot,const MotorCommand *command)
{
 MotorContext *saved=ctx;uint32_t result;
 if(slot>=APP_MOTOR_COUNT)return RESULT_RANGE;
 ctx=&motors[slot];result=execute(command);
 if(result!=RESULT_PENDING)ctx->s.result=ctx->s.reject_reason=result;
 ctx=saved;return result;
}
uint32_t AppMotor_LocalResult(uint32_t slot)
{return motors[slot].job?RESULT_PENDING:motors[slot].s.result;}
uint32_t AppMotor_RemoteTargets(float first,float second)
{
 uint32_t i,result=AppMotor_PairReady(2);float targets[2];
 if(result)return result;
 if(BspCan_IsBusOff())return RESULT_BUSOFF;
 targets[0]=first;targets[1]=second;
 for(i=0;i<APP_MOTOR_COUNT;i++)
  if(!finite_value(targets[i]) || fabsf(targets[i])>motors[i].active.speed_limit_rad_s)return RESULT_RANGE;
 AppMotor_PairTargets(first,second);return RESULT_OK;
}
uint32_t AppMotor_PairReady(uint32_t mode)
{
 uint32_t i;
 if(!g_debug.setup.dual_ready || g_debug.setup.busy)return RESULT_CONFIRM;
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  MotorContext *m=&motors[i];
  if(m->s.mode!=mode || !m->armed || !m->prepared || m->job)return RESULT_STATE;
  if(!m->fb.feedback_count || HAL_GetTick()-m->fb.feedback_ms>APP_FEEDBACK_TIMEOUT_MS)return RESULT_OFFLINE;
  if(m->s.state==MOTOR_FAULT || m->health_fault || m->wall_fault)return RESULT_PROTECTION;
 }
 return RESULT_OK;
}
void AppMotor_PairTargets(float first,float second)
{
 uint32_t i;float targets[2];targets[0]=first;targets[1]=second;
 for(i=0;i<APP_MOTOR_COUNT;i++) {
  float limit=motors[i].active.speed_limit_rad_s;
  motors[i].s.target_rad_s=fmaxf(-limit,fminf(limit,targets[i]));
  if(motors[i].armed)motors[i].s.state=targets[i]==0?MOTOR_STOPPING:MOTOR_RUNNING;
 }
}
float AppMotor_FeedbackSpeed(uint32_t slot){return motors[slot].fb.velocity_rad_s;}
float AppMotor_FeedbackCurrent(uint32_t slot){return motors[slot].s.current_a;}
float AppMotor_TargetSpeed(uint32_t slot){return motors[slot].s.target_rad_s;}
void AppMotor_SetHealth(uint32_t fault)
{uint32_t i;for(i=0;i<APP_MOTOR_COUNT;i++)motors[i].health_fault=fault;}
