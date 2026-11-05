#!/usr/bin/env bash
# 内存看门狗：MemAvailable 低于阈值时终止机器人栈的全部进程。
#
# 默认行为：
#   · 每 2 秒读一次 /proc/meminfo 的 MemAvailable
#   · 连续 3 次（约 6 秒）低于 MEM_THRESHOLD_MB=800 才触发，避免瞬时抖动
#   · 先给栈主脚本和各自进程组发 SIGINT（与 Ctrl-C 相同，会走 DISARM/清理）
#   · 5 秒未退出则 SIGTERM，再 3 秒未退出则 SIGKILL
#   · 触发后冷却 120 秒，避免重启瞬间再次触发
#
# 只终止机器人栈（导航 / AI / 深度 / 人像 / 手部），不会杀桌面、SSH、
# VSCode 或 systemd —— 否则树莓派会失去远程恢复能力。
#
# 可调环境变量：
#   MEM_THRESHOLD_MB=800    可用内存阈值（MiB）
#   MEM_CHECK_INTERVAL=2    采样间隔（秒）
#   MEM_CONFIRM_SAMPLES=3   连续超限采样次数
#   MEM_KILL_GRACE=5        SIGINT 后等待秒数
#   MEM_COOLDOWN=120        触发后冷却秒数
#   MEM_DRY_RUN=1           只打印将要终止的进程，不发信号（测试用）
#
# 手动测试：
#   MEM_DRY_RUN=1 MEM_THRESHOLD_MB=999999 bash scripts/mem_watchdog.sh

set -uo pipefail

THRESHOLD_MB="${MEM_THRESHOLD_MB:-800}"
INTERVAL="${MEM_CHECK_INTERVAL:-2}"
CONFIRM="${MEM_CONFIRM_SAMPLES:-3}"
GRACE="${MEM_KILL_GRACE:-5}"
COOLDOWN="${MEM_COOLDOWN:-120}"
DRY_RUN="${MEM_DRY_RUN:-0}"

# 栈进程匹配规则；顺序无关。
STACK_PATTERN='ros2 launch robot_base_bringup|ros2 launch largemodel|ros2 launch orbbec_camera|person_detector_node\.py|hand_detector_host\.py|hand_pose_bridge\.py|micromamba run -n (ros_jazzy|ai_host)|scripts/start_host_stack\.sh'

log() {
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

avail_mb() {
  awk '/^MemAvailable:/ {printf "%d", $2 / 1024}' /proc/meminfo 2>/dev/null
}

stack_pids() {
  # shellcheck disable=SC2086
  pgrep -f "$STACK_PATTERN" 2>/dev/null | sort -n | uniq
}

stack_groups() {
  local pid
  for pid in $(stack_pids); do
    ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' '
  done | sort -n | uniq
}

is_stack_group() {
  local pgid="$1"
  ps -eo pgid=,args= 2>/dev/null \
    | awk -v g="$pgid" '$1 == g { $1=""; print }' \
    | grep -qE 'robot_base_bringup|largemodel|orbbec_camera|person_detector|hand_detector|hand_pose_bridge|micromamba|start_host_stack'
}

signal_pid() {
  local sig="$1" pid="$2"
  if [[ "$DRY_RUN" == "1" ]]; then
    log "[dry-run] kill -$sig $pid ($(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | cut -c1-100))"
    return 0
  fi
  kill -"$sig" "$pid" 2>/dev/null || true
}

signal_group() {
  local sig="$1" pgid="$2"
  if [[ "$DRY_RUN" == "1" ]]; then
    log "[dry-run] kill -$sig -- -$pgid"
    return 0
  fi
  kill -"$sig" -- "-$pgid" 2>/dev/null || true
}

kill_stack() {
  local avail_before="$1"
  local sups groups pgid

  sups="$(pgrep -f 'scripts/start_host_stack\.sh' 2>/dev/null | sort -n | uniq || true)"
  groups="$(stack_groups || true)"

  log "可用内存 ${avail_before}MB < ${THRESHOLD_MB}MB，终止机器人栈"
  if [[ -n "$sups" ]]; then
    log "栈主进程: $(echo "$sups" | tr '\n' ' ')"
  fi

  # 1) 先让栈主脚本走自己的 trap（会 DISARM 并清理各进程组）
  for pid in $sups; do
    signal_pid INT "$pid"
  done

  # 2) 同时通知所有栈进程组，防止主脚本已不在时残留
  for pgid in $groups; do
    if is_stack_group "$pgid"; then
      signal_group INT "$pgid"
    fi
  done

  sleep "$GRACE"

  # 3) 仍在的进程组升级为 SIGTERM
  local left=0
  for pgid in $(stack_groups || true); do
    if is_stack_group "$pgid"; then
      signal_group TERM "$pgid"
      left=1
    fi
  done
  if [[ "$left" == "1" ]]; then
    sleep 3
  fi

  # 4) 最后 SIGKILL
  for pgid in $(stack_groups || true); do
    if is_stack_group "$pgid"; then
      signal_group KILL "$pgid"
    fi
  done

  if [[ "$DRY_RUN" != "1" ]]; then
    sleep 2
    log "终止完成：可用内存 ${avail_mb:-?}MB（触发前 ${avail_before}MB）"
  else
    log "[dry-run] 不发信号，结束"
  fi
}

log "启动：阈值 ${THRESHOLD_MB}MB，采样 ${INTERVAL}s×${CONFIRM}，冷却 ${COOLDOWN}s，dry_run=${DRY_RUN}"

below=0
last_heartbeat=0
while :; do
  avail="$(avail_mb)"
  now="$(date +%s)"

  if [[ -z "$avail" || "$avail" -le 0 ]]; then
    log "读取 MemAvailable 失败，跳过本次采样"
    sleep "$INTERVAL"
    continue
  fi

  if (( avail < THRESHOLD_MB )); then
    below=$((below + 1))
    log "采样超限 ${below}/${CONFIRM}：可用 ${avail}MB < ${THRESHOLD_MB}MB"
    if (( below >= CONFIRM )); then
      kill_stack "$avail"
      below=0
      sleep "$COOLDOWN"
      continue
    fi
  else
    if (( below > 0 )); then
      log "内存恢复到 ${avail}MB，计数清零"
    fi
    below=0
  fi

  # 每 60 秒输出一次心跳，方便 journalctl 观察
  if (( now - last_heartbeat >= 60 )); then
    log "心跳：可用 ${avail}MB / 阈值 ${THRESHOLD_MB}MB"
    last_heartbeat="$now"
  fi

  sleep "$INTERVAL"
done
