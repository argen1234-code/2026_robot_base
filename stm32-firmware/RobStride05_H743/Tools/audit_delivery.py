"""Record existing build evidence. No download or hardware control."""
from pathlib import Path
import re,json,hashlib,xml.etree.ElementTree as ET
from datetime import datetime,timezone,timedelta
root=Path(__file__).resolve().parents[1]
mdk=root/'MDK-ARM'
tree=ET.parse(mdk/'RobStride05_H743.uvprojx')
paths=[n.text for n in tree.findall('.//FilePath')]
assert all((mdk/s.replace('\\','/')).is_file() for s in paths),'Missing project files'
assert len(paths)==len(set(paths)),'Duplicate source paths'
for n in tree.findall('.//Cads/VariousControls/IncludePath'):
 for s in n.text.split(';'):assert (mdk/s.replace('\\','/')).is_dir(),s
assert tree.findtext('.//TargetArmAds/LDads/ScatterFile')=='RobStride05_H743.sct'
assert tree.findtext('.//Utilities/Flash1/UpdateFlashBeforeDebugging')=='0'
assert tree.findtext('.//TargetCommonOption/Device')=='STM32H743IITx'
src='\n'.join(p.read_text(encoding='utf-8-sig') for d in ['Core','App','BSP','Devices'] for p in (root/d).rglob('*.c'))
assert 'SCB_EnableDCache(' not in src
assert 'HAL_DMA_Start(' not in src and 'HAL_DMA_Init(' not in src
assert 'portable/RVDS/ARM_CM7/r0p1/port.c' in '\n'.join(paths)
build=(root/'Docs/build_output.log').read_text(encoding='utf-8-sig',errors='replace')
host=(root/'Docs/host_checks.log').read_text(encoding='utf-8-sig',errors='replace')
assert '0 Error(s), 0 Warning(s).' in build
assert 'ALL HOST CHECKS PASSED' in host
m=re.search(r'Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)',build)
assert m
c,ro,rw,zi=map(int,m.groups());art={};tz=timezone(timedelta(hours=8))
for ext in ['axf','hex']:
 p=mdk/'Objects'/('RobStride05_H743.'+ext)
 art[ext]={'path':str(p.relative_to(root)),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'modified_shanghai':datetime.fromtimestamp(p.stat().st_mtime,tz).isoformat()}
passes=sum(s.startswith('PASS ') for s in host.splitlines())
data={'as_of_shanghai':datetime.now(tz).isoformat(),'project_file_entries':len(paths),'all_sources_local':True,'errors':0,'warnings':0,'code_bytes':c,'ro_bytes':ro,'rw_bytes':rw,'zi_bytes':zi,'host_check_groups':passes,'artifacts':art,'flashed':False,'hardware_tested':False}
(root/'Docs/delivery_audit.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
report=f'''# 实际验证与构建记录

记录时间：{data['as_of_shanghai']}（上海）。以下是实际工具输出，未执行下载或电机实测。

| 层次 | 结果 | 证据与边界 |
|---|---|---|
| 工程检查 | 通过 | {len(paths)} 个文件条目存在、无重复、均为本工程相对路径，含 CM7/r0p1 端口和 scatter |
| CubeMX | 已实际生成和再次加载/生成 | cubemx_generate.log、cubemx_regenerate.log；RC5 生成后必须运行 integrate.py |
| Keil 编译/链接 | **0 错误、0 警告** | build_output.log、build_run.log；ARMCC5.06u7 build960、STM32H7xx_DFP2.7.0 |
| PC 软件检查 | **{passes} 组 PASS** | host_checks.log；编译实际 app_motor.c、robstride05.c，使用假 RTOS/CAN 对端 |
| 下载 | 未执行 | 首次下载前由用户核对接线、供电与限制 |
| CAN/OLED/电机实测 | 未执行 | 按 04_上板验收.md 执行；不以软件通过代替硬件观测 |

## 实际构建大小

~~~text
Program Size: Code={c} RO-data={ro} RW-data={rw} ZI-data={zi}
RobStride05_H743.axf - 0 Error(s), 0 Warning(s).
~~~

只读代码/数据共 {c+ro} 字节；RW+ZI 共 {rw+zi} 字节，含静态 RTOS 堆。动态栈最大使用量、总线波形和实际实时性仍需上板验证。

| 产物 | 字节 | SHA256 |
|---|---:|---|
| AXF | {art['axf']['bytes']} | {art['axf']['sha256']} |
| HEX | {art['hex']['bytes']} | {art['hex']['sha256']} |

AXF 时间：{art['axf']['modified_shanghai']}。AXF/HEX 在 MDK-ARM/Objects，map 在 MDK-ARM/Listings；机器可读记录见 delivery_audit.json。

## 软件检查覆盖

- 启动不使能；RS05 参数帧、小端浮点、05 量程、反馈 ID 和版本消息区分。
- 非法命令、NaN、越界、确认键拒绝；奇/偶序号完整快照提交，一次性命令不重放。
- 写入/读回、超时门禁、使能/运行/停止、故障不恢复；无电机超时功能时拒绝运行。
- 电机掉电或外部改变保护参数后拒绝使能；发送失败和 Bus-off 拒绝。
- 清错不抹掉详细锁存故障，等待新零故障帧；反馈过期、窗口超时、超速及故障触发生产监测。
- 模拟 HAL tick 不变但独立计时前进的 CPU 间断，拒绝旧运动帧和提交；TIM2 被冻结时拒绝运动。

未在 PC 中验证 HAL I²C 电气恢复、FDCAN ACK、ISR 抢占、探针 Live Watch、Keil 命令文件解释器或转矩释放。Tests 不参与固件编译，未添加另一套上板演示控制任务。

## 配置检查

HAL v1.11.6、FreeRTOS V10.6.2、CMSIS-RTOS2。DCache 在 IOC 关闭，入口显式 Disable，应用源码无 EnableDCache；未配置 DMA。TIM2 是 BSP 独占、无 IRQ 的运行间断计时，不要在 CubeMX 重复分配。

ST-Link、内部 Flash 算法路径、三个 Watch 根结构和 Periodic Window Update 已写入选项；未接探针验证。进入调试前自动更新 Flash 已关闭，构建脚本不下载。
'''
(root/'Docs/05_验证记录.md').write_text(report,encoding='utf8')
for p in [root/'README.md',*(root/'Docs').glob('*.md')]:
 for link in re.findall(r'\]\(([^)]+)\)',p.read_text(encoding='utf8')):
  if '://' not in link and not link.startswith('#'):assert (p.parent/link).exists(),(str(p),link)
print(json.dumps(data,ensure_ascii=False,indent=2))
