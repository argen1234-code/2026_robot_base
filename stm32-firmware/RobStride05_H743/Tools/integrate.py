"""Restore the validated HAL+FreeRTOS startup after CubeMX generation.
Baseline: 2026-10-22 RTOS port, both caches OFF, four application tasks.
The old ROB2 migrator was incompatible with that port and is replaced.
Update cubemx_baseline explicitly when changing generated peripheral/startup
configuration. App/BSP/Devices stay live and are never restored from templates.
"""
from pathlib import Path
import shutil
import re
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
for source in (root/'Tools/cubemx_baseline').rglob('*'):
 if source.is_file():
  target=root/source.relative_to(root/'Tools/cubemx_baseline')
  if target.read_bytes()!=source.read_bytes():shutil.copy2(source,target)
proj=root/'MDK-ARM/RobStride05_H743.uvprojx'
tree=ET.parse(proj)
cpu=tree.find('.//TargetCommonOption/Cpu')
cpu.text=re.sub(r'CLOCK\(\d+\)','CLOCK(480000000)',cpu.text)
groups=tree.find('.//Target/Groups')
hal_files=next(g for g in groups if any('stm32h7xx_hal.c'==f.findtext('FileName') for f in g.findall('Files/File'))).find('Files')
for name in ('stm32h7xx_hal_uart.c','stm32h7xx_hal_uart_ex.c'):
 if not any(f.findtext('FileName')==name for f in hal_files):
  f=ET.SubElement(hal_files,'File')
  ET.SubElement(f,'FileName').text=name
  ET.SubElement(f,'FileType').text='1'
  ET.SubElement(f,'FilePath').text='../Drivers/STM32H7xx_HAL_Driver/Src/'+name
for name in ('App','BSP','Devices'):
 group=next(g for g in groups if g.findtext('GroupName')==name)
 files=group.find('Files')
 files.clear()
 for p in sorted((root/name).glob('*')):
  if p.suffix not in ('.c','.h'):continue
  f=ET.SubElement(files,'File')
  ET.SubElement(f,'FileName').text=p.name
  ET.SubElement(f,'FileType').text='1' if p.suffix=='.c' else '5'
  ET.SubElement(f,'FilePath').text='../'+name+'/'+p.name
ET.indent(tree,space='  ')
tree.write(proj,encoding='utf-8',xml_declaration=True)
options=root/'MDK-ARM/RobStride05_H743.uvoptx'
if options.exists():
 text=options.read_text(encoding='utf-8')
 updated=re.sub(r'<CLKADS>\d+</CLKADS>','<CLKADS>480000000</CLKADS>',text)
 if updated!=text:options.write_text(updated,encoding='utf-8')
source=(root/'Core/Src/main.c').read_text('utf-8')
assert 'SCB_Enable' not in source and 'SCB_Disable' not in source
assert 'AppTasks_Start();' in (root/'Core/Src/freertos.c').read_text('utf-8')
print('Integration OK: validated RTOS startup, caches OFF, dual motor application')
