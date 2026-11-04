from pathlib import Path

root = Path(__file__).resolve().parents[1]
d = {
 'File.Version':'6','KeepUserPlacement':'false','Mcu.CPN':'STM32H743IIT6',
 'Mcu.Family':'STM32H7','Mcu.Name':'STM32H743IITx','Mcu.UserName':'STM32H743IITx',
 'Mcu.Package':'LQFP176','Mcu.ThirdPartyNb':'0','Mcu.UserConstants':'',
 'MxCube.Version':'6.17.0','MxDb.Version':'DB.6.0.170',
 'ProjectManager.ProjectName':'RobStride05_H743','ProjectManager.ProjectFileName':'RobStride05_H743.ioc',
 'ProjectManager.TargetToolchain':'MDK-ARM V5','ProjectManager.CompilerOptimize':'2',
 'ProjectManager.FirmwarePackage':'STM32Cube FW_H7 V1.13.0','ProjectManager.LastFirmware':'false',
 'ProjectManager.LibraryCopy':'1','ProjectManager.KeepUserCode':'true',
 'ProjectManager.MainLocation':'Core/Src','ProjectManager.CoupleFile':'true',
 'ProjectManager.HeapSize':'0x200','ProjectManager.StackSize':'0x1000',
 'ProjectManager.FreePins':'false','ProjectManager.BackupPrevious':'false',
 'ProjectManager.NoMain':'false','ProjectManager.DeletePrevious':'true',
 'ProjectManager.UnderRoot':'false','ProjectManager.ToolChainLocation':'',
 'ProjectManager.DeviceId':'STM32H743IITx','ProjectManager.DefaultFWLocation':'true',
 'NVIC.PriorityGroup':'NVIC_PRIORITYGROUP_4',
 'CORTEX_M7.IPParameters':'CPU_DCache,CPU_ICache',
 'CORTEX_M7.CPU_DCache':'Disabled','CORTEX_M7.CPU_ICache':'Enabled',
 'RCC.HSE_VALUE':'25000000','RCC.PLLSourceVirtual':'RCC_PLLSOURCE_HSE',
 'RCC.DIVM1':'5','RCC.DIVN1':'160','RCC.DIVP1':'4','RCC.DIVQ1':'10','RCC.DIVR1':'4',
 'RCC.SYSCLKSource':'RCC_SYSCLKSOURCE_PLLCLK','RCC.HPRE':'RCC_HCLK_DIV2',
 'RCC.D1PPRE':'RCC_APB3_DIV2','RCC.D2PPRE1':'RCC_APB1_DIV2','RCC.D2PPRE2':'RCC_APB2_DIV2','RCC.D3PPRE':'RCC_APB4_DIV2',
 'RCC.SYSCLKFreq_VALUE':'200000000','RCC.CortexFreq_Value':'200000000','RCC.HCLKFreq_Value':'100000000',
 'RCC.FDCANFreq_Value':'80000000','RCC.FDCANClockSelection':'RCC_FDCANCLKSOURCE_PLL',
 'RCC.I2C123ClockSelection':'RCC_I2C123CLKSOURCE_HSI','RCC.I2C123Freq_Value':'64000000',
 'RCC.PWR_Regulator_Voltage_Scale':'PWR_REGULATOR_VOLTAGE_SCALE1',
 'FDCAN1.FrameFormat':'FDCAN_FRAME_CLASSIC','FDCAN1.Mode':'FDCAN_MODE_NORMAL',
 'FDCAN1.NominalPrescaler':'4','FDCAN1.NominalSyncJumpWidth':'4','FDCAN1.NominalTimeSeg1':'15','FDCAN1.NominalTimeSeg2':'4',
 'FDCAN1.AutoRetransmission':'DISABLE','FDCAN1.ExtFiltersNbr':'1',
 'FDCAN1.RxFifo0ElmtsNbr':'16','FDCAN1.TxFifoQueueElmtsNbr':'3',
 'FDCAN1.TxFifoQueueMode':'FDCAN_TX_FIFO_OPERATION',
 'I2C2.Timing':'0x10707DBC','I2C2.IPParameters':'Timing',
 'FREERTOS.Interface':'CMSIS_V2','FREERTOS.Tasks01':'defaultTask,24,128,StartDefaultTask,Default,NULL,Dynamic,NULL,NULL',
 'FREERTOS.configTOTAL_HEAP_SIZE':'24576','FREERTOS.configCHECK_FOR_STACK_OVERFLOW':'2',
 'FREERTOS.configUSE_MALLOC_FAILED_HOOK':'1',
 'FREERTOS.IPParameters':'Interface,Tasks01,configTOTAL_HEAP_SIZE,configCHECK_FOR_STACK_OVERFLOW,configUSE_MALLOC_FAILED_HOOK',
 'SYS.TimebaseSource':'TIM6','SYS.IPParameters':'TimebaseSource',
 'VP_SYS_VS_tim6.Mode':'TIM6','VP_SYS_VS_tim6.Signal':'SYS_VS_tim6',
 'VP_FREERTOS_VS_CMSIS_V2.Mode':'CMSIS_V2','VP_FREERTOS_VS_CMSIS_V2.Signal':'FREERTOS_VS_CMSIS_V2',
}
ips=['CORTEX_M7','FDCAN1','FREERTOS','I2C2','NVIC','RCC','SYS']
for i,x in enumerate(ips):d[f'Mcu.IP{i}']=x
d['Mcu.IPNb']=str(len(ips))
pins={'PH0-OSC_IN (PH0)':'RCC_OSC_IN','PH1-OSC_OUT (PH1)':'RCC_OSC_OUT',
 'PA13 (JTMS/SWDIO)':'DEBUG_JTMS-SWDIO','PA14 (JTCK/SWCLK)':'DEBUG_JTCK-SWCLK',
 'PB8':'FDCAN1_RX','PB9':'FDCAN1_TX','PB10':'I2C2_SCL','PB11':'I2C2_SDA',
 'VP_SYS_VS_tim6':None,'VP_FREERTOS_VS_CMSIS_V2':None}
for i,(p,s) in enumerate(pins.items()):
 d[f'Mcu.Pin{i}']=p
 if s:
  key=p.replace(' ',r'\ ');d[key+'.Signal']=s;d[key+'.Locked']='true'
  if p.startswith('PH'):d[key+'.Mode']='HSE-External-Oscillator'
  if p in ['PB10','PB11']:d[key+'.Mode']='I2C'
  if p in ['PB8','PB9']:d[key+'.Mode']='Normal'
d['Mcu.PinsNb']=str(len(pins))
for irq,prio in [('FDCAN1_IT0_IRQn',6),('TIM6_DAC_IRQn',4),('SysTick_IRQn',15),('PendSV_IRQn',15),('SVCall_IRQn',0),('NonMaskableInt_IRQn',0),('HardFault_IRQn',0),('MemoryManagement_IRQn',0),('BusFault_IRQn',0),('UsageFault_IRQn',0),('DebugMonitor_IRQn',0)]:
 d['NVIC.'+irq]=rf'true\:{prio}\:0\:false\:false\:true\:true\:true\:false'
for ip in ['RCC','FDCAN1']:
 d[ip+'.IPParameters']=','.join(k.split('.',1)[1] for k in d if k.startswith(ip+'.') and not k.endswith('IPParameters'))
(root/'RobStride05_H743.ioc').write_text('#MicroXplorer Configuration settings - do not modify\n'+'\n'.join(f'{k}={v}' for k,v in sorted(d.items()))+'\n',encoding='utf8')
