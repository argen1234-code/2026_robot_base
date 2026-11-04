"""Read-only clock/timebase/WS2812 verification on the running Rev.V board.
Peripheral addresses come from STM32H743 CMSIS; RAM addresses from this AXF.
Run with the console disconnected and Keil Debug closed.
"""
import json
from pathlib import Path
import struct
import time
from elftools.elf.elffile import ELFFile
import flash_and_verify as f

def main():
    fw=f.load_axf(f.DEFAULT_AXF)
    with fw['path'].open('rb') as stream:
        elf=ELFFile(stream)
        symbols={s.name:(s['st_value'],s['st_size']) for s in elf.get_section_by_name('.symtab').iter_symbols()}
    addresses={
        'idcode':0x5C001000,'pllckselr':0x58024428,'pll1divr':0x58024430,
        'd1cfgr':0x58024418,'d2cfgr':0x5802441C,'d3cfgr':0x58024420,
        'd2ccip1r':0x58024450,'d2ccip2r':0x58024454,'cfgr':0x58024410,
        'pwr_d3cr':0x58024818,'pwr_cr3':0x5802480C,'syscfg_pwrcr':0x5800042C,
        'flash_acr':0x52002000,'systick_load':0xE000E014,'ccr':0xE000ED14,
        'tim2_psc':0x40000028,'tim3_psc':0x40000428,'tim3_arr':0x4000042C,
        'tim6_psc':0x40001028,'tim6_arr':0x4000102C,
        'i2c2_timing':0x40005810,'fdcan_nbtp':0x4000A01C}
    session=f.session_open(f.pick_probe()['id'])
    try:
        session.open();target=session.target
        for seg in fw['segments']:
            assert bytes(target.read_memory_block8(seg['address'],len(seg['data'])))==seg['data'],'Flash mismatch'
        regs={k:target.read32(a) for k,a in addresses.items()}
        n=1+(regs['pll1divr']&511);p=1+((regs['pll1divr']>>9)&127);q=1+((regs['pll1divr']>>16)&127)
        m=(regs['pllckselr']>>4)&63;vco=25000000//m*n
        clocks={'cpu_hz':vco//p,'hclk_hz':vco//p//2,'pclk_hz':vco//p//4,'fdcan_hz':vco//q,
                'SystemCoreClock':target.read32(symbols['SystemCoreClock'][0])}
        checks={'rev_v':regs['idcode']>>16==0x2003,'hse_pll':regs['pllckselr']&3==2,
            'clock_dividers':(regs['d1cfgr'],regs['d2cfgr'],regs['d3cfgr'])==(0x48,0x440,0x40),
            'maximum_clocks':clocks=={'cpu_hz':480000000,'hclk_hz':240000000,'pclk_hz':120000000,'fdcan_hz':80000000,'SystemCoreClock':480000000},
            'ldo_vos0_ready':bool(regs['pwr_cr3']&2) and regs['pwr_d3cr']&0xE000==0xE000 and bool(regs['syscfg_pwrcr']&1),
            'flash_latency4':regs['flash_acr']&15==4,'caches_off':regs['ccr']&0x30000==0,
            'rtos_1khz':regs['systick_load']==479999,'wall_time_1mhz':regs['tim2_psc']==239,
            'hal_time_1khz':(regs['tim6_psc'],regs['tim6_arr'])==(239,999),
            'ws2812_800khz':(regs['tim3_psc'],regs['tim3_arr'])==(0,299) and regs['cfgr']&(1<<15)==0,
            'i2c_unchanged':regs['i2c2_timing']==0x10707DBC and (regs['d2ccip2r']>>12)&3==2,
            'fdcan_1mbps':(regs['d2ccip1r']>>28)&3==1 and ((regs['fdcan_nbtp']>>16)&511)+1==4 and ((regs['fdcan_nbtp']>>8)&255)+1==15 and (regs['fdcan_nbtp']&127)+1==4}
        addr,size=symbols['waveform'];words=struct.unpack('<'+'H'*(size//2),bytes(target.read_memory_block8(addr,size)))
        checks['all16_pixel_waveform']=len(words)==708 and set(words[2:386])<={77,204} and 77 in words and 204 in words and not any(words[386:])
        def sample():
            return f.decode(fw['type'],bytes(target.read_memory_block8(fw['address'],fw['size'])))
        begin=time.monotonic();first=sample();time.sleep(1.2);last=sample();elapsed=time.monotonic()-begin
        tick_ms=(last['system']['tick_ms']-first['system']['tick_ms'])&0xFFFFFFFF
        checks['real_time_tick']=abs(tick_ms-elapsed*1000)<80
        checks['tasks_advance']=all(last['system'][k]>first['system'][k] for k in ('loop_heartbeat','health_heartbeat')) and last['oled']['heartbeat']>first['oled']['heartbeat']
        checks['both_online_disabled']=all(last[k]['online'] and not last[k]['enabled'] and not last[k]['enable_requested'] for k in ('motor','motor2'))
        checks['can_oled_clean']=all(last['motor']['can'][k]==0 for k in ('errors','bus_off','rx_drop','tx_fail')) and last['oled']['refresh_fail']==0
        result={'passed':all(checks.values()),'checks':checks,'clocks':clocks,'registers':{k:hex(v) for k,v in regs.items()},'tick_ms':tick_ms,'host_elapsed_ms':elapsed*1000,'axf_sha256':fw['sha256']}
        Path('Docs/clock480_registers.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps(result,indent=2));assert result['passed'],'Clock verification failed'
    finally:session.close()

if __name__=='__main__':main()
