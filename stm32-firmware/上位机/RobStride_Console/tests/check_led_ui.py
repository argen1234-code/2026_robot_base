"""Live production UI/firmware acceptance; changes lights only, never enables motors."""
import argparse
import json
import time
from pathlib import Path
from playwright.sync_api import sync_playwright,expect

NAMES=['自定义循环','流动彩虹','双向彗星','极光波浪','火焰','流动等离子','往返扫描','纯色常亮']

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--url',default='http://127.0.0.1:8766')
    ap.add_argument('--prefix',default='led_console');args=ap.parse_args()
    out=Path(__file__).resolve().parents[3]/'RobStride05_H743/Docs'
    errors=[];samples=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1200},device_scale_factor=1)
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(args.url)
        expect(page.locator('#connect')).to_be_enabled(timeout=15000)
        page.click('#connect');expect(page.locator('#connect')).to_contain_text('断开',timeout=40000)
        page.click('[data-view=led]');expect(page.locator('#led-status')).to_contain_text('16 颗')
        def status():return page.request.get(args.url+'/api/status').json()
        def apply():
            page.click('#led-apply');expect(page.locator('#led-sync')).to_have_text('与板上设置一致',timeout=6000)
        def motors_off(s):
            assert s['connected'] and s['led_supported']
            assert all(s['snapshot'][k]['enabled']==0 and s['snapshot'][k]['enable_requested']==0 and s['snapshot'][k]['target_rad_s']==0 for k in ('motor','motor2'))
        try:
            page.select_option('#led-mode','7');page.fill('#led-red','17');page.fill('#led-green','34');page.fill('#led-blue','51')
            page.fill('#led-brightness','32');page.fill('#led-speed','150');apply()
            expect(page.locator('#led-color')).to_have_value('#112233')
            for mode in range(1,8):
                page.select_option('#led-mode',str(mode));apply()
                expect(page.locator('#led-live-mode')).to_have_text(NAMES[mode])
                s=status();motors_off(s);samples.append(s['snapshot']['led'])
            page.click('#led-save');page.fill('#led-brightness','81');page.click('#led-load')
            expect(page.locator('#led-brightness')).to_have_value('32')
            page.click('#led-off');expect(page.locator('#led-live-mode')).to_have_text('已关闭')
            time.sleep(0.2);off=status()['snapshot']['led']['frames_done'];time.sleep(0.25)
            assert status()['snapshot']['led']['frames_done']==off
            page.check('#led-enabled');page.click('#led-all')
            while page.locator('#led-playlist li').count()>1:
                page.locator('#led-playlist button[data-op=remove]').last.click()
            page.select_option('#led-add-mode','7');page.click('#led-add')
            page.locator('#led-playlist button[data-op=up]').last.click()
            page.click('#led-add') # [7,1,7], with a duplicate and custom order
            page.fill('#led-interval','1');page.fill('#led-speed','400');apply()
            transitions=[];deadline=time.monotonic()+3.4
            while time.monotonic()<deadline:
                s=status();motors_off(s);l=s['snapshot']['led'];samples.append(l)
                assert l['cycle_modes'][:3]==[7,1,7] and l['mode']==0
                assert l['active_mode']==[7,1,7][l['cycle_index']]
                if not transitions or transitions[-1]!=l['cycle_index']:transitions.append(l['cycle_index'])
                time.sleep(0.12)
            assert transitions[:4]==[0,1,2,0],transitions
            page.click('#led-save');page.reload()
            page.click('[data-view=led]');expect(page.locator('#led-status')).to_contain_text('16 颗')
            page.click('#led-load');expect(page.locator('#led-playlist li')).to_have_count(3)
            before=status();page.click('#connect');expect(page.locator('#connect')).to_contain_text('连接设备')
            time.sleep(1.2);page.click('#connect');expect(page.locator('#connect')).to_contain_text('断开',timeout=40000)
            after=status();motors_off(after)
            assert after['snapshot']['led']['frames_done']>before['snapshot']['led']['frames_done']
            assert after['snapshot']['led']['cycle_modes'][:3]==[7,1,7]
            page.click('#led-read');expect(page.locator('#led-sync')).to_have_text('与板上设置一致')
            page.screenshot(path=str(out/(args.prefix+'.png')),full_page=True)
            # Return to the six-effect default auto program for delivery.
            page.click('#led-default');apply();final=status();motors_off(final)
            assert final['snapshot']['led']['mode']==0 and final['snapshot']['led']['errors']==0
            assert not errors,errors
            out.joinpath(args.prefix+'.json').write_text(json.dumps({'passed':True,'errors':errors,'transitions':transitions,'samples':samples,'final':final},ensure_ascii=False,indent=2),encoding='utf-8')
            # Check narrow-screen layout as well; do not send new light commands.
            page.set_viewport_size({'width':730,'height':1100})
            page.screenshot(path=str(out/(args.prefix+'_narrow.png')),full_page=True)
        finally:
            if status()['connected']:
                page.click('#connect');expect(page.locator('#connect')).to_contain_text('连接设备')
            browser.close()
    print('PASS live UI: all 7 effects, RGB, brightness/speed, off/resume, custom ordered repeated playlist, local save/load, loop across page reload and disconnect; motors remain disabled; no JS errors')

if __name__=='__main__':main()
