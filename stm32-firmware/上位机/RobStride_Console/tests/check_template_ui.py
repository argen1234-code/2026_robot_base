"""Browser regression for the template UI. Never opens a hardware connection.

Real server smoke checks use disconnected status; connected controls are tested
with browser-intercepted API fixtures decoded from the checked-in AXF schema.
"""
import argparse
import copy
import json
from pathlib import Path
import sys

from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from profile import decode
from server import DEFAULTS

MODULES = ['control', 'telemetry', 'diagnostics', 'led', 'guide']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://127.0.0.1:8876')
    ap.add_argument('--output', type=Path, default=ROOT / 'review')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    errors = []
    checks = []
    with sync_playwright() as p:
        browser = p.chromium.launch(channel='msedge', headless=True)
        context = browser.new_context(viewport={'width':1500, 'height':1050}, reduced_motion='reduce')
        page = context.new_page()
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto(args.url)
        page.wait_for_function('() => !!data && !busy',timeout=30000)
        expect(page.locator('#slide-control')).to_have_class('monitor-slide is-active')
        for selector in ('#prepare','#enable','#confirmed','#setup-details','#chassis-prepare','#chassis-enable'):
            expect(page.locator(selector)).to_have_count(0)
        expect(page.locator('#run')).to_be_disabled()
        expect(page.locator('#module-previous')).to_be_disabled()
        page.screenshot(path=str(args.output / '01-home-desktop.png'))
        page.click('#enter-console')
        page.locator('.monitor-navigation').scroll_into_view_if_needed()
        page.evaluate('window.scrollTo(0,0)')
        page.screenshot(path=str(args.output / '02-control-desktop.png'), full_page=True)
        page.fill('#target', '0.7')
        page.click('#select-motor-1')
        expect(page.locator('#target')).to_have_value('1')
        page.fill('#target', '-0.4')
        page.click('#select-motor-0')
        expect(page.locator('#target')).to_have_value('0.7')
        page.click('#select-motor-1')
        expect(page.locator('#target')).to_have_value('-0.4')
        checks.append('independent motor drafts and offline movement gates')
        # Unit toggles edit the same draft, never submit a motor request.
        page.click('[data-unit=rpm]')
        assert abs(float(page.input_value('#target')) - (-.4 * 30 / 3.141592653589793)) < .01
        page.click('[data-unit=rad]')
        for name in MODULES:
            page.click('#tab-' + name)
            expect(page.locator('#tab-' + name)).to_have_attribute('aria-selected', 'true')
            assert page.locator('#slide-' + name + ' .module-window').evaluate('(e)=>!e.inert')
            for other in MODULES:
                if other != name:
                    assert page.locator('#slide-' + other + ' .module-window').evaluate('(e)=>e.inert')
        expect(page.locator('#module-next')).to_be_disabled()
        page.locator('#tab-guide').focus()
        page.keyboard.press('Home')
        expect(page.locator('#tab-control')).to_be_focused()
        page.keyboard.press('ArrowRight')
        expect(page.locator('#tab-telemetry')).to_be_focused()
        page.keyboard.press('End')
        expect(page.locator('#tab-guide')).to_be_focused()
        page.click('#tab-control')
        page.locator('#target').focus()
        page.keyboard.press('ArrowRight')
        expect(page.locator('#tab-control')).to_have_attribute('aria-selected','true')
        checks.append('all modules, inactive controls inert, bounded arrows and keyboard navigation')
        page.click('#tab-led')
        page.select_option('#led-mode','7')
        page.fill('#led-red','17')
        page.fill('#led-green','34')
        page.fill('#led-blue','51')
        expect(page.locator('#led-color')).to_have_value('#112233')
        page.click('#led-all')
        expect(page.locator('#led-playlist li')).to_have_count(7)
        page.locator('#led-playlist [data-op=remove]').last.click()
        page.select_option('#led-add-mode','7')
        page.click('#led-add')
        page.locator('#led-playlist [data-op=up]').last.click()
        page.click('#led-save')
        page.fill('#led-brightness','81')
        page.click('#led-load')
        expect(page.locator('#led-brightness')).to_have_value('64')
        expect(page.locator('#led-apply')).to_be_disabled()
        page.evaluate('window.scrollTo(0,0)')
        page.screenshot(path=str(args.output / '03-led-desktop.png'),full_page=True)
        checks.append('LED RGB, ordered playlist, local save/load, offline apply gate')
        page.click('#open-settings')
        expect(page.locator('#close-settings')).to_be_focused()
        page.keyboard.press('Shift+Tab')
        expect(page.locator('#reset-settings')).to_be_focused()
        page.keyboard.press('Tab')
        expect(page.locator('#close-settings')).to_be_focused()
        page.uncheck('#motion-enabled')
        page.check('#compact-layout')
        page.uncheck('#show-logs')
        page.locator('#accent-color').fill('#72d4ff')
        page.screenshot(path=str(args.output / '04-settings-desktop.png'))
        page.keyboard.press('Escape')
        expect(page.locator('#settings-drawer')).to_be_hidden()
        expect(page.locator('#open-settings')).to_be_focused()
        page.reload()
        page.wait_for_function('() => !!data && !busy',timeout=30000)
        assert page.evaluate("getComputedStyle(document.documentElement).getPropertyValue('--accent').trim()") == '#72d4ff'
        expect(page.locator('#log-section')).to_be_hidden()
        page.click('#open-settings')
        expect(page.locator('#compact-layout')).to_be_checked()
        expect(page.locator('#motion-enabled')).not_to_be_checked()
        page.click('#reset-settings')
        page.click('#close-settings')
        checks.append('settings modal focus trap, Escape, persistence and reset')
        for width in [1920,1280,768,390]:
            page.set_viewport_size({'width':width,'height':1000})
            for name in MODULES:
                page.click('#tab-' + name)
                page.wait_for_timeout(80)
                assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1'), (width,name,'page overflow')
                box = page.locator('#slide-'+name+' .module-window').bounding_box()
                assert box['x'] >= -1 and box['x']+box['width'] <= width+1, (width,name,box)
            if width in (768,390):
                page.click('#tab-control')
                page.evaluate('window.scrollTo(0,0)')
                page.screenshot(path=str(args.output / f'05-control-{width}.png'),full_page=True)
        checks.append('1920 / 1280 / 768 / 390 layouts without horizontal overflow')
        caption=page.locator('#slide-control .slide-caption')
        caption.dispatch_event('touchstart',{'touches':[{'identifier':0,'clientX':280,'clientY':30}]})
        caption.dispatch_event('touchend',{'changedTouches':[{'identifier':0,'clientX':80,'clientY':35}]})
        expect(page.locator('#tab-telemetry')).to_have_attribute('aria-selected','true')
        checks.append('touch swipe on card caption')
        page.set_viewport_size({'width':1500,'height':1050})
        page.emulate_media(reduced_motion='no-preference')
        page.evaluate('window.scrollTo(0,0)')
        page.wait_for_timeout(200)
        field1=page.locator('#pixelField').evaluate('(e)=>e.toDataURL()')
        page.wait_for_timeout(200)
        field2=page.locator('#pixelField').evaluate('(e)=>e.toDataURL()')
        assert field1 != field2, 'pixel field must animate when enabled'
        page.click('#tab-control')
        page.wait_for_timeout(700)
        page.locator('#slide-control').screenshot(path=str(args.output/'07-control-panel.png'))
        page.click('#tab-telemetry')
        page.wait_for_timeout(700)
        page.locator('#slide-telemetry').screenshot(path=str(args.output/'08-telemetry-panel.png'))
        checks.append('normal animation mode, live pixel animation and carousel positioning')
        page.emulate_media(reduced_motion='reduce')
        # Everything below uses intercepted HTTP; no production controller command
        # or connection can reach a probe, including the Escape stop-all check.
        page.set_viewport_size({'width':1500,'height':1050})
        status = page.request.get(args.url+'/api/status').json()
        profile = json.loads((ROOT/'firmware_profile.json').read_text('utf-8'))
        snap = decode(profile['schema'], bytes(profile['schema']['size']))
        snap['led'] = decode(profile['led']['schema'],bytes(profile['led']['schema']['size']))
        snap.update(coherent=True,coherent2=True)
        snap['setup']['dual_ready'] = 1
        for index,key in enumerate(('motor','motor2')):
            snap[key].update(**DEFAULTS)
            snap[key].update(state=2,mode=2,enabled=1,enable_requested=1,online=1,configured_id=127-index,feedback_age_ms=1,
                             timeout_motor_supported=1,current_age_ms=1,voltage_age_ms=1,
                             velocity_rad_s=.5,position_rad=.2,torque_nm=.1,temperature_c=25,bus_voltage_v=24)
        snap['led'].update(enabled=1,mode=7,active_mode=7,brightness=64,speed_percent=100,color_order=0,
                           red=17,green=34,blue=51,cycle_interval_ms=8000,cycle_modes=[1,2,3,4,5,6,0,0],
                           cycle_count=6,ready=1,led_count=16)
        status.update(connected=True,initializing=False,motors_ready=True,snapshot=snap,sample_age_ms=1,motor_count=2,led_supported=True,
                      pending=None,pending_motors=[None,None],continuous=False,continuous_motors=[False,False],
                      setup_pending=None,history_count=5)
        status['profile']['led_supported']=True
        submitted=[]
        def api(route):
            action=route.request.url.rsplit('/',1)[-1]
            if action=='status':
                snap['system']['tick_ms']+=20
                route.fulfill(json=status)
            elif action=='session':route.fulfill(json={'token':'browser-fixture'})
            elif action=='export':route.fulfill(body='motor,motor_id,velocity_rad_s\n0,127,0.5\n',content_type='text/csv')
            elif route.request.method=='POST':
                payload=route.request.post_data_json
                submitted.append((action,copy.deepcopy(payload)))
                if action=='command':
                    m=snap['motor' if payload.get('motor',0)==0 else 'motor2']
                    m['state']={1:1,2:2,3:3,4:2,5:0,6:0}[payload['command']]
                if action=='stop_all':
                    for key in ('motor','motor2'):snap[key]['state']=0
                result={'seq':2}
                if action=='probes':result=[{'uid':'fixture','description':'Browser fixture — no hardware'}]
                route.fulfill(json={'ok':True,'data':result})
            else:raise AssertionError(action)
        page.route('**/api/*',api)
        page.reload()
        page.wait_for_function('() => !!data && !busy',timeout=30000)
        page.click('#enter-console')
        for button,command in [('run',3),('stop',4),('disable',5)]:
            expect(page.locator('#'+button)).to_be_enabled(timeout=10000)
            page.click('#'+button)
            page.wait_for_timeout(250)
            assert any(a=='command' and payload['command']==command and payload.get('motor',0)==0 for a,payload in submitted), (button,submitted)
        page.click('#select-motor-1')
        page.click('#run')
        page.wait_for_timeout(250)
        assert submitted[-1][0]=='command' and submitted[-1][1]['motor']==1
        page.click('#open-settings')
        page.keyboard.press('Escape')
        page.wait_for_timeout(250)
        assert submitted[-1][0]=='stop_all',submitted[-1]
        page.click('#tab-diagnostics')
        page.click('#clear-fault')
        page.wait_for_timeout(250)
        assert submitted[-1][1]['command']==6 and submitted[-1][1]['motor']==1
        page.click('#tab-telemetry')
        page.click('#pause-chart')
        expect(page.locator('#pause-chart')).to_have_attribute('aria-label','继续曲线')
        with page.expect_download() as download:
            page.click('#export')
        assert download.value.suggested_filename.endswith('.csv')
        page.click('#tab-led')
        expect(page.locator('#led-status')).to_contain_text('16 颗')
        page.click('#led-apply')
        page.wait_for_timeout(250)
        assert submitted[-1][0]=='led',submitted[-1]
        page.screenshot(path=str(args.output / '06-fixture-led.png'),full_page=True)
        checks.append('HTTP fixtures: connection-ready run / stop / disable / clear, motor routing, Escape in drawer, CSV and LED')
        checks.append('no real hardware connection or command submitted')
        from check_effects_ui import check_effects
        checks.extend(check_effects(page, context, args.output, status, snap, submitted))
        assert not errors, errors
        report={'passed':True,'hardware_tested':False,'javascript_errors':errors,'checks':checks,'fixture_requests':submitted}
        (args.output/'ui-regression.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        context.close()
        browser.close()
    print(json.dumps({'passed':True,'checks':checks},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
