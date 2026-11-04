"""Connection UI contract with intercepted APIs; never accesses a probe."""
import argparse
import copy
import json
from pathlib import Path
import sys
from playwright.sync_api import sync_playwright,expect

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from profile import decode
from server import DEFAULTS,LIMITS


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--url',default='http://127.0.0.1:8766')
    ap.add_argument('--output',type=Path,default=ROOT/'review/auto-ready-connection');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    profile=json.loads((ROOT/'firmware_profile.json').read_text('utf-8'))
    snap=decode(profile['schema'],bytes(profile['schema']['size']))
    snap.update(coherent=True,coherent2=True,chassis_coherent=True)
    snap['setup']['dual_ready']=1
    for i,name in enumerate(('motor','motor2')):
        snap[name].update(**DEFAULTS,state=2,mode=2,enabled=1,enable_requested=1,
            configured_id=127-i,online=1,feedback_age_ms=0)
    snap['chassis']['imu_online']=1
    status=dict(connected=False,initializing=False,motors_ready=False,snapshot=snap,
        sample_age_ms=0,motor_count=2,pending_motors=[None,None],setup_pending=None,
        drive_supported=True,chassis_supported=True,led_supported=True,logs=[],probes=[],
        limits=LIMITS,defaults=DEFAULTS,profile={'source':'fixture'},history_count=0)
    sent=[];errors=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='msedge',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1100},reduced_motion='reduce')
        page.on('pageerror',lambda e:errors.append(str(e)))
        def route(r):
            action=r.request.url.rsplit('/',1)[-1]
            if action=='session':r.fulfill(json={'token':'fixture'});return
            if action=='status':r.fulfill(json=status);return
            payload=r.request.post_data_json;sent.append((action,copy.deepcopy(payload)))
            result={}
            if action=='probes':result=[{'id':'fixture','name':'No hardware'}]
            if action=='connect':
                status.update(connected=True,initializing=True,startup_message='正在自动零速使能双电机…')
            r.fulfill(json={'ok':True,'data':result})
        page.route('**/api/*',route);page.goto(args.url)
        page.wait_for_function('() => data && !busy && document.querySelector("#probe").value')
        page.click('#enter-console')
        page.fill('#speed-limit','12');page.fill('#current-limit','3')
        page.click('#select-motor-1');page.fill('#accel-limit','25')
        page.click('#connect');page.wait_for_function('() => data.initializing && !busy')
        payload=next(payload for action,payload in sent if action=='connect')
        assert payload['motor_params']==[
            dict(speed_limit_rad_s=12,current_limit_a=3,accel_rad_s2=20),
            dict(speed_limit_rad_s=33,current_limit_a=2.4,accel_rad_s2=25)]
        expect(page.locator('#run')).to_be_disabled()
        expect(page.locator('#notice-text')).to_contain_text('自动零速使能')
        status.update(initializing=False,motors_ready=True,startup_message='双电机已就绪，可直接运行')
        page.wait_for_function('() => data.motors_ready')
        expect(page.locator('#run')).to_be_enabled()
        page.click('#chassis-panel summary');expect(page.locator('#chassis-angle')).to_be_enabled()
        page.click('#drive-panel summary');expect(page.locator('#drive-enter')).to_be_enabled()
        for selector in ('#prepare','#enable','#confirmed','#setup-details','#chassis-prepare','#chassis-enable'):
            expect(page.locator(selector)).to_have_count(0)
        assert not any(action in ('command','setup','chassis_prepare','chassis_enable') for action,_ in sent)
        assert not errors,errors
        page.locator('#chassis-panel').screenshot(path=str(args.output/'ready-controls.png'))
        report=dict(passed=True,hardware_tested=False,javascript_errors=errors,requests=sent,
            checks=['both per-motor limits submitted on connect','movement blocked during automatic initialization',
                    'ready state enables angle, wheel-speed and WASD controls','manual setup controls absent'])
        (args.output/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        browser.close()
    print('PASS automatic connection UI; intercepted APIs only')


if __name__=='__main__':main()
