"""Browser PID integration with intercepted telemetry; no probe or physical motion."""
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
    parser=argparse.ArgumentParser();parser.add_argument('--url',default='http://127.0.0.1:29843')
    parser.add_argument('--output',type=Path,default=ROOT/'review/chassis');args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    profile=json.loads((ROOT/'firmware_profile.json').read_text('utf-8'))
    snap=decode(profile['schema'],bytes(profile['schema']['size']))
    cfg=dict(left_slot=1,left_sign=-1,right_sign=1,yaw_sign=1,target_deg=0,left_rad_s=0,right_rad_s=0,
        turn_limit_rad_s=2,angle_kp=.04,angle_ki=0,angle_kd=.005)
    snap.update(coherent=True,coherent2=True,chassis_coherent=True)
    snap['setup']['dual_ready']=1
    for name,ident in (('motor',127),('motor2',126)):snap[name].update(configured_id=ident,state=2,mode=2,enabled=1,enable_requested=1,online=1,feedback_age_ms=0,timeout_motor_supported=1,**DEFAULTS)
    c=snap['chassis'];c.update(imu_online=1,config=copy.deepcopy(cfg),yaw_deg=0)
    c['imu'].update(ready=1,angle_frames=100,gyro_frames=100)
    status=dict(connected=True,initializing=False,motors_ready=True,snapshot=snap,sample_age_ms=0,motor_count=2,pending=None,
        pending_motors=[None,None],setup_pending=None,continuous=False,continuous_motors=[False,False],
        led_supported=True,drive_supported=True,chassis_supported=True,drive_active=False,
        profile={'source':'Browser fixture; no hardware','axf_sha256':'fixture'},history_count=0,
        logs=[],probes=[],defaults=DEFAULTS,limits=LIMITS)
    sent=[];errors=[];checks=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='msedge',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1100},reduced_motion='reduce')
        page.on('pageerror',lambda e:errors.append(str(e)))
        def route(r):
            action=r.request.url.rsplit('/',1)[-1]
            if action=='session':r.fulfill(json={'token':'fixture'});return
            if action=='status':
                snap['system']['tick_ms']+=20;c['published_tick_ms']=snap['system']['tick_ms']
                c['imu']['angle_tick_ms']=c['published_tick_ms'];c['imu']['angle_frames']+=1
                r.fulfill(json=status);return
            payload=r.request.post_data_json;sent.append((action,payload));result={}
            if action=='chassis_prepare':
                for name in ('motor','motor2'):snap[name].update(state=1,mode=2)
            if action=='chassis_enable':
                for name in ('motor','motor2'):snap[name].update(state=2,mode=2,enabled=1,enable_requested=1)
            if action=='chassis':
                cfg.update(payload['config']);c['config']=copy.deepcopy(cfg);c['executed_seq']+=2
                command=payload['command'];c['target_deg']=cfg['target_deg']
                if command in (2,3):c.update(active=1,mode=2 if command==3 else 1)
                if command==4:c.update(active=0,mode=0)
                if command==5:c.update(yaw_deg=0)
            if action=='drive_enter':status['drive_active']=True;snap['drive']['active']=1;result={'owner':'fixture-owner'}
            if action=='drive_exit':status['drive_active']=False;snap['drive']['active']=0
            if action=='probes':result=[]
            r.fulfill(json={'ok':True,'data':result})
        page.route('**/api/*',route);page.goto(args.url)
        page.wait_for_function('() => data?.connected && !busy')
        page.click('#chassis-panel summary')
        for selector in ('#chassis-prepare','#chassis-enable','#confirmed','#setup-details'):
            expect(page.locator(selector)).to_have_count(0)
        expect(page.locator('#chassis-speed')).to_be_enabled()
        expect(page.locator('#run')).to_be_enabled()
        expect(page.locator('.chassis-gain-group')).to_have_count(1)
        expect(page.locator('#chassis-left_kp')).to_have_count(0)
        expect(page.locator('#chassis-right_kp')).to_have_count(0)
        checks.append('connection-ready motors allow direct heading, speed and independent control without manual setup')
        for angle in (90,0,-90,360):
            page.fill('#chassis-target_deg',str(angle));page.click('#chassis-angle')
            page.wait_for_function('(angle) => data.snapshot.chassis.target_deg===angle',arg=angle)
            cmd=next(payload for action,payload in reversed(sent) if action=='chassis' and payload['command']==3)
            assert cmd['config']['target_deg']==angle
            assert (cmd['config']['left_slot'],cmd['config']['left_sign'],cmd['config']['right_sign'])==(1,-1,1)
            expect(page.locator('#run')).to_be_disabled()
        checks.append('absolute +90/0/-90/360 commands preserve screenshot wheel assignment and mounting polarity')
        page.fill('#chassis-angle_kp','0.05');page.click('#chassis-apply')
        page.wait_for_function('() => data.snapshot.chassis.config.angle_kp===0.05')
        assert c['active'];checks.append('live PID changes preserve active heading control')
        page.click('#chassis-stop');expect(page.locator('#chassis-zero')).to_be_enabled()
        page.click('#chassis-zero')
        page.fill('#chassis-left_rad_s','1');page.fill('#chassis-right_rad_s','1');page.click('#chassis-speed')
        command=next(v for a,v in reversed(sent) if a=='chassis' and v['command']==2)
        assert command['config']['left_rad_s']==command['config']['right_rad_s']==1
        page.click('#chassis-stop')
        checks.append('paired forward-positive speed targets, stop, and explicit zero are connected')
        page.click('#drive-panel summary')
        page.select_option('#drive-left','0');page.select_option('#drive-left-sign','1');page.select_option('#drive-right-sign','-1')
        page.click('#drive-enter');expect(page.locator('#drive-exit')).to_be_enabled()
        command=next(v for a,v in reversed(sent) if a=='chassis' and v['command']==1)
        assert command['config']=={'left_slot':0,'left_sign':1,'right_sign':-1}
        page.keyboard.down('w');page.wait_for_timeout(250);page.keyboard.up('w')
        assert any(a=='drive_update' and v['targets']==[1,-1] for a,v in sent)
        page.click('#drive-exit');checks.append('WASD sends changed wheel mapping into the same chassis configuration')
        page.select_option('#drive-left','1');page.select_option('#drive-left-sign','-1');page.select_option('#drive-right-sign','1')
        page.fill('#chassis-target_deg','90');page.click('#chassis-apply')
        c.update(yaw_deg=32,target_deg=90,angle_error_deg=58,turn_rad_s=1.5,
            left_target_rad_s=1,right_target_rad_s=1,left_speed_rad_s=.95,right_speed_rad_s=.98,left_iq_a=.4,right_iq_a=.42)
        page.wait_for_timeout(400);expect(page.locator('#chassis-yaw')).to_contain_text('32.00')
        page.fill('#chassis-angle_kp','');page.click('#chassis-stop')
        assert next(v for a,v in reversed(sent) if a=='chassis')['command']==4
        page.fill('#chassis-angle_kp','0.05')
        expect(page.locator('#chassis-empty')).to_be_hidden()
        for metric in ('angle','left','right','current'):page.select_option('#chassis-metric',metric)
        page.select_option('#chassis-metric','angle')
        page.locator('#chassis-panel').screenshot(path=str(args.output/'chassis-desktop.png'))
        page.set_viewport_size({'width':390,'height':1000});page.wait_for_timeout(250)
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
        assert page.locator('#chassis-target_deg').bounding_box()['width']>200
        page.locator('#chassis-panel').screenshot(path=str(args.output/'chassis-mobile.png'))
        checks.append('telemetry, four chart metrics, desktop/mobile layout render without horizontal overflow')
        assert not errors,errors;browser.close()
    (args.output/'checks.json').write_text(json.dumps(dict(passed=True,checks=checks,errors=errors,
        physical_motion_tested=False),ensure_ascii=False,indent=2),encoding='utf-8')
    print('PASS chassis browser:',len(checks),'checks; no physical motion')

if __name__=='__main__':main()
