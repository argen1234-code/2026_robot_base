"""Browser WASD regression with intercepted telemetry; never connects a probe."""
import argparse
import json
from pathlib import Path
import sys
from playwright.sync_api import sync_playwright,expect

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from profile import decode
from server import DEFAULTS,LIMITS


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8876')
    parser.add_argument('--output',type=Path,default=ROOT/'review/wasd')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    profile=json.loads((ROOT/'firmware_profile.json').read_text('utf-8'))
    snap=decode(profile['schema'],bytes(profile['schema']['size']))
    snap.update(coherent=True,coherent2=True)
    snap['setup']['dual_ready']=1
    for name in ('motor','motor2'):
        snap[name].update(state=2,online=1,enabled=1,enable_requested=1,feedback_age_ms=0,
                          timeout_motor_supported=1,**DEFAULTS)
    status=dict(connected=True,snapshot=snap,sample_age_ms=0,motor_count=2,
        pending=None,pending_motors=[None,None],setup_pending=None,continuous=False,
        continuous_motors=[False,False],led_supported=True,drive_supported=True,drive_active=False,
        profile={'source':'Browser fixture; no hardware','axf_sha256':'fixture'},
        history_count=0,logs=[],probes=[],defaults=DEFAULTS,limits=LIMITS)
    submitted=[];errors=[];checks=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='msedge',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1100},reduced_motion='reduce')
        page.add_init_script("""(() => {
            const interval = window.setInterval.bind(window);
            window.setInterval = (callback, delay, ...args) => interval(
                () => { if (delay !== 200 || !window.fixtureAcquiring) callback(...args); }, delay);
        })();""")
        page.on('pageerror',lambda e:errors.append(str(e)))
        def route(r):
            action=r.request.url.rsplit('/',1)[-1]
            if action=='status':
                r.fulfill(json=status);return
            if action=='session':r.fulfill(json={'token':'fixture'});return
            payload=r.request.post_data_json
            submitted.append((action,payload))
            result={}
            if action=='drive_enter':status['drive_active']=True;snap['drive']['active']=1;result={'owner':'owner'}
            elif action in ('drive_exit','stop_all'):
                status['drive_active']=False;snap['drive']['active']=0
            elif action=='probes':result=[]
            r.fulfill(json={'ok':True,'data':result})
        page.route('**/api/*',route)
        page.goto(args.url)
        page.wait_for_function('() => data?.connected && !busy')
        page.click('#drive-panel summary')
        expect(page.locator('#drive-left')).to_have_value('1')
        expect(page.locator('#drive-left-sign')).to_have_value('-1')
        expect(page.locator('#drive-right-sign')).to_have_value('1')
        # Already-enabled motors do not require repeating the preparation
        # checkboxes, including a second tab with fresh UI drafts.
        page.click('#select-motor-1')
        page.fill('#drive-speed','33')
        def enter():
            expect(page.locator('#drive-enter')).to_be_enabled()
            # Keep the intercepted pre-acquire telemetry response from
            # arriving after the fixture has granted a new control session.
            page.evaluate("""async () => {
                window.fixtureAcquiring = true;
                while (polling) await new Promise(resolve => setTimeout(resolve, 5));
                document.getElementById('drive-enter').click();
            }""")
            expect(page.locator('#drive-exit')).to_be_enabled()
            page.evaluate('window.fixtureAcquiring = false')
        for state in (2,3,4):
            snap['motor'].update(state=state,velocity_rad_s=0.065,target_rad_s=1,applied_rad_s=0.7,
                                 timeout_motor_supported=0)
            snap['motor2'].update(state=state,velocity_rad_s=-0.08,target_rad_s=-1,applied_rad_s=-0.7,
                                  timeout_motor_supported=0)
            page.wait_for_timeout(250)
            enter()
            payload=next(v for a,v in reversed(submitted) if a=='drive_enter')
            assert payload=={},payload
            page.click('#drive-exit')
            expect(page.locator('#drive-enter')).to_be_enabled()
        checks.append('entry accepts enabled/running/stopping and nonzero feedback without repeating confirmation checkboxes')
        def vector(value):
            for _ in range(40):
                if submitted and any(a=='drive_update' and all(abs(x-y)<1e-6 for x,y in zip(v['targets'],value)) for a,v in submitted):return
                page.wait_for_timeout(25)
            raise AssertionError((value,submitted[-10:]))
        def key(code,wanted):
            submitted.clear();page.keyboard.down(code);vector(wanted)
        enter()
        expect(page.locator('#run')).to_be_disabled()
        key('w',[33,-33]);submitted.clear();page.keyboard.up('w');vector([0,0])
        key('s',[-33,33]);page.keyboard.up('s')
        key('a',[33,33]);page.keyboard.up('a')
        key('d',[-33,-33]);page.keyboard.up('d')
        for forward,turn,wanted in (
            ('w','a',[33,0]),('w','d',[0,-33]),
            ('s','a',[0,33]),('s','d',[-33,0]),
            ('w','s',[0,0]),('a','d',[0,0])):
            page.keyboard.down(forward);key(turn,wanted)
            submitted.clear();page.keyboard.up(turn);page.keyboard.up(forward);vector([0,0])
        checks.append('vehicle default: W/S unchanged; A/D and all four diagonal turns corrected at full 33 rad/s; opposing keys cancel and keyup zeros both')
        key('w',[33,-33]);submitted.clear();page.keyboard.press('Space');vector([0,0])
        page.keyboard.up('w')
        checks.append('Space clears held vector')
        page.click('#open-settings');submitted.clear()
        page.keyboard.press('w');page.wait_for_timeout(220)
        assert not any(a=='drive_update' and any(v['targets']) for a,v in submitted)
        page.click('#close-settings')
        key('w',[33,-33]);submitted.clear();page.evaluate("window.dispatchEvent(new Event('blur'))")
        expect(page.locator('#drive-enter')).to_be_enabled()
        assert any(a=='drive_exit' for a,_ in submitted)
        page.keyboard.up('w')
        checks.append('editing/settings ignore movement keys; window blur releases the session')
        enter();page.click('#tab-led')
        page.wait_for_function('() => !data.drive_active')
        assert submitted[-1][0]=='drive_exit'
        page.click('#tab-control');enter();page.keyboard.press('Escape')
        page.wait_for_function('() => !data.drive_active')
        assert any(a=='stop_all' for a,_ in submitted)
        checks.append('module switch releases; original Esc all-disable stays available')
        page.select_option('#drive-left','0');page.select_option('#drive-left-sign','1');page.select_option('#drive-right-sign','-1')
        enter();key('w',[33,-33]);page.keyboard.up('w')
        key('a',[-33,-33]);page.keyboard.up('a')
        key('d',[33,33]);page.keyboard.up('d')
        page.click('#drive-exit')
        page.select_option('#drive-left','1');page.select_option('#drive-left-sign','-1');page.select_option('#drive-right-sign','1')
        checks.append('left/right motor assignment and mounting polarity are configurable')
        page.locator('#drive-panel').scroll_into_view_if_needed()
        page.screenshot(path=str(args.output/'drive-desktop.png'))
        page.set_viewport_size({'width':390,'height':1000})
        page.locator('#drive-panel').scroll_into_view_if_needed()
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1')
        page.screenshot(path=str(args.output/'drive-mobile.png'))
        assert not errors,errors
        browser.close()
    (args.output/'checks.json').write_text(json.dumps({'passed':True,'checks':checks,'errors':errors,
        'physical_motion_tested':False},ensure_ascii=False,indent=2),encoding='utf-8')
    print('PASS WASD browser:',len(checks),'checks; no hardware connection')


if __name__=='__main__':main()
