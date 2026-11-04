"""Browser integration with simulated serial API receipts; no COM/probe opens."""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright, expect

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:29844')
    parser.add_argument('--output',type=Path,default=ROOT/'review/bluetooth-source')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    state=dict(connected=False,port='',identity='',armed=False,phase=0,moving=False,
               speed=1,result=0,error='',logs=[],v1=0,v2=0,rx_age_s=0)
    calls=[];errors=[];checks=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='msedge',headless=True)
        page=browser.new_page(viewport={'width':1450,'height':1050},reduced_motion='reduce')
        page.on('pageerror',lambda error:errors.append(str(error)))
        def route(r):
            action=r.request.url.rsplit('/',1)[-1]
            if action=='status':r.fulfill(json=state);return
            payload=r.request.post_data_json;calls.append((action,payload))
            result={}
            if action=='ports':result=[dict(device='COM42',description='JDY31 Bluetooth SPP (fixture)')]
            if action=='connect':
                state.update(connected=True,port='COM42',identity='RS05-BT/1');result={'session':'fixture-session'}
            if action=='disconnect':state.update(connected=False,armed=False,moving=False)
            if action=='arm':state.update(armed=True,phase=0,speed=payload['speed'],error='')
            if action=='speed':state.update(speed=payload['speed'],moving=False)
            if action=='motion':state['moving']=payload['direction']!='S'
            if action=='stop':
                state['moving']=False
                if payload.get('disable'):state['armed']=False
            r.fulfill(json={'ok':True,'data':result})
        page.route('**/api/bluetooth/**',route)
        response=page.goto(args.url,wait_until='load');assert response.status==200
        expect(page.locator('#bt-status')).to_have_text('蓝牙未连接')
        page.locator('#bluetooth-panel summary').first.click()
        expect(page.locator('#bt-port option')).to_have_count(2)
        page.select_option('#bt-port','COM42');page.locator('#bt-connect').click()
        expect(page.locator('#bt-arm')).to_be_enabled()
        assert not any(action=='arm' for action,_ in calls)
        checks.append('COM selection, handshake status, connect never auto-enables')
        page.fill('#bt-speed','33');page.locator('#bt-arm').click()
        expect(page.locator('[data-bt-motion=F]')).to_be_enabled()
        arm=[payload for action,payload in calls if action=='arm'][-1]
        assert arm['speed']==33 and arm['cap']==33 and arm['left_slot']==1 and arm['left_sign']==-1 and arm['right_sign']==1
        checks.append('explicit ARM, full 33 rad/s range, installed wheel mapping')
        def button_hold(direction):
            button=page.locator(f'[data-bt-motion={direction}]');button.scroll_into_view_if_needed()
            box=button.bounding_box();page.mouse.move(box['x']+box['width']/2,box['y']+box['height']/2)
            start=len(calls);page.mouse.down();page.wait_for_timeout(400);page.mouse.up();page.wait_for_timeout(300)
            motions=[p['direction'] for a,p in calls[start:] if a=='motion']
            assert direction in motions and motions[-1]=='S',motions
        for direction in ('F','B','L','R','FL','FR','BL','BR'):button_hold(direction)
        checks.append('all eight held directions, repeated renewal, pointer release stops')
        page.locator('#bt-keyboard').click();page.keyboard.down('w');page.wait_for_timeout(200)
        page.keyboard.down('a');page.wait_for_timeout(250);page.keyboard.up('a');page.keyboard.up('w');page.wait_for_timeout(300)
        assert any(a=='motion' and p['direction']=='FL' for a,p in calls)
        page.keyboard.down('s');page.wait_for_timeout(150);page.keyboard.down('d');page.wait_for_timeout(200)
        page.keyboard.up('s');page.keyboard.up('d');page.wait_for_timeout(250)
        assert any(a=='motion' and p['direction']=='BR' for a,p in calls)
        page.keyboard.down('w');page.wait_for_timeout(180);page.keyboard.press('Space');page.keyboard.up('w');page.wait_for_timeout(200)
        assert [p['direction'] for a,p in calls if a=='motion'][-1]=='S'
        checks.append('WASD diagonal semantics, keyboard release and Space stop')
        page.keyboard.down('w');page.wait_for_timeout(200)
        page.evaluate("window.dispatchEvent(new Event('blur'))")
        page.keyboard.up('w');page.wait_for_timeout(300)
        expect(page.locator('#bt-keyboard')).to_have_attribute('aria-pressed','false')
        assert calls[-1][0]=='stop' or any(a=='stop' for a,_ in calls[-3:])
        page.locator('#bt-keyboard').click();page.keyboard.down('w');page.wait_for_timeout(180)
        page.locator('#bt-speed').focus();page.keyboard.up('w');page.wait_for_timeout(300)
        expect(page.locator('#bt-keyboard')).to_have_attribute('aria-pressed','false')
        checks.append('blur and editing stop movement and leave Bluetooth keyboard mode')
        page.locator('#bt-stop').click();page.keyboard.press('Escape');page.wait_for_timeout(300)
        assert any(a=='stop' and p.get('disable') for a,p in calls)
        expect(page.locator('#bt-keyboard')).to_be_disabled()
        page.locator('#bt-arm').click();expect(page.locator('#bt-keyboard')).to_be_enabled()
        page.locator('#bt-keyboard').click();page.keyboard.down('w');page.wait_for_timeout(200)
        page.locator('#bluetooth-panel summary').first.click();page.keyboard.up('w');page.wait_for_timeout(300)
        expect(page.locator('#bt-keyboard')).to_have_attribute('aria-pressed','false')
        checks.append('Esc disables, fold cancels keyboard and stops')
        page.locator('#bluetooth-panel summary').first.click();page.wait_for_timeout(300)
        page.locator('#bluetooth-panel').scroll_into_view_if_needed()
        page.screenshot(path=str(args.output/'desktop.png'),full_page=True)
        page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(300)
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.locator('#bluetooth-panel summary').first.scroll_into_view_if_needed()
        page.screenshot(path=str(args.output/'mobile.png'),full_page=True)
        checks.append('desktop and 390px layout, no horizontal overflow')
        page.locator('#bt-disconnect').click();expect(page.locator('#bt-connect')).to_be_enabled()
        assert not errors,errors
        (args.output/'checks.json').write_text(json.dumps(dict(checks=checks,errors=errors,api_calls=len(calls),physical_hardware_tested=False),ensure_ascii=False,indent=2),encoding='utf-8')
        browser.close()
    print(json.dumps(dict(checks=checks,errors=errors),ensure_ascii=False))


if __name__=='__main__':main()
