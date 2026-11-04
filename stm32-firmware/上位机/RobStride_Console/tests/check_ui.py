"""Production UI smoke check in a clean Edge session, optionally live SWD.
Never checks isolation/prepare/enable/run, or submits any motion command.
"""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright,expect

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8766')
    parser.add_argument('--live',action='store_true')
    args=parser.parse_args()
    out=Path(__file__).resolve().parents[3]/'RobStride05_H743/Docs'
    errors=[]
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path='C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless=True)
        page=browser.new_page(viewport={'width':1500,'height':1100},device_scale_factor=1)
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto(args.url)
        expect(page.locator('#connect')).to_be_enabled(timeout=15000)
        if args.live:
            if not page.request.get(args.url+'/api/status').json()['connected']:
                page.click('#connect')
            expect(page.locator('#connect')).to_contain_text('断开',timeout=40000)
            expect(page.locator('#summary-0')).to_contain_text('目标')
        assert page.locator('#prepare').is_disabled()
        assert page.locator('#enable').is_disabled()
        assert page.locator('#run').is_disabled()
        page.fill('#target','0.7')
        page.click('#select-motor-1')
        assert '电机 2' in page.inner_text('#selected-label')
        assert float(page.input_value('#target'))==1
        page.fill('#target','-0.4')
        page.click('#select-motor-0')
        assert float(page.input_value('#target'))==0.7
        page.click('#select-motor-1')
        assert float(page.input_value('#target'))==-0.4
        page.click('#setup-details summary')
        assert page.locator('#change-id').is_disabled()
        assert page.locator('#verify-pair').is_disabled()
        page.locator('#setup-details summary').click()
        page.screenshot(path=str(out/'dual_motor_console_live.png'),full_page=True)
        status=page.request.get(args.url+'/api/status').json()
        assert not errors,errors
        if args.live:
            assert status['connected'] and status['motor_count']==2
            assert not status['snapshot']['motor']['enable_requested']
            assert not status['snapshot']['motor2']['enable_requested']
            page.click('#connect') # production disconnect requests BOTH disabled
            expect(page.locator('#connect')).to_contain_text('连接设备')
        out.joinpath('dual_motor_ui_check.json').write_text(json.dumps({'passed':True,'live':args.live,'errors':errors,'status':status},ensure_ascii=False,indent=2),encoding='utf-8')
        browser.close()
    print('PASS production UI: live channels, independent editing, movement gates, setup gates, no JavaScript errors')

if __name__=='__main__':main()
