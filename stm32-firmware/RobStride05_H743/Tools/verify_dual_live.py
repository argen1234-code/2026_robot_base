"""Bounded physical commissioning through the PRODUCTION console HTTP API.
Requires --motion-confirmed and operator-secured, unloaded motors.
Uses exactly the same prepare/enable/run/stop/disable path as the browser.
"""
import argparse
import json
from pathlib import Path
import time
import urllib.request

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8765')
    parser.add_argument('--motion-confirmed',action='store_true',required=True)
    args=parser.parse_args()
    url=args.url
    def get(path):return json.load(urllib.request.urlopen(url+path,timeout=10))
    token=get('/api/session')['token']
    def post(action,payload):
        request=urllib.request.Request(url+'/api/'+action,data=json.dumps(payload).encode(),headers={
            'Content-Type':'application/json','Origin':url,'X-Console-Token':token})
        try:return json.load(urllib.request.urlopen(request,timeout=40))['data']
        except urllib.error.HTTPError as exc:raise RuntimeError(exc.read().decode()) from exc
    log=[]
    record={'operator_motion_confirmed':True,'complete':False,'samples':log}
    def sample(label):
        status=get('/api/status')
        if not status['connected']:raise RuntimeError('Console disconnected: '+str(status['logs'][-3:]))
        s=status['snapshot']
        for name,key in (('motor','coherent'),('motor2','coherent2')):
            m=s[name]
            if not s[key] or not m['online'] or m['feedback_age_ms']+status['sample_age_ms']>=150:
                raise RuntimeError(f'{name}: stale telemetry')
            if m['fault'] or m['fault_bits'] or m['health_fault'] or m['state']==5:
                raise RuntimeError(f'{name}: fault {m}')
        log.append({'label':label,'host_monotonic':time.monotonic(),'snapshot':s})
        return s
    params={'target_rad_s':0,'speed_limit_rad_s':6,'current_limit_a':1,'accel_rad_s2':5,
            'confirmed':True,'compatibility_confirmed':True}
    def command(slot,cmd,target=0):
        receipt=post('command',{**params,'motor':slot,'command':cmd,'target_rad_s':target})
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            time.sleep(.06);s=sample(f'command {slot+1}/{cmd}/{target}')
            m=s['motor' if slot==0 else 'motor2']
            if m['executed_seq']==receipt['seq']:
                if m['result']!=0:raise RuntimeError(f'command failed {slot}: {m}')
                return
        raise RuntimeError('Command receipt timeout')
    def hold(label,duration,expected,tolerance=.18):
        end=time.monotonic()+duration;values=[[],[]]
        while time.monotonic()<end:
            time.sleep(.1);s=sample(label)
            for i,name in enumerate(('motor','motor2')):values[i].append(s[name]['velocity_rad_s'])
        # The first half may include acceleration/deceleration; use settled tail.
        means=[sum(v[len(v)//2:])/len(v[len(v)//2:]) for v in values]
        if any(abs(a-b)>tolerance for a,b in zip(means,expected)):
            raise RuntimeError(f'{label}: expected {expected}, measured mean {means}')
        print(json.dumps({'phase':label,'mean_rad_s':means,'expected':expected}),flush=True)
        record.setdefault('phases',[]).append({'phase':label,'mean_rad_s':means,'expected':expected})
    try:
        status=get('/api/status')
        if not status['connected']:
            probes=post('probes',{})
            if len(probes)!=1:raise RuntimeError('Select one probe before this test')
            post('connect',{'probe_id':probes[0]['id']})
        s=sample('before')
        if not s['setup']['dual_ready']:
            post('setup',{'command':1,'pair_confirmed':True})
            time.sleep(.3)
            if not sample('pair verified')['setup']['dual_ready']:raise RuntimeError('Pair verification failed')
        for slot in (0,1):command(slot,1)
        for slot in (0,1):command(slot,2)
        hold('both enabled at zero',1.0,[0,0])
        command(0,3,.5);hold('M1 +0.5, M2 zero',1.5,[.5,0])
        command(0,3,-.5);hold('M1 -0.5, M2 zero',1.5,[-.5,0])
        command(0,4);hold('M1 stop, M2 zero',1.2,[0,0])
        command(1,3,.5);hold('M2 +0.5, M1 zero',1.5,[0,.5])
        command(1,3,-.5);hold('M2 -0.5, M1 zero',1.5,[0,-.5])
        command(0,3,2);command(1,3,-2);hold('both +2 / -2',2.0,[2,-2])
        command(0,3,5);command(1,3,-5);hold('both +5 / -5',2.5,[5,-5],.25)
        command(0,3,-2);command(1,3,2);hold('both reverse -2 / +2',3.0,[-2,2],.25)
        command(0,4);hold('M1 stopped while M2 maintains +2',2.0,[0,2])
        record['complete']=True
    finally:
        try:
            post('stop_all',{})
            time.sleep(.5)
            final=sample('final both disabled')
            record['final_disabled']=all(not final[name]['enabled'] and not final[name]['enable_requested'] and final[name]['target_rad_s']==0 for name in ('motor','motor2'))
            if not record['final_disabled']:raise RuntimeError('BOTH DISABLE not confirmed')
        finally:
            Path(__file__).resolve().parents[1].joinpath('Docs/dual_motor_motion_live.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    if record['complete']:print('PASS physical dual motor motion; both disabled',flush=True)

if __name__=='__main__':main()
