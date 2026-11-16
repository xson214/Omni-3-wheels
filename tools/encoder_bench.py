"""Bounded, one-wheel-at-a-time encoder bench test. Wheels must be off ground.
Does NOT measure counts per mechanical revolution or unlock calibrated motion.
"""
import argparse
import json
import time
from pathlib import Path
import serial

class Robot:
    def __init__(self, port):
        self.port = serial.Serial(port, 115200, timeout=.04, write_timeout=.5)
    def send(self, line):
        self.port.write((line+'\n').encode())
    def lines_until(self, predicate, timeout=1):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            line=self.port.readline().decode('utf-8','replace').strip()
            if line and predicate(line): return line
        raise TimeoutError('No expected firmware response')
    def stop(self):
        self.send('S')
        self.lines_until(lambda s:s=='OK S')
    def ticks(self):
        self.send('?')
        line=self.lines_until(lambda s:s.startswith('P,') and ',ticks,' in s)
        return list(map(int,line.split(',ticks,')[1].split(',')))
    def pulse(self, wheel, pwm, duration=600, heartbeat=True):
        self.stop()
        time.sleep(.6)
        start=self.ticks()
        self.send(f'T {wheel} {pwm} {duration}')
        reply=self.lines_until(lambda s:s=='OK T' or s.startswith('ERR T'))
        if reply!='OK T': raise RuntimeError(reply)
        deadline=time.monotonic()+duration/1000+.08
        next_h=time.monotonic()
        samples=[]
        states=[]
        while time.monotonic()<deadline:
            now=time.monotonic()
            if now>=next_h:
                if heartbeat: self.send('H')
                self.send('?')
                next_h=now+.06
            line=self.port.readline().decode('utf-8','replace').strip()
            if line.startswith('state,'): states.append({'t':round(now,4),'line':line})
            if line.startswith('P,') and ',ticks,' in line:
                samples.append({'t':round(now,4),'ticks':list(map(int,line.split(',ticks,')[1].split(','))),'omega_rad_s':list(map(float,line.split(',w,')[1].split(',ticks,')[0].split(',')))})
        self.stop()
        time.sleep(.65)
        end=self.ticks()
        delta=[b-a for a,b in zip(start,end)]
        result={'wheel':wheel,'pwm':pwm,'duration_ms':duration,'heartbeat':heartbeat,'start':start,'end':end,'delta':delta,'samples':samples,'states':states}
        print(json.dumps({k:v for k,v in result.items() if k not in ('samples','states')}),flush=True)
        if any(abs(delta[i])>3 for i in range(3) if i!=wheel-1):
            raise RuntimeError('Unexpected encoder moved: check wheel/channel mapping; stopped')
        return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='COM16')
    parser.add_argument('--wheel',type=int,choices=[1,2,3],required=True)
    parser.add_argument('--pwm',type=int,required=True)
    parser.add_argument('--duration',type=int,default=600)
    parser.add_argument('--no-heartbeat',action='store_true')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if not 1<=abs(args.pwm)<=160 or not 50<=args.duration<=1500: parser.error('PWM 1..160, duration 50..1500ms')
    robot=Robot(args.port)
    try:
        result=robot.pulse(args.wheel,args.pwm,args.duration,not args.no_heartbeat)
        Path(args.output).write_text(json.dumps(result,indent=2),encoding='utf-8')
    finally:
        try: robot.stop()
        finally: robot.port.close()
if __name__=='__main__': main()
