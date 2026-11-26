"""Locally taught SO-101 follower path. Hardware imports occur only for USB commands."""
import hashlib
import json
import time
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .model import JOINTS
from .path import check_grid, trajectory, selection

ARM=list(JOINTS)


@contextmanager
def connection(port,robot_id):
    from lerobot.robots.so_follower import SO101Follower,SO101FollowerConfig
    robot=SO101Follower(SO101FollowerConfig(port=port,id=robot_id,use_degrees=True,cameras={}))
    if not set(ARM+['gripper'])==set(robot.calibration):
        raise ValueError('calibrate this follower and reuse the same robot ID')
    bus=robot.bus
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise ValueError('stored motor calibration does not match the connected arm')
        calibration={j:asdict(c) for j,c in robot.calibration.items()}
        digest=hashlib.sha256(json.dumps(calibration,sort_keys=True).encode()).hexdigest()
        limits=np.array([(calibration[j]['range_max']-calibration[j]['range_min'])*180/4095 for j in ARM])
        yield bus,digest,limits
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def angles(bus):
    current=bus.sync_read('Present_Position',ARM,num_retry=1)
    result=np.array([current[j] for j in ARM],float)
    if not np.isfinite(result).all(): raise RuntimeError('nonfinite motor observation')
    return result


def stationary(bus):
    a=angles(bus)
    time.sleep(.15)
    b=angles(bus)
    if np.max(abs(a-b))>.6: raise ValueError('support the arm and hold the pose still')
    return b


def torque(bus):
    return [bus.read('Torque_Enable',j,normalize=False) for j in ARM]


def profile_path(args):
    if not args.id or any(not(c.isascii() and (c.isalnum() or c in '-_')) for c in args.id):
        raise ValueError('use a short alphanumeric robot ID')
    return Path('local')/args.id/(args.color+'.json')


def teach(args):
    path=profile_path(args)
    if path.exists(): raise ValueError(f'{path} exists; move the old profile aside before teaching a new mount')
    path.parent.mkdir(parents=True,exist_ok=True)
    with connection(args.port,args.id) as (bus,digest,limits):
        if any(torque(bus)): raise ValueError('support the arm, then use release before hand teaching')
        grid=np.empty((2,3,3,5))
        print('Support the arm. Keep the roll and nib inclination consistent. Follow the nine printed crosses.')
        for row in range(3):
            for col in range(3):
                for level,label in enumerate(['writing contact','8 mm spacer on the same cross']):
                    input(f'R{row+1}C{col+1}, {label}. Hold still and press Enter: ')
                    grid[level,row,col]=stationary(bus)
        check_grid(grid)
        if np.any(abs(grid)>limits-.5):raise ValueError('a taught point is too close to a motor stop')
        record={'schema':1,'source':'hand-taught','id':args.id,'color':args.color,'calibration':digest,'grid':grid.tolist()}
        with path.open('x') as f: json.dump(record,f,indent=1)
        print('Saved',path)


def hold(bus):
    measured=bus.sync_read('Present_Position',ARM,normalize=False,num_retry=1)
    bus.sync_write('Goal_Position',measured,normalize=False)


def follow(bus,q,marks,ids,log,rate=30.):
    current=stationary(bus)
    if np.max(abs(current-q[0]))>3.0:
        raise ValueError('start with the nib at taught R1C1 hover within 3 degrees of the recorded pose')
    if any(bus.read('Operating_Mode',j,normalize=False)!=0 for j in ARM):
        raise ValueError('configure the follower motors for position control')
    states=torque(bus)
    if len(set(states))!=1:
        raise ValueError('unexpected mix of enabled and disabled arm joints')
    hold(bus)
    start=time.monotonic()
    try:
        bus.enable_torque(ARM)
        for i,target in enumerate(q):
            deadline=time.monotonic()+3.
            while True:
                current=angles(bus)
                error=target-current
                if np.max(abs(error))>12:
                    raise RuntimeError('arm is too far from the planned path')
                goal=current+error*min(1.,.8/max(np.max(abs(error)),1e-9))
                bus.sync_write('Goal_Position',dict(zip(ARM,goal.tolist())))
                log.write(json.dumps({'s':round(time.monotonic()-start,3),'point':i,'stroke':int(ids[i]),
                                      'ink':bool(marks[i]),'goal_deg':goal.tolist(),'measured_deg':current.tolist()})+'\n')
                if time.monotonic()>deadline:
                    raise RuntimeError(f'tracking timeout at point {i}; inspect the sheet before resuming')
                time.sleep(1/rate)
                if np.max(abs(error))<.65:break
    finally:
        hold(bus)


def run(args):
    path=profile_path(args)
    record=json.loads(path.read_text())
    if (record['schema'],record['source'],record['id'],record['color'])!=(1,'hand-taught',args.id,args.color):
        raise ValueError('wrong arm or marker profile')
    batch=selection(args.color,args.start,args.count,args.mode=='swatch')
    q,_,marks,ids=trajectory(batch,np.array(record['grid']),air=args.mode=='air')
    print(f'{len(batch)} strokes, {len(q)} joint targets. From R1C1 hover back to R1C1 hover.')
    print('Starting arm angles in degrees:', dict(zip(ARM, q[0].round(2).tolist())))
    if not args.execute:return
    if not args.port:raise ValueError('provide --port to execute')
    with connection(args.port,args.id) as (bus,digest,limits):
        if record['calibration']!=digest or np.any(abs(q)>limits-.5):
            raise ValueError('motor calibration changed or route exceeds this arm’s calibrated range')
        folder=path.parent/'logs';folder.mkdir(exist_ok=True)
        logfile=folder/f'{time.time_ns()}-{args.color}-{args.mode}-{args.start}.jsonl'
        print('Motor log',logfile,flush=True)
        with logfile.open('x',buffering=1) as log:
            log.write(json.dumps({'mode':args.mode,'colour':args.color,'start':args.start,'labels':[s[1] for s in batch]})+'\n')
            follow(bus,q,marks,ids,log)
            log.write('{"completed":true}\n')


def release(args):
    with connection(args.port,args.id) as (bus,_,__):
        if not args.supported: raise ValueError('physically support the arm first; then pass --supported')
        bus.disable_torque(ARM)
        print('Five arm joints released. The gripper setting is untouched.')
