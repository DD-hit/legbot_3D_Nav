"""Record the Building baseline round trip in an isolated, sourced Humble environment."""
import argparse
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--robot-profile', type=Path, default=Path(__file__).resolve().parents[1]/'src/legbot_bringup/config/robot.yaml')
    args = parser.parse_args()
    root = args.directory.resolve()
    profile = args.robot_profile.resolve()
    robot = yaml.safe_load(profile.read_text())
    root.mkdir()
    processes = []
    def launch(name, command):
        with (root/(name+'.log')).open('x') as log:
            child = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        processes.append(child)
    def api(path, body=None):
        data = None if body is None else json.dumps(body).encode()
        return json.load(urlopen(Request(f'http://127.0.0.1:{args.port}/api/{path}', data=data,
                                        headers={'Content-Type':'application/json'}), timeout=5))
    def wait(predicate, timeout, starting=False):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            if any(process.poll() is not None for process in processes):
                raise RuntimeError('Capture process exited before mission completion')
            try:
                state = api('state')
            except URLError:
                if not starting: raise
            else:
                if predicate(state): return state
            time.sleep(.5)
        raise TimeoutError('Survey timed out')
    try:
        for name,topics in [('raw',[robot['raw_cloud'],robot['imu'],'/clock','/tf_static']),
                            ('evaluation',['/Odometry_gazebo',robot['odometry'],'/navigation/task_status'])]:
            launch(name, ['ros2','bag','record','--use-sim-time','-o',name,*topics])
        launch('simulation',['ros2','launch','legbot_bringup','pct_cross_floor_demo.launch.py','gui:=false','rviz:=false',
                             'navigation_source:=map_localized','web_console:=true',f'web_port:={args.port}',
                             'publish_ground_truth:=true',f'robot_profile:={profile}'])
        wait(lambda state:state.get('standing'),120,True)
        entry = dict(x=-5.5,y=6.,floor=0,yaw=0.)
        api('seed',dict(entry,yaw=.03))
        wait(lambda state:state['ready'],50)
        task_id=uuid.uuid4().hex
        api('tasks',dict(id=task_id,points=[dict(x=2.,y=-3.,floor=3,yaw=0.),entry]))
        state=wait(lambda state:state['task']['id']==task_id and (state['task']['phase'] in ('succeeded','failed','canceled') or
                   (state['task']['phase']=='stopping' and state['localization']['state']=='LOST')),1400)
        (root/'mission.json').write_text(json.dumps(state,indent=2,ensure_ascii=False)+'\n')
        time.sleep(4)
        if state['task']['phase']!='succeeded':print('Navigation did not confirm arrival:', state['task'],flush=True)
    finally:
        for child in processes:
            if child.poll() is None:os.killpg(child.pid, signal.SIGINT)
        for child in processes:
            try: child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                child.wait(timeout=10)
    for name in ['raw','evaluation']:
        if not (root/name/'metadata.yaml').is_file():raise RuntimeError(f'Incomplete {name} bag')


if __name__=='__main__':
    main()
