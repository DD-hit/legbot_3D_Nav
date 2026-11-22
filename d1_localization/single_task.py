import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid

RUNNER = '/workspace/navigation_ws/d1-planning-preview/single.sh'


class SingleTask:
    def __init__(self):
        self.lock = threading.Lock()
        self.process = None
        self.state = None
        self.route = []

    def snapshot(self):
        with self.lock:
            return self.state, self.route

    def start(self, body, atlas):
        received = time.monotonic()
        with self.lock:
            if self.process is not None:
                raise ValueError('已有任务')
            if len(body['points']) != 1:
                raise ValueError('本次真机测试只支持单点')
            goal = body['points'][0]
            x,y,z = atlas.point(goal,True)
            yaw = float(goal['yaw'])
            if not math.isfinite(yaw):
                raise ValueError('终点朝向无效')
            task_id = uuid.UUID(body['id']).hex
            if self.state and self.state['id'] == task_id:
                raise ValueError('该任务已提交')
            p = subprocess.Popen(['bash',RUNNER,'--goal',str(x),str(y),'--yaw',str(yaw)],
                                 env=dict(os.environ,D1_TASK_PARENT=str(os.getpid()),D1_RECOVERY_PROTOCOL='1',D1_REQUEST_MONOTONIC=str(received)),start_new_session=True, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
            self.route = []
            self.process = p
            self.state = dict(id=task_id,phase='planning',points=[[x,y,z]],headings=[yaw],index=0)
            threading.Thread(target=self.watch,args=(p,z),daemon=True).start()

    def watch(self,p,z):
        reason = '单点运行异常'
        arrived = False
        for line in p.stdout:
            line = line.strip()
            with self.lock:
                if self.state['phase'] == 'canceled':
                    continue
                if line == 'WAIT_LOCALIZATION':
                    self.state = dict(self.state,phase='relocalizing')
                    self.route = []
                elif line == 'REPLAN_LOCALIZED':
                    self.state = dict(self.state,phase='planning')
                elif line.startswith('{"start"'):
                    self.route = [[x,y,z] for x,y in json.loads(line)['map_route']]
                elif line=='TRACKING' and self.state['phase']=='planning':
                    self.state = dict(self.state,phase='walking')
                elif line=='ARRIVED':
                    arrived = True
                elif 'Error:' in line:
                    reason = line.split('Error:')[-1].strip()
        code = p.wait()
        with self.lock:
            if self.state['phase'] != 'canceled':
                ok = arrived and code==0
                self.state = dict(self.state,phase='succeeded' if ok else 'failed',
                                  index=int(ok),reason='' if ok else reason)
            self.process = None

    def cancel(self,task_id=None):
        with self.lock:
            if self.process is None:
                return
            if task_id and uuid.UUID(task_id).hex != self.state['id']:
                raise ValueError('任务编号不一致')
            self.state = dict(self.state,phase='canceled',reason='用户取消任务')
            try:
                os.killpg(self.process.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
            return True
