"""Short-lived, single-owner joystick control; runs in the bridge's ROS executor."""
import math
import speed_settings
import secrets
import time

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter

SERVICE = '/d1/manual_drive'
TIMEOUT = .35


class ManualDrive:
    def __init__(self, bridge):
        self.bridge = bridge
        self.token = ''
        self.owner = ""
        self.expires = 0.
        self.revision = 0
        bridge.create_service(SetParametersAtomically, SERVICE, self.request)

    def request(self, request, response):
        try:
            values = {item.name: Parameter.from_parameter_msg(item).value for item in request.parameters}
            result = self.apply(**values)
            response.result = SetParametersResult(successful=True, reason=result)
        except (TypeError, ValueError, RuntimeError, IndexError) as error:
            response.result = SetParametersResult(successful=False, reason=str(error))
        return response

    def apply(self, action, owner='', token='', seq=0, x=0., y=0., turn=0.):
        b = self.bridge
        now = time.monotonic()
        if action == 'connect':
            if self.owner and now < self.expires or b.armed or b.pending_stop or b.moving:
                raise ValueError('控制权已被占用或正在停车')
            if not b.sdk.checkConnect():
                raise ValueError('SDK 未连接')
            b.set_recovery(False)
            self.owner = secrets.token_urlsafe(24)
            self.expires = now + 5.
            return self.owner
        if not self.owner or now >= self.expires or owner != self.owner:
            raise ValueError('请先连接控制权')
        if action == 'heartbeat':
            self.expires = now + 5.
            return '已连接'
        if action == 'release':
            self.owner = ''
            self.expires = 0.
            b.disarm('manual ownership released')
            return '已释放控制权'
        if action in ('stand','lie'):
            from std_srvs.srv import Trigger
            if now < b.posture_ready_at:
                raise ValueError('请等待当前姿态切换完成')
            result = b.posture('standUp' if action=='stand' else 'lieDown', Trigger.Response())
            if not result.success:
                raise ValueError(result.message)
            return result.message
        if action == 'begin':
            if b.armed or b.moving or b.pending_stop or now < max(b.manual_until, b.posture_ready_at):
                raise ValueError('已有运动指令或姿态切换尚未完成')
            if not b.sdk.checkConnect() or b.sdk.getCurrentCtrlmode() not in (1, 18) or not .20 <= b.sdk.getPosition()[2] <= .45:
                raise ValueError('请确认机器狗已连接并站稳')
            self.limits = speed_settings.load()['manual']
            self.token = secrets.token_urlsafe(24)
            self.revision = 0
            b.manual_until = now + TIMEOUT
            b.velocity = (0., 0., 0.)
            return self.token
        if not self.token or token != self.token or now >= b.manual_until:
            raise ValueError('控制已释放，请重新按住')
        if action == 'end':
            self.token = ''
            b.disarm('joystick released')
            return '已释放'
        if action != 'drive' or type(seq) is not int or seq <= self.revision:
            raise ValueError('控制指令已过期')
        if not all(type(v) in (float, int) and math.isfinite(v) for v in (x,y,turn)) or math.hypot(x,y)>1.00001 or abs(turn)>1 or (turn and (x or y)):
            raise ValueError('控制量无效')
        self.revision = seq
        vx = 0.
        if abs(x)*.12 >= .05:
            # Map the joystick's existing forward range onto the chosen ceiling.
            fraction = max(0., (abs(x)*.12-.05)/(.12-.05))
            vx = math.copysign(.05 + fraction*(self.limits['forward']-.05), x)
        vy = 0.
        if abs(y)*.12 >= .10:
            fraction = min(1., (abs(y)*.12-.10)/.02)
            vy = math.copysign(.10 + fraction*(self.limits['lateral']-.10), y)
        yaw = self.limits['yaw']*turn
        b.velocity = (vx, vy, yaw if abs(yaw) >= .02 else 0.)
        b.manual_until = now + TIMEOUT
        b.moving = b.moving or any(b.velocity)
        return '手动控制中'
