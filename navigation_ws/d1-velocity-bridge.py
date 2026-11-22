import argparse
import os
import fcntl
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool, Trigger


STOP = (0., 0., 0.)
COMMAND_TIMEOUT = .30
SENSOR_TIMEOUT = .80
CONTROL_PORT = 43988
JOG = {
    'forward': (.10, 0., 0.), 'backward': (-.10, 0., 0.),
    'left': (0., .10, 0.), 'right': (0., -.10, 0.),
    'turn_left': (0., 0., .20), 'turn_right': (0., 0., -.20),
}


class VelocityBridge(Node):
    def __init__(self, sdk=None):
        super().__init__('d1_velocity_bridge')
        self.sdk = sdk
        self.nav_limits = (.20, .12, .45) if os.getenv("D1_NAV_SPEED") == "normal" else (.12, .12, .25)
        self.armed = False
        self.moving = False
        self.velocity = STOP
        self.command_at = 0.
        self.localization_at = 0.
        self.cloud_at = 0.
        self.localized = False
        self.reason = 'disarmed'
        self.audit = self.create_publisher(Twist, '/d1/velocity_applied', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.recovery_allowed = False
        self.recovery_state = self.create_publisher(Bool, '/d1/recovery_allowed', latched)
        self.recovery_state.publish(Bool(data=False))
        self.create_service(SetBool, '/d1/resume_motion', self.resume)
        self.state = self.create_publisher(Bool, '/d1/motion_armed', latched)
        self.state.publish(Bool(data=False))
        self.create_subscription(Bool, '/localization/ready', self.localization, latched)
        self.create_subscription(PointCloud2, '/fast_lio/cloud_registered', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, '/navigation/approved_velocity', self.command, 1)
        self.create_subscription(Empty, '/d1/cancel_motion', lambda _: self.disarm('cancel'), 10)
        self.create_service(SetBool, '/d1/arm_motion', self.arm)
        self.manual_until = 0.
        self.posture_ready_at = 0.
        if sdk is not None:
            from manual_drive import ManualDrive
            self.joystick = ManualDrive(self)
            from functools import partial
            for action in JOG:
                self.create_service(Trigger, '/d1/' + action, partial(self.jog, action))
            self.create_service(Trigger, '/d1/stand', lambda req, res: self.posture('standUp', res))
            self.create_service(Trigger, '/d1/lie', lambda req, res: self.posture('lieDown', res))
            self.create_service(Trigger, '/d1/stop', self.stop)
        self.last_sent = STOP
        self.pending_stop = False
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def posture(self, method, response):
        if self.armed or self.moving or self.manual_until:
            response.message = '请先停止并解除导航授权'
            return response
        if self.pending_stop or not self.sdk.checkConnect():
            response.message = 'SDK 未连接或仍在等待停车确认'
            return response
        result = getattr(self.sdk, method)()
        response.success = result == 0
        if response.success:
            self.posture_ready_at = time.monotonic() + 5.
        response.message = '指令已接受' if response.success else f'SDK 返回 {result:#x}'
        return response

    def jog(self, action, request, response):
        now = time.monotonic()
        if self.armed or self.manual_until or self.pending_stop:
            response.message = '正在执行其他运动或等待停车'
            return response
        if now < self.posture_ready_at or not self.sdk.checkConnect():
            response.message = '请等待姿态切换完成并检查 SDK 连接'
            return response
        if self.sdk.getCurrentCtrlmode() not in (1, 18) or not .20 <= self.sdk.getPosition()[2] <= .45:
            response.message = '请先确认机器狗已站稳'
            return response
        self.velocity = JOG[action]
        self.manual_until = now + .30
        self.moving = True
        response.success = self.emit(self.velocity)
        if not response.success:
            self.disarm('manual command rejected')
        response.message = '已开始短动' if response.success else 'SDK 未接受速度指令'
        return response

    def stop(self, request, response):
        self.disarm('web stop')
        response.success = not self.pending_stop
        response.message = '已解除运动授权' if response.success else '停车指令尚未确认'
        return response

    def localization(self, message):
        self.localized = message.data
        self.localization_at = time.monotonic()
        if not message.data and self.armed:
            self.disarm('localization lost')

    def cloud(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        if message.header.frame_id == 'odom' and message.width * message.height > 0 and 0 <= age < 500_000_000:
            self.cloud_at = time.monotonic()

    def sensor_fault(self, now):
        if not self.localized or now - self.localization_at > SENSOR_TIMEOUT:
            return 'no fresh localization'
        if now - self.cloud_at > SENSOR_TIMEOUT:
            return 'no fresh point cloud'
        return ''

    def set_recovery(self, allowed):
        self.recovery_allowed = allowed
        self.recovery_state.publish(Bool(data=allowed))

    def resume(self, request, response):
        if not request.data or not self.recovery_allowed or self.armed:
            response.message = '自动恢复授权已撤销'
            return response
        return self.arm(request, response)

    def arm(self, request, response):
        if not request.data:
            self.disarm('disabled')
            response.success, response.message = True, 'disarmed'
            return response
        now = time.monotonic()
        fault = self.sensor_fault(now)
        if getattr(getattr(self,'joystick',None),'expires',0.) > now or self.manual_until or now < self.posture_ready_at:
            fault = "manual motion or posture change in progress"
        if not fault and self.pending_stop:
            fault = 'previous stop not acknowledged'
        if not fault and self.sdk is not None:
            if not self.sdk.checkConnect():
                fault = 'D1 SDK is disconnected'
            else:
                mode = self.sdk.getCurrentCtrlmode()
                position = self.sdk.getPosition()
                height = position[2] if len(position) == 3 else float('nan')
                if mode not in (1, 18) or not .20 <= height <= .45:
                    fault = f'D1 must be standing (mode={mode}, height={height:.3f} m)'
        if not fault:
            import speed_settings
            try:
                profile = speed_settings.load()['navigation']
                self.nav_limits = (profile['forward'], profile['lateral'], profile['yaw'])
            except (OSError, ValueError) as error:
                fault = str(error)
        if fault:
            response.message = fault
            return response
        self.command_at, self.velocity = now, STOP
        self.set_recovery(False)
        self.armed = True
        self.reason = 'armed'
        self.state.publish(Bool(data=True))
        response.success, response.message = True, 'armed; waiting for a new velocity command'
        return response

    def command(self, message):
        if not self.armed:
            return
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.disarm('invalid velocity')
            return
        vx, vy, yaw = (max(-limit, min(limit, value)) for value, limit in zip(values, self.nav_limits))
        self.velocity = (vx if abs(vx) >= .05 else 0., vy if abs(vy) >= .10 else 0., yaw if abs(yaw) >= .02 else 0.)
        self.command_at = time.monotonic()
        if self.velocity == STOP and self.moving:
            self.pending_stop = True
            self.stop_if_needed()
            if self.pending_stop:
                self.disarm('zero velocity rejected')
            else:
                self.moving = False

    def emit(self, values):
        if self.sdk is not None:
            if not self.sdk.checkConnect():
                return False
            result = self.sdk.move(*values)
            if result != 0:
                self.get_logger().error(f'D1 move returned {result:#x}')
                return False
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = values
        self.audit.publish(message)
        self.last_sent = values
        return True

    def disarm(self, reason):
        self.set_recovery(self.armed and reason == 'localization lost')
        self.pending_stop = self.pending_stop or self.moving or self.last_sent != STOP
        if hasattr(self, "joystick"):
            self.joystick.token = ""
        self.manual_until = 0.
        self.armed = self.moving = False
        self.velocity = STOP
        self.reason = reason
        self.state.publish(Bool(data=False))
        self.get_logger().info(f'motion disarmed: {reason}')
        self.stop_if_needed()

    def stop_if_needed(self):
        if self.pending_stop and self.emit(STOP):
            self.pending_stop = False

    def tick(self):
        if self.manual_until:
            if time.monotonic() >= self.manual_until or not self.emit(self.velocity):
                self.disarm('manual pulse ended')
            return
        if not self.armed:
            self.stop_if_needed()
            return
        now = time.monotonic()
        fault = self.sensor_fault(now)
        if fault or now - self.command_at > COMMAND_TIMEOUT:
            self.disarm(fault or 'velocity command timeout')
        elif self.velocity != STOP:
            self.moving = True
            if not self.emit(self.velocity):
                self.disarm('SDK rejected velocity')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware', action='store_true')
    options = parser.parse_args()
    sdk = lock = None
    if options.hardware:
        lock = Path('/workspace/agibot_d1_sdk/control.lock').open('a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Another D1 controller owns the SDK')
        if any(f':{CONTROL_PORT:04X}' in row.split()[1].upper() for row in Path('/proc/net/udp').read_text().splitlines()[1:]):
            raise SystemExit('The SDK UDP port is already in use')
        from mc_sdk_zsl_1_py import HighLevel
        sdk = HighLevel()
        sdk.initRobot('192.168.168.169', CONTROL_PORT, '192.168.168.168')
        deadline = time.monotonic() + 4.
        while not sdk.checkConnect() and time.monotonic() < deadline:
            time.sleep(.1)
        if not sdk.checkConnect():
            raise SystemExit('D1 SDK connection unavailable')
    rclpy.init()
    node = VelocityBridge(sdk)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.disarm('shutdown')
        if sdk is not None and (node.pending_stop or node.moving or node.last_sent != STOP):
            try:
                for _ in range(3):
                    if sdk.checkConnect():
                        sdk.move(*STOP)
                    time.sleep(.05)
            except Exception as error:
                print(f'D1 shutdown stop could not be confirmed: {error}', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    main()
