import argparse
import fcntl
import os
import select
import signal
import sys
import termios
import time
import tty


MOTION = {'w': (0.12, 0., 0.), 's': (-0.12, 0., 0.),
          'a': (0., 0., 0.25), 'd': (0., 0., -0.25),
          'q': (0., 0.12, 0.), 'e': (0., -0.12, 0.)}
STOP = (0., 0., 0.)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dry-run', action='store_true')
    options = parser.parse_args()
    if not sys.stdin.isatty():
        raise SystemExit('需要交互终端，请用 ssh -t 和 docker exec -it')
    print('1 站立 · 2 趴下 · W/S 前后 · A/D 转向 · Q/E 横移 · 空格停车 · X 退出')
    print('每次方向键输入授权 0.35 秒运动。确认现场安全后，先输入 YES 并回车。')
    while True:
        try:
            answer = input('连接确认（YES 继续，X 退出）：').strip().upper()
        except (EOFError, KeyboardInterrupt):
            return
        if answer == 'X':
            return
        if answer == 'YES':
            break
        print('请先输入 YES 并回车。连接成功后再按 1 站立。')
    dog = None
    lock = None
    if not options.dry_run:
        from mc_sdk_zsl_1_py import HighLevel
        lock = open('/workspace/agibot_d1_sdk/control.lock', 'a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('已有 D1 控制程序，请先退出再连接')
        dog = HighLevel()
        dog.initRobot('192.168.168.169', 43988, '192.168.168.168')
        for _ in range(20):
            if dog.checkConnect():
                break
            time.sleep(.2)
        else:
            raise RuntimeError('D1 SDK 未连接')
    print('已就绪。按 1 站立，站起后等约 5 秒再按方向键。', flush=True)

    def command(method, *args):
        if dog is None:
            print(method, *args, flush=True)
            return
        if not dog.checkConnect():
            raise RuntimeError('D1 SDK 已断开')
        result = getattr(dog, method)(*args)
        if result != 0:
            raise RuntimeError(f'{method} 返回错误 {result:#x}')

    def terminate(signum, frame):
        raise KeyboardInterrupt()

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, terminate)
    terminal = sys.stdin.fileno()
    previous = termios.tcgetattr(terminal)
    active = STOP
    expiry = 0.
    next_command = 0.
    enabled_after = math_inf = float('inf')
    try:
        tty.setcbreak(terminal)
        while True:
            now = time.monotonic()
            if active != STOP and now >= expiry:
                command('move', *STOP)
                active = STOP
            if active != STOP and now >= next_command:
                command('move', *active)
                next_command = now + .05
            ready, _, _ = select.select([terminal], [], [], .02)
            if not ready:
                continue
            keys = os.read(terminal, 128).decode(errors='ignore').lower()
            if not keys or 'x' in keys or '\x1b' in keys:
                break
            if keys == 'p' and dog is not None and dog.checkConnect():
                print(f'机身高度 {dog.getPosition()[2]:.3f} m，控制状态 {dog.getCurrentCtrlmode()}', flush=True)
                continue
            if ' ' in keys or '2' in keys:
                if active != STOP:
                    command('move', *STOP)
                active = STOP
                if '2' in keys:
                    enabled_after = math_inf
                    command('lieDown')
                continue
            if '1' in keys and enabled_after == math_inf:
                command('standUp')
                enabled_after = time.monotonic() + 5.
                continue
            key = keys[-1]
            if key in MOTION and now >= enabled_after:
                active = MOTION[key]
                expiry = now + .35
                next_command = now
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(terminal, termios.TCSADRAIN, previous)
        if enabled_after != math_inf:
            try:
                for _ in range(3):
                    command('move', *STOP)
                    time.sleep(.04)
            except RuntimeError as error:
                print(f'停车指令未确认，请现场停车：{error}', file=sys.stderr)
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    main()
