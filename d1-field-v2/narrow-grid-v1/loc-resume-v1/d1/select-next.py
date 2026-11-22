"""Select the comparison runner for the next goal submitted from the existing D1 web UI."""
from pathlib import Path
import fcntl
import os
import shutil
import sys

base = Path('/workspace/navigation_ws/d1-planning-preview')
trial = Path('/workspace/d1_scan_community')
runner = base / 'single.sh'
selector = trial / 'next-web-task'
dispatch = """# One-shot Upstream community comparison selected by the operator.
if [[ ${1:-} == --goal ]] && rmdir /workspace/d1_scan_community/next-web-task 2>/dev/null; then
  exec bash /workspace/d1_scan_community/d1/run.sh "$@"
fi
"""
with (base / 'single.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if sys.argv[1:] == ['--cancel']:
        if selector.exists():
            selector.rmdir()
    else:
        if Path('/workspace/d1_jazzy_trial/next-web-task').exists():
            raise RuntimeError('另一个对照版已预约下一次任务')
        text = runner.read_text()
        if dispatch not in text:
            backup = trial / 'single.before-selector.sh'
            if backup.exists():
                raise RuntimeError('启动脚本已变化，请先核对备份')
            shutil.copy2(runner, backup)
            first, rest = text.split('\n', 1)
            staged = base / '.single-trial-selector.sh'
            staged.write_text(first+'\n'+dispatch+rest)
            shutil.copymode(runner, staged)
            os.replace(staged, runner)
        selector.mkdir(exist_ok=True)
