#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-cache-dir -r scripts/requirements.txt
.venv/bin/python - <<'PY'
import hashlib
from pathlib import Path
import urllib.request

target = Path('src/tb4_gesture_control/models/gesture_recognizer.task')
url = ('https://storage.googleapis.com/mediapipe-models/gesture_recognizer/'
       'gesture_recognizer/float16/1/gesture_recognizer.task')
expected = '97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482'
if not target.is_file():
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read()
    import zipfile
    import io
    if not zipfile.is_zipfile(io.BytesIO(data)):
        raise RuntimeError('Downloaded model is not a MediaPipe task bundle')
    if hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError('Downloaded model checksum mismatch')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
checksum = hashlib.sha256(target.read_bytes()).hexdigest()
if checksum != expected:
    raise RuntimeError(f'Model checksum mismatch: {target}')
print('Model:', target, 'SHA256:', checksum)
PY
source /opt/ros/humble/setup.bash
.venv/bin/python - <<'PY'
import cv2, mediapipe, numpy, rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
print('Imports OK:', 'cv2', cv2.__version__, 'mediapipe', mediapipe.__version__,
      'numpy', numpy.__version__)
PY
