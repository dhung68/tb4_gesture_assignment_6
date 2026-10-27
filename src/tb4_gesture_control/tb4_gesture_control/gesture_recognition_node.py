"""Laptop webcam -> MediaPipe Tasks -> event gestures and fresh-frame heartbeat."""

import math
import os
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .gesture_filter import GestureFilter, WaveDetector


GESTURE_DEFAULTS = {
    'camera_index': 0,
    'confidence_threshold': 0.65,
    'stable_frame_count': 4,
    'gesture_cooldown': 0.6,
    'neutral_reset_seconds': 0.6,
    'wave_min_amplitude': 0.08,
    'wave_min_direction_changes': 2,
    'wave_window_seconds': 2.5,
    'wave_reversal_distance': 0.025,
    'show_window': True,
    'model_path': '',
}
SUPPORTED = {'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'}
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14),
    (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)


class GestureRecognitionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gesture_recognition_node', **kwargs)
        self.capture = None
        self.recognizer = None
        self.cv2 = None
        c = {name: self.declare_parameter(name, default).value
             for name, default in GESTURE_DEFAULTS.items()}
        for name, value in c.items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if not 0 <= c['confidence_threshold'] <= 1:
            raise ValueError('confidence_threshold must be in [0, 1]')
        if c['camera_index'] < 0 or c['stable_frame_count'] < 1:
            raise ValueError('camera_index >= 0 and stable_frame_count >= 1 required')
        for name in ('neutral_reset_seconds', 'wave_min_amplitude', 'wave_window_seconds',
                     'wave_reversal_distance', 'wave_min_direction_changes'):
            if c[name] <= 0:
                raise ValueError(f'{name} must be positive')
        if c['gesture_cooldown'] < 0:
            raise ValueError('gesture_cooldown must be nonnegative')
        if c['wave_reversal_distance'] > c['wave_min_amplitude']:
            raise ValueError('wave_reversal_distance must not exceed wave_min_amplitude')
        self.config = c
        self.filter = GestureFilter(c['stable_frame_count'], c['gesture_cooldown'],
                                    c['neutral_reset_seconds'])
        self.wave = WaveDetector(c['wave_window_seconds'], c['wave_min_amplitude'],
                                 c['wave_min_direction_changes'], c['wave_reversal_distance'],
                                 c['neutral_reset_seconds'], c['stable_frame_count'])
        self.wave_published = False
        self.gesture_pub = self.create_publisher(String, '/detected_gesture', 10)
        self.heartbeat_pub = self.create_publisher(Empty, '/gesture_heartbeat', 1)
        self.create_subscription(String, '/gesture_control_state', self.on_state, 1)
        self.robot_state = 'IDLE -'
        self.last_heartbeat = float('-inf')
        self.last_frame = None
        self.last_timestamp_ms = -1
        self.fps = 0.0
        self.failed_reads = 0
        self._open_perception()
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame, clock=self.wall_clock)
        self.get_logger().info('Webcam ready. One hand; Q/Esc closes perception safely.')

    def _open_perception(self):
        try:
            import cv2
            import mediapipe as mp
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision

            self.cv2, self.mp = cv2, mp
            if self.config['show_window'] and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
                raise RuntimeError('No display; use show_window:=false for headless testing')
            model = Path(self.config['model_path']).expanduser() if self.config['model_path'] else (
                Path(get_package_share_directory('tb4_gesture_control')) /
                'models/gesture_recognizer.task')
            if not model.is_file():
                raise FileNotFoundError(f'Model missing: {model}; run scripts/setup.sh, then rebuild')
            options = vision.GestureRecognizerOptions(
                base_options=python.BaseOptions(model_asset_path=str(model),
                                               delegate=python.BaseOptions.Delegate.CPU),
                running_mode=vision.RunningMode.VIDEO, num_hands=1)
            self.recognizer = vision.GestureRecognizer.create_from_options(options)
            self.capture = cv2.VideoCapture(self.config['camera_index'], cv2.CAP_V4L2)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.capture.isOpened():
                raise RuntimeError(f'Cannot open webcam index {self.config["camera_index"]}; '
                                   'check /dev/video*, access permissions and camera users')
        except Exception:
            self.close_perception()
            raise

    def on_state(self, message):
        self.robot_state = message.data

    def process_frame(self):
        cv2 = self.cv2
        ok, frame = self.capture.read()
        if not ok or frame is None:
            # No independent heartbeat timer: failed/stuck capture or inference
            # necessarily expires the controller's wall-clock watchdog.
            self.failed_reads += 1
            self.filter.update(None, time.monotonic())
            self.wave.update(None, False, time.monotonic())
            if self.failed_reads == 1:
                self.get_logger().error('Camera frame unavailable; heartbeat suspended')
            if self.failed_reads >= 10:
                raise RuntimeError('Camera failed for 10 consecutive reads')
            return
        self.failed_reads = 0
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        now = time.monotonic()
        timestamp_ms = max(self.last_timestamp_ms + 1, int(now * 1000))
        self.last_timestamp_ms = timestamp_ms
        result = self.recognizer.recognize_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), timestamp_ms)
        now = time.monotonic()
        if self.last_frame is not None:
            instant = 1.0 / max(now - self.last_frame, 1e-6)
            self.fps = instant if self.fps == 0 else 0.85 * self.fps + 0.15 * instant
        self.last_frame = now
        # Heartbeat proves that capture AND inference completed, even with no hand.
        if now - self.last_heartbeat >= 0.2:
            self.heartbeat_pub.publish(Empty())
            self.last_heartbeat = now
        raw, score = 'None', 0.0
        landmarks = result.hand_landmarks[0] if result.hand_landmarks else None
        if result.gestures and result.gestures[0]:
            category = result.gestures[0][0]
            raw, score = category.category_name, category.score
        valid = raw if (raw in SUPPORTED and landmarks is not None
                        and score >= self.config['confidence_threshold']) else None
        palm_x = (sum(landmarks[i].x for i in (0, 5, 9, 13, 17)) / 5
                  if landmarks else None)
        pose = raw if (landmarks and raw != 'None'
                       and score >= self.config['confidence_threshold']) else None
        self.wave.update(palm_x, valid == 'Open_Palm', now, pose=pose)
        if not self.wave.latched:
            self.wave_published = False
        # Preserve a geometry-qualified Wave through None flicker while normal
        # gesture debounce/cooldown runs. Other poses, especially Fist/Thumb_Up,
        # still go through their existing stop/cancel/confirmation path.
        effective = ('Wave' if self.wave.latched and landmarks is not None
                     and (valid == 'Open_Palm' or (valid is None and not self.wave_published))
                     else valid)
        event = self.filter.update(effective, now)
        if event == 'Wave':
            if self.wave_published:
                event = None
            else:
                self.wave_published = True
        if event:
            self.gesture_pub.publish(String(data=event))
            self.get_logger().info(f'Gesture event: {event}')
        if self.config['show_window']:
            self.draw(frame, landmarks, raw, score)
            cv2.imshow('Bai 6 - TurtleBot 4 Gesture Control (Q: quit)', frame)
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                raise KeyboardInterrupt
            if cv2.getWindowProperty('Bai 6 - TurtleBot 4 Gesture Control (Q: quit)',
                                     cv2.WND_PROP_VISIBLE) < 1:
                raise KeyboardInterrupt

    def draw(self, frame, landmarks, raw, score):
        cv2 = self.cv2
        height, width = frame.shape[:2]
        if landmarks:
            points = [(int(p.x * width), int(p.y * height)) for p in landmarks]
            for a, b in HAND_CONNECTIONS:
                cv2.line(frame, points[a], points[b], (0, 220, 0), 2)
            for point in points:
                cv2.circle(frame, point, 3, (0, 120, 255), -1)
        status = 'NO HAND' if not landmarks else self.robot_state
        lines = [f'Raw: {raw}  confidence: {score:.2f}',
                 f'Stable: {self.filter.stable or "-"}  FPS: {self.fps:.1f}',
                 f'Robot: {self.robot_state}', f'Status: {status}',
                 f'Wave tracking: {"ON" if self.wave.tracking else "OFF"}',
                 f'Wave amp: {self.wave.amplitude:.3f} / {self.config["wave_min_amplitude"]:.3f}',
                 f'Wave reversals: {self.wave.reversals}  '
                 f'legs: {self.wave.direction_changes}/{self.config["wave_min_direction_changes"]}',
                 f'Wave points: {self.wave.points}']
        for i, text in enumerate(lines):
            y = 25 + 26 * i
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    def close_perception(self):
        if self.capture is not None:
            self.capture.release()
        if self.recognizer is not None:
            self.recognizer.close()
        if self.cv2 is not None and self.config['show_window']:
            self.cv2.destroyAllWindows()


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    import signal

    def terminate(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = GestureRecognitionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close_perception()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
