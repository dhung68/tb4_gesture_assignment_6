"""Temporal filters with monotonic timestamps; no ROS or camera dependencies."""

from collections import deque
import math


class GestureFilter:
    """Consecutive-frame debounce, neutral rearm and event edge detection.

    Stop/cancel use the first confident frame and bypass cooldown. They never
    command motion, so safety takes precedence over normal stabilization.
    """

    def __init__(self, stable_frames=4, cooldown=0.6, neutral_seconds=0.6):
        self.stable_frames = stable_frames
        self.cooldown = cooldown
        self.neutral_seconds = neutral_seconds
        self.candidate = None
        self.count = 0
        self.latched = None
        self.neutral_since = None
        self.last_event = float('-inf')
        self.stable = None

    def update(self, gesture, now):
        if gesture is None:
            self.candidate, self.count, self.stable = None, 0, None
            if self.neutral_since is None:
                self.neutral_since = now
            if now - self.neutral_since >= self.neutral_seconds:
                self.latched = None
            return None
        self.neutral_since = None
        if gesture != self.candidate:
            self.candidate, self.count, self.stable = gesture, 1, None
        else:
            self.count += 1
        safety = gesture in ('Open_Palm', 'Closed_Fist')
        if self.count < (1 if safety else self.stable_frames):
            return None
        self.stable = gesture
        if self.latched == gesture:
            return None
        if not safety and now - self.last_event < self.cooldown:
            return None
        self.latched, self.last_event = gesture, now
        return gesture


class WaveDetector:
    """Arm on a stable palm, then track geometry despite classifier flicker.

    ``changes`` counts amplitude-qualified alternating motion legs, including
    the first established direction: two legs mean left->right->left (or its
    mirror). ``reversals`` reports actual sign reversals for the debug overlay.
    Every qualifying leg must cover amplitude, not just a short jitter reversal.
    """

    def __init__(self, window=2.5, amplitude=0.08, changes=2,
                 reversal_distance=0.025, reset_seconds=0.6, stable_frames=4):
        self.window = window
        self.amplitude_threshold = amplitude
        self.changes = changes
        self.reversal_distance = reversal_distance
        self.reset_seconds = reset_seconds
        self.stable_frames = stable_frames
        self.history = deque()
        self.reset()

    def reset(self):
        self.history.clear()
        self.tracking = False
        self.latched = False
        self.open_count = 0
        self.other_pose = None
        self.other_count = 0
        self.missing_since = None
        self.neutral_since = None
        self.amplitude = 0.0
        self.reversals = 0
        self.direction_changes = 0

    @property
    def points(self):
        return len(self.history)

    def update(self, x, open_palm, now, pose=None):
        if x is None or not math.isfinite(x):
            self.open_count = 0
            if self.missing_since is None:
                self.missing_since = now
            if now - self.missing_since >= self.reset_seconds:
                self.reset()
            return False
        self.missing_since = None
        if open_palm:
            self.other_pose, self.other_count, self.neutral_since = None, 0, None
        elif pose is not None:
            self.neutral_since = None
            if pose == self.other_pose:
                self.other_count += 1
            else:
                self.other_pose, self.other_count = pose, 1
            if self.other_count >= self.stable_frames:
                self.reset()
                return False
        else:
            self.other_pose, self.other_count = None, 0
            # While collecting a wave, landmarks alone keep tracking alive.
            # After firing, a sustained neutral pose may rearm the next session.
            if self.latched:
                if self.neutral_since is None:
                    self.neutral_since = now
                if now - self.neutral_since >= self.reset_seconds:
                    self.reset()
                    return False
        if not self.tracking:
            self.open_count = self.open_count + 1 if open_palm else 0
            if self.open_count < self.stable_frames:
                return False
            self.tracking = True
        if self.latched:
            return False  # Keep the successful trajectory visible until rearmed.
        self.history.append((now, x))
        while self.history and now - self.history[0][0] > self.window:
            self.history.popleft()
        self._measure()
        if (self.amplitude + 1e-9 >= self.amplitude_threshold
                and self.direction_changes >= max(2, self.changes) and self.reversals >= 1):
            self.latched = True
            return True
        return False

    def _measure(self):
        if not self.history:
            self.amplitude, self.reversals, self.direction_changes = 0.0, 0, 0
            return False
        xs = [p[1] for p in self.history]
        self.amplitude = max(xs) - min(xs)
        anchor = extreme = xs[0]
        direction, turns = 0, 0
        legs = []

        def finish_leg():
            if (abs(extreme - anchor) + 1e-9 >= self.amplitude_threshold
                    and (not legs or legs[-1] != direction)):
                legs.append(direction)

        for value in xs[1:]:
            if direction == 0:
                if abs(value - anchor) >= self.reversal_distance:
                    direction = 1 if value > anchor else -1
                    extreme = value
            elif direction > 0:
                if value >= extreme:
                    extreme = value
                elif extreme - value >= self.reversal_distance:
                    finish_leg()
                    anchor = extreme
                    direction, extreme, turns = -1, value, turns + 1
            else:
                if value <= extreme:
                    extreme = value
                elif value - extreme >= self.reversal_distance:
                    finish_leg()
                    anchor = extreme
                    direction, extreme, turns = 1, value, turns + 1
        if direction:
            finish_leg()
        self.reversals, self.direction_changes = turns, len(legs)
