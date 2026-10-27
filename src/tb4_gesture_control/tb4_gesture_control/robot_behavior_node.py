"""Nonblocking confirmed behaviors, with wall-clock watchdog and stop priority."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Empty, String


BEHAVIOR_DEFAULTS = {
    'forward_speed': 0.15,
    'backward_speed': 0.15,
    'angular_speed': 0.6,
    'circle_linear_speed': 0.14,
    'circle_angular_speed': 0.35,
    'greeting_phase_duration': 0.7,
    'forward_back_phase_duration': 2.0,
    'direction_change_pause': 0.2,
    'circle_duration': 30.0,  # Simulation/ROS-time fallback; odometry normally ends the circle.
    'control_timeout': 1.0,
}
SELECTIONS = {'Wave': 'GREETING', 'Pointing_Up': 'FORWARD_BACK', 'Victory': 'CIRCLE'}


class RobotBehaviorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_behavior_node', **kwargs)
        self.config = {}
        for name, default in BEHAVIOR_DEFAULTS.items():
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            self.config[name] = value
        self.state = 'IDLE'
        self.pending_behavior = None
        self.phases = []
        self.phase_index = 0
        self.phase_started = 0.0
        self.last_heartbeat = None
        self.watchdog_tripped = False
        self.odom_yaw = None
        self.odom_received = None
        self.circle_last_yaw = None
        self.circle_accumulated_yaw = 0.0
        # Depth one avoids a backlog of obsolete velocity commands/heartbeats.
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=1))
        self.state_pub = self.create_publisher(String, '/gesture_control_state', 1)
        self.create_subscription(String, '/detected_gesture', self.on_gesture, 10)
        self.create_subscription(Empty, '/gesture_heartbeat', self.on_heartbeat, 1)
        self.create_subscription(Odometry, '/odom', self.on_odometry, qos_profile_sensor_data)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.wall_clock)
        self.publish_velocity()
        self.publish_state()
        self.get_logger().info('IDLE; waiting for perception heartbeat and selection + Thumb_Up')

    def healthy(self, now):
        return (self.last_heartbeat is not None
                and now - self.last_heartbeat <= self.config['control_timeout'])

    def on_heartbeat(self, _message):
        self.last_heartbeat = time.monotonic()
        self.watchdog_tripped = False

    def behavior_time(self):
        """ROS time follows /clock in Gazebo; never use it for heartbeat age."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odometry(self, message):
        q = message.pose.pose.orientation
        components = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in components):
            return
        norm = math.sqrt(sum(v * v for v in components))
        if norm < 1e-9:
            return
        x, y, z, w = (v / norm for v in components)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.odom_yaw = yaw
        self.odom_received = time.monotonic()
        if self.state == 'EXECUTING' and self.pending_behavior == 'CIRCLE':
            delta = yaw - self.circle_last_yaw
            delta = math.atan2(math.sin(delta), math.cos(delta))
            self.circle_accumulated_yaw += delta
            self.circle_last_yaw = yaw
            # Signed accumulation: opposite rotation/jitter does not add progress.
            if self.circle_accumulated_yaw >= math.tau - 1e-3:
                self.stop('Circle complete: odometry reached 2*pi')

    def publish_velocity(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.velocity_pub.publish(message)

    def publish_state(self):
        detail = self.pending_behavior or '-'
        self.state_pub.publish(String(data=f'{self.state} {detail}'))

    def stop(self, reason):
        self.publish_velocity()
        self.phases.clear()
        self.pending_behavior = None
        self.state = 'IDLE'
        self.circle_last_yaw = None
        self.publish_state()
        self.get_logger().info(f'{reason} -> IDLE')

    def reset_progress(self):
        self.phases.clear()
        self.phase_index = 0
        self.phase_started = 0.0
        self.circle_last_yaw = None
        self.circle_accumulated_yaw = 0.0

    def temporary_stop(self):
        self.publish_velocity()
        self.reset_progress()
        self.state = 'SELECTED' if self.pending_behavior is not None else 'IDLE'
        self.publish_state()
        self.get_logger().info(f'STOP -> {self.state}; pending={self.pending_behavior or "-"}')

    def on_gesture(self, message):
        gesture = message.data
        # Safety callbacks are always accepted, even without heartbeat.
        if gesture == 'Open_Palm':
            self.temporary_stop()
            return
        if gesture == 'Closed_Fist':
            self.stop('CANCEL')
            self.reset_progress()
            return
        if not self.healthy(time.monotonic()):
            self.stop('Perception unavailable; gesture rejected')
            return
        if self.state == 'EXECUTING':
            return  # A running behavior can only be interrupted by stop/cancel/watchdog.
        if gesture in SELECTIONS:
            self.pending_behavior = SELECTIONS[gesture]
            self.state = 'SELECTED'
            self.publish_velocity()
            self.publish_state()
            self.get_logger().info(f'SELECTED {self.pending_behavior}; show Thumb_Up to confirm')
        elif gesture == 'Thumb_Up' and self.state == 'SELECTED' and self.pending_behavior:
            now = self.behavior_time()
            if self.get_parameter('use_sim_time').value and now <= 0:
                self.stop('No simulation clock; confirmation rejected')
                return
            if self.pending_behavior == 'CIRCLE':
                if (self.odom_received is None or time.monotonic() - self.odom_received
                        > self.config['control_timeout']):
                    self.stop('No fresh odometry; circle confirmation rejected')
                    return
                self.circle_last_yaw = self.odom_yaw
                self.circle_accumulated_yaw = 0.0
            self.phases = self.make_phases(self.pending_behavior)
            self.phase_index = 0
            self.phase_started = now
            self.state = 'EXECUTING'
            self.publish_state()
            self.get_logger().info(f'EXECUTING {self.pending_behavior}')

    def make_phases(self, behavior):
        c = self.config
        if behavior == 'GREETING':
            # Equal left/right durations; three complete pairs.
            motion = [(c['greeting_phase_duration'], 0.0, sign * c['angular_speed'])
                      for _ in range(3) for sign in (1, -1)]
        elif behavior == 'FORWARD_BACK':
            motion = [phase for _ in range(4) for phase in (
                (c['forward_back_phase_duration'], c['forward_speed'], 0.0),
                (c['forward_back_phase_duration'], -c['backward_speed'], 0.0))]
        else:
            return [(c['circle_duration'], c['circle_linear_speed'], c['circle_angular_speed'])]
        # Reuse ordinary timed phases: stop before every direction reversal,
        # including the transition from one pair/cycle to the next.
        phases = []
        for index, phase in enumerate(motion):
            if index:
                phases.append((c['direction_change_pause'], 0.0, 0.0))
            phases.append(phase)
        return phases

    def tick(self):
        now = time.monotonic()
        if not self.healthy(now):
            if not self.watchdog_tripped:
                self.stop('WATCHDOG timeout')
                self.watchdog_tripped = True
            self.publish_velocity()
        elif self.state == 'EXECUTING':
            behavior_now = self.behavior_time()
            if behavior_now < self.phase_started:
                self.stop('ROS clock moved backward')
                return
            duration, linear, angular = self.phases[self.phase_index]
            if behavior_now - self.phase_started >= duration:
                self.phase_index += 1
                if self.phase_index >= len(self.phases):
                    self.stop('Circle odometry timeout (ROS time)'
                              if self.pending_behavior == 'CIRCLE' else 'Behavior complete')
                else:
                    # Start a fresh phase at the actual transition time. Avoid
                    # skipping motion phases when the executor was delayed.
                    self.phase_started = behavior_now
                    _, linear, angular = self.phases[self.phase_index]
                    self.publish_velocity(linear, angular)
            else:
                self.publish_velocity(linear, angular)
        else:
            self.publish_velocity()
        self.publish_state()

    def shutdown_stop(self):
        if self.context.ok():
            self.stop('Shutdown')
            self.publish_velocity()


def main(args=None):
    # Handle Ctrl+C ourselves, allowing a final zero Twist before ROS shutdown.
    from rclpy.signals import SignalHandlerOptions
    import signal

    def terminate(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = RobotBehaviorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
