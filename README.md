# TurtleBot 4 Gesture Control – Assignment 6

Điều khiển TurtleBot 4 trong Gazebo bằng cử chỉ tay từ webcam sử dụng MediaPipe và ROS 2 Humble.

## Gestures

| Gesture | Action |
|---|---|
| Wave | Select Greeting |
| Pointing Up | Select Forward/Backward |
| Victory | Select Circle |
| Thumb Up | Confirm / restart from beginning |
| Open Palm | Stop; keep selection |
| Closed Fist | Cancel; clear selection |

## Requirements

- Ubuntu 22.04, ROS 2 Humble, Python 3.10.
- TurtleBot 4 Simulator (Ignition Fortress), desktop/OpenGL, webcam `/dev/video0`.
- Công cụ build và venv: `sudo apt install python3-colcon-common-extensions python3-venv`.
- Nếu chưa có simulator: `sudo apt install ros-humble-turtlebot4-simulator`.

## Setup

```bash
cd tb4_gesture_assignment_6
bash scripts/setup.sh
```

Setup tạo `.venv` riêng và kiểm tra model MediaPipe đi kèm (tự tải model chính thức nếu thiếu).

## Build

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python /usr/bin/colcon build --packages-select tb4_gesture_control
source install/setup.bash
source scripts/env.sh
```

## Run

Mở cả ba terminal tại thư mục repo. Dùng cùng `ROS_DOMAIN_ID` và RMW nếu đã tùy chỉnh.

Terminal 1 – TurtleBot 4 Simulation:

```bash
source scripts/env.sh
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze model:=standard
```

Terminal 2 – Undock (chờ Gazebo spawn xong và đang Play):

```bash
source scripts/env.sh
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
ros2 param set /motion_control safety_override backup_only
```

Chờ undock thành công trước khi điều khiển. `backup_only` chỉ dùng cho mô phỏng để cho phép lùi; khôi phục bằng `ros2 param set /motion_control safety_override none`.

Terminal 3 – Gesture Control:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/env.sh
ros2 launch tb4_gesture_control gesture_control.launch.py use_sim_time:=true
```

Chọn behavior rồi Thumb Up để chạy. Open Palm dừng và giữ lựa chọn; Thumb Up chạy lại từ đầu. Closed Fist hủy lựa chọn. Q/Esc hoặc Ctrl+C để thoát.

Optional debug (terminal khác tại thư mục repo):

```bash
source scripts/env.sh
ros2 topic echo /detected_gesture
```
