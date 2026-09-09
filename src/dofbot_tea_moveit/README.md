# dofbot_tea_moveit

Bài toán rót trà cho Dofbot 6DOF bằng MoveIt2 (C++): gắp ly → gắp ấm →
rót → đặt về. Chạy trên `move_group` + ros2_controllers thật (fake/hardware),
không cần camera.

## Kiến trúc

```
tea_backend.launch.py      tea_task.launch.py
  (static_tf,                 (tea_task node:
   robot_state_publisher,      đọc config/tea_task.yaml,
   ros2_control_node,           lập plan + thực thi
   spawners,                   từng bước gắp/rót)
   move_group)
```

- `src/tea_task.cpp`: node `tea_moveit_app`, đọc `config/tea_task.yaml`
  (tọa độ bàn/ly/ấm, tốc độ, tolerance) + `config/dofbot_tea.srdf`.
- Services: `/tea/next` (chạy bước tiếp theo), `/tea/status` (trạng thái),
  `/tea/abort` (dừng khẩn).
- `src/smoke.cpp` / `src/xyz_test.cpp`: node test nhanh qua
  `launch/smoke.launch.py` và `launch/xyz_test.launch.py`.

## Cài đặt (standalone, chỉ cần repo này)

```bash
git clone <url> dofbot_robot_arm_6dof
cd dofbot_robot_arm_6dof
./setup_standalone.sh   # cài dep, lấy dofbot_urdf, colcon build
source install/setup.bash
```

Build lại sau khi sửa code (chạy tại workspace root):

```bash
cd ~/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash
colcon build --packages-select dofbot_tea_moveit
source install/setup.bash
```

## Chạy

Mỗi terminal đều source trước:

```bash
source /opt/ros/humble/setup.bash
source ~/dofbot_robot_arm_6dof/install/setup.bash
```

```bash
# terminal 1: move_group + controllers
ros2 launch dofbot_tea_moveit tea_backend.launch.py
# terminal 2: task rót trà
ros2 launch dofbot_tea_moveit tea_task.launch.py
# terminal 3 (optional): RViz lite (mặc định, phù hợp máy yếu)
ros2 launch dofbot_tea_moveit tea_rviz.launch.py
```

Launch Tea dùng robot `dofbot.urdf.xacro` đầy đủ; RViz dùng profile lite
`tea_view_lite.rviz`: chỉ hiển thị robot và proxy primitive nhẹ cho ly/ấm,
không render STL gốc gần 500k triangles. Scene
chỉ publish khi task thay đổi trạng thái, nên không có redraw định kỳ lúc idle.
Khi cần debug quỹ đạo, bật display `MotionPlanning (enable for trajectory)`
trong profile lite; nó tắt mặc định để giữ FPS.

Điều khiển task qua service (terminal đã source):

```bash
ros2 service call /tea/status std_srvs/srv/Trigger '{}'
ros2 service call /tea/next std_srvs/srv/Trigger '{}'
ros2 service call /tea/abort std_srvs/srv/Trigger '{}'
```

## Chỉnh trước khi chạy

- `config/tea_task.yaml`: `cup_x/y`, `teapot_x/y`, `table_*`, `velocity_scaling`,
  `pour_joint_delta_deg` → khớp vị trí ly/ấm/bàn thật của bạn.
- Tune từng tham số vài mm/độ rồi chạy lại `/tea/next` để kiểm tra từng bước.
