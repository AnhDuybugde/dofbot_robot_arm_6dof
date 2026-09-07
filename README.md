# dofbot_tea_chess — standalone workspace (no dofbot_ws needed)

Repo này là một colcon workspace hoàn chỉnh. Clone về là build/chạy được luôn.

## Layout

```
dofbot_tea_chess/            <- workspace root (chạy colcon build TẠI ĐÂY)
  src/
    chess_moveit_demo/       <- bài toán cờ (Stockfish self-play + MoveIt2 pick-place)
    dofbot_tea_moveit/       <- bài toán trà (C++ MoveIt2)
    dofbot_moveit/           <- vendored MoveIt config (Yahboom)
    pymoveit2/               <- vendored + patch no-spin (xem VENDOR.txt)
    dofbot_urdf/             <- KHÔNG vendor (~104M), setup script tự copy từ máy robot
  setup_standalone.sh
```

## Clone + build (máy mới, chỉ cần repo này)

```bash
git clone <url> dofbot_tea_chess
cd dofbot_tea_chess
./setup_standalone.sh
source install/setup.bash
```

`setup_standalone.sh` làm:
1. `apt install stockfish` + `pip install chess`
2. Lấy `dofbot_urdf`: copy từ `~/dofbot_ws/src/dofbot_urdf` nếu máy còn ws cũ,
   nếu không thì báo lỗi và yêu cầu copy package Yahboom vào `src/dofbot_urdf`.
3. `colcon build` 5 package.

## Chạy (mỗi terminal đều source trước)

```bash
source /opt/ros/humble/setup.bash
source ~/dofbot_tea_chess/install/setup.bash
```

Chess base (không MoveIt):
```bash
ros2 launch chess_moveit_demo chess_base.launch.py
# terminal khác:
ros2 service call /chess/start std_srvs/srv/Trigger '{}'
```

Chess full sim (MoveIt + RViz):
```bash
ros2 launch chess_moveit_demo chess_sim.launch.py
# terminal khác:
ros2 service call /chess/check_reachability std_srvs/srv/Trigger '{}'
ros2 service call /chess/start std_srvs/srv/Trigger '{}'
```

Tea:
```bash
# terminal 1: move_group + controllers
ros2 launch dofbot_tea_moveit tea_backend.launch.py
# terminal 2: task
ros2 launch dofbot_tea_moveit tea_task.launch.py
# terminal 3 (optional): RViz
ros2 launch dofbot_tea_moveit tea_rviz.launch.py
```

## Ghi chú vendor

- `src/pymoveit2/VENDOR.txt`: snapshot upstream `149c164` + patch bắt buộc
  (bỏ `rclpy.spin_once` → `time.sleep`), nếu checkout upstream sạch sẽ treo.
- `src/dofbot_moveit`: config Yahboom, `exec_depend` vào `dofbot_urdf`.
