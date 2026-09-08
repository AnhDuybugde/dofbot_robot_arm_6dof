# dofbot_tea_chess — standalone workspace (no dofbot_ws needed)

Repo này là một colcon workspace hoàn chỉnh. Clone về là build/chạy được luôn.
Mặc định là bản **lightweight**: ưu tiên RViz/MoveIt phản hồi mượt trên Dofbot,
không bật các thành phần camera 3D hay mô phỏng không dùng.

## Layout

```
dofbot_tea_chess/            <- workspace root (chạy colcon build TẠI ĐÂY)
  src/
    chess_moveit_demo/       <- bài toán cờ (Stockfish self-play + MoveIt2 pick-place)
    dofbot_tea_moveit/       <- bài toán trà (C++ MoveIt2)
    dofbot_moveit/           <- vendored MoveIt config (Yahboom)
    pymoveit2/               <- vendored + patch no-spin (xem VENDOR.txt)
    dofbot_urdf/             <- có sẵn cục bộ (~104M), nhưng Git bỏ qua (không push)
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

## File Yahboom không đưa lên Git

`src/dofbot_urdf/` đã có trong bản workspace cục bộ này để chạy độc lập, nhưng
được giữ trong `.gitignore` vì chứa mesh STL khoảng 104 MB. Khi clone repo trên
máy khác, tải driver/tài nguyên Yahboom tại
[Google Drive (driver)](https://drive.google.com/drive/folders/1N8DdsQJRkj8_7xfk3T-jFWvssnKH7QYD?usp=sharing),
rồi đặt **cả thư mục** `dofbot_urdf` vào `src/dofbot_urdf/` trước khi build.

### Driver cho robot thật (làm thủ công)

Demo tea/chess hiện dùng `mock_components/GenericSystem`, tức là mô phỏng
MoveIt/RViz; `dofbot_driver` **không cần** để build hay chạy mô phỏng. Nếu muốn
thử gửi `/joint_states` sang cánh tay thật, copy nguyên package sau từ driver
Yahboom vào `src/dofbot_driver/`:

```
dofbot_driver/package.xml
dofbot_driver/setup.py
dofbot_driver/setup.cfg
dofbot_driver/resource/dofbot_driver
dofbot_driver/dofbot_driver/__init__.py
dofbot_driver/dofbot_driver/dofbot_driver.py
```

Node cần chạy là `ros2 run dofbot_driver dofbot_driver`; nó còn cần thư viện
Python `Arm_Lib` của Yahboom. Không cần copy `arm_driver.py`, các file
AprilTag, màu sắc, hay `config/offset_value.yaml` cho luồng trà/cờ hiện tại.
Lưu ý: đây chỉ là bridge đơn giản từ `/joint_states` sang servo; để dùng robot
thật an toàn, vẫn cần thay hardware `mock_components/GenericSystem` trong
`dofbot_moveit` bằng ros2_control hardware interface phù hợp và calibrate trước.

## Chế độ lightweight

Các thay đổi này chỉ giảm tải hiển thị và Planning Scene; luồng gắp/rót và luật
cờ không đổi.

- **Không khởi tạo Octomap/depth camera**: `sensors_3d.yaml` để `sensors: []`.
  Dofbot demo không dùng camera sâu, nên tránh một monitor rỗng cập nhật liên tục.
- **RViz ít render hơn**: tắt scene geometry/collision/octomap mặc định, tắt
  animation lặp và giảm tốc độ hiển thị trajectory. Robot visual và marker bàn
  cờ/ly/ấm vẫn hiện.
- **Cờ**: lúc khởi tạo, bàn và 32 quân chỉ publish MarkerArray một lần thay vì
  publish đầy đủ sau từng quân.
- **Trà**: mesh visual của ly/ấm vẫn đầy đủ, nhưng mesh dùng để tính collision
  được lấy thưa khoảng 75 lần (ly) và 150 lần (ấm); visual chỉ refresh mỗi 2 s
  khi idle, còn lúc task thay đổi scene vẫn publish ngay.

Đánh đổi: collision của ly/ấm nhẹ hơn nhưng **kém chính xác hơn** bản mesh đầy
đủ. Dùng bản này để mô phỏng, tune toạ độ và kiểm tra luồng; trước khi chạy
robot thật gần vật thể, cần kiểm tra lại đường đi và khoảng hở trong RViz.

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
# terminal 3 (optional): RViz lite (mặc định, phù hợp máy yếu)
ros2 launch dofbot_tea_moveit tea_rviz.launch.py
```

Chess và Tea mặc định dùng `dofbot_lite`: 14 box primitive thay cho hơn 80 MB
mesh STL visual của robot. Tên link/joint, collision proxy và MoveIt
kinematics giữ nguyên; chỉ hình robot trong RViz trở nên tối giản để tăng FPS.

`tea_rviz.launch.py` mặc định dùng profile lite: robot + proxy primitive nhẹ
cho ly/ấm, không render hai STL gốc gần 500k triangles. `/tea_scene` chỉ được
publish khi task đổi trạng thái. Khi cần debug quỹ đạo, bật display
`MotionPlanning (enable for trajectory)` trong profile lite; nó tắt mặc định
để giữ FPS.

## Ghi chú vendor

- `src/pymoveit2/VENDOR.txt`: snapshot upstream `149c164` + patch bắt buộc
  (bỏ `rclpy.spin_once` → `time.sleep`), nếu checkout upstream sạch sẽ treo.
- `src/dofbot_moveit`: config Yahboom, `exec_depend` vào `dofbot_urdf`.
