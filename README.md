# dofbot_robot_arm_6dof — standalone workspace (no dofbot_ws needed)

Repo này là một colcon workspace hoàn chỉnh. Clone về là build/chạy được luôn.
Mặc định là bản **lightweight**: ưu tiên RViz/MoveIt phản hồi mượt trên Dofbot,
không bật các thành phần camera 3D hay mô phỏng không dùng.

> Workspace root hiện tại là `dofbot_robot_arm_6dof`.
> Mọi lệnh dưới đây giả định bạn đang đứng tại workspace root.

## Layout

```
dofbot_robot_arm_6dof/      <- workspace root (chạy colcon build TẠI ĐÂY)
  src/
    chess_moveit_demo/       <- cờ vua (Stockfish self-play + MoveIt2 pick-place, Python)
    dofbot_tea_moveit/       <- rót trà (C++ MoveIt2)
    dofbot_moveit/           <- vendored MoveIt config (Yahboom)
    pymoveit2/               <- vendored + patch no-spin (xem VENDOR.txt)
    dofbot_urdf/             <- Yahboom meshes (~104M), Git bỏ qua (xem dưới)
  setup_standalone.sh
```

## Clone + build (máy mới, chỉ cần repo này)

```bash
git clone <url> dofbot_robot_arm_6dof
cd dofbot_robot_arm_6dof
./setup_standalone.sh
source install/setup.bash
```

`setup_standalone.sh` làm:
1. `apt install stockfish` + `pip install chess`
2. Lấy `dofbot_urdf`: dùng sẵn `src/dofbot_urdf` nếu có, nếu không thì copy từ
   `~/dofbot_ws/src/dofbot_urdf` / `/home/yahboom/dofbot_ws/src/dofbot_urdf` /
   `~/Dofbot/dofbot_urdf`; nếu không tìm thấy thì báo lỗi và dừng.
3. `colcon build --packages-select pymoveit2 dofbot_urdf dofbot_moveit dofbot_tea_moveit chess_moveit_demo`

Build lại sau khi sửa code (tại workspace root):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select chess_moveit_demo   # hoặc dofbot_tea_moveit
source install/setup.bash
```

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
source ~/dofbot_robot_arm_6dof/install/setup.bash
# (hoặc nếu đang đứng tại workspace root: source install/setup.bash)
```

### Chess base (không MoveIt — calibrate luật/visual)
```bash
ros2 launch chess_moveit_demo chess_base.launch.py
# terminal khác:
ros2 service call /chess/start std_srvs/srv/Trigger '{}'
```
Chế độ này không import/chạy `pymoveit2`, `move_group` hay controller:
chỉ kiểm tra Stockfish self-play, luật `python-chess`, ACK và visual đủ 32 quân.
Chi tiết: `src/chess_moveit_demo/README.md`.

### Chess full sim (MoveIt + RViz)
```bash
ros2 launch chess_moveit_demo chess_sim.launch.py
# terminal khác (đã source):
ros2 service call /chess/check_reachability std_srvs/srv/Trigger '{}'
ros2 service call /chess/start std_srvs/srv/Trigger '{}'
```

Luồng mới (READY-gated, fail-loud):
- `pick_place_node` tự dựng PlanningScene **nguyên tử 1 board + 32 quân = 33 objects**,
  retry tới khi `move_group` serve (budget 600s, retry mỗi 10s, kiểm tra pose lại
  với tolerance 5mm), rồi kiểm tra planner + controller + `/joint_states` tươi
  (<1.0s). Xong mới publish `READY` lên `/chess/system_ready` (transient-local latch).
- `chess_brain_node` **chờ READY** mới cho `/chess/start`. Start khi chưa READY
  trả `success=False`. Board của brain chỉ commit **sau ACK** khớp
  `command_id:uci`; NACK (`/chess/move_failed`) dừng + **khóa restart**
  (`locked_after_failure`) — phải restart launch mới chơi ván mới. Watchdog 600s
  chống treo không ACK/NACK.
- `/chess/check_reachability`: kiểm tra 2 tầng (quét nhanh 64 ô + dry-run chuỗi
  runtime trên 10 ô đại diện + 16 slot discard). Trên sim, deep-check **có execute
  thật + attach/detach** rồi trả quân về ô nguồn, collision vẫn bật. Khi lỗi in
  block `REACHABILITY-FAIL-SUMMARY` (`scope | phase | case | reason`) + dòng
  `COLLISION-CONTACT` nếu MoveIt phát hiện cặp va chạm cụ thể.
- Trước khi dùng arm thật, đặt `REACHABILITY_EXECUTE_ON_FAKESYSTEM=False`
  trong `chess_utils.py`.

### Thông số bàn cờ/robot hiện tại (CHỐT trong `chess_utils.py`)

- Bàn thật 24cm: tâm `(0.2015, -0.0005)`, mặt `z=0.005`, ô `26mm`,
  tâm a1 `(0.1105, -0.0915)`. Mapping **X=rank, Y=file** (hàng 1 gần robot).
- TCP gắp theo loại quân (`PIECE_GRIP_Z_SIM`, neo tốt p=55mm):
  `p=55, r=58, n=59, b=61, q=64, k=66mm`. Collision là cylinder có margin
  (+3mm cao, +0.5–1mm radius); visual là marker RViz.
- Approach/lift/retreat chung `65mm`; riêng hàng 1 gần đế (`c1/d1/e1/f1`)
  hạ approach còn `45mm`, và `e1/d1` có override tuyệt đối (`e1→70mm`,
  `d1→80mm`) vì KDL position-only không có nghiệm ở vùng cao.
- Candidate gắp: thử tâm ô trước, rồi lệch hữu hạn `3 → 6 → 8mm`
  (tối đa `8.5mm` = bán kính tốt, timeout 120s/ô), chấm điểm theo
  vị trí/tilt/quãng joint/margin limit, cache offset theo ô. Điểm đặt,
  collision và visual luôn ở tâm ô.
- Tilt 5-DOF: target ≤11°, hard-reject >26°; mặc định dùng candidate scoring
  (`USE_UPRIGHT_ORIENTATION_CONSTRAINT=False`).
- Quân bị ăn vào "nghĩa địa" lưới `4x4=16 slot` tại `(0.09, -0.13)`,
  bước `33mm` (bản cũ xếp 1 hàng +Y vô hạn đã gây treo vì ngoài tầm với).
- Attach/detach chuẩn MoveIt2 qua `/apply_planning_scene`: mở ACM tạm thời
  giữa quân mục tiêu với ngón kẹp/mặt bàn, giữ tới khi về HOME rồi mới đóng;
  marker visual đổi sang frame `Gripping_point_Link` nên quân "dính" theo tay
  trên RViz. Mỗi quân giữ id cố định `piece_000…` (không đặt id theo ô).
- Robot: base `base_link` (+x trước, +y ngang), TCP `Gripping_point_Link`,
  IK position-only 5-DOF, gripper chỉ command `Rlink1_Joint`
  (`0.0`=mở, `1.57`=đóng, các joint L/R là mimic).

### Cổng an toàn phần cứng (chưa calibrate → chặn execute thật)

- `TCP_OFFSET_CALIBRATED=False`: sim chạy được, hardware execute bị chặn.
  Đo `OFFSET = measured_tcp_z - BOARD_TOP_Z - PIECE_GRASP_HEIGHT[type]` trên
  phần cứng rồi mới bật.
- `GRIPPER_CALIBRATION_TABLE=[]`: `gripper_width_to_joint_angle()` raise,
  giữ gripper binary; cấm nội suy tuyến tính `angle = width/max*1.57` vì mimic
  phi tuyến. Cần đo FK RViz + hiệu chỉnh servo tại 20/14/12mm.
- `HARDWARE_SAFE_VELOCITY_SCALE=0.25` khi test phần cứng tốc độ thấp/không tải.

### Tea
```bash
# terminal 1: move_group + controllers
ros2 launch dofbot_tea_moveit tea_backend.launch.py
# terminal 2: task
ros2 launch dofbot_tea_moveit tea_task.launch.py
# terminal 3 (optional): RViz lite (mặc định, phù hợp máy yếu)
ros2 launch dofbot_tea_moveit tea_rviz.launch.py
# điều khiển từng bước:
ros2 service call /tea/status std_srvs/srv/Trigger '{}'
ros2 service call /tea/next std_srvs/srv/Trigger '{}'
ros2 service call /tea/abort std_srvs/srv/Trigger '{}'
```
`tea_backend` dùng `config/tea_joint_limits.yaml` conservative (1.0 rad/s)
thay cho limit ảo 1000 rad/s của config generic; task đọc
`config/tea_task.yaml` + `config/dofbot_tea.srdf`. Chỉnh `cup_x/y`,
`teapot_x/y`, `table_*`, `velocity_scaling`, `pour_joint_delta_deg` theo
bàn thật. Chi tiết: `src/dofbot_tea_moveit/README.md`.

Chess (`config/chess_lite.rviz`) và Tea (`config/tea_view_lite.rviz`) mặc định dùng
profile RViz lite: robot + proxy primitive nhẹ, tắt `MotionPlanning` trajectory,
scene geometry/collision/octomap mặc định để giữ FPS trên máy yếu.
Tên link/joint, collision proxy và MoveIt kinematics giữ nguyên.
Khi cần debug quỹ đạo, bật display `MotionPlanning (enable for trajectory)`
trong panel Displays (FPS sẽ giảm). `/tea_scene` chỉ publish khi task đổi trạng
thái; scene cờ `33 objects` verify nguyên tử lúc init, visual marker publish
một lần thay vì sau từng quân. Gói `dofbot_lite.urdf.xacro`
(14 box thay mesh STL) vẫn có sẵn trong `dofbot_moveit/config` + launch
`demo_lite.launch.py` để test khi cần, nhưng `chess_sim`/`tea_backend` hiện
dùng `dofbot.urdf.xacro` đầy đủ.

## Ghi chú vendor

- `src/pymoveit2/VENDOR.txt`: snapshot upstream `149c164` + patch bắt buộc
  (bỏ `rclpy.spin_once` → `time.sleep`), nếu checkout upstream sạch sẽ treo.
- `src/dofbot_moveit`: config Yahboom, `exec_depend` vào `dofbot_urdf`.
