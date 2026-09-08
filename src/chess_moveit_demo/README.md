# chess_moveit_demo

## Chạy base, không MoveIt

Khi đang calibrate vùng với tới hoặc va chạm của Dofbot, dùng chế độ này để
kiểm tra trọn luồng Stockfish, luật `python-chess`, ACK và visual đủ 32 quân.
Nó không import hay chạy `pymoveit2`, `move_group` hoặc controller robot:

```bash
cd ~/dofbot_tea_chess
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch chess_moveit_demo chess_base.launch.py
```

Trong terminal khác gọi `ros2 service call /chess/start std_srvs/srv/Trigger '{}'`.

Khung sườn end-to-end: robot 6DOF "chơi cờ" trong RViz/MoveIt2, không cần camera
hay bàn cờ thật — Stockfish tự chơi cả 2 bên (self-play). Robot mô phỏng chỉ
thực hiện quân Trắng; quân Đen tự dịch marker/Planning Scene. Các lượt robot
dùng MoveIt2 với Planning Scene được cập nhật động.

## Kiến trúc

```
chess_brain_node          pick_place_node
  (python-chess    --/chess/move-->   (pymoveit2:
   + Stockfish                          move_to_pose,
   self-play)                           add/remove/move
        <--/chess/move_done--          collision object,
                                        gripper)
```

- `chess_brain_node`: giữ 1 `chess.Board()`, hỏi Stockfish lần lượt từng nước.
  Payload có `execution`: quân **Trắng** là `robot` (arm gắp/thả), quân **Đen**
  là `virtual` (tự dịch marker/scene, arm đứng yên). Brain luôn CHỜ
  `/chess/move_done` trước nước kế tiếp.
- `pick_place_node`: nhận nước đi, phân rã thành
  HOME joint PTP → Pre-grasp (OMPL) → Cartesian Pick/Lift → Move (OMPL) →
  Cartesian Place → HOME joint PTP (thêm bước "discard" nếu ăn quân, và 2 lần
  pick-place nếu nhập thành), gọi
  MoveIt2 thực thi, đồng thời
  add/remove/move các collision object (box cho bàn cờ, cylinder cho từng quân)
  trong `chess_utils.py`.
- `chess_utils.py`: mapping ô cờ (`e4`) → toạ độ Cartesian, chiều cao gắp và độ
  mở gripper theo từng loại quân — đây là những con số BẮT BUỘC PHẢI CHỈNH theo
  robot/bàn cờ ảo của bạn.

## Cài đặt (standalone, chỉ cần repo này)

```bash
git clone <url> dofbot_tea_chess
cd dofbot_tea_chess
./setup_standalone.sh   # cài stockfish + python-chess, lấy dofbot_urdf, colcon build
source install/setup.bash
```

Build lại sau khi sửa code (chạy tại workspace root):

```bash
cd ~/dofbot_tea_chess
source /opt/ros/humble/setup.bash
colcon build --packages-select chess_moveit_demo
source install/setup.bash
```

Yêu cầu: `stockfish` (`which stockfish`), `pip install chess`,
`pymoveit2` + `dofbot_moveit` đã vendored sẵn trong `src/`.

## Việc BẮT BUỘC phải chỉnh trước khi chạy

1. `chess_moveit_demo/pick_place_node.py`:
   - `JOINT_NAMES`, `BASE_LINK`, `END_EFFECTOR`, `GROUP_NAME`, `GRIPPER_JOINT`
     → lấy đúng từ gói MoveIt2 config bạn đã tạo cho task ấm trà (SRDF/joint
     names giống hệt, không cần làm lại).
   - `STOCKFISH_PATH` trong `chess_brain_node.py` → đường dẫn binary thật.
2. `chess_moveit_demo/chess_utils.py`:
   - `BOARD_ORIGIN`, `SQUARE_SIZE`, `BOARD_Z` → toạ độ bàn cờ ảo trong scene
     của bạn (nếu chưa có mesh bàn cờ, cứ để robot thao tác trên một mặt phẳng
     tưởng tượng, không bắt buộc phải có model 3D bàn cờ).
   - `PICK_TCP_Z` → cao độ của **Gripping_point_Link** khi kẹp quân. Đây là
     tham số phải tune riêng (mặc định `0.055` m), không phải chiều cao quân.
   - `PIECE_SPECS` → chiều cao collision/visual và độ mở gripper từng loại quân.
3. `launch/chess_sim.launch.py` dùng sẵn `dofbot_moveit` đã vendored trong
   `src/` — không cần đổi gì thêm.

### Dofbot hiện có trong workspace này

- `base_link`: gốc hệ toạ độ; `+x` là hướng ra phía trước robot, `+y` là ngang.
- Bàn mặc định có tâm gần `(0.13, 0.0, 0.005)` m, cùng vùng làm việc với task trà.
- TCP là `Gripping_point_Link`. URDF định nghĩa fixed transform
  `Gripping_Joint rpy=(pi, -pi/2, 0)`. Cấu hình Dofbot hiện dùng **position-only
  IK** (5 DOF), nên demo gửi target XYZ cho TCP và không ép quaternion 6D.
  Đây tránh lỗi không có nghiệm orientation ở các ô cờ.
- Gripper chỉ command `Rlink1_Joint`: `0.0` = mở, `1.57` rad = đóng. Các joint
  L/R còn lại là mimic; không publish trực tiếp `/joint_states` để thử gripper.

## Chạy

Mỗi terminal đều source trước:

```bash
source /opt/ros/humble/setup.bash
source ~/dofbot_tea_chess/install/setup.bash
```

Lite sim mặc định (MoveIt + RViz nhẹ):

```bash
ros2 launch chess_moveit_demo chess_sim.launch.py
```

RViz mở với robot và đủ bàn cờ, nhưng robot là `dofbot_lite` (14 box primitive
thay cho mesh STL high-poly) và `MotionPlanning` tắt để tránh render planning
scene/trajectory nặng (phù hợp máy yếu). Link/joint, IK và collision của MoveIt
không đổi. Khi cần debug quỹ đạo, tick
`MotionPlanning (enable for trajectory)` trong panel **Displays**; FPS sẽ giảm.
Khi muốn bắt đầu self-play, mở terminal
khác (đã source workspace) rồi gọi:

```bash
ros2 service call /chess/start std_srvs/srv/Trigger '{}'
```

Node brain sẽ publish nước Trắng đầu tiên để robot thực hiện; sau ACK, nước
Đen tự cập nhật trên RViz rồi tới lượt Trắng kế tiếp.

Trước khi bấm `/chess/start`, kiểm tra vùng làm việc của đúng vị trí bàn hiện
tại (2 tầng: quét nhanh 64 ô + dry-run đúng chuỗi runtime trên 12 ô đại diện
và khu discard):

```bash
ros2 service call /chess/check_reachability std_srvs/srv/Trigger '{}'
```

Ở sim (FakeSystem) tầng deep **có di chuyển arm thật** (execute dry-run không
kẹp quân); ở robot thật chỉ plan-only. `success=False` nếu bất kỳ phase nào
(approach/hạ/nâng Cartesian/chuyển ô/discard) lỗi — khác bản cũ luôn True.

Terminal đang chạy launch sẽ in danh sách ô không có IK ở cả `approach` và
`pick`. Bảng approach đã đo IK thực tế (KDL position-only): `c1`/`f1` dùng
pre-grasp TCP `z=0.100 m`, riêng `d1`→`z=0.080 m` và `e1`→`z=0.070 m` (hai ô
file giữa sát đế không có nghiệm KDL ở vùng cao — xem `SQUARE_APPROACH_OFFSET`
trong `chess_utils.py`); các ô còn lại dùng `z=0.125 m`.
Đoạn mang quân theo phương ngang vẫn luôn ở `z=0.125 m`. Nếu còn ô lỗi sau
kiểm tra, chỉnh `BOARD_ORIGIN`, `SQUARE_SIZE` hoặc xoay/đặt lại bàn; không chạy
self-play trên layout đó. Tune `PICK_TCP_Z` từng bước 2--3 mm trong RViz đến
khi hai ngón kẹp đúng thân quân mà TCP không chạm bàn.

## Quân cờ có thực sự "dính" theo tay gắp không?

Có. Bản này dùng cơ chế **attach/detach collision object** đúng chuẩn MoveIt2
(qua service `/apply_planning_scene`) thay vì chỉ xoá-thêm object ở vị trí mới:

- Trước lúc hạ gắp, `_take_piece_from_world()` chỉ gỡ collision của **quân đang
  gắp**. Vì vậy MoveIt không loại toàn bộ IK chỉ vì ngón phải chạm quân mục tiêu;
  bàn và mọi quân khác vẫn là chướng ngại. Nếu Cartesian pick thất bại, object
  này được thêm lại ngay tại ô nguồn.
- Lúc gripper vừa đóng, `_attach_piece()` gắn object vào `END_EFFECTOR` với
  offset lấy từ TF của TCP, đồng thời khai báo các link ngón là `touch_links`.
  Trong suốt Lift → Move, marker visual cũng đổi sang frame
  `Gripping_point_Link`, nên quân cờ trôi theo cánh tay thật sự trên RViz, kể
  cả trong FakeSystem đang tắt collision object.
- Lúc gripper vừa mở ở bước Place → `_detach_piece()` gỡ khỏi tay, thêm lại
  thành world object đứng yên tại ô đích.

Mỗi quân cờ giữ **1 id cố định** (`piece_000`, `piece_001`, ...) theo dõi qua
`self.piece_id_by_square`, KHÔNG đặt tên id theo ô cờ — vì một ô có thể được
nhiều quân khác nhau chiếm giữ theo thời gian, đặt tên theo ô sẽ gây trùng id.

`main()` phải dùng `MultiThreadedExecutor` (không phải `rclpy.spin()` mặc
định) vì `_execute_move` chạy trên thread riêng và gọi service attach/detach
một cách đồng bộ (poll `future.done()`), cần executor xử lý song song.

## Vì sao không cần tải STL bàn cờ/quân cờ

- Bàn cờ: 1 `add_collision_box` phẳng là đủ để MoveIt2 biết "không được đâm
  xuyên qua mặt bàn" — không cần mesh chi tiết vì hình dạng là hình hộp đơn giản.
- Quân cờ: dùng `add_collision_cylinder` xấp xỉ theo chiều cao từng loại quân
  (đã khai báo sẵn trong `PIECE_SPECS`) — đủ chính xác để tính va chạm và
  gắp/thả, không cần độ chi tiết thẩm mỹ như ấm trà/ly (nơi hình dạng tay cầm,
  vòi rót phức tạp buộc phải dùng mesh thật). Nếu sau này muốn RViz nhìn đẹp
  như bàn cờ thật, mới cần thêm STL chỉ cho phần **visual** (không ảnh hưởng
  logic pick-place), giữ nguyên phần **collision** là primitive cho nhẹ.

## Ghi chú thu nhỏ vấn đề va chạm bạn đang gặp ở task ấm trà

Toàn bộ phần `_setup_initial_scene` / `_move_piece_collision` ở đây chính là
"bài tập" quản lý Planning Scene động — kỹ năng này dùng lại y hệt cho task ấm
trà: mỗi khi tay robot cầm ấm trà di chuyển, bạn cũng cần cập nhật (hoặc gắn
attach_object) để MoveIt2 biết ấm trà "dính" theo end-effector thay vì đứng
yên gây self-collision giả. Nếu muốn, gửi mình đoạn SRDF hoặc log lỗi collision
của task ấm trà để mình xem cụ thể chỗ nào đang vướng.
