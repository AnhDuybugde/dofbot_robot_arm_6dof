# simplify-chess-game

Package ROS 2 độc lập để robot chơi cờ bằng **replay trajectory joint-space đã
calibration**. Nó không import MoveIt và không dùng OMPL, Cartesian planner,
IK/FK runtime, quaternion/orientation constraints, candidate offsets, IK seeds,
hay replanning.

Runtime chỉ thực hiện:

```text
HOME -> route(source) -> reverse(route(source)) -> route(target) -> reverse(route(target))
```

Mỗi `route(square)` bắt đầu bằng `HOME_JOINTS`; route quay về là đúng
`list(reversed(route(square)))`, không phải đảo dấu các joint angle.

## Interface ROS được tái sử dụng

- Arm joints: `arm1_Joint` … `arm5_Joint`
- Arm action: `/arm_group_controller/follow_joint_trajectory`
- Gripper joint: `Rlink1_Joint`
- Gripper action: `/grip_group_controller/gripper_cmd`
- State source: `/joint_states`

Các tên này khớp với `dofbot_moveit/config/ros2_controllers.yaml`; package này
không phụ thuộc vào hoặc khởi chạy MoveIt pipeline cũ.

## Route database

`config/square_routes.yaml` có đủ 64 entry `a1` … `h8`. Ban đầu tất cả là
`UNCALIBRATED`; không có joint value nào được tự suy diễn từ XYZ. Runtime chỉ
cho phép entry có cả `status: VALIDATED` và `validated: true`.

Mỗi entry cần dạng:

```yaml
b2:
  status: VALIDATED
  validated: true
  route:
    - [0.0, 0.0, 0.0, 0.0, 0.0] # HOME
    - [q1, q2, q3, q4, q5]      # safe waypoint
    - [q1, q2, q3, q4, q5]      # b2 grasp/drop pose
  notes: tested with pieces on neighboring squares
```

Collision safety là điều phải xác nhận trong lúc teach/test route. Không có
dynamic collision planner tại runtime.

`discard_route` cũng có sẵn với cùng metadata/state, dành cho capture ở mốc
sau. Castling sẽ chỉ ghép hai lần `move` bình thường.

## Build và chạy

```bash
colcon build --packages-select simplify_chess_game
source install/setup.bash
ros2 run simplify_chess_game calibrate_square b2
ros2 run simplify_chess_game chess_cli
```

Khi chạy từ checkout, tool mặc định ghi vào
`simplify-chess-game/config/square_routes.yaml`. Với bản đã cài ở nơi chỉ đọc,
truyền một file copy có thể ghi được:

```bash
ros2 run simplify_chess_game calibrate_square b2 --routes /path/to/square_routes.yaml
ros2 run simplify_chess_game chess_cli --routes /path/to/square_routes.yaml
```

## Calibration

`calibrate_square b2` mở REPL. Route luôn khởi tạo với HOME. Dùng tool jog
thấp tầng hiện có hoặc `goto q1 q2 q3 q4 q5`, sau đó `capture` để đọc chính
`/joint_states` và thêm waypoint. Các command chính:

```text
home | goto q1 q2 q3 q4 q5 | capture | show | pop | clear
save | reload | test | test-reverse | validate | unvalidate | notes <text> | quit
```

`save` luôn lưu `UNCALIBRATED`. Sau khi test forward/reverse an toàn với bàn
thật, chạy `validate` mới cho phép chess runtime dùng route.

## Chess CLI

```text
move> home
move> test b2
move> test-reverse b2
move> b2 b4
move> status
move> list-routes
move> quit
```

Trong move bình thường gripper chỉ có `PRE_CLOSE`, `CLOSE`, `PRE_CLOSE`
(release), cấu hình tại `config/gripper.yaml`. `piece_grip_adjustment` chỉ là
placeholder cho tương lai; milestone này dùng một giá trị chung.

Mỗi waypoint/gripper action được timeout, cancel khi timeout và log JSONL gồm
timestamp, square, index, target, actual `/joint_states`, gripper state và kết
quả. Nếu action hoặc endpoint check lỗi, execution dừng ngay; không fallback
sang một route/IK/planner khác.
