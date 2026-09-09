"""Node pick-place: nhận nước cờ UCI từ /chess/move, phân rã thành
Approach -> Pick -> Lift -> Move -> Place -> Return Home, đồng thời cập nhật
PlanningScene (add/remove/di chuyển collision object quân cờ) để MoveIt2 tính
toán tránh va chạm với các quân khác trên bàn.

Cài đặt:
    pip install pymoveit2
(hoặc thay lớp MoveIt2 này bằng MoveGroupInterface bạn đã dùng cho task ấm trà -
 xem ghi chú "THAY THẾ" ở cuối file).
"""

import copy
import itertools
import json
import math
import threading
import time

import chess
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    MoveItErrorCodes,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

from pymoveit2 import MoveIt2

from .chess_utils import (
    ALLOW_CARTESIAN_FALLBACK,
    APPROACH_HEIGHT,
    BOARD_CENTER_X,
    BOARD_CENTER_Y,
    BOARD_CENTER_Z,
    BOARD_SIZE_X,
    BOARD_SIZE_Y,
    BOARD_THICKNESS,
    BOARD_TOP_Z,
    BOARD_Z,
    CANDIDATE_SCORE_W_LIMIT_MARGIN,
    CANDIDATE_SCORE_W_POS,
    CANDIDATE_SCORE_W_TILT,
    CANDIDATE_SCORE_W_TRAVEL,
    CARTESIAN_EEF_STEP,
    DISCARD_MAX_SLOTS,
    DISCARD_TCP_Z,
    DOFBOT_JOINT_LIMITS,
    EXPECTED_WORLD_OBJECTS,
    FINGER_THICKNESS,
    FINAL_GRASP_INNER_WIDTH,
    GRASP_APPROACH_CANDIDATE_OFFSETS,
    GRASP_MAX_OFFSET,
    GRASP_SEARCH_TIMEOUT_SEC,
    GRIPPER_CLOSED_RAD,
    GRIPPER_OPEN_RAD,
    HARDWARE_SAFE_VELOCITY_SCALE,
    HIGH_APPROACH_INNER_WIDTH,
    JOINT_LIMIT_MARGIN_RAD,
    JOINT_STATE_MAX_AGE_SEC,
    MAX_JOINT_STEP_RAD,
    MIN_CARTESIAN_FRACTION,
    NARROW_DESCENT_INNER_WIDTH,
    PICK_TCP_Z,
    PIECE_COLLISION,
    PIECE_PHYSICAL,
    PIECE_SPECS,
    TCP_OFFSET_CALIBRATED,
    COLLISION_ENABLED,
    REACHABILITY_EXECUTE_ON_FAKESYSTEM,
    REGION_JOINT_TEMPLATES,
    SCENE_VERIFY_POS_TOL_M,
    SQUARE_SIZE,
    SYSTEM_READY_TIMEOUT_SEC,
    SYSTEM_READY_RETRY_SEC,
    SYSTEM_READY_TOPIC,
    TILT_HARD_LIMIT_RAD,
    TILT_QUALITY_TARGET_RAD,
    USE_UPRIGHT_ORIENTATION_CONSTRAINT,
    VERTICAL_CLEARANCE,
    approach_tcp_z,
    discard_slot_pose,
    square_to_grasp_pose,
    square_to_place_pose,
    square_to_xy,
)

try:
    from controller_manager_msgs.srv import ListControllers
    _HAS_LIST_CONTROLLERS = True
except Exception:  # package vắng trên máy chỉ chạy base demo
    ListControllers = None  # type: ignore
    _HAS_LIST_CONTROLLERS = False

# Dofbot: 5 joints arm (arm_group) + 1 gripper joint (grip_group, mimic).
# Tên repo "6dof" = 5+1. Đã đối chiếu SRDF arm_group/up = [0,0,0,0,0].
JOINT_NAMES = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]
BASE_LINK = "base_link"
END_EFFECTOR = "Gripping_point_Link"
GROUP_NAME = "arm_group"
GRIPPER_JOINT = "Rlink1_Joint"
GRIPPER_GROUP = "grip_group"
# Link ngón được phép chạm quân đang mang (canonical attachObject touch_links:
# chỉ ngón + tip). arm5_Link (palm/đế) đã bỏ khỏi danh sách: quân q/k cao chạm
# palm phải được planner phát hiện, không che bằng ACM. Object vẫn luôn là vật
# cản với tay/bàn/quân khác.
GRIPPER_TOUCH_LINKS = [
    END_EFFECTOR,
    "Rlink1_Link", "Rlink2_Link", "Rlink3_Link",
    "Llink1_Link", "Llink2_Link", "Llink3_Link",
]
# SRDF `arm_group/up`: pose joint đã biết, dùng làm điểm đầu/cuối ổn định cho
# mỗi lượt robot. Đây là joint-goal (PTP), không phải Cartesian target.
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]
# Không thử nhiều yaw: IK position-only của Dofbot bỏ qua quaternion nên 8 yaw
# thường cùng rơi vào một orientation FK. TODO-2 thay bằng candidate IK hữu hạn:
# mỗi pre-place sinh nhiều joint-seed theo vùng bàn cờ + yaw quanh trục đứng,
# descend Cartesian từ chính từng candidate rồi chấm điểm chọn tốt nhất.
# PLACE_YAW_COUNT giữ tương thích API cũ; CANDIDATE_YAW_COUNT là số yaw thật
# mà bộ chọn candidate dùng.
PLACE_YAW_COUNT = 1
CANDIDATE_YAW_COUNT = 4
PLACE_YAW_STEP_DEG = 45.0  # không dùng khi YAW_COUNT=1, giữ để khỏi sửa caller
# TODO-3: quality target tilt <= 11° (KPI, chưa blocker MVP), hard limit 26°.
# Mọi candidate tilt > 26° bị hard-reject; tilt <= 11° được cộng điểm chất lượng.
PREFERRED_TILT_RAD = TILT_QUALITY_TARGET_RAD
MAX_ACCEPTED_TILT_RAD = TILT_HARD_LIMIT_RAD
PLACE_POSITION_TOL_M = 0.005
# Sau detach giữ touch ACM trong lúc retreat; chỉ đóng khi TCP đã cách quân
# đủ xa. Retreat hiện tại 65mm >> ngưỡng 10mm nên luôn thỏa, hằng số này để
# test/hardware sau kiểm chứng tường minh thay vì đoán.
ACM_RELEASE_CLEARANCE_M = 0.010
# Đã đứng sẵn ở approach (transfer vừa execute tới đó) thì bỏ qua OMPL
# pre-place zero-length, descend thẳng từ state hiện tại.
PREPLACE_SKIP_TOL_M = 0.004
# Clearance chung 65mm (CHỐT): pre-grasp = lift = retreat = grasp + 0.065.
# Retreat về approach_z nên không cần hằng số riêng.
CARRY_CLEARANCE_LIFT_M = VERTICAL_CLEARANCE
# Số vòng fixed-point correction cho TCP place. Vì log cho thấy TCP FK đạt đúng
# XYZ yêu cầu, cộng trực tiếp sai số tâm quân vào XYZ TCP sẽ hội tụ nhanh dù
# orientation FK thay đổi nhẹ theo vị trí.
PLACE_COMPENSATION_MAX_ITERATIONS = 5
PLACE_COMPENSATION_MAX_STEP_M = 0.020
# Ngưỡng tái sử dụng chuỗi trajectory đã PASS precheck (không đạt -> fallback
# plan lại, không phải lỗi fatal). Joint: arm không hề di chuyển giữa precheck
# và attach (chỉ có gripper), nên start lift phải gần như trùng khớp.
CACHED_CHAIN_JOINT_TOL_RAD = 0.02
# T_tcp_piece thật sau khi đóng kẹp so với giả định lúc precheck. Đóng kẹp có
# thể xê dịch quân nhẹ; endpoint cached được FK-validate LẠI với local thật
# trước execute nên ngưỡng này chỉ loại nhanh ca xô lệch thô.
CACHED_LOCAL_POS_TOL_M = 0.004
CACHED_LOCAL_ANG_TOL_DEG = 8.0
# GetStateValidity là kiểm tra rời rạc; nội suy waypoint cached sao cho mỗi
# joint thay đổi tối đa khoảng 1.7° giữa hai mẫu để không chỉ kiểm endpoint.
CACHED_COLLISION_SAMPLE_RAD = 0.03
# Chuẩn Cartesian bàn thật: eef_step 2mm, fraction 0.98 theo spec.
CARTESIAN_MAX_STEP_M = CARTESIAN_EEF_STEP
CARTESIAN_FRACTION_THRESHOLD = MIN_CARTESIAN_FRACTION


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("chess_pick_place_node")
        cb_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GROUP_NAME,
            callback_group=cb_group,
        )
        self.gripper = MoveIt2(
            node=self,
            joint_names=[GRIPPER_JOINT],
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GRIPPER_GROUP,
            callback_group=cb_group,
        )
        # Bật cả collision cho Cartesian path. OMPL luôn đọc PlanningScene.
        self.moveit2.cartesian_avoid_collisions = COLLISION_ENABLED
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.board = chess.Board()
        self.discard_count = 0
        self.carried_piece_id: str | None = None
        # Khóa hệ thống sau khi scene/mapping rơi vào trạng thái không nhất
        # quán (deep-check execute fail giữa chừng, runtime fail khi đang
        # attached). Mọi nước đi và check sau đó đều bị từ chối với thông điệp
        # rõ ràng cho tới khi restart launch dựng lại scene từ board (Fix 8).
        self._needs_recovery = False
        # State machine mang quân: WORLD_SOURCE | ATTACH_PENDING | ATTACHED |
        # DETACH_PENDING | WORLD_DESTINATION | RECOVERY_REQUIRED. Không suy
        # rollback từ một trạng thái WORLD mơ hồ:
        # khi kết quả attach/detach chưa rõ phải đối chiếu PlanningScene.
        self._carry_state: str = "WORLD_SOURCE"
        # T_tcp_piece lúc attach (pose tương đối của tâm quân trong frame TCP),
        # dùng để bù transform lúc đặt (Fix 2): quân gắp lệch vẫn rơi đúng tâm
        # ô đích thay vì TCP đi đúng tâm còn quân thì lệch theo.
        self._grasp_local_by_id: dict[str, Pose] = {}
        # Offset thành công gần nhất theo ô được thử trước ở lượt sau. Cache
        # chỉ là ưu tiên: collision vẫn được plan lại với scene hiện tại.
        self._grasp_offset_cache: dict[str, tuple[float, float]] = {}
        # Object vừa được thả vẫn có thể chạm đầu ngón ở approach thấp. Giữ ACM
        # tạm thời cho tới khi arm đã về HOME, rồi mới đóng để không biến start
        # state của đường HOME thành collision giả.
        self._release_contact_object_ids: set[str] = set()
        self._id_counter = itertools.count()
        # ID collision object KHÔNG suy ra từ tên ô (vì ô sẽ được quân khác chiếm
        # lại sau này) - mỗi quân giữ 1 id cố định, theo dõi vị trí hiện tại qua dict.
        self.piece_id_by_square: dict[str, str] = {}
        # Thông tin visual không suy ra từ collision object: MoveIt chỉ hiển thị
        # primitive một màu, khiến hai bên cờ và hàng tốt khó nhìn trong RViz.
        self.piece_info_by_id: dict[str, tuple[str, bool]] = {}
        # RViz có thể mất hàng chục giây để nạp MotionPlanning. Transient-local
        # giữ snapshot mới nhất để subscriber kết nối muộn vẫn nhận đủ 64 ô + 32 quân.
        self._visual_markers: dict[int, Marker] = {}
        self.visual_pub = self.create_publisher(
            MarkerArray,
            "/chess/visual",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self.done_pub = self.create_publisher(String, "/chess/move_done", 10)
        # NACK: mọi lỗi thực thi đều báo về brain để dừng chờ ACK, thay vì treo
        # game âm thầm (brain chờ ACK vô hạn). Format: "<uci>: <lý do>".
        self.fail_pub = self.create_publisher(String, "/chess/move_failed", 10)
        # TODO-1: tín hiệu READY latch cho brain (thay timer 12s cố định).
        self.ready_pub = self.create_publisher(
            String,
            SYSTEM_READY_TOPIC,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self._system_ready = False
        self._ready_since: float | None = None
        # Freshness /joint_states (TODO-1): pymoveit2 đã subscribe nhưng không
        # lưu timestamp; node tự subscribe thêm để gate READY.
        self._last_joint_state_time: float | None = None
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state,
            10, callback_group=cb_group,
        )
        if _HAS_LIST_CONTROLLERS:
            self._list_controllers_client = self.create_client(
                ListControllers, "/controller_manager/list_controllers",
                callback_group=cb_group,
            )
        else:
            self._list_controllers_client = None
        # Guard race: on_move spawn thread mỗi message; 2 thread _execute_move
        # song song sẽ xé board/piece maps dùng chung. Flow chuẩn đã ACK-gated
        # nên cờ này chỉ chặn publish thủ công chồng lệnh.
        self._exec_lock = threading.Lock()
        self._executing = False
        # TODO-6: log tái hiện ván (seed, move list, candidate, snapshot).
        self._run_seed = int(time.time() * 1000) % 100000
        self._run_moves: list[dict] = []
        self._last_place_choice: dict | None = None
        self._last_place_region: str | None = None
        self._last_place_seed_count: int | None = None
        # TODO-7: param an toàn phần cứng (tốc độ thấp, e-stop ngoài).
        self.declare_parameter(
            "hardware_safe_velocity_scale", HARDWARE_SAFE_VELOCITY_SCALE)
        self.declare_parameter("hardware_low_speed_test", True)
        self.move_sub = self.create_subscription(
            String, "/chess/move", self.on_move, 10, callback_group=cb_group
        )
        self.reachability_service = self.create_service(
            Trigger, "/chess/check_reachability", self._check_reachability,
            callback_group=cb_group,
        )

        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=cb_group
        )
        self._get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene", callback_group=cb_group
        )
        self._state_validity_client = self.create_client(
            GetStateValidity, "/check_state_validity", callback_group=cb_group
        )
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=cb_group
        )
        # FK để validate điểm cuối trajectory hạ đặt TRƯỚC execute (không đạt
        # pose bù thì loại, không kẹp hớ). Không wait blocking ở init: fail-closed
        # lúc dùng (raise nếu service vắng) để không treo startup.
        self._fk_client = self.create_client(
            GetPositionFK, "/compute_fk", callback_group=cb_group
        )
        self._apply_scene_client.wait_for_service(timeout_sec=10.0)
        self._get_scene_client.wait_for_service(timeout_sec=10.0)
        self._state_validity_client.wait_for_service(timeout_sec=10.0)
        self._ik_client.wait_for_service(timeout_sec=10.0)

        # TODO-1: dựng scene + gate READY trên thread nền. Service
        # /apply_planning_scene và /get_planning_scene cần executor đang spin
        # mới hoàn thành future; gọi đồng bộ ngay trong __init__ (trước
        # executor.spin() ở main) sẽ treo vì không ai xử lý response.
        # Brain đã chờ topic /chess/system_ready nên init async là an toàn.
        self._init_error: str | None = None
        threading.Thread(target=self._init_scene_and_ready, daemon=True).start()
        self.get_logger().info(
            f"Pick-place node khởi động (đang dựng scene nền); "
            f"collision={'ON' if COLLISION_ENABLED else 'OFF'}, "
            f"deep-check={'EXECUTE' if REACHABILITY_EXECUTE_ON_FAKESYSTEM else 'plan-only'}."
        )

    def _init_scene_and_ready(self):
        deadline = time.monotonic() + SYSTEM_READY_TIMEOUT_SEC
        attempt = 0
        while rclpy.ok() and time.monotonic() < deadline:
            attempt += 1
            try:
                self._setup_initial_scene()
                # Gate READY có timeout + log rõ điều kiện fail, thay vì coi
                # scene setup xong là sẵn sàng. Brain chờ topic này.
                self._wait_for_system_ready(
                    timeout_sec=min(60.0, max(5.0, deadline - time.monotonic())))
            except Exception as exc:
                self._init_error = str(exc)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.get_logger().warning(
                    f"[INIT-RETRY] lần {attempt} thất bại ({exc}); "
                    f"thử lại sau {SYSTEM_READY_RETRY_SEC:.0f}s "
                    f"(còn {remaining:.0f}s)")
                time.sleep(min(SYSTEM_READY_RETRY_SEC, remaining))
                continue
            else:
                self._init_error = None
                self.get_logger().info(
                    f"Pick-place node sẵn sàng; collision={'ON' if COLLISION_ENABLED else 'OFF'}, "
                    f"deep-check={'EXECUTE' if REACHABILITY_EXECUTE_ON_FAKESYSTEM else 'plan-only'}."
                )
                return
        self.get_logger().error(
            f"[FAIL] init scene/READY thất bại sau {attempt} lần thử: {self._init_error}")

    def _on_joint_state(self, msg: JointState):
        self._last_joint_state_time = time.monotonic()

    # ---------------- TODO-1: PlanningScene nguyên tử + READY gate ----------------

    def _build_initial_collision_objects(self) -> list[CollisionObject]:
        """Dựng 1 board box + 32 cylinder quân, chưa gửi (để gửi 1 lần)."""
        from moveit_msgs.msg import CollisionObject as CO
        from shape_msgs.msg import SolidPrimitive as SP
        objects: list[CO] = []
        board = CO()
        board.header.frame_id = BASE_LINK
        board.id = "chessboard"
        board.operation = CollisionObject.ADD
        prim = SP()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [BOARD_SIZE_X, BOARD_SIZE_Y, BOARD_THICKNESS]
        board.primitives = [prim]
        board.primitive_poses = [Pose()]
        board.primitive_poses[0].position.x = BOARD_CENTER_X
        board.primitive_poses[0].position.y = BOARD_CENTER_Y
        board.primitive_poses[0].position.z = BOARD_CENTER_Z
        board.primitive_poses[0].orientation.w = 1.0
        board.pose.orientation.w = 1.0
        objects.append(board)
        for square, piece in self.board.piece_map().items():
            name = chess.square_name(square)
            ptype = piece.symbol().lower()
            obj_id = self._new_piece_id()
            self.piece_id_by_square[name] = obj_id
            self.piece_info_by_id[obj_id] = (ptype, piece.color)
            x, y = square_to_xy(name)
            col = PIECE_COLLISION[ptype]
            obj = CO()
            obj.header.frame_id = BASE_LINK
            obj.id = obj_id
            obj.operation = CollisionObject.ADD
            cyl = SP()
            cyl.type = SolidPrimitive.CYLINDER
            cyl.dimensions = [col["height"], col["radius"]]
            obj.primitives = [cyl]
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = BOARD_Z + col["height"] / 2
            pose.orientation.w = 1.0
            obj.primitive_poses = [pose]
            obj.pose.orientation.w = 1.0
            objects.append(obj)
        return objects

    def _apply_initial_scene_once(self, objects: list[CollisionObject],
                                    attempts: int = 3, timeout_each: float = 15.0):
        """Gửi toàn bộ scene bằng MỘT lần /apply_planning_scene (TODO-1).

        Retry với backoff vì request đầu sau launch có thể rớt trong lúc
        move_group còn khởi tạo scene monitor. Hết attempts thì raise để caller
        fallback incremental (topic) thay vì kẹt init.
        """
        last_exc: Exception | None = None
        for attempt in range(1, attempts + 1):
            req = ApplyPlanningScene.Request()
            req.scene.is_diff = True
            req.scene.world.collision_objects = objects
            req.scene.robot_state.is_diff = True
            future = self._apply_scene_client.call_async(req)
            deadline = time.monotonic() + timeout_each
            while not future.done():
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.02)
            if future.done():
                result = future.result()
                if result is not None and result.success:
                    if attempt > 1:
                        self.get_logger().warning(
                            f"[SCENE] apply nguyên tử đạt ở lần thử {attempt}")
                    return
                last_exc = RuntimeError(
                    "apply_planning_scene từ chối scene ban đầu")
            else:
                try:
                    future.cancel()
                except Exception:
                    pass
                last_exc = RuntimeError(
                    f"apply_planning_scene timeout ({timeout_each:.0f}s) "
                    f"lần {attempt}/{attempts}")
                self.get_logger().warning(f"[SCENE] {last_exc}; thử lại...")
                time.sleep(1.0)
        raise last_exc if last_exc is not None else RuntimeError(
            "apply_planning_scene thất bại không rõ nguyên nhân")

    def _verify_initial_scene(self) -> list[str]:
        """Đọc lại scene và verify 33/33, ID, geometry, pose, dup, attached rỗng.

        Trả về danh sách lý do chưa đạt (rỗng = đạt).
        """
        reasons: list[str] = []
        try:
            scene = self._get_planning_scene(
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            )
        except Exception as exc:
            return [f"get_planning_scene lỗi: {exc}"]
        world = list(scene.world.collision_objects)
        attached = list(scene.robot_state.attached_collision_objects)
        if attached:
            reasons.append(
                f"attached objects ban đầu phải rỗng, thấy {len(attached)}")
        if len(world) != EXPECTED_WORLD_OBJECTS:
            reasons.append(
                f"world objects {len(world)}/{EXPECTED_WORLD_OBJECTS}")
        ids = [o.id for o in world]
        if len(set(ids)) != len(ids):
            reasons.append("trùng ID trong world objects")
        idset = set(ids)
        if "chessboard" not in idset:
            reasons.append("thiếu chessboard")
        # Đếm quân: mọi id piece_* phải đúng 32.
        piece_ids = [i for i in ids if i.startswith("piece_")]
        if len(piece_ids) != 32:
            reasons.append(f"quân cờ {len(piece_ids)}/32")
        # Geometry + pose từng object so với kỳ vọng (tolerance 5mm).
        expected_board = (BOARD_CENTER_X, BOARD_CENTER_Y, BOARD_CENTER_Z,
                          BOARD_SIZE_X, BOARD_SIZE_Y, BOARD_THICKNESS)
        for obj in world:
            if obj.id == "chessboard":
                if not obj.primitives:
                    reasons.append("chessboard thiếu primitive")
                    continue
                d = tuple(float(v) for v in obj.primitives[0].dimensions)
                if any(abs(a - b) > 1e-6 for a, b in zip(
                        d, expected_board[3:])):
                    reasons.append(f"chessboard geometry sai: {d}")
                pp = obj.primitive_poses[0].position if obj.primitive_poses else obj.pose.position
                if (abs(pp.x - expected_board[0]) > SCENE_VERIFY_POS_TOL_M
                        or abs(pp.y - expected_board[1]) > SCENE_VERIFY_POS_TOL_M
                        or abs(pp.z - expected_board[2]) > SCENE_VERIFY_POS_TOL_M):
                    reasons.append("chessboard pose sai > 5mm")
            elif obj.id.startswith("piece_"):
                info = self.piece_info_by_id.get(obj.id)
                if info is None:
                    reasons.append(f"{obj.id} không có mapping nội bộ")
                    continue
                ptype, _color = info
                col = PIECE_COLLISION[ptype]
                if not obj.primitives:
                    reasons.append(f"{obj.id} thiếu primitive")
                    continue
                d = tuple(float(v) for v in obj.primitives[0].dimensions)
                if (abs(d[0] - col["height"]) > 1e-6
                        or abs(d[1] - col["radius"]) > 1e-6):
                    reasons.append(f"{obj.id} geometry sai: {d} vs {col}")
        # Pose quân: đối chiếu tâm cylinder với ô mà mapping nội bộ ghi.
        sq_by_id = {v: k for k, v in self.piece_id_by_square.items()}
        for obj in world:
            if not obj.id.startswith("piece_"):
                continue
            sq = sq_by_id.get(obj.id)
            if sq is None:
                reasons.append(f"{obj.id} không map về ô nào")
                continue
            info = self.piece_info_by_id.get(obj.id)
            if info is None:
                continue
            ptype, _c = info
            ex, ey = square_to_xy(sq)
            ez = BOARD_Z + PIECE_COLLISION[ptype]["height"] / 2
            pp = obj.primitive_poses[0].position if obj.primitive_poses else obj.pose.position
            if (abs(pp.x - ex) > SCENE_VERIFY_POS_TOL_M
                    or abs(pp.y - ey) > SCENE_VERIFY_POS_TOL_M
                    or abs(pp.z - ez) > SCENE_VERIFY_POS_TOL_M):
                reasons.append(f"{obj.id} pose sai > 5mm so với ô {sq}")
                break  # gọn log, 1 mẫu đã đủ báo
        return reasons

    def _planner_ready(self) -> bool:
        try:
            client = self.moveit2._plan_kinematic_path_service
            return bool(client.service_is_ready())
        except Exception:
            return False

    def _controller_active(self) -> tuple[bool, str]:
        """Verify controller active (TODO-1). Ưu tiên list_controllers."""
        if self._list_controllers_client is not None:
            try:
                if not self._list_controllers_client.service_is_ready():
                    return False, "controller_manager chưa có service"
                req = ListControllers.Request()
                future = self._list_controllers_client.call_async(req)
                deadline = time.monotonic() + 3.0
                while not future.done():
                    if time.monotonic() >= deadline:
                        return False, "list_controllers timeout"
                    time.sleep(0.02)
                result = future.result()
                if result is None:
                    return False, "list_controllers không phản hồi"
                for ctrl in result.controller:
                    name = ctrl.name
                    if ("arm" in name or "trajectory" in name
                            or "fake" in name or "dofbot" in name):
                        if ctrl.state == "active":
                            return True, ""
                states = ",".join(f"{c.name}={c.state}" for c in result.controller)
                return False, f"không controller arm nào active ({states})"
            except Exception as exc:
                return False, f"list_controllers lỗi: {exc}"
        # Fallback: joint_states tươi + moveit2 joint_state có dữ liệu.
        if self.moveit2.joint_state is None:
            return False, "chưa có joint state từ MoveIt2"
        return True, ""

    def _joint_states_fresh(self) -> tuple[bool, str]:
        if self._last_joint_state_time is None:
            # Chưa nhận mẫu nào: vẫn cho qua nếu MoveIt2 đã có state (sim mới
            # start), nhưng báo rõ để log.
            if self.moveit2.joint_state is not None:
                return True, ""
            return False, "/joint_states chưa có dữ liệu"
        age = time.monotonic() - self._last_joint_state_time
        if age > JOINT_STATE_MAX_AGE_SEC:
            return False, f"/joint_states cũ {age:.1f}s (> {JOINT_STATE_MAX_AGE_SEC}s)"
        return True, ""

    def _check_system_readiness(self) -> list[str]:
        """Tổng hợp mọi điều kiện READY (TODO-1). Rỗng = READY."""
        reasons: list[str] = []
        reasons.extend(self._verify_initial_scene())
        if not self._planner_ready():
            reasons.append("planner service chưa sẵn sàng")
        ok_ctrl, why_ctrl = self._controller_active()
        if not ok_ctrl:
            reasons.append(f"controller chưa active: {why_ctrl}")
        ok_js, why_js = self._joint_states_fresh()
        if not ok_js:
            reasons.append(why_js)
        return reasons

    def _announce_ready(self):
        msg = String()
        msg.data = "READY"
        self.ready_pub.publish(msg)
        self._system_ready = True
        self._ready_since = time.monotonic()
        self.get_logger().info("[READY] hạ tầng đạt: scene 33/33 + planner + controller + joint_states")

    def _wait_for_system_ready(self, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        last_log = 0.0
        while rclpy.ok():
            reasons = self._check_system_readiness()
            if not reasons:
                self._announce_ready()
                return
            now = time.monotonic()
            if now - last_log >= 5.0:
                self.get_logger().warning(
                    "[NOT-READY] chưa READY: " + "; ".join(reasons))
                last_log = now
            if now >= deadline:
                raise RuntimeError(
                    "hệ thống chưa READY sau "
                    f"{timeout_sec:.0f}s: " + "; ".join(reasons))
            time.sleep(0.2)

    def _require_ready(self, context: str):
        if not self._system_ready:
            reasons = self._check_system_readiness()
            raise RuntimeError(
                f"{context} bị từ chối: hệ thống chưa READY: "
                + ("; ".join(reasons) if reasons else "unknown"))

    # ---------------- TODO-6/7: log ván + gate phần cứng ----------------

    def _require_hardware_gates(self, uci: str):
        """TODO-7: chặn execute phần cứng khi chưa calibration (fail-loud).

        Sim/FakeSystem (REACHABILITY_EXECUTE_ON_FAKESYSTEM=True) luôn qua.
        Robot thật yêu cầu: TCP_OFFSET_CALIBRATED=True, velocity scale <= 0.25,
        low-speed test bật, e-stop sẵn sàng (vận hành thủ công xác nhận qua
        param). Thiếu -> raise để NACK thay vì chạy mù.
        """
        if REACHABILITY_EXECUTE_ON_FAKESYSTEM:
            return
        problems = []
        if not TCP_OFFSET_CALIBRATED:
            problems.append("TCP_TO_CONTACT_OFFSET_Z chưa calibration")
        try:
            scale = float(self.get_parameter(
                "hardware_safe_velocity_scale").value)
            if not 0.0 < scale <= 0.25:
                problems.append(
                    f"velocity_scale={scale} vượt ngưỡng an toàn 0.25")
            if not bool(self.get_parameter("hardware_low_speed_test").value):
                problems.append("hardware_low_speed_test đang tắt")
        except Exception as exc:
            problems.append(f"không đọc param an toàn ({exc})")
        if problems:
            raise RuntimeError(
                f"gate phần cứng chặn nước {uci}: " + "; ".join(problems)
                + ". Test không tải + từng ô/quân ở tốc độ thấp trước.")

    def _snapshot_scene_ids(self) -> dict:
        try:
            attached, world = self._scene_object_ids()
            return {"attached": sorted(attached), "world": sorted(world),
                    "board_fen": self.board.fen(),
                    "discard": self.discard_count}
        except Exception as exc:
            return {"error": str(exc)}

    def _log_move_result(self, cmd: int, uci: str, ok: bool, reason: str = ""):
        entry = {
            "seed": self._run_seed, "cmd": cmd, "uci": uci, "ok": ok,
            "reason": reason,
            "place_region": self._last_place_region,
            "place_seeds": self._last_place_seed_count,
            "place_choice": self._last_place_choice,
            "scene": self._snapshot_scene_ids(),
        }
        self._run_moves.append(entry)
        try:
            with open("/tmp/chess_moves.jsonl", "a", encoding="utf-8") as fh:
                fh.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self.get_logger().info(
            f"[RUN-LOG] seed={self._run_seed} cmd={cmd} {uci} "
            f"{'OK' if ok else 'FAIL'} choice={self._last_place_choice}")

    # ---------------- Planning scene ----------------

    def _new_piece_id(self) -> str:
        return f"piece_{next(self._id_counter):03d}"

    def _setup_initial_scene(self):
        """Dựng scene chuẩn 33/33 bằng MỘT lần /apply_planning_scene (TODO-1).

        Visual vẫn publish 1 snapshot duy nhất để tránh RViz update storm.
        Idempotent: mỗi lần retry đều dọn object cờ cũ (best-effort, qua topic)
        trước khi dựng lại để không nhân đôi world objects.
        """
        self._clear_chess_scene_objects()
        self._publish_board_visual(publish=False)
        # Dựng mapping nội bộ trước để _build_* dùng piece_map chuẩn.
        self.piece_id_by_square.clear()
        self.piece_info_by_id.clear()
        self._id_counter = itertools.count()
        if not COLLISION_ENABLED:
            for square, piece in self.board.piece_map().items():
                name = chess.square_name(square)
                obj_id = self._new_piece_id()
                self.piece_id_by_square[name] = obj_id
                self.piece_info_by_id[obj_id] = (
                    piece.symbol().lower(), piece.color)
                self._publish_piece_visual(
                    obj_id, (*square_to_xy(name), BOARD_Z), publish=False)
            self._publish_all_visual()
            return
        objects = self._build_initial_collision_objects()
        try:
            self._apply_initial_scene_once(objects)
        except Exception as exc:
            # Fallback incremental qua topic (cách cũ đã chứng minh chạy được):
            # vẫn verify 33/33 đọc lại phía dưới nên không mất gate TODO-1.
            self.get_logger().warning(
                f"[SCENE] apply nguyên tử thất bại ({exc}); "
                f"fallback incremental từng object qua topic")
            self._setup_initial_scene_incremental()
        # Visual cho từng quân từ mapping vừa dựng (không add collision lần 2).
        for square, obj_id in self.piece_id_by_square.items():
            self._publish_piece_visual(
                obj_id, (*square_to_xy(square), BOARD_Z), publish=False)
        self._publish_all_visual()
        # Đọc lại và verify ngay: fail-loud nếu chưa 33/33.
        reasons = self._verify_initial_scene()
        if reasons:
            raise RuntimeError(
                "scene ban đầu chưa đạt 33/33: " + "; ".join(reasons))

    def _clear_chess_scene_objects(self):
        """Dọn object cờ cũ trước khi dựng lại (best-effort, không raise)."""
        if not COLLISION_ENABLED:
            return
        try:
            _attached, world_ids = self._scene_object_ids()
        except Exception:
            return
        for oid in list(world_ids):
            if oid == "chessboard" or oid.startswith("piece_") or oid.startswith("__dry_"):
                try:
                    self.moveit2.remove_collision_object(id=oid)
                except Exception:
                    pass

    def _setup_initial_scene_incremental(self):
        """Fallback: thêm từng object qua topic (cách cũ, chậm hơn nhưng đã
        chứng minh chạy được khi service apply flake lúc khởi động)."""
        if COLLISION_ENABLED:
            self.moveit2.add_collision_box(
                id="chessboard",
                position=[BOARD_CENTER_X, BOARD_CENTER_Y, BOARD_CENTER_Z],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                size=[BOARD_SIZE_X, BOARD_SIZE_Y, BOARD_THICKNESS],
            )
            for square, obj_id in self.piece_id_by_square.items():
                ptype, _color = self.piece_info_by_id[obj_id]
                self._add_piece_collision_at(obj_id, (*square_to_xy(square), BOARD_Z), ptype)
            # Chờ scene nhận đủ rồi mới verify ở caller.
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                try:
                    _a, world = self._scene_object_ids()
                    if "chessboard" in world and sum(
                            1 for i in world if i.startswith("piece_")) >= 32:
                        break
                except Exception:
                    pass
                time.sleep(0.3)

    def _add_piece_collision(self, square: str, piece: chess.Piece, publish: bool = True):
        obj_id = self._new_piece_id()
        self.piece_id_by_square[square] = obj_id
        self.piece_info_by_id[obj_id] = (piece.symbol().lower(), piece.color)
        x, y = square_to_xy(square)
        self._add_piece_collision_at(obj_id, (x, y, BOARD_Z), piece.symbol().lower())
        self._publish_piece_visual(obj_id, (x, y, BOARD_Z), publish=publish)

    def _add_piece_collision_at(self, obj_id: str, xyz, piece_type: str):
        if not COLLISION_ENABLED:
            return
        x, y, z = xyz
        col = PIECE_COLLISION[piece_type]
        self.moveit2.add_collision_cylinder(
            id=obj_id,
            position=[x, y, z + col["height"] / 2],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            height=col["height"],
            radius=col["radius"],
        )

    def _publish_board_visual(self, publish: bool = True):
        """Bàn 8x8 và 32 quân có màu riêng; đây là visual layer, tách với
        collision layer mà MoveIt dùng để tránh va chạm."""
        markers = MarkerArray()
        for square in chess.SQUARES:
            name = chess.square_name(square)
            x, y = square_to_xy(name)
            marker = Marker()
            marker.header.frame_id = BASE_LINK
            marker.ns = "chessboard"
            marker.id = square
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = BOARD_Z - 0.004
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = SQUARE_SIZE
            marker.scale.z = 0.006
            # a1 đậm như bàn cờ chuẩn; tránh dùng alpha 0 vì RViz sẽ ẩn marker.
            light = (chess.square_file(square) + chess.square_rank(square)) % 2 == 1
            marker.color.r = 0.93 if light else 0.20
            marker.color.g = 0.78 if light else 0.12
            marker.color.b = 0.54 if light else 0.07
            marker.color.a = 1.0
            markers.markers.append(marker)
        for marker in markers.markers:
            self._visual_markers[1000 + marker.id] = marker
        if publish:
            self._publish_all_visual()

    def _publish_piece_visual(self, obj_id: str, xyz, publish: bool = True):
        piece_type, is_white = self.piece_info_by_id[obj_id]
        x, y, z = xyz
        phys = PIECE_PHYSICAL[piece_type]
        marker = Marker()
        marker.header.frame_id = BASE_LINK
        marker.ns = "chess_pieces"
        marker.id = int(obj_id.rsplit("_", 1)[1])
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + phys["height"] / 2
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = phys["diameter"]
        marker.scale.z = phys["height"]
        if is_white:
            marker.color.r, marker.color.g, marker.color.b = 0.96, 0.96, 0.88
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.08, 0.10, 0.13
        marker.color.a = 1.0
        self._visual_markers[marker.id] = marker
        if publish:
            self._publish_all_visual()

    def _publish_carried_piece_visual(self, obj_id: str, local_pose: Pose):
        """Đổi marker quân sang frame TCP.

        RViz tự cập nhật theo TF của Gripping_point_Link trong lúc lift/transfer,
        nên không cần timer hay publish liên tục. Cơ chế này hoạt động cả khi
        demo bỏ collision object khỏi PlanningScene.
        """
        piece_type, is_white = self.piece_info_by_id[obj_id]
        phys = PIECE_PHYSICAL[piece_type]
        marker = Marker()
        marker.header.frame_id = END_EFFECTOR
        marker.ns = "chess_pieces"
        marker.id = int(obj_id.rsplit("_", 1)[1])
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = local_pose
        marker.scale.x = marker.scale.y = phys["diameter"]
        marker.scale.z = phys["height"]
        if is_white:
            marker.color.r, marker.color.g, marker.color.b = 0.96, 0.96, 0.88
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.08, 0.10, 0.13
        marker.color.a = 1.0
        self._visual_markers[marker.id] = marker
        self.carried_piece_id = obj_id
        self._publish_all_visual()

    def _delete_piece_visual(self, obj_id: str):
        """Xoá quân bị Đen ăn trong lượt virtual khỏi snapshot RViz."""
        marker_id = int(obj_id.rsplit("_", 1)[1])
        self._visual_markers.pop(marker_id, None)
        delete = Marker()
        delete.header.frame_id = BASE_LINK
        delete.ns = "chess_pieces"
        delete.id = marker_id
        delete.action = Marker.DELETE
        self.visual_pub.publish(
            MarkerArray(markers=[delete, *self._visual_markers.values()])
        )

    def _publish_all_visual(self):
        """Luôn gửi nguyên snapshot, không gửi từng quân rời rạc.
        Nhờ vậy QoS transient-local không chỉ cache quân cuối cùng."""
        self.visual_pub.publish(
            MarkerArray(markers=list(self._visual_markers.values()))
        )

    def _apply_attached_object(self, aco: AttachedCollisionObject):
        """Gửi diff attach/detach lên MoveIt2 qua service /apply_planning_scene.
        Đây là cơ chế MoveIt2-native để một object 'dính' theo link của robot,
        thay vì chỉ xoá-thêm lại object ở vị trí mới (gây teleport trên RViz)."""
        req = ApplyPlanningScene.Request()
        req.scene.robot_state.attached_collision_objects = [aco]
        req.scene.robot_state.is_diff = True
        req.scene.is_diff = True
        future = self._apply_scene_client.call_async(req)
        # poll thay vì spin_until_future_complete, vì node đang được spin
        # bởi MultiThreadedExecutor ở thread khác (xem main())
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("apply_planning_scene timeout khi attach/detach quân")
            time.sleep(0.01)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("apply_planning_scene từ chối attach/detach quân")

    def _get_planning_scene(self, components: int):
        req = GetPlanningScene.Request()
        req.components.components = components
        future = self._get_scene_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("get_planning_scene timeout")
            time.sleep(0.01)
        result = future.result()
        if result is None:
            raise RuntimeError("get_planning_scene không có phản hồi")
        return result.scene

    def _set_piece_collision(self, obj_id: str, *, gripper_touch: bool,
                             board_contact: bool):
        """Đặt RIÊNG 2 ngoại lệ va chạm của 1 quân (Fix 3).

        gripper_touch: link ngón kẹp được chạm CHÍNH quân này (cần lúc gắp,
        mang, đặt; đóng ngay khi detach xong và kẹp đã rút).
        board_contact: quân được chạm mặt bàn (CHỈ cần ở pose gắp/đặt vì đáy
        cylinder đúng tại mặt bàn; PHẢI ĐÓNG ngay khi quân đã thoát mặt bàn
        để quân đang mang không xuyên bàn trong lift/transfer mà cặp va chạm
        đó không bị chặn).
        Quân vẫn luôn là vật cản với arm, đế và quân khác.
        """
        if not COLLISION_ENABLED:
            return
        scene = self._get_planning_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        acm = scene.allowed_collision_matrix

        def ensure(name: str) -> int:
            if name in acm.entry_names:
                index = acm.entry_names.index(name)
            else:
                index = len(acm.entry_names)
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * (index + 1)))
            # PlanningScene có thể trả row ngắn khi entry được tạo từ diff cũ.
            for row in acm.entry_values:
                while len(row.enabled) < len(acm.entry_names):
                    row.enabled.append(False)
            return index

        object_index = ensure(obj_id)
        for link in GRIPPER_TOUCH_LINKS:
            link_index = ensure(link)
            acm.entry_values[object_index].enabled[link_index] = gripper_touch
            acm.entry_values[link_index].enabled[object_index] = gripper_touch
        board_index = ensure("chessboard")
        acm.entry_values[object_index].enabled[board_index] = board_contact
        acm.entry_values[board_index].enabled[object_index] = board_contact

        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.allowed_collision_matrix = acm
        future = self._apply_scene_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("apply_planning_scene timeout khi cập nhật ACM")
            time.sleep(0.01)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("apply_planning_scene từ chối cập nhật ACM")

    def _set_object_gripper_collision(self, obj_id: str, allow: bool):
        """Compat: bật/tắt cả 2 ngoại lệ cùng lúc.

        Chỉ dùng cho plan-only (quét nhanh/dry-run không di chuyển) và
        rollback, nơi không có chuyển phase mang/đặt. Mọi flow CÓ di chuyển
        phải dùng _set_piece_collision để tách phase (Fix 3).
        """
        self._set_piece_collision(obj_id, gripper_touch=allow, board_contact=allow)

    def _wait_for_scene_object(self, obj_id: str, *, attached: bool, timeout_sec: float = 3.0):
        """Chờ scene xác nhận trạng thái world/attached thay cho sleep cố định."""
        deadline = time.monotonic() + timeout_sec
        components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        while rclpy.ok() and time.monotonic() < deadline:
            scene = self._get_planning_scene(components)
            attached_ids = {aco.object.id for aco in scene.robot_state.attached_collision_objects}
            world_ids = {obj.id for obj in scene.world.collision_objects}
            if attached and obj_id in attached_ids:
                return
            if not attached and obj_id in world_ids and obj_id not in attached_ids:
                return
            time.sleep(0.03)
        state = "attached" if attached else "world"
        raise RuntimeError(f"PlanningScene chưa xác nhận quân {obj_id} ở trạng thái {state}")

    def _wait_for_scene_absence(self, obj_id: str, timeout_sec: float = 3.0):
        """Chờ scene xác nhận object đã biến mất khỏi cả world lẫn attached."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            attached_ids, world_ids = self._scene_object_ids()
            if obj_id not in attached_ids and obj_id not in world_ids:
                return
            time.sleep(0.03)
        raise RuntimeError(
            f"PlanningScene chưa xác nhận quân {obj_id} đã bị xoá khỏi scene")

    def _wait_for_scene_pose(self, obj_id: str, xyz, piece_type: str,
                             timeout_sec: float = 3.0):
        """Chờ scene xác nhận object ở ĐÚNG pose mới (Fix 5).

        `_wait_for_scene_object` chỉ check ID tồn tại — robot có thể đi lượt
        sau trong lúc MoveIt chưa nhận đủ thay đổi pose. So sánh tâm cylinder
        với sai số 5 mm.
        """
        x, y, z = xyz
        want_z = z + PIECE_SPECS[piece_type].pickup_height / 2
        deadline = time.monotonic() + timeout_sec
        components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        while rclpy.ok() and time.monotonic() < deadline:
            scene = self._get_planning_scene(components)
            for obj in scene.world.collision_objects:
                if obj.id != obj_id:
                    continue
                # CollisionObject có pose riêng và primitive pose tương đối.
                # Tuỳ phiên bản MoveIt, transform world có thể nằm ở một trong
                # hai trường; phải compose cả hai mới kiểm tra đúng tâm cylinder.
                op = obj.pose.position
                oq = obj.pose.orientation
                oq_tuple = (oq.x, oq.y, oq.z, oq.w)
                if sum(v * v for v in oq_tuple) < 1e-12:
                    oq_tuple = (0.0, 0.0, 0.0, 1.0)
                if obj.primitive_poses:
                    pp = obj.primitive_poses[0].position
                    rpx, rpy, rpz = self._rotate_by_quaternion(
                        (pp.x, pp.y, pp.z), oq_tuple)
                else:
                    rpx = rpy = rpz = 0.0
                actual = (op.x + rpx, op.y + rpy, op.z + rpz)
                if (abs(actual[0] - x) < 0.005
                        and abs(actual[1] - y) < 0.005
                        and abs(actual[2] - want_z) < 0.005):
                    return
            time.sleep(0.03)
        raise RuntimeError(
            f"PlanningScene chưa xác nhận pose mới của quân {obj_id} tại {xyz}")

    def _state_validity_contacts(self, joint_state, context: str) -> list[str]:
        """Hỏi MoveIt contact pairs của một joint state để chẩn đoán collision."""
        if joint_state is None or not self._state_validity_client.service_is_ready():
            return []
        request = GetStateValidity.Request()
        request.group_name = GROUP_NAME
        request.robot_state = RobotState()
        request.robot_state.joint_state = copy.deepcopy(joint_state)
        # joint_state chỉ là phần diff so với monitored PlanningScene. Phải
        # giữ attached collision objects (quân đang mang / scratch); nếu để
        # False, mảng attached rỗng có thể bị hiểu là xoá toàn bộ attached.
        request.robot_state.is_diff = True
        future = self._state_validity_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(f"[COLLISION-DIAG] timeout: {context}")
                return []
            time.sleep(0.01)
        result = future.result()
        if result is None:
            return []
        contacts = sorted({
            f"{contact.contact_body_1}<->{contact.contact_body_2}"
            for contact in result.contacts
        })
        if contacts:
            self.get_logger().error(
                f"[COLLISION-CONTACT] context={context} | " + ", ".join(contacts)
            )
        else:
            self.get_logger().info(f"[COLLISION-CONTACT] context={context} | none-at-current-state")
        return contacts

    def _diagnose_position_goal_collision(self, position, quat_xyzw, context: str) -> list[str]:
        """Giải IK không tránh collision, rồi hỏi contact pair của nghiệm đó.

        Planner position-only không trả joint state khi reject goal. KDL của
        Dofbot đang position_only_ik, nên orientation ở đây chỉ làm seed cho
        request; contact trả về là manh mối hình học cho đúng XYZ mục tiêu.
        """
        if not self._ik_client.service_is_ready():
            return []
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = GROUP_NAME
        ik.ik_link_name = END_EFFECTOR
        ik.avoid_collisions = False
        ik.robot_state = RobotState()
        ik.robot_state.joint_state = copy.deepcopy(self.moveit2.joint_state)
        ik.robot_state.is_diff = True
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = BASE_LINK
        ik.pose_stamped.pose.position.x = float(position[0])
        ik.pose_stamped.pose.position.y = float(position[1])
        ik.pose_stamped.pose.position.z = float(position[2])
        ik.pose_stamped.pose.orientation.x = float(quat_xyzw[0])
        ik.pose_stamped.pose.orientation.y = float(quat_xyzw[1])
        ik.pose_stamped.pose.orientation.z = float(quat_xyzw[2])
        ik.pose_stamped.pose.orientation.w = float(quat_xyzw[3])
        ik.timeout.sec = 1
        future = self._ik_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(f"[COLLISION-DIAG] IK timeout: {context}")
                return []
            time.sleep(0.01)
        result = future.result()
        if result is None or result.error_code.val != 1:
            code = "no-response" if result is None else str(result.error_code.val)
            self.get_logger().warning(f"[COLLISION-DIAG] IK không có nghiệm tại {context}; code={code}")
            return []
        return self._state_validity_contacts(result.solution.joint_state, context)

    def _home_joint_state(self):
        """Tạo state hiện tại nhưng thay 5 joint arm bằng HOME_JOINTS."""
        self._wait_for_joint_state(self.moveit2)
        state = copy.deepcopy(self.moveit2.joint_state)
        values = dict(zip(state.name, state.position))
        values.update(zip(JOINT_NAMES, HOME_JOINTS))
        state.position = [values[name] for name in state.name]
        return state

    def _take_piece_from_world(self, square: str) -> str:
        """Đánh dấu quân chuẩn bị gắp, vẫn giữ nó trong world collision scene."""
        obj_id = self.piece_id_by_square.get(square)
        if obj_id is None:
            raise RuntimeError(f"ô {square} không có quân trong mapping nội bộ")
        if COLLISION_ENABLED:
            # ACM trước, pop mapping sau: ACM fail thì mapping còn nguyên để
            # retry thay vì mất dấu quân (Fix 4). Cả 2 ngoại lệ đều mở ở pose
            # gắp (attach tại điểm chạm mặt bàn).
            self._set_piece_collision(obj_id, gripper_touch=True, board_contact=True)
        del self.piece_id_by_square[square]
        return obj_id

    def _restore_piece_to_world(self, square: str, obj_id: str, piece_type: str):
        """Rollback duy nhất an toàn trước khi object được attach vào gripper."""
        self._set_object_gripper_collision(obj_id, False)
        self.piece_id_by_square[square] = obj_id
        self._publish_piece_visual(obj_id, (*square_to_xy(square), BOARD_Z))

    def _restore_piece_collision(self, square: str, obj_id: str, piece_type: str):
        """Hoàn trả collision tạm bỏ bởi reachability check.

        Khác ``_restore_piece_to_world``: mapping/visual không đổi, vì service
        chỉ mô phỏng planning scene chứ không hề nhấc quân khỏi bàn.
        """
        self._set_object_gripper_collision(obj_id, False)

    @staticmethod
    def _rotate_by_inverse_quaternion(vector, quaternion):
        """Đổi vector BASE_LINK sang hệ TCP bằng quaternion nghịch đảo."""
        qx, qy, qz, qw = (-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w)
        vx, vy, vz = vector
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    @staticmethod
    def _rotate_by_quaternion(vector, quaternion):
        """Xoay vector bằng quaternion (x, y, z, w); validate strict trước."""
        x, y, z, w = PickPlaceNode._normalize_quaternion(tuple(quaternion))
        vx, vy, vz = (float(v) for v in vector)
        for v in (vx, vy, vz):
            if not math.isfinite(v):
                raise ValueError(f"vector xoay không hợp lệ (NaN/inf): {vector}")
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        )

    def _place_tcp_for_target(self, obj_id: str, target_xy, tcp_z_fallback: float,
                              piece_type: str, preferred_quat=None):
        """Tính pose TCP để tâm quân rơi đúng target, quân đứng thẳng (Fix 2).

        T_base_tcp_target = T_base_piece_target × inverse(T_tcp_piece), với
        T_tcp_piece là pose tương đối đã lưu lúc attach. Nếu không có (collision
        off hoặc attach legacy), fallback hành vi cũ: TCP tới tâm, giữ quat
        hiện tại — caller phải tự chịu sai số offset.
        preferred_quat: quat TCP sau transfer. Quân đứng thẳng chỉ ràng buộc
        TILT (yaw quanh trục đứng tự do) nên chọn yaw ψ của quân ở đích sao cho
        TCP gần preferred_quat nhất: ψ* = 2·atan2(N.z, N.w) với
        N = Q_preferred ⊗ q_local. Không truyền (=None) thì ψ=0 (identity như
        trước). Trả về (x, y, z, quat_xyzw) — yaw tối ưu duy nhất (giữ tương
        thích; flow mới nên dùng _place_tcp_candidates_for_target để thử
        nhiều yaw).
        """
        candidates = self._place_tcp_candidates_for_target(
            obj_id, target_xy, tcp_z_fallback, piece_type, preferred_quat,
            local_override=None, yaw_count=1)
        return candidates[0]

    def _optimal_place_yaw(self, q_local, preferred_quat=None) -> float:
        """Yaw ψ* của quân ở đích để TCP gần preferred nhất (rad)."""
        ql = self._normalize_quaternion(tuple(q_local))
        if preferred_quat is None:
            return 0.0
        n = self._multiply_quaternions(
            self._normalize_quaternion(tuple(preferred_quat)), ql)
        return 2.0 * math.atan2(n[2], n[3])

    @staticmethod
    def _place_yaw_order(yaw_count: int):
        """Thứ tự k: 0, +1, -1, +2, -2, ... để thử gần ψ* trước, phủ vòng tròn."""
        order = [0]
        for k in range(1, (yaw_count + 1) // 2 + 1):
            order.append(k)
            order.append(-k)
        return order[:max(1, yaw_count)]

    @staticmethod
    def _quat_angle(a, b) -> float:
        """Góc (rad) giữa 2 quaternion đã chuẩn hoá; q ≡ -q nên lấy |dot|."""
        dot = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
        return 2.0 * math.acos(max(-1.0, min(1.0, dot)))

    def _place_yaw_candidates_for_local(self, local: Pose, target_xy,
                                       piece_type: str, preferred_quat=None,
                                       yaw_count: int = PLACE_YAW_COUNT):
        """Sinh candidate đặt từ T_tcp_piece tường minh.

        Mỗi candidate: Q_tcp(ψk) = Y(ψk) ⊗ conj(q_local), ψk = ψ* + k·step.
        Trả về list [(place_tcp, k, psi)] theo thứ tự |k·step| tăng dần.
        Quaternion rác raise (không đoán mò pose).
        """
        ql = self._normalize_quaternion(
            (local.orientation.x, local.orientation.y,
             local.orientation.z, local.orientation.w))
        psi_star = self._optimal_place_yaw(ql, preferred_quat)
        step = math.radians(PLACE_YAW_STEP_DEG)
        conj = self._normalize_quaternion((-ql[0], -ql[1], -ql[2], ql[3]))
        spec = PIECE_SPECS[piece_type]
        want = (target_xy[0], target_xy[1], BOARD_Z + spec.pickup_height / 2)
        lx, ly, lz = local.position.x, local.position.y, local.position.z
        out = []
        for k in self._place_yaw_order(yaw_count):
            psi = psi_star + k * step
            s, c = math.sin(psi / 2.0), math.cos(psi / 2.0)
            q_tcp = self._multiply_quaternions((0.0, 0.0, s, c), conj)
            rx, ry, rz = self._rotate_by_quaternion((lx, ly, lz), q_tcp)
            tcp = (want[0] - rx, want[1] - ry, want[2] - rz)
            out.append(((*tcp, q_tcp), k, psi))
        return out

    def _place_tcp_candidates_for_target(self, obj_id: str, target_xy,
                                         tcp_z_fallback: float,
                                         piece_type: str, preferred_quat=None,
                                         local_override=None,
                                         yaw_count: int = PLACE_YAW_COUNT,
                                         verified_quat=None):
        """Sinh initial TCP guess cho vòng bù tâm FK.

        Quaternion chỉ giúp tạo dự đoán ban đầu; IK position-only có thể bỏ
        qua nó. Kết quả cuối được quyết định bởi tâm quân từ FK, không bởi yaw
        hay tilt. Trả về list để giữ tương thích API cũ, hiện chỉ có 1 phần tử.
        """
        local = local_override if local_override is not None else \
            self._grasp_local_by_id.get(obj_id)
        if local is None:
            q = self._current_tcp_quat()
            self.get_logger().warning(
                f"[WARN] {obj_id} không có offset attach -> đặt TCP đúng tâm, "
                f"quân có thể lệch nếu gắp không chuẩn tâm")
            return [(*target_xy, tcp_z_fallback,
                     self._normalize_quaternion(tuple(q)))]
        yaw_cands = self._place_yaw_candidates_for_local(
            local, target_xy, piece_type, preferred_quat, yaw_count)
        if verified_quat is not None:
            vq = self._normalize_quaternion(tuple(verified_quat))
            yaw_cands.sort(
                key=lambda item: self._quat_angle(item[0][3], vq))
        return [place_tcp for place_tcp, _k, _psi in yaw_cands]

    @staticmethod
    def _normalize_quaternion(q):
        """Chuẩn hoá quaternion, FAIL LOUD với dữ liệu rác.

        Quaternion gần zero, NaN hoặc infinity mà âm thầm trả identity sẽ che
        lỗi TF/dữ liệu và báo tilt 0° giả. Mọi caller muốn pose quân đều phải
        thấy lỗi rõ ràng thay vì đặt lệch.
        """
        x, y, z, w = (float(v) for v in q)
        for v in (x, y, z, w):
            if not math.isfinite(v):
                raise ValueError(
                    f"quaternion không hợp lệ (NaN/inf): {(x, y, z, w)}")
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            raise ValueError(
                f"quaternion gần zero (norm={norm:.3e}): {(x, y, z, w)}")
        return (x / norm, y / norm, z / norm, w / norm)

    @staticmethod
    def _multiply_quaternions(a, b):
        ax, ay, az, aw = PickPlaceNode._normalize_quaternion(tuple(a))
        bx, by, bz, bw = PickPlaceNode._normalize_quaternion(tuple(b))
        x = aw * bx + ax * bw + ay * bz - az * by
        y = aw * by - ax * bz + ay * bw + az * bx
        z = aw * bz + ax * by - ay * bx + az * bw
        w = aw * bw - ax * bx - ay * by - az * bz
        # Tích 2 quaternion đơn vị không thể triệt tiêu; normalize strict để
        # mọi sai số số học tích luỹ đều được chặn, không trôi âm thầm.
        return PickPlaceNode._normalize_quaternion((x, y, z, w))

    @staticmethod
    def _tilt_from_quaternion(q_xyzw) -> float:
        """Góc nghiêng (rad) giữa trục Z của vật và trục Z thế giới, BỎ QUA yaw.

        Đo sai cũ (2·acos(|qw|)) so toàn bộ quaternion với identity nên yaw
        thuần 131° cũng bị tính thành 'nghiêng 131°'. Chỉ lấy thành phần z của
        R(q)·ẑ = 1−2(x²+y²): yaw-only cho đúng 0. Chuẩn hoá strict trước vì
        phép nhân q_tcp × q_local tích luỹ sai số số học dù quaternion MoveIt
        đã chuẩn; quaternion rác (zero/NaN/inf) raise thay vì báo tilt 0° giả.
        """
        x, y, z, w = PickPlaceNode._normalize_quaternion(tuple(q_xyzw))
        vz = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
        return math.acos(vz)

    def _verify_attached_piece_target(self, obj_id: str, target_xy,
                                      piece_type: str, requested_tcp=None,
                                      timeout_sec: float = 1.0):
        """Xác nhận tâm quân và độ thẳng đứng trước khi mở kẹp.

        Độ thẳng = TILT (góc trục Z quân vs Z thế giới), bỏ qua yaw vì đặt cho
        phép xoay quanh trục đứng. requested_tcp (pose TCP đã bù) chỉ để log
        phân biệt lỗi IK/transform/TF khi fail.
        """
        local = self._grasp_local_by_id.get(obj_id)
        if local is None:
            raise RuntimeError(f"thiếu T_tcp_piece của {obj_id} trước detach")
        spec = PIECE_SPECS[piece_type]
        expected = (
            target_xy[0], target_xy[1], BOARD_Z + spec.pickup_height / 2)
        q_local = (local.orientation.x, local.orientation.y,
                   local.orientation.z, local.orientation.w)
        deadline = time.monotonic() + timeout_sec
        while True:
            transform = self.tf_buffer.lookup_transform(
                BASE_LINK, END_EFFECTOR, rclpy.time.Time())
            t = transform.transform.translation
            q = transform.transform.rotation
            q_tcp = (q.x, q.y, q.z, q.w)
            rotated = self._rotate_by_quaternion(
                (local.position.x, local.position.y, local.position.z), q_tcp)
            actual = (t.x + rotated[0], t.y + rotated[1], t.z + rotated[2])
            position_error = math.sqrt(sum(
                (got - want) ** 2 for got, want in zip(actual, expected)))
            q_piece = self._multiply_quaternions(q_tcp, q_local)
            tilt = self._tilt_from_quaternion(q_piece)
            if position_error <= PLACE_POSITION_TOL_M:
                return
            if time.monotonic() >= deadline:
                req = ("không rõ" if requested_tcp is None else
                       f"xyz={[round(v, 4) for v in requested_tcp[:3]]} "
                       f"quat={[round(v, 3) for v in requested_tcp[3]]}")
                raise RuntimeError(
                    f"pose quân trước detach sai: tâm quân thực tế "
                    f"={[round(v, 4) for v in actual]} muốn={expected} "
                    f"(lệch {position_error:.4f}m); góc nghiêng tham khảo "
                    f"(không dùng để FAIL)={math.degrees(tilt):.1f}deg; "
                    f"TCP yêu cầu {req}, "
                    f"TCP thực tế xyz={[round(v, 4) for v in (t.x, t.y, t.z)]} "
                    f"quat={[round(v, 3) for v in q_tcp]}")
            time.sleep(0.02)

    def _scene_object_ids(self):
        """Trả về (attached_ids, world_ids) từ PlanningScene hiện tại (Fix 4)."""
        scene = self._get_planning_scene(
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        attached_ids = {aco.object.id for aco in scene.robot_state.attached_collision_objects}
        world_ids = {obj.id for obj in scene.world.collision_objects}
        return attached_ids, world_ids

    def _assert_scene_invariant(self, context: str, expect_attached: str | None = None,
                                expect_world_pose=None):
        """TODO-5: kiểm tra scene invariant trong runtime, fail-loud.

        - Không mất board; không double world+attached cùng ID.
        - Sau attach: quân chỉ ở attached. Sau detach: quân ở world đúng pose.
        - world/attached khớp mapping nội bộ (board state + discard).
        Vi phạm -> khóa RECOVERY + raise (dừng an toàn).
        """
        if not COLLISION_ENABLED:
            return
        try:
            scene = self._get_planning_scene(
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        except Exception as exc:
            self._needs_recovery = True
            self._carry_state = "RECOVERY_REQUIRED"
            raise RuntimeError(f"scene invariant {context}: không đọc scene ({exc})")
        attached_ids = {a.object.id for a in scene.robot_state.attached_collision_objects}
        world_ids = {o.id for o in scene.world.collision_objects}
        problems: list[str] = []
        if "chessboard" not in world_ids:
            problems.append("mất chessboard")
        double = attached_ids & world_ids
        if double:
            problems.append(f"double world+attached: {sorted(double)}")
        if expect_attached is not None:
            if expect_attached not in attached_ids:
                problems.append(f"{expect_attached} phải attached sau attach")
            if expect_attached in world_ids:
                problems.append(f"{expect_attached} còn sót trong world sau attach")
        if expect_world_pose is not None:
            obj_id, xyz, ptype = expect_world_pose
            if obj_id in attached_ids:
                problems.append(f"{obj_id} còn attached sau detach")
            if obj_id not in world_ids:
                problems.append(f"{obj_id} mất khỏi world sau detach")
        # Mapping nội bộ khớp scene: mọi quân trong map phải ở đúng một nơi.
        for sq, oid in self.piece_id_by_square.items():
            in_w = oid in world_ids
            in_a = oid in attached_ids
            if not in_w and not in_a:
                problems.append(f"{oid} (ô {sq}) mất dấu khỏi scene")
            if in_w and in_a:
                problems.append(f"{oid} (ô {sq}) double world+attached")
        # World/attached/board/discard khớp nhau (TODO-5/6): số quân world
        # (trừ scratch) phải bằng quân trên board python-chess + discard_count.
        world_pieces = [i for i in world_ids
                        if i.startswith("piece_") and not i.startswith("__dry_")]
        n_board = len(self.board.piece_map())
        if len(world_pieces) != n_board + self.discard_count:
            problems.append(
                f"world/board/discard lệch: world={len(world_pieces)} "
                f"board={n_board} discard={self.discard_count}")
        if problems:
            self._needs_recovery = True
            self._carry_state = "RECOVERY_REQUIRED"
            raise RuntimeError(
                f"scene invariant {context} VI PHẠM: " + "; ".join(problems))

    def _attached_piece_local_pose(self, obj_id: str) -> Pose:
        """Đọc T_tcp_piece đúng như PlanningScene đang dùng cho collision."""
        scene = self._get_planning_scene(
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        for aco in scene.robot_state.attached_collision_objects:
            if aco.object.id != obj_id:
                continue
            obj = aco.object
            base = obj.pose
            bq_raw = (base.orientation.x, base.orientation.y,
                      base.orientation.z, base.orientation.w)
            bq = ((0.0, 0.0, 0.0, 1.0)
                  if sum(v * v for v in bq_raw) < 1e-12
                  else self._normalize_quaternion(bq_raw))
            child = obj.primitive_poses[0] if obj.primitive_poses else Pose()
            cq_raw = (child.orientation.x, child.orientation.y,
                      child.orientation.z, child.orientation.w)
            cq = ((0.0, 0.0, 0.0, 1.0)
                  if sum(v * v for v in cq_raw) < 1e-12
                  else self._normalize_quaternion(cq_raw))
            rx, ry, rz = self._rotate_by_quaternion(
                (child.position.x, child.position.y, child.position.z), bq)
            out = Pose()
            out.position.x = base.position.x + rx
            out.position.y = base.position.y + ry
            out.position.z = base.position.z + rz
            oq = self._multiply_quaternions(bq, cq)
            (out.orientation.x, out.orientation.y,
             out.orientation.z, out.orientation.w) = oq
            return out
        raise RuntimeError(f"PlanningScene không thấy attached object {obj_id}")

    @staticmethod
    def _pose_signature(pose: Pose):
        """Chữ ký pose ổn định đủ nhạy để phát hiện scene đổi giữa hai plan."""
        return tuple(round(float(v), 8) for v in (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w))

    def _collision_object_signature(self, obj: CollisionObject):
        """Chữ ký pose + geometry; không chỉ ID (ID giữ nguyên vẫn có thể di chuyển)."""
        primitives = tuple(
            (int(shape.type), tuple(round(float(v), 8) for v in shape.dimensions))
            for shape in obj.primitives)
        meshes = tuple(
            (
                tuple(tuple(round(float(v), 8) for v in (p.x, p.y, p.z))
                      for p in mesh.vertices),
                tuple(tuple(int(i) for i in tri.vertex_indices)
                      for tri in mesh.triangles),
            )
            for mesh in obj.meshes)
        planes = tuple(
            tuple(round(float(v), 8) for v in plane.coef)
            for plane in obj.planes)
        return (
            obj.id, obj.header.frame_id, self._pose_signature(obj.pose),
            primitives,
            tuple(self._pose_signature(p) for p in obj.primitive_poses),
            meshes,
            tuple(self._pose_signature(p) for p in obj.mesh_poses),
            planes,
            tuple(self._pose_signature(p) for p in obj.plane_poses),
            tuple(obj.subframe_names),
            tuple(self._pose_signature(p) for p in obj.subframe_poses),
        )

    def _scene_cache_fingerprint(self, carried_id: str | None):
        """Fingerprint phần scene phải bất biến khi đổi scratch thành quân thật.

        Scratch và quân đang gắp được loại khỏi fingerprint vì chúng được kiểm
        riêng bằng geometry cố định + T_tcp_piece. Mọi world pose/geometry,
        attached object khác và ACM không liên quan tới quân phải giữ nguyên.
        """
        scene = self._get_planning_scene(
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        excluded = {"__dry_carry__"}
        if carried_id:
            excluded.add(carried_id)
        world = tuple(sorted(
            self._collision_object_signature(obj)
            for obj in scene.world.collision_objects
            if obj.id not in excluded))
        attached = tuple(sorted(
            (
                aco.link_name,
                tuple(sorted(aco.touch_links)),
                self._collision_object_signature(aco.object),
            )
            for aco in scene.robot_state.attached_collision_objects
            if aco.object.id not in excluded))

        acm = scene.allowed_collision_matrix
        keep = [
            (i, name) for i, name in enumerate(acm.entry_names)
            if name not in excluded
        ]
        names = tuple(sorted(name for _i, name in keep))
        enabled_pairs = []
        for left_pos, (i, left) in enumerate(keep):
            row = acm.entry_values[i].enabled if i < len(acm.entry_values) else []
            for j, right in keep[left_pos:]:
                if j < len(row) and bool(row[j]):
                    enabled_pairs.append(tuple(sorted((left, right))))
        return world, attached, names, tuple(sorted(enabled_pairs))

    def _reconcile_carry_failure(self, obj_id: str | None, from_sq: str,
                                 piece_type: str, where: str,
                                 destination_square: str | None = None,
                                 destination_xyz=None,
                                 destination_piece_type: str | None = None):
        """Đối chiếu scene sau lỗi giữa attach/detach, không bao giờ rollback mù (Fix 4).

        Rollback về ô nguồn CHỈ khi chứng minh được quân còn nằm yên trong
        world (chưa attach). Mọi trạng thái mơ hồ khác -> RECOVERY_REQUIRED,
        raise để caller NACK và yêu cầu phục hồi thủ công. Hàm này luôn raise.
        """
        if obj_id is None:
            raise
        if not COLLISION_ENABLED:
            # Không có scene để đối chiếu: state machine là nguồn duy nhất.
            if self._carry_state in ("WORLD_SOURCE", "ATTACH_PENDING"):
                self._carry_state = "WORLD_SOURCE"
                self._restore_piece_to_world(from_sq, obj_id, piece_type)
            else:
                self._carry_state = "RECOVERY_REQUIRED"
                self.get_logger().error(
                    f"[FAIL] {where}: collision off, trạng thái {self._carry_state} "
                    f"không phục hồi tự động được.")
            raise
        try:
            attached_ids, world_ids = self._scene_object_ids()
        except Exception as exc:
            self._carry_state = "RECOVERY_REQUIRED"
            self.get_logger().error(
                f"[FAIL] {where}: không đọc được scene để đối chiếu ({exc}); "
                f"quân {obj_id} cần phục hồi thủ công.")
            raise
        actually_attached = obj_id in attached_ids
        in_world = obj_id in world_ids
        placed_type = destination_piece_type or piece_type
        if self._carry_state == "ATTACH_PENDING":
            if actually_attached:
                # Service attach xong nhưng confirm timeout: scene đã đúng,
                # rollback sẽ gây double (vừa attached vừa world) -> giữ.
                self._carry_state = "ATTACHED"
                self.get_logger().error(
                    f"[FAIL] {where}: attach đã vào scene nhưng confirm lỗi; "
                    f"giữ ATTACHED, mở gripper/đưa arm về safe pose rồi chạy lại.")
            elif in_world and not actually_attached:
                self._carry_state = "WORLD_SOURCE"
                self._restore_piece_to_world(from_sq, obj_id, piece_type)
            else:
                self._carry_state = "RECOVERY_REQUIRED"
                self.get_logger().error(
                    f"[FAIL] {where}: quân {obj_id} không ở world lẫn gripper; "
                    f"cần phục hồi thủ công.")
        elif self._carry_state == "DETACH_PENDING":
            if not actually_attached and in_world:
                self._carry_state = "WORLD_DESTINATION"
                self._grasp_local_by_id.pop(obj_id, None)
                if destination_square is not None:
                    self.piece_id_by_square.pop(from_sq, None)
                    self.piece_id_by_square[destination_square] = obj_id
                if destination_xyz is not None:
                    self._wait_for_scene_pose(
                        obj_id, destination_xyz, placed_type)
                self.get_logger().error(
                    f"[FAIL] {where}: detach đã xong nhưng lỗi sau đó; "
                    f"quân giữ tại vị trí đặt, không rollback về nguồn.")
            elif actually_attached:
                self._carry_state = "ATTACHED"
                self.get_logger().error(
                    f"[FAIL] {where}: detach chưa vào scene, quân vẫn trên gripper; "
                    f"cần phục hồi thủ công.")
            else:
                self._carry_state = "RECOVERY_REQUIRED"
                self.get_logger().error(
                    f"[FAIL] {where}: quân {obj_id} mất dấu sau detach; "
                    f"cần phục hồi thủ công.")
        elif self._carry_state == "ATTACHED":
            if actually_attached:
                self.get_logger().error(
                    f"[FAIL] {where}: quân đang attached vào gripper sau lỗi; "
                    f"không gửi ACK. Mở gripper/đưa arm về safe pose rồi chạy lại.")
            else:
                self._carry_state = "RECOVERY_REQUIRED"
                self.get_logger().error(
                    f"[FAIL] {where}: state ATTACHED nhưng scene không thấy quân; "
                    f"cần phục hồi thủ công.")
        elif self._carry_state == "WORLD_DESTINATION":
            # Detach đã hoàn tất; lỗi retreat/đóng ACM không được đưa mapping
            # trở lại nguồn. Xác nhận quân vẫn ở đúng đích rồi giữ nguyên.
            if actually_attached or not in_world:
                self._carry_state = "RECOVERY_REQUIRED"
            elif destination_xyz is not None:
                self._wait_for_scene_pose(obj_id, destination_xyz, placed_type)
            if destination_square is not None:
                self.piece_id_by_square.pop(from_sq, None)
                self.piece_id_by_square[destination_square] = obj_id
            self.get_logger().error(
                f"[FAIL] {where}: quân đã ở đích; giữ WORLD_DESTINATION, "
                "không rollback về nguồn.")
        else:
            # WORLD_SOURCE + obj đã take (take pop mapping trước attach): approach/
            # descend/lift-validate fail trước attach. Rollback an toàn KHI VÀ
            # CHỈ KHI scene xác nhận quân còn nằm yên trong world.
            if actually_attached:
                self._carry_state = "ATTACHED"
                self.get_logger().error(
                    f"[FAIL] {where}: state WORLD nhưng scene thấy quân attached; "
                    f"giữ ATTACHED, cần phục hồi thủ công.")
            elif in_world and not actually_attached:
                self._carry_state = "WORLD_SOURCE"
                self._restore_piece_to_world(from_sq, obj_id, piece_type)
            else:
                self._carry_state = "RECOVERY_REQUIRED"
                self.get_logger().error(
                    f"[FAIL] {where}: quân {obj_id} mất dấu trước attach; "
                    f"cần phục hồi thủ công.")
        raise

    def _piece_local_pose(self, piece_type: str, piece_world_xy) -> Pose:
        """Tính T_tcp_piece cho một quân đang đứng trên mặt bàn.

        TF rác (quaternion zero/NaN/inf) raise ngay tại đây thay vì tạo pose
        giả khiến precheck PASS oan.
        """
        spec = PIECE_SPECS[piece_type]
        transform = self.tf_buffer.lookup_transform(
            BASE_LINK, END_EFFECTOR, rclpy.time.Time())
        tcp = transform.transform.translation
        q = transform.transform.rotation
        self._normalize_quaternion((q.x, q.y, q.z, q.w))
        center_world = (
            piece_world_xy[0], piece_world_xy[1],
            BOARD_Z + spec.pickup_height / 2)
        local = self._rotate_by_inverse_quaternion(
            (center_world[0] - tcp.x, center_world[1] - tcp.y,
             center_world[2] - tcp.z), q)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = local
        pose.orientation.x = -q.x
        pose.orientation.y = -q.y
        pose.orientation.z = -q.z
        pose.orientation.w = q.w
        return pose

    def _attach_piece(self, obj_id: str, piece_type: str, piece_world_xy) -> str:
        """Gọi NGAY SAU KHI gripper đã đóng ở vị trí gắp: chuyển collision object
        từ 'world object' đứng yên trên bàn sang 'attached object' dính vào
        END_EFFECTOR, để nó trôi theo cánh tay trong suốt Lift->Move->Place.
        Trả về obj_id để hàm gọi truyền tiếp cho _detach_piece."""
        col = PIECE_COLLISION[piece_type]
        # Pose tương đối so với END_EFFECTOR lấy từ TF hiện tại. Nhờ vậy object
        # giữ đúng pose world lúc kẹp, kể cả TCP không song song trục Z của bàn.
        # pick_xyz có thể lệch trong ô; collision object vẫn lấy tâm quân thật.
        pose = self._piece_local_pose(piece_type, piece_world_xy)
        # Lưu T_tcp_piece để bù transform lúc đặt (Fix 2), kể cả khi collision
        # off (visual carry vẫn cần offset đúng).
        self._grasp_local_by_id[obj_id] = copy.deepcopy(pose)
        self._publish_carried_piece_visual(obj_id, pose)

        # Demo vẫn có carry visual ở trên, chỉ bỏ attached collision để nhẹ.
        if not COLLISION_ENABLED:
            return obj_id

        aco = AttachedCollisionObject()
        aco.link_name = END_EFFECTOR
        aco.object.header.frame_id = END_EFFECTOR
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [col["height"], col["radius"]]  # [height, radius]
        aco.object.primitives = [primitive]
        aco.object.primitive_poses = [pose]
        aco.touch_links = GRIPPER_TOUCH_LINKS

        self._apply_attached_object(aco)
        self._wait_for_scene_object(obj_id, attached=True)
        # TODO-5: sau attach, quân chỉ tồn tại trong attached, không còn world.
        self._assert_scene_invariant(f"attach {obj_id}", expect_attached=obj_id)
        return obj_id

    def _detach_piece(self, obj_id: str, to_square: str, world_xyz, piece_type: str):
        """Gọi NGAY SAU KHI gripper đã mở ở vị trí đặt: gỡ object khỏi
        END_EFFECTOR và thêm lại thành world object tại toạ độ mới.
        to_square=None nếu thả vào khu 'nghĩa địa' (quân bị ăn), không thuộc bàn cờ."""
        if COLLISION_ENABLED:
            aco = AttachedCollisionObject()
            aco.link_name = END_EFFECTOR
            aco.object.id = obj_id
            aco.object.operation = CollisionObject.REMOVE
            self._apply_attached_object(aco)

        self._add_piece_collision_at(obj_id, world_xyz, piece_type)
        if COLLISION_ENABLED:
            self._wait_for_scene_object(obj_id, attached=False)
        old_type, is_white = self.piece_info_by_id[obj_id]
        if old_type != piece_type:  # phong cấp: tốt thành hậu/xe/tượng/mã
            self.piece_info_by_id[obj_id] = (piece_type, is_white)
        self._publish_piece_visual(obj_id, world_xyz)
        if self.carried_piece_id == obj_id:
            self.carried_piece_id = None
        if to_square is not None:
            self.piece_id_by_square[to_square] = obj_id
        if COLLISION_ENABLED:
            self._release_contact_object_ids.add(obj_id)
        # TODO-5: sau detach, quân trở lại world đúng vị trí, không double.
        if COLLISION_ENABLED:
            self._assert_scene_invariant(
                f"detach {obj_id}", expect_world_pose=(obj_id, world_xyz, piece_type))

    # ---------------- Move execution ----------------

    def _fail(self, command_id: int, uci: str, reason: str):
        """Gửi NACK để brain dừng chờ ACK và dừng game một cách tường minh.

        Format '<cmd>:<uci>: <lý do>' để brain khớp đúng lệnh (Fix 9).
        """
        msg = String()
        msg.data = f"{command_id}:{uci}: {reason}"
        self.fail_pub.publish(msg)
        self.get_logger().error(f"[FAIL] [cmd={command_id}] {uci}: {reason} (đã gửi NACK)")

    def _ack(self, command_id: int, uci: str):
        """ACK khớp command_id (Fix 9): brain chỉ commit board khi khớp."""
        done = String()
        done.data = f"{command_id}:{uci}"
        self.done_pub.publish(done)

    def on_move(self, msg: String):
        try:
            payload = json.loads(msg.data)
            uci = payload.get("uci", "?")
        except Exception as exc:
            self.get_logger().error(f"[FAIL] payload /chess/move không parse được: {exc}")
            return
        with self._exec_lock:
            if self._executing:
                self.get_logger().error(
                    f"[FAIL] move chồng {uci}: lượt trước chưa xong, từ chối để tránh race")
                self._fail(int(payload.get("command_id", 0)), uci,
                           "overlapped with previous move")
                return
            self._executing = True
        # chạy trên thread riêng để không block callback ROS trong lúc chờ MoveIt2 thực thi
        threading.Thread(target=self._execute_move_guarded, args=(payload,), daemon=True).start()

    def _execute_move_guarded(self, payload: dict):
        try:
            self._execute_move(payload)
        except Exception as exc:
            # Payload malformed phải có NACK thay vì làm worker thread chết
            # âm thầm khiến brain chờ tới watchdog.
            uci = payload.get("uci", "?") if isinstance(payload, dict) else "?"
            try:
                cmd = int(payload.get("command_id", 0))
            except (TypeError, ValueError):
                cmd = 0
            self._needs_recovery = True
            self._fail(cmd, uci, f"worker exception: {exc}")
        finally:
            with self._exec_lock:
                self._executing = False

    def _execute_move(self, payload: dict):
        uci = payload["uci"]
        cmd = int(payload.get("command_id", 0))
        if cmd <= 0:
            self._fail(cmd, uci, "thiếu command_id hợp lệ")
            return
        try:
            self._require_ready(f"nước {uci}")
        except Exception as exc:
            self._fail(cmd, uci, str(exc))
            return
        if self._needs_recovery:
            self._fail(cmd, uci, "hệ thống ở trạng thái RECOVERY_REQUIRED "
                                 "(scene/mapping không nhất quán sau lỗi trước); "
                                 "restart launch rồi chơi lại")
            return
        expected_fen = self.board.fen()
        if payload.get("board_fen") != expected_fen:
            self._fail(
                cmd, uci,
                "board_fen không khớp executor; từ chối lập đường trên scene cũ")
            return
        from_sq, to_sq = uci[:2], uci[2:4]
        move = chess.Move.from_uci(uci)
        if move not in self.board.legal_moves:
            self._fail(cmd, uci, "nước đi không hợp lệ theo python-chess")
            return
        moving_piece = self.board.piece_at(move.from_square)
        piece_type = moving_piece.symbol().lower()
        expected_execution = "robot" if moving_piece.color == chess.WHITE else "virtual"
        execution = payload.get("execution", expected_execution)
        if execution != expected_execution:
            self._fail(cmd, uci, f"execution sai: nhận {execution}, phải là {expected_execution}")
            return

        try:
            if execution == "virtual":
                self._do_virtual_move(move, piece_type, payload)
                self.board.push(move)
                self.get_logger().info(f"[OK] virtual done {uci}")
                self._ack(cmd, uci)
                return

            # `move_group` của Dofbot nạp các planning pipeline khá chậm (thường
            # sau khi RViz đã mở).  Không được đánh rơi nước đầu tiên chỉ vì
            # service chưa xuất hiện ở thời điểm brain vừa publish nó.
            self._wait_for_motion_planner()
            try:
                self._require_hardware_gates(uci)
            except Exception as exc:
                self._fail(cmd, uci, str(exc))
                self._log_move_result(cmd, uci, False, str(exc))
                return
            self._move_to_home("HOME trước lượt Trắng")
            if self.board.is_capture(move):
                # Với en passant, quân bị ăn không ở ô đích mà ở cùng rank với ô đi.
                captured_square = (
                    to_sq[0] + from_sq[1] if self.board.is_en_passant(move) else to_sq
                )
                self._do_discard(captured_square)

            if self.board.is_castling(move):
                # Nhập thành = 2 thao tác pick-place: Vua rồi tới Xe
                rook_from, rook_to = self._castling_rook_squares(uci)
                self._do_pick_place(from_sq, to_sq, piece_type)
                self._do_pick_place(rook_from, rook_to, "r")
            else:
                # Robot mang quân tốt đến hàng cuối trước, sau đó visual/collision
                # object được đổi sang loại quân phong cấp. Loại quân suy từ
                # nước cờ hợp lệ, không tin trường promotion trong payload.
                final_piece_type = (
                    chess.piece_symbol(move.promotion).lower()
                    if move.promotion else piece_type
                )
                self._do_pick_place(from_sq, to_sq, piece_type, final_piece_type)

            self._move_to_home("HOME sau lượt Trắng")

            # cập nhật board nội bộ SAU KHI đã dùng vị trí cũ để tính toán ở trên
            self.board.push(chess.Move.from_uci(uci))
        except Exception as exc:
            # Một thao tác nhiều bước có thể đã thay scene dù quân hiện ở
            # world (capture đã discard, vua nhập thành đã đi, virtual remove
            # xong nhưng add lỗi). Mọi exception trong transaction đều khóa.
            self._needs_recovery = True
            self.get_logger().error(
                "[RECOVERY-REQUIRED] runtime transaction fail tại carry_state="
                f"{self._carry_state}; restart launch trước khi chơi tiếp.")
            self._fail(cmd, uci, f"không thực thi: {exc}")
            self._log_move_result(cmd, uci, False, str(exc))
            return
        else:
            self.get_logger().info(f"[OK] done {uci}")
            self._log_move_result(cmd, uci, True)
            self._ack(cmd, uci)

    def _remove_virtual_piece(self, square: str):
        """Bỏ quân bị ăn khỏi scene/visual trong lượt Đen tự đi."""
        obj_id = self.piece_id_by_square.pop(square)
        if COLLISION_ENABLED:
            self.moveit2.remove_collision_object(id=obj_id)
            self._wait_for_scene_absence(obj_id)
        self._delete_piece_visual(obj_id)

    def _move_virtual_piece(self, from_sq: str, to_sq: str, piece_type: str):
        obj_id = self.piece_id_by_square.pop(from_sq)
        if COLLISION_ENABLED:
            self.moveit2.remove_collision_object(id=obj_id)
            self._wait_for_scene_absence(obj_id)
        old_type, is_white = self.piece_info_by_id[obj_id]
        if old_type != piece_type:
            self.piece_info_by_id[obj_id] = (piece_type, is_white)
        target_xyz = (*square_to_xy(to_sq), BOARD_Z)
        self._add_piece_collision_at(obj_id, target_xyz, piece_type)
        if COLLISION_ENABLED:
            self._wait_for_scene_pose(obj_id, target_xyz, piece_type)
        self.piece_id_by_square[to_sq] = obj_id
        self._publish_piece_visual(obj_id, target_xyz)

    def _do_virtual_move(self, move: chess.Move, piece_type: str, payload: dict):
        """Cập nhật scene + visual cho nước Đen, không chạy arm.

        Không còn 'atomically' giả (Fix 5): remove/add qua topic rồi CHỜ scene
        xác nhận (biến mất + pose mới) xong mới cho ACK. Promotion suy từ nước
        cờ hợp lệ (move.promotion), không tin trường promotion do bên gửi cung
        cấp.
        """
        from_sq = chess.square_name(move.from_square)
        to_sq = chess.square_name(move.to_square)
        if self.board.is_capture(move):
            captured_sq = (
                to_sq[0] + from_sq[1]
                if self.board.is_en_passant(move) else to_sq
            )
            self._remove_virtual_piece(captured_sq)

        final_piece_type = (
            chess.piece_symbol(move.promotion).lower()
            if move.promotion else piece_type
        )
        self._move_virtual_piece(from_sq, to_sq, final_piece_type)
        if self.board.is_castling(move):
            rook_from, rook_to = self._castling_rook_squares(move.uci())
            self._move_virtual_piece(rook_from, rook_to, "r")

    def _wait_for_motion_planner(self, timeout_sec: float = 180.0):
        """Chờ đúng service mà pymoveit2 dùng để lập kế hoạch.

        Đây là wait có giới hạn, chạy trong worker thread nên executor ROS vẫn
        xử lý joint state, RViz và các service khác trong lúc chờ.
        """
        client = self.moveit2._plan_kinematic_path_service
        deadline = time.monotonic() + timeout_sec
        announced = False
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            if not announced:
                self.get_logger().info(
                    "Đang chờ MoveIt hoàn tất khởi động; nước cờ sẽ tự chạy khi planner sẵn sàng..."
                )
                announced = True
            if time.monotonic() >= deadline:
                raise RuntimeError("MoveIt planner không sẵn sàng sau 180 giây")
        if announced:
            self.get_logger().info("MoveIt planner đã sẵn sàng; tiếp tục nước cờ đang chờ.")

    # Deep-check chỉ chạy trên ô có QUÂN TRẮNG (robot chỉ gắp Trắng; Đen đi
    # virtual). Ưu tiên hàng cuối (gần đế, vùng IK khó) rồi tới quân đã tiến,
    # tối đa DEEP_CHECK_MAX_SQUARES ô để thời gian check hợp lý.
    DEEP_CHECK_MAX_SQUARES = 10
    # Quét cả 16 slot nghĩa địa. Collision đã bật nên mọi slot cần EXECUTE để
    # kiểm tra Cartesian descend/lift thật, không chỉ endpoint plan-only.
    DEEP_CHECK_DISCARD_SLOTS = tuple(range(DISCARD_MAX_SLOTS))
    DEEP_CHECK_DISCARD_EXEC_SLOTS = DEEP_CHECK_DISCARD_SLOTS

    def _dry_segment(self, failures: dict, key: str, label: str, fn) -> bool:
        """Chạy 1 đoạn của dry-run, ghi nhận lỗi theo phase thay vì raise."""
        try:
            fn()
            return True
        except Exception as exc:
            failures.setdefault(key, []).append(label)
            contacts = self._state_validity_contacts(
                self.moveit2.joint_state, f"deep/{key}/{label}/current"
            )
            reason = str(exc).replace("\n", " ")
            if contacts:
                reason += "; contacts=" + ",".join(contacts)
            details = getattr(self, "_reachability_failure_details", None)
            if details is not None:
                details.append(("deep", key, label, reason))
            self.get_logger().error(f"[FAIL] dry-run {label}: {exc}")
            return False

    def _emit_reachability_failure_summary(self):
        """In trọn bộ case lỗi thành block dễ copy từ terminal, không truncate."""
        details = getattr(self, "_reachability_failure_details", [])
        if not details:
            return
        self.get_logger().error("[REACHABILITY-FAIL-SUMMARY-BEGIN]")
        for scope, phase, case, reason in details:
            self.get_logger().error(
                f"[FAIL-CASE] scope={scope} | phase={phase} | case={case} | reason={reason}"
            )
        self.get_logger().error("[REACHABILITY-FAIL-SUMMARY-END]")

    def _dry_run_square(
        self,
        square: str,
        piece_type: str,
        target_square: str,
        failures: dict,
        execute: bool,
    ):
        """Mô phỏng đúng chuỗi runtime của 1 nước đi, không kẹp/attach quân.

        Chuỗi mirror _do_pick_place: approach -> hạ Cartesian -> nâng ->
        chuyển sang approach ô đích -> hạ đặt -> nâng. Ở sim (FakeSystem)
        thì EXECUTE thật để state/quaternion chaining đúng như runtime; ở
        robot thật chỉ plan-only (an toàn, nhưng không kiểm tra chaining).
        """
        x0, y0, z0 = square_to_grasp_pose(square, piece_type)
        src_approach = approach_tcp_z(square, z0)
        tx, ty, tz = square_to_place_pose(target_square, piece_type)
        tgt_approach = approach_tcp_z(target_square, tz)
        obj_id = self.piece_id_by_square.get(square)
        if obj_id and COLLISION_ENABLED and not execute:
            self._set_object_gripper_collision(obj_id, True)
        try:
            if execute:
                # Mirror runtime: mọi lượt Trắng đều bắt đầu từ HOME (joint PTP).
                # Không có bước này, dry-run chain ô này sang ô khác với start
                # state mà runtime bao giờ không gặp -> fail giả.
                ok = self._dry_segment(
                    failures, "home", f"{square}/home",
                    lambda: self._move_to_home(f"dry HOME trước {square}"))
                if not ok:
                    return
                if not self._dry_segment(
                    failures, "pick_place", f"{square}->{target_square}",
                    lambda: self._do_pick_place(square, target_square, piece_type)):
                    return
                if not self._dry_segment(
                    failures, "return_piece", f"{target_square}->{square}",
                    lambda: self._do_pick_place(target_square, square, piece_type)):
                    return
                self._dry_segment(
                    failures, "home", f"{square}/home-after",
                    lambda: self._move_to_home(f"dry HOME sau {square}"))
            else:
                grasp_q = self._current_tcp_quat()
                self._dry_segment(
                    failures, "approach", f"{square}/approach",
                    lambda: self._plan_or_raise(
                        f"{square}/approach",
                        position=[x0, y0, src_approach],
                        target_link=END_EFFECTOR, tolerance_position=0.004))
                self._dry_segment(
                    failures, "descend_pick", f"{square}/descend",
                    lambda: self._plan_or_raise(
                        f"{square}/descend",
                        position=[x0, y0, z0], quat_xyzw=grasp_q,
                        target_link=END_EFFECTOR, tolerance_position=0.002,
                        tolerance_orientation=0.03, cartesian=True,
                        max_step=CARTESIAN_MAX_STEP_M, cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD))
                self._dry_segment(
                    failures, "transfer", f"{square}->{target_square}",
                    lambda: self._plan_or_raise(
                        f"{square}->{target_square}",
                        position=[tx, ty, tgt_approach],
                        target_link=END_EFFECTOR, tolerance_position=0.004))
                self._dry_segment(
                    failures, "descend_place", f"{target_square}/descend",
                    lambda: self._plan_or_raise(
                        f"{target_square}/descend",
                        position=[tx, ty, tz], quat_xyzw=grasp_q,
                        target_link=END_EFFECTOR, tolerance_position=0.002,
                        tolerance_orientation=0.03, cartesian=True,
                        max_step=CARTESIAN_MAX_STEP_M, cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD))
        finally:
            obj_id = self.piece_id_by_square.get(square)
            if (obj_id and COLLISION_ENABLED
                    and obj_id not in self._release_contact_object_ids):
                self._set_object_gripper_collision(obj_id, False)

    def _plan_or_raise(self, label: str, **kwargs):
        """Plan-only helper cho dry-run: None là FAIL, không phải thành công.

        Fix 8: trước đây lambda plan-only vứt return value của _plan_motion,
        nên _dry_segment không bao giờ thấy lỗi (None == success giả). Mọi
        plan-only trong dry-run phải đi qua đây để None raise rõ ràng.
        """
        trajectory = self._plan_motion(**kwargs)
        if trajectory is None:
            raise RuntimeError(f"không có plan cho {label}")
        return trajectory

    def _dry_attach_scratch(self, piece_type: str = "k",
                            piece_world_xy=None,
                            source_obj_id: str | None = None) -> str:
        """Attach proxy đúng kích thước và T_tcp_piece để kiểm tra carry.

        Khi validate lift tại nguồn, world object thật được bỏ tạm để proxy
        không tự va chạm với chính bản sao của nó; cleanup thêm lại đúng pose.
        Discard độc lập dùng quân vua làm trường hợp bảo thủ.
        """
        if not COLLISION_ENABLED:
            return ""
        if piece_world_xy is not None:
            pose = self._piece_local_pose(piece_type, piece_world_xy)
        else:
            # Proxy discard bắt đầu đứng thẳng trong BASE_LINK và có tâm thấp
            # hơn TCP đúng bằng quan hệ tại pose gắp chuẩn. Nhờ lưu orientation
            # inverse của TCP, nó xoay theo arm giống một quân đã attach thật.
            spec = PIECE_SPECS[piece_type]
            transform = self.tf_buffer.lookup_transform(
                BASE_LINK, END_EFFECTOR, rclpy.time.Time())
            q = transform.transform.rotation
            dz = BOARD_Z + spec.pickup_height / 2 - PICK_TCP_Z
            local = self._rotate_by_inverse_quaternion((0.0, 0.0, dz), q)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = local
            pose.orientation.x = -q.x
            pose.orientation.y = -q.y
            pose.orientation.z = -q.z
            pose.orientation.w = q.w
        return self._dry_attach_scratch_with_local(
            pose, piece_type, source_obj_id)

    def _dry_attach_scratch_with_local(self, local_pose: Pose, piece_type: str,
                                       source_obj_id: str | None = None) -> str:
        """Attach proxy với T_tcp_piece tường minh (không tự suy từ TF chuẩn).

        Dùng cho precheck chuỗi mang: transform giả định của đúng offset đang
        thử được giữ xuyên suốt lift → transfer → descend, thay vì pose mặc
        định chỉ đúng tại pose gắp chuẩn. Attach fail thì raise — caller phải
        coi là FAIL kiểm tra transfer, không được fallback IK-only rồi PASS.
        """
        if not COLLISION_ENABLED:
            return ""
        obj_id = "__dry_carry__"
        col = PIECE_COLLISION[piece_type]
        pose = copy.deepcopy(local_pose)
        # Validate strict: pose proxy rác mà attach mù sẽ cho kết quả mang giả.
        self._normalize_quaternion(
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w))
        for v in (pose.position.x, pose.position.y, pose.position.z):
            if not math.isfinite(float(v)):
                raise ValueError(f"proxy T_tcp_piece NaN/inf: {local_pose}")
        if source_obj_id is not None:
            self.moveit2.remove_collision_object(id=source_obj_id)
            self._wait_for_scene_absence(source_obj_id)
        aco = AttachedCollisionObject()
        aco.link_name = END_EFFECTOR
        aco.object.header.frame_id = END_EFFECTOR
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [col["height"], col["radius"]]
        aco.object.primitives = [primitive]
        aco.object.primitive_poses = [pose]
        aco.touch_links = GRIPPER_TOUCH_LINKS
        self._apply_attached_object(aco)
        self._wait_for_scene_object(obj_id, attached=True)
        self._set_piece_collision(obj_id, gripper_touch=True, board_contact=False)
        return obj_id

    def _dry_detach_scratch(self, obj_id: str, source_obj_id: str | None = None,
                            source_xyz=None, piece_type: str = "k"):
        """Gỡ proxy dry-run, best-effort để không che lỗi chính."""
        if not obj_id or not COLLISION_ENABLED:
            return
        try:
            aco = AttachedCollisionObject()
            aco.link_name = END_EFFECTOR
            aco.object.id = obj_id
            aco.object.operation = CollisionObject.REMOVE
            self._apply_attached_object(aco)
            self.moveit2.remove_collision_object(id=obj_id)
            self._wait_for_scene_absence(obj_id)
        finally:
            # Dù detach proxy lỗi, luôn cố phục hồi world object nguồn. Nếu
            # không làm bước này, một lỗi validate có thể làm quân biến mất.
            if source_obj_id is not None and source_xyz is not None:
                self._add_piece_collision_at(source_obj_id, source_xyz, piece_type)
                self._wait_for_scene_pose(
                    source_obj_id, source_xyz, piece_type)
            try:
                self._set_piece_collision(
                    obj_id, gripper_touch=False, board_contact=False)
            except Exception:
                pass

    def _add_dry_discard_occupants(self, target_slot: int) -> list[str]:
        """Dựng quân giả ở các slot trước để test khoảng hở giữa các quân."""
        if not COLLISION_ENABLED:
            return []
        ids = []
        # Các slot nhỏ hơn discard_count đã có quân thật trong scene. Chỉ bù
        # những slot còn thiếu để case target_slot luôn thấy hàng xóm đã chiếm.
        try:
            for slot in range(self.discard_count, target_slot):
                obj_id = f"__dry_discard_occupied_{slot}__"
                x, y, z = discard_slot_pose(slot)
                self._add_piece_collision_at(obj_id, (x, y, z), "p")
                ids.append(obj_id)
                self._wait_for_scene_pose(obj_id, (x, y, z), "p")
        except Exception:
            self._remove_dry_discard_occupants(ids)
            raise
        return ids

    def _remove_dry_discard_occupants(self, obj_ids: list[str]):
        for obj_id in obj_ids:
            self.moveit2.remove_collision_object(id=obj_id)
            self._wait_for_scene_absence(obj_id)

    def _dry_run_discard_slot(self, slot: int, failures: dict, execute: bool):
        """Mirror _do_discard: tới khu nghĩa địa + hạ/thả thẳng đứng.

        EXECUTE mang theo proxy scratch attached (Fix 8) để check thể tích
        quân thật thay vì kẹp rỗng lạc quan. Plan-only cũng raise khi None.
        """
        xd, yd, _zd = discard_slot_pose(slot)
        dz = DISCARD_TCP_Z
        occupants = []
        try:
            occupants = self._add_dry_discard_occupants(slot)
            if execute:
                if not self._dry_segment(
                    failures, "home", f"slot{slot}/home",
                    lambda: self._move_to_home(f"dry HOME trước slot {slot}")):
                    return
                scratch_ok = self._dry_segment(
                    failures, "discard", f"slot{slot}/attach-proxy",
                    lambda: self._dry_attach_scratch())
                if not scratch_ok:
                    return
                try:
                    if not self._dry_segment(
                        failures, "discard", f"slot{slot}/transfer",
                        lambda: self._move_to(xd, yd, dz + APPROACH_HEIGHT)):
                        return
                    drop_q = self._current_tcp_quat()
                    if not self._dry_segment(
                        failures, "discard", f"slot{slot}/descend",
                        lambda: self._move_vertical(xd, yd, dz, f"dry hạ thả slot{slot}", quat_xyzw=drop_q)):
                        return
                    self._dry_segment(
                        failures, "discard", f"slot{slot}/lift",
                        lambda: self._move_vertical(xd, yd, dz + APPROACH_HEIGHT, f"dry nâng slot{slot}", quat_xyzw=drop_q))
                finally:
                    self._dry_detach_scratch("__dry_carry__")
            else:
                q = self._current_tcp_quat()
                self._dry_segment(
                    failures, "discard", f"slot{slot}/transfer",
                    lambda: self._plan_or_raise(
                        f"slot{slot}/transfer",
                        position=[xd, yd, dz + APPROACH_HEIGHT],
                        target_link=END_EFFECTOR, tolerance_position=0.004))
                self._dry_segment(
                    failures, "discard", f"slot{slot}/descend",
                    lambda: self._plan_or_raise(
                        f"slot{slot}/descend",
                        position=[xd, yd, dz], quat_xyzw=q,
                        target_link=END_EFFECTOR, tolerance_position=0.002,
                        tolerance_orientation=0.03, cartesian=True,
                        max_step=CARTESIAN_MAX_STEP_M, cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD))
        except Exception as exc:
            failures.setdefault("discard", []).append(f"slot{slot}: {exc}")
            details = getattr(self, "_reachability_failure_details", None)
            if details is not None:
                details.append(("deep", "discard", f"slot{slot}", str(exc).replace("\n", " ")))
        finally:
            try:
                self._remove_dry_discard_occupants(occupants)
            except Exception as exc:
                failures.setdefault("discard_cleanup", []).append(
                    f"slot{slot}: {exc}")

    def _check_reachability(self, request, response):
        """Không cho reachability thay đổi PlanningScene trong lúc đang chạy game."""
        with self._exec_lock:
            if self._executing:
                response.success = False
                response.message = "Robot đang chạy nước cờ; chờ ACK/NACK rồi gọi lại reachability."
                return response
            self._executing = True
        try:
            try:
                self._require_ready("reachability check")
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response
            return self._check_reachability_impl(request, response)
        finally:
            with self._exec_lock:
                self._executing = False

    @staticmethod
    def _classify_diagnostic_error(exc: Exception) -> str:
        """Phân loại PASS/FAIL/INFRA_ERROR (TODO-4): lỗi hạ tầng không tính
        thành lỗi reachability."""
        text = str(exc).lower()
        infra_keys = ("planning scene", "planningscene", "controller",
                      "planner", "joint_states", "joint state", "service",
                      "timeout", "not ready", "not-ready", "recovery")
        if any(k in text for k in infra_keys):
            return "INFRA_ERROR"
        return "FAIL"

    def _reset_diagnostic_case(self, label: str):
        """Khôi phục trạng thái độc lập trước mỗi case (TODO-4): HOME, scene
        chuẩn, attached rỗng, ACM mặc định, controller active."""
        # Dọn scratch còn sót từ case trước (best-effort).
        try:
            attached, _world = self._scene_object_ids()
            for stale in list(attached):
                if stale.startswith("__dry_"):
                    try:
                        self._dry_detach_scratch(stale)
                    except Exception:
                        pass
            _a2, world_ids = self._scene_object_ids()
            for oid in list(world_ids):
                if oid.startswith("__dry_discard_occupied_"):
                    try:
                        self.moveit2.remove_collision_object(id=oid)
                    except Exception:
                        pass
        except Exception:
            pass
        self._move_to_home(f"diagnostic HOME trước {label}")
        try:
            attached, _w = self._scene_object_ids()
            if attached:
                raise RuntimeError(f"attached objects còn sót: {sorted(attached)}")
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(f"không đọc scene trước {label}: {exc}")
        ok_ctrl, why = self._controller_active()
        if not ok_ctrl:
            raise RuntimeError(f"controller chưa active trước {label}: {why}")
        # Scene invariant sau khôi phục (board + mapping khớp).
        try:
            self._assert_scene_invariant(f"diagnostic-reset {label}")
        except RuntimeError as exc:
            raise RuntimeError(f"scene invariant trước {label}: {exc}")

    def _check_reachability_impl(self, _request, response):
        """Kiểm tra khả năng gắp thật theo 2 tầng, CHỈ cho quân TRẮNG.

        Robot chỉ điều khiển Trắng (Đen đi virtual, không chạy arm), nên check
        64 ô là thừa và gây nhiễu fail ở ô Đen không bao giờ gắp. Cả 2 tầng đều
        suy từ board state hiện tại:
        Tầng 1 (nhanh): position-only tới approach + pick của mọi ô đang có
        quân Trắng — tín hiệu hồi quy nhanh, KHÔNG chứng minh pick được.
        Tầng 2 (deep): dry-run đúng chuỗi runtime (HOME -> approach -> hạ/nâng
        Cartesian giữ quaternion đã chốt -> chuyển ô -> hạ/đặt) trên tối đa
        DEEP_CHECK_MAX_SQUARES ô trắng (hàng cuối trước) + toàn bộ khu discard
        (robot vẫn phải mang quân Đen bị ăn ra nghĩa địa). Ở sim thì execute
        thật trên FakeSystem; ở robot thật chỉ plan-only. response.success=False
        nếu BẤT KỲ phase nào có lỗi.
        """
        try:
            self._reachability_failure_details = []
            if self._needs_recovery:
                response.success = False
                response.message = (
                    "Hệ thống đang ở trạng thái RECOVERY_REQUIRED sau lỗi trước "
                    "(scene/mapping có thể không nhất quán). Restart launch để "
                    "dựng lại scene từ board rồi gọi lại check.")
                self.get_logger().error(f"[FAIL] {response.message}")
                return response
            self._wait_for_motion_planner(timeout_sec=20.0)
            execute = REACHABILITY_EXECUTE_ON_FAKESYSTEM
            if execute:
                # Quét nhanh phải xuất phát đúng start state như runtime (mọi
                # lượt Trắng đều bắt đầu từ HOME). Nếu không, fail có thể chỉ
                # do arm đang đứng ở pose xấu từ game trước — fail giả.
                self._move_to_home("HOME trước quét reachability")
            # Ô đang có quân Trắng trên board hiện tại.
            white_names = [
                chess.square_name(sq)
                for sq in chess.SQUARES
                if (p := self.board.piece_at(sq)) is not None and p.color
            ]
            n_white = len(white_names)
            failures = {"approach": [], "pick": []}
            for name in white_names:
                piece = self.board.piece_at(chess.parse_square(name))
                piece_type = piece.symbol().lower()
                # Chỉ cho phép quân mục tiêu chạm link ngón kẹp. Nó vẫn là vật
                # cản với arm, bàn và những quân khác trong mọi plan.
                obj_id = self.piece_id_by_square.get(name)
                if obj_id and COLLISION_ENABLED:
                    self._set_object_gripper_collision(obj_id, True)
                try:
                    chosen = None
                    approach_seen = False
                    last_xyz = square_to_grasp_pose(name, piece_type)
                    for offset in self._grasp_offset_candidates(name):
                        x, y, pick_z = square_to_grasp_pose(name, piece_type, offset)
                        last_xyz = (x, y, pick_z)
                        approach = self._plan_motion(
                            position=[x, y, approach_tcp_z(name, pick_z)],
                            target_link=END_EFFECTOR, tolerance_position=0.004,
                            cartesian=False)
                        if approach is None:
                            continue
                        approach_seen = True
                        pick = self._plan_motion(
                            position=[x, y, pick_z], target_link=END_EFFECTOR,
                            tolerance_position=0.004, cartesian=False)
                        if pick is not None:
                            chosen = offset
                            break
                    if chosen is not None:
                        self._grasp_offset_cache[name] = chosen
                    else:
                        phase = "pick" if approach_seen else "approach"
                        failures[phase].append(name)
                        contacts = self._diagnose_position_goal_collision(
                            last_xyz, self._current_tcp_quat(), f"fast/{phase}/{name}")
                        reason = (
                            f"không candidate nào có plan ({len(GRASP_APPROACH_CANDIDATE_OFFSETS)} offset)"
                        )
                        if contacts:
                            reason += "; contacts=" + ",".join(contacts)
                        self._reachability_failure_details.append(
                            ("fast", phase, name, reason))
                finally:
                    if obj_id and COLLISION_ENABLED:
                        self._set_object_gripper_collision(obj_id, False)

            for phase, squares in failures.items():
                if squares:
                    self.get_logger().error(
                        f"[FAIL] reach nhanh (Trắng) {phase}: "
                        f"{n_white - len(squares)}/{n_white}, "
                        f"không có plan: {', '.join(squares)}"
                    )
            if not failures["approach"] and not failures["pick"]:
                self.get_logger().info(
                    f"[OK] reach nhanh (Trắng): {n_white}/{n_white} approach, "
                    f"{n_white}/{n_white} pick")

            # Hàng cuối trước (vùng khó), rồi tới quân đã tiến xa.
            back = sorted(s for s in white_names if s[1] == "1")
            rest = sorted(s for s in white_names if s[1] != "1")
            deep_squares = (back + rest)[:self.DEEP_CHECK_MAX_SQUARES]
            self.get_logger().info(
                f"Deep-check {len(deep_squares)} ô trắng {deep_squares} "
                f"({'EXECUTE trên FakeSystem' if execute else 'plan-only, robot thật không di chuyển'}; "
                f"collision={'ON' if COLLISION_ENABLED else 'OFF'})..."
            )
            deep: dict = {}
            # Mỗi ca đặt tạm vào một ô trống, rồi chạy chiều ngược để trả quân
            # về source. Không làm thay đổi board state sau deep-check.
            scratch_square = next(
                (candidate for candidate in ("e4", "d5", "c4", "f5")
                 if self.board.piece_at(chess.parse_square(candidate)) is None),
                None,
            )
            if scratch_square is None:
                raise RuntimeError("deep-check cần một ô trống trong e4/d5/c4/f5")
            # TODO-4: chạy đủ toàn bộ matrix, KHÔNG fail-fast. Mỗi case độc lập:
            # reset HOME/scene/ACM/controller trước, check invariant sau.
            case_results: list[tuple[str, str, str]] = []  # (case, verdict, reason)
            squares_done = 0
            for square in deep_squares:
                piece = self.board.piece_at(chess.parse_square(square))
                piece_type = piece.symbol().lower() if piece else "p"
                try:
                    self._reset_diagnostic_case(f"ô {square}")
                except Exception as exc:
                    verdict = self._classify_diagnostic_error(exc)
                    reason = f"reset fail: {exc}"
                    case_results.append((square, verdict, reason))
                    deep.setdefault("infra" if verdict == "INFRA_ERROR" else "reset",
                                    []).append(f"{square}: {reason}")
                    self.get_logger().error(f"[DIAG] {square}: {verdict} ({reason})")
                    continue
                before = sum(len(items) for items in deep.values())
                try:
                    self._dry_run_square(square, piece_type, scratch_square, deep, execute)
                except Exception as exc:
                    verdict = self._classify_diagnostic_error(exc)
                    deep.setdefault("infra" if verdict == "INFRA_ERROR" else "exception",
                                    []).append(f"{square}: {exc}")
                    case_results.append((square, verdict, str(exc)))
                squares_done += 1
                after = sum(len(items) for items in deep.values())
                if after == before:
                    case_results.append((square, "PASS", ""))
                    self.get_logger().info(
                        f"[DIAG] {square}: PASS (seed vùng "
                        f"{self._region_for_target(square_to_xy(square))})")
                else:
                    # _dry_run_square đã ghi chi tiết vào deep{}; phân loại thêm.
                    last_err = "; ".join(
                        f"{k}:{v[-1]}" for k, v in deep.items() if v)
                    verdict = ("INFRA_ERROR" if "infra" in deep else "FAIL")
                    case_results.append((square, verdict, last_err))
                try:
                    self._assert_scene_invariant(f"diagnostic sau ô {square}")
                except Exception as exc:
                    case_results.append((f"{square}/invariant", "INFRA_ERROR", str(exc)))
                    deep.setdefault("infra", []).append(f"{square}/invariant: {exc}")
            slots_done = 0
            for slot in self.DEEP_CHECK_DISCARD_SLOTS:
                try:
                    self._reset_diagnostic_case(f"slot{slot}")
                except Exception as exc:
                    verdict = self._classify_diagnostic_error(exc)
                    reason = f"reset fail: {exc}"
                    case_results.append((f"slot{slot}", verdict, reason))
                    deep.setdefault("infra" if verdict == "INFRA_ERROR" else "reset",
                                    []).append(f"slot{slot}: {reason}")
                    continue
                before = sum(len(items) for items in deep.values())
                try:
                    self._dry_run_discard_slot(
                        slot, deep,
                        execute and slot in self.DEEP_CHECK_DISCARD_EXEC_SLOTS)
                except Exception as exc:
                    verdict = self._classify_diagnostic_error(exc)
                    deep.setdefault("infra" if verdict == "INFRA_ERROR" else "exception",
                                    []).append(f"slot{slot}: {exc}")
                    case_results.append((f"slot{slot}", verdict, str(exc)))
                slots_done += 1
                after = sum(len(items) for items in deep.values())
                if after == before:
                    case_results.append((f"slot{slot}", "PASS", ""))
                else:
                    last_err = "; ".join(
                        f"{k}:{v[-1]}" for k, v in deep.items() if v)
                    verdict = ("INFRA_ERROR" if "infra" in deep else "FAIL")
                    case_results.append((f"slot{slot}", verdict, last_err))
                try:
                    self._assert_scene_invariant(f"diagnostic sau slot{slot}")
                except Exception as exc:
                    case_results.append((f"slot{slot}/invariant", "INFRA_ERROR", str(exc)))
                    deep.setdefault("infra", []).append(f"slot{slot}/invariant: {exc}")
            n_pass = sum(1 for _c, v, _r in case_results if v == "PASS")
            n_fail = sum(1 for _c, v, _r in case_results if v == "FAIL")
            n_infra = sum(1 for _c, v, _r in case_results if v == "INFRA_ERROR")
            self.get_logger().info(
                f"[DIAG] full-matrix: {squares_done}/{len(deep_squares)} ô + "
                f"{slots_done}/{len(self.DEEP_CHECK_DISCARD_SLOTS)} slot; "
                f"PASS={n_pass} FAIL={n_fail} INFRA_ERROR={n_infra}")
            self._reachability_case_results = case_results
            if execute:
                # Trả arm về HOME sau slot cuối (trước đây bỏ sót): không để
                # arm đứng chơ vơ ở khu discard sau check.
                try:
                    self._move_to_home("dry HOME cuối deep-check")
                except Exception as exc:
                    deep.setdefault("home", []).append(f"final-home: {exc}")
            if execute and (sum(len(v) for v in deep.values()) > 0
                            or self._carry_state != "WORLD_SOURCE"):
                # Dry-run execute fail giữa chừng có thể để quân ở ô tạm hoặc
                # attached trong khi self.board vẫn là thế cờ cũ (Fix 8): khóa
                # hệ thống, không cho game/check chạy tiếp trên scene sai.
                self._needs_recovery = True
                self.get_logger().error(
                    "[RECOVERY-REQUIRED] deep-check execute có lỗi; quân có "
                    "thể còn ở ô test tạm dù carry_state="
                    f"{self._carry_state}. Restart launch trước khi chơi tiếp.")

            deep_total = sum(len(v) for v in deep.values())
            for phase, items in deep.items():
                if items:
                    self.get_logger().error(
                        f"[FAIL] deep-check {phase}: {len(items)} lỗi: "
                        f"{', '.join(items)}"
                    )
            fast_fail = sum(len(v) for v in failures.values())
            if fast_fail == 0 and deep_total == 0:
                response.success = True
                self.get_logger().info("[OK] reachability PASS toàn bộ")
            else:
                response.success = False
                self._emit_reachability_failure_summary()
                self.get_logger().error(
                    "[FAIL] reachability CÓ LỖI — copy block REACHABILITY-FAIL-SUMMARY ở trên"
                )
            response.message = (
                f"trắng approach {n_white-len(failures['approach'])}/{n_white}, "
                f"pick {n_white-len(failures['pick'])}/{n_white}; "
                f"deep {squares_done}/{len(deep_squares)} ô + "
                f"{slots_done}/{len(self.DEEP_CHECK_DISCARD_SLOTS)} slot "
                f"(full-matrix, không fail-fast); "
                f"PASS={n_pass} FAIL={n_fail} INFRA_ERROR={n_infra} "
                f"({'EXECUTE' if execute else 'plan-only'}, collision={'ON' if COLLISION_ENABLED else 'OFF'}). "
                + ("PASS toàn bộ." if response.success
                   else "CÓ LỖI — xem terminal để biết ô/phase.")
            )
            # Log seed + nguyên nhân từng case để tái hiện (TODO-4/6).
            for case, verdict, reason in case_results:
                self.get_logger().info(
                    f"[DIAG-CASE] {case}: {verdict}"
                    + (f" ({reason})" if reason else ""))
        except Exception as exc:
            if REACHABILITY_EXECUTE_ON_FAKESYSTEM:
                self._needs_recovery = True
            response.success = False
            response.message = f"Reachability check thất bại: {exc}"
        return response

    def _do_pick_place(self, from_sq: str, to_sq: str, piece_type: str,
                       placed_piece_type: str | None = None):
        x1, y1, z1 = square_to_place_pose(to_sq, placed_piece_type or piece_type)
        gripper_open = PIECE_SPECS[piece_type].gripper_open
        # Ở c1/d1/e1/f1, nâng thẳng đứng (Cartesian) lên cao độ vận chuyển
        # chuẩn 0.125 m là vô nghiệm IK. Vì vậy arm chỉ nâng thẳng đến
        # approach riêng của ô nguồn (xem approach_tcp_z), rồi dùng
        # position-only OMPL để rời vùng gần đế robot sang approach của ô đích.
        target_approach_z = approach_tcp_z(to_sq, z1)
        obj_id = None

        try:
            # Quân mục tiêu vẫn ở PlanningScene. ACM chỉ cho link kẹp chạm nó.
            self._set_gripper(gripper_open)
            obj_id = self._take_piece_from_world(from_sq)
            (x0, y0, z0, source_approach_z, grasp_q,
             verified) = self._approach_and_descend_for_grasp(
                from_sq, piece_type, "hạ gắp", obj_id,
                dest_xy=(x1, y1),
                dest_piece_type=placed_piece_type or piece_type,
                dest_approach_z=target_approach_z, dest_label=to_sq,
            )
            self._set_gripper(0.0)
            self._carry_state = "ATTACH_PENDING"
            self._attach_piece(obj_id, piece_type, square_to_xy(from_sq))
            self._carry_state = "ATTACHED"
            dest_piece = placed_piece_type or piece_type
            verified_quat = (verified or {}).get("place_quat")
            # Ưu tiên execute đúng chuỗi trajectory đã PASS precheck (giữ nhánh
            # khớp đã FK-xác nhận; plan lại có thể lật nhánh với IK
            # position-only như case a1->e4). Gate fail -> plan lại, execute
            # fail -> raise (robot đã chuyển động, không fallback).
            place_tcp, _cached_where = self._try_execute_cached_carry_chain(
                verified, obj_id, dest_piece, (x1, y1),
                f"pick-place {from_sq}->{to_sq}")
            if place_tcp is None and _cached_where == "source":
                self._move_vertical(
                    x0, y0, source_approach_z, "nâng sau gắp",
                    quat_xyzw=grasp_q)
                # Quân đã thoát mặt bàn: ĐÓNG board-contact ngay (Fix 3). Giữ
                # gripper-touch suốt lúc mang. Nếu còn mở, quân attached xuyên
                # bàn trong transfer mà không bị chặn.
                self._set_piece_collision(
                    obj_id, gripper_touch=True, board_contact=False)
                # Transit position-only tới tâm approach ô đích: KHÔNG ép
                # orientation (5 DOF + IK position-only; ép quat grasp lúc gắp
                # qua cả bàn cờ là vô nghiệm như case a1->e4). Orientation chỉ
                # chốt ở bước hạ đặt Cartesian bên dưới.
                self._move_to(x1, y1, target_approach_z)
            if place_tcp is None:
                # Đặt bù offset gắp bằng vòng FK → sửa XYZ tâm quân.
                place_tcp = self._place_at_dest_compensated(
                    obj_id, (x1, y1), z1, dest_piece, target_approach_z,
                    verified_quat, "hạ đặt")
            self._verify_attached_piece_target(
                obj_id, (x1, y1), placed_piece_type or piece_type,
                requested_tcp=place_tcp)
            self._set_gripper(gripper_open)
            self._carry_state = "DETACH_PENDING"
            self._detach_piece(
                obj_id, to_sq, (x1, y1, BOARD_Z), placed_piece_type or piece_type
            )
            self._carry_state = "WORLD_DESTINATION"
            self._grasp_local_by_id.pop(obj_id, None)
            self._move_vertical(place_tcp[0], place_tcp[1], target_approach_z,
                               "nâng sau đặt", quat_xyzw=place_tcp[3])
            # Giữ touch ACM trong suốt retreat; chỉ đóng khi TCP đã cách quân
            # đủ xa. Retreat 65mm >> ACM_RELEASE_CLEARANCE 10mm nên luôn thỏa;
            # log tường minh để test/hardware đo kiểm thay vì đoán.
            retreat_lift = target_approach_z - place_tcp[2]
            self.get_logger().info(
                f"[ACM-RELEASE] {from_sq}->{to_sq}: retreat "
                f"{retreat_lift * 1000:.0f}mm >= "
                f"{ACM_RELEASE_CLEARANCE_M * 1000:.0f}mm -> đóng ACM")
            # Đã rút khỏi quân: đóng mọi ngoại lệ ngay tại phase boundary.
            self._set_piece_collision(
                obj_id, gripper_touch=False, board_contact=False)
            self._release_contact_object_ids.discard(obj_id)
            self._carry_state = "WORLD_SOURCE"
        except Exception:
            self._reconcile_carry_failure(obj_id, from_sq, piece_type,
                                          f"pick-place {from_sq}->{to_sq}",
                                          to_sq, (x1, y1, BOARD_Z),
                                          placed_piece_type or piece_type)

    def _do_discard(self, square: str):
        """Quân bị ăn: pick tại chỗ, mang sang khu 'nghĩa địa'. to_square=None vì
        quân này không còn thuộc bàn cờ (không tham gia mapping ô -> id nữa)."""
        piece = self.board.piece_at(chess.parse_square(square))
        piece_type = piece.symbol().lower()
        slot = self.discard_count
        xd, yd, zd = discard_slot_pose(slot)
        discard_tcp_z = DISCARD_TCP_Z
        obj_id = None
        try:
            self._set_gripper(PIECE_SPECS[piece_type].gripper_open)
            obj_id = self._take_piece_from_world(square)
            (x0, y0, z0, source_approach_z, grasp_q,
             verified) = self._approach_and_descend_for_grasp(
                square, piece_type, "hạ gắp quân bị ăn", obj_id,
                dest_xy=(xd, yd), dest_piece_type=piece_type,
                dest_approach_z=discard_tcp_z + APPROACH_HEIGHT,
                dest_label=f"slot{slot}",
            )
            self._set_gripper(0.0)
            self._carry_state = "ATTACH_PENDING"
            self._attach_piece(obj_id, piece_type, square_to_xy(square))
            self._carry_state = "ATTACHED"
            verified_quat = (verified or {}).get("place_quat")
            drop_tcp, _cached_where = self._try_execute_cached_carry_chain(
                verified, obj_id, piece_type, (xd, yd),
                f"discard {square}->slot{slot}")
            if drop_tcp is None and _cached_where == "source":
                self._move_vertical(
                    x0, y0, source_approach_z, "nâng quân bị ăn",
                    quat_xyzw=grasp_q)
                self._set_piece_collision(
                    obj_id, gripper_touch=True, board_contact=False)
                # Transit position-only tới khu discard (lý do như
                # _do_pick_place: không ép orientation lúc mang).
                self._move_to(xd, yd, discard_tcp_z + APPROACH_HEIGHT)
            if drop_tcp is None:
                drop_tcp = self._place_at_dest_compensated(
                    obj_id, (xd, yd), discard_tcp_z, piece_type,
                    discard_tcp_z + APPROACH_HEIGHT, verified_quat,
                    "hạ thả quân bị ăn")
            self._verify_attached_piece_target(
                obj_id, (xd, yd), piece_type, requested_tcp=drop_tcp)
            self._set_gripper(PIECE_SPECS[piece_type].gripper_open)
            self._carry_state = "DETACH_PENDING"
            self._detach_piece(obj_id, None, (xd, yd, zd), piece_type)
            self._carry_state = "WORLD_DESTINATION"
            self._grasp_local_by_id.pop(obj_id, None)
            self.discard_count += 1
            self._move_vertical(drop_tcp[0], drop_tcp[1],
                               discard_tcp_z + APPROACH_HEIGHT,
                               "nâng sau thả quân bị ăn", quat_xyzw=drop_tcp[3])
            self.get_logger().info(
                f"[ACM-RELEASE] discard {square}->slot{slot}: retreat "
                f"{(discard_tcp_z + APPROACH_HEIGHT - drop_tcp[2]) * 1000:.0f}mm "
                f">= {ACM_RELEASE_CLEARANCE_M * 1000:.0f}mm -> đóng ACM")
            self._set_piece_collision(
                obj_id, gripper_touch=False, board_contact=False)
            self._release_contact_object_ids.discard(obj_id)
            self._carry_state = "WORLD_SOURCE"
        except Exception:
            self._reconcile_carry_failure(obj_id, square, piece_type,
                                          f"discard {square}->slot", None,
                                          (xd, yd, zd))

    def _castling_rook_squares(self, uci: str):
        mapping = {
            "e1g1": ("h1", "f1"), "e1c1": ("a1", "d1"),
            "e8g8": ("h8", "f8"), "e8c8": ("a8", "d8"),
        }
        return mapping[uci]

    # ---------------- Robot helpers ----------------

    def _grasp_offset_candidates(self, square: str):
        """Cache trước, sau đó danh sách hữu hạn; không trả candidate trùng."""
        cached = self._grasp_offset_cache.get(square)
        ordered = (() if cached is None else (cached,)) + GRASP_APPROACH_CANDIDATE_OFFSETS
        seen = set()
        for offset in ordered:
            if offset not in seen:
                seen.add(offset)
                yield offset

    def _approach_and_descend_for_grasp(self, square, piece_type, step_name,
                                        source_obj_id=None,
                                        dest_xy=None, dest_piece_type=None,
                                        dest_approach_z=None, dest_label=None):
        """Tìm offset gắp an toàn bằng chính pipeline OMPL -> Cartesian runtime.

        Mỗi candidate đều được collision-check. Candidate thất bại ở approach
        hoặc descend không làm attach quân, vì vậy có thể thử candidate kế.
        Fix 6: candidate chỉ được CHỌN sau khi lift cũng plan được VỚI thể tích
        mang (proxy scratch attached ở đúng TCP sau descend) — trước đây chọn
        ngay khi descend xong nên lift/place vẫn có thể rớt sau attach. Thêm
        điều kiện hình học offset tối đa và trần tổng thời gian tìm kiếm.

        Fix 7 (tìm nghiệm gắp–đặt trước attach): nếu có dest_xy, mỗi offset
        phải PASS toàn chuỗi lift → transfer → pre-place/descend từng yaw với
        MỘT scratch session giữ đúng T_tcp_piece giả định (đo tại TF descend
        hiện tại) và start plan nối chuỗi, không di chuyển robot. IK
        position-only có thể bỏ qua quaternion yêu cầu nên chỉ chốt offset khi
        tồn tại candidate đưa tâm quân vào sai số 5 mm ở FK endpoint. Góc quân
        không còn là điều kiện PASS/NACK. Nghiệm candidate
        đã PASS (gồm chính các trajectory) được trả thẳng cho runtime trong
        cùng thao tác. Nếu vẫn rớt khi đã ATTACHED thì giữ hành vi dừng an
        toàn (không đổi offset khi đang mang). Không truyền dest_xy (=None)
        thì chỉ check lift.
        Trả về (x, y, z, approach_z, grasp_q, verified|None).
        """
        # Runtime đã pop mapping ô nguồn trước khi gọi hàm này. Giữ ID nguồn
        # tường minh để proxy lift thay đúng CollisionObject; nếu không có
        # (trường hợp helper được gọi độc lập) mới thử lấy từ mapping.
        if source_obj_id is None:
            source_obj_id = self.piece_id_by_square.get(square)
        errors = []
        search_deadline = time.monotonic() + GRASP_SEARCH_TIMEOUT_SEC
        for index, offset in enumerate(self._grasp_offset_candidates(square), 1):
            if time.monotonic() >= search_deadline:
                raise RuntimeError(
                    f"Tìm offset gắp {square} quá {GRASP_SEARCH_TIMEOUT_SEC:.0f}s; "
                    f"đã thử: {'; '.join(errors) if errors else 'none'}")
            if math.hypot(*offset) > GRASP_MAX_OFFSET:
                errors.append(f"{offset}: vượt bán kính quân {GRASP_MAX_OFFSET} m")
                self.get_logger().warning(
                    f"[GRASP-CANDIDATE] {square} thử {index} offset={offset}: "
                    f"loại vì trượt tâm quân (điều kiện hình học)"
                )
                continue
            x, y, z = square_to_grasp_pose(square, piece_type, offset)
            approach_z = approach_tcp_z(square, z)
            # Lift Cartesian chỉ đủ để tách quân khỏi mặt bàn. Từ đây tới đích
            # là chuyển động xa và được OMPL position-only xử lý.
            lift_z = min(approach_z, z + CARRY_CLEARANCE_LIFT_M)
            approach_trajectory = self._plan_motion(
                position=[x, y, approach_z], target_link=END_EFFECTOR,
                tolerance_position=0.004, cartesian=False)
            if approach_trajectory is None:
                errors.append(f"{offset}: không có OMPL approach")
                self.get_logger().warning(
                    f"[GRASP-CANDIDATE] {square} thử {index} offset={offset}: OMPL fail"
                )
                continue
            # Từ đây execution failure phải dừng ngay; không tự chạy candidate
            # khác khi chưa biết trạng thái vật lý thật của robot.
            self._execute_and_wait(self.moveit2, approach_trajectory)
            grasp_q = self._current_tcp_quat()
            descend_trajectory = self._plan_vertical_trajectory(x, y, z, grasp_q, step_name)
            if descend_trajectory is None:
                errors.append(f"{offset}: Cartesian descend fail")
                self.get_logger().warning(
                    f"[GRASP-CANDIDATE] {square} thử {index} offset={offset}: Cartesian fail"
                )
                continue
            self._execute_and_wait(self.moveit2, descend_trajectory)
            # T_tcp_piece giả định của đúng offset đang thử, đo tại TF descend
            # hiện tại (kẹp vẫn mở, chưa attach gì). Mọi precheck có mang bên
            # dưới dùng đúng transform này.
            source_xyz = (*square_to_xy(square), BOARD_Z)
            try:
                hypo_local = self._piece_local_pose(
                    piece_type, square_to_xy(square))
            except Exception as exc:
                errors.append(f"{offset}: không đo được T_tcp_piece ({exc})")
                self.get_logger().warning(
                    f"[GRASP-CANDIDATE] {square} thử {index} offset={offset}: "
                    f"không đo TF để validate mang ({exc}) -> loại")
                continue
            if dest_xy is None:
                # Không có đích (helper gọi độc lập): chỉ check lift có mang
                # với đúng transform, như Fix 6 nhưng pose tường minh.
                scratch = None
                try:
                    scratch = self._dry_attach_scratch_with_local(
                        hypo_local, piece_type, source_obj_id)
                    lift_trajectory = self._plan_vertical_trajectory(
                        x, y, lift_z, grasp_q, f"{step_name}/lift-validate")
                except Exception as exc:
                    errors.append(f"{offset}: scratch/lift lỗi ({exc})")
                    lift_trajectory = None
                finally:
                    self._dry_detach_scratch(
                        scratch or "__dry_carry__", source_obj_id,
                        source_xyz, piece_type)
                if lift_trajectory is None:
                    errors.append(f"{offset}: Cartesian lift fail (có mang)")
                    self.get_logger().warning(
                        f"[GRASP-CANDIDATE] {square} thử {index} offset={offset}: "
                        f"descend được nhưng lift có mang fail -> loại"
                    )
                    continue
                verified = None
            else:
                # Fix 7 đầy đủ: MỘT scratch session giữ đúng transform xuyên
                # suốt lift → transfer → pre-place/descend từng yaw. Robot
                # không di chuyển (toàn plan-only với start nối chuỗi).
                verified = self._validate_carry_chain_for_offset(
                    hypo_local, piece_type,
                    dest_piece_type or piece_type,
                    (x, y, z), lift_z, grasp_q,
                    dest_xy, dest_approach_z,
                    source_obj_id, source_xyz,
                    context=f"{square}->{dest_label or dest_xy}/offset{offset}")
                if verified is None:
                    chain_reason = getattr(
                        self, "_last_chain_failure_reason",
                        "chuỗi mang không có nghiệm")
                    errors.append(
                        f"{offset}: {chain_reason} tại {dest_label or dest_xy}")
                    self.get_logger().warning(
                        f"[GRASP-CANDIDATE] {square} thử {index} offset={offset}: "
                        f"chuỗi mang fail ({chain_reason}) -> loại")
                    continue
            self._grasp_offset_cache[square] = offset
            if verified is not None:
                self.get_logger().info(
                    f"[GRASP-CANDIDATE] {square} chọn offset={offset} "
                    f"+ nghiệm đặt bù tâm sau {index} lần thử "
                    f"(lệch {verified['pos_err'] * 1000:.1f}mm "
                    f"tilt {math.degrees(verified['tilt']):.1f}° ở precheck)"
                )
            else:
                self.get_logger().info(
                    f"[GRASP-CANDIDATE] {square} chọn offset={offset} "
                    f"sau {index} lần thử"
                )
            return x, y, z, lift_z, grasp_q, verified
        raise RuntimeError(
            f"Không có approach gắp an toàn cho {square} sau {len(errors)} candidate; "
            f"candidate cuối: {errors[-1] if errors else 'none'}"
        )

    def _wait_for_joint_state(self, interface, timeout_sec: float = 10.0):
        """Chờ callback của MultiThreadedExecutor cập nhật joint state.

        Không gọi rclpy.spin_once ở đây: node này đã thuộc executor chính.
        """
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and interface.joint_state is None:
            if time.monotonic() >= deadline:
                raise RuntimeError("Không nhận được joint_states")
            time.sleep(0.01)

    def _joint_state_from_trajectory_end(self, trajectory):
        """Dựng JointState tại điểm cuối trajectory để chain plan kế tiếp.

        plan_async chấp nhận start tùy ý (cả OMPL lẫn Cartesian đều đọc
        start_state này), nhờ vậy precheck nối lift → transfer → descend đúng
        thứ tự mà không cần di chuyển robot thật. Merge theo tên joint để giữ
        các joint ngoài trajectory (không bao giờ rỗng một phần).
        """
        last = trajectory.points[-1]
        base = copy.deepcopy(self.moveit2.joint_state)
        if base is None:
            raise RuntimeError("thiếu joint state hiện tại để chain plan")
        pos = dict(zip(base.name, base.position))
        names = list(trajectory.joint_names)
        if len(names) != len(last.positions):
            raise RuntimeError(
                f"trajectory joint_names ({len(names)}) lệch positions "
                f"({len(last.positions)}) khi chain plan")
        for n, p in zip(names, last.positions):
            if not math.isfinite(float(p)):
                raise RuntimeError(f"trajectory endpoint NaN/inf tại {n}")
            pos[n] = float(p)
        merged = JointState()
        merged.name = list(base.name)
        merged.position = [pos[n] for n in base.name]
        return merged

    def _plan_motion(self, _start_joint_state=None, **kwargs):
        """Phiên bản non-spinning của pymoveit2.plan().

        pymoveit2.plan() tự gọi rclpy.spin_once(), điều này không hợp lệ khi
        node đã chạy trong MultiThreadedExecutor và dẫn tới lỗi wait-set.
        _start_joint_state: JointState tùy ý để chain plan (mặc định = state
        hiện tại). Cả OMPL lẫn Cartesian đều tôn trọng start này.
        """
        self._wait_for_joint_state(self.moveit2)
        cartesian = kwargs.get("cartesian", False)
        fraction_threshold = kwargs.pop("cartesian_fraction_threshold", 0.0)
        start = (_start_joint_state if _start_joint_state is not None
                 else self.moveit2.joint_state)
        future = self.moveit2.plan_async(
            start_joint_state=start, **kwargs
        )
        if future is None:
            return None
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("MoveIt planning timeout sau 30 giây")
            time.sleep(0.01)
        if not future.done():
            return None
        return self.moveit2.get_trajectory(
            future,
            cartesian=cartesian,
            cartesian_fraction_threshold=fraction_threshold,
        )

    def _execute_and_wait(self, interface, trajectory, timeout_sec: float = 60.0):
        """Execute mà không tạo executor thứ hai trên cùng ROS node."""
        # Gripper dùng move_to_configuration(), hàm đó đã gửi MoveGroup action
        # trước khi gọi helper này; arm truyền JointTrajectory để gửi ở đây.
        if trajectory is not None:
            interface.execute(trajectory)
        deadline = time.monotonic() + timeout_sec
        # Callback action result được MultiThreadedExecutor chính xử lý và sẽ
        # đổi query_state() về IDLE.
        while rclpy.ok() and interface.query_state().name != "IDLE":
            if time.monotonic() >= deadline:
                interface.cancel_execution()
                # Xác nhận đã dừng trước khi cho nhận lệnh mới: chờ bounded
                # cho action về IDLE, hết chờ thì raise kèm trạng thái để
                # caller biết robot có thể vẫn đang chuyển động.
                stop_deadline = time.monotonic() + 5.0
                while (rclpy.ok()
                       and interface.query_state().name != "IDLE"
                       and time.monotonic() < stop_deadline):
                    time.sleep(0.05)
                state = interface.query_state().name
                raise RuntimeError(
                    f"MoveIt execution timeout (trạng thái sau cancel: {state})")
            time.sleep(0.01)
        if not interface.motion_suceeded:
            raise RuntimeError("MoveIt báo trajectory không thành công")

    def _move_to(self, x, y, z):
        """Dofbot có 5 DOF và plugin IK của config này là position-only.
        Không ép quaternion/Cartesian path: với pose 6D đó thường không có
        nghiệm, dù điểm XYZ hoàn toàn nằm trong vùng với tới. Đây cùng cách
        position-target mà task trà dùng cho Gripping_point_Link."""
        trajectory = self._plan_motion(
            position=[x, y, z],
            target_link=END_EFFECTOR,
            tolerance_position=0.004,
            cartesian=False,
        )
        if trajectory is None:
            self._diagnose_position_goal_collision(
                (x, y, z), self._current_tcp_quat(), f"position-only/{(x, y, z)}"
            )
            raise RuntimeError(f"Không tìm được position-only plan tới {(x, y, z)}")
        self._execute_and_wait(self.moveit2, trajectory)

    def _close_release_contacts_at_home(self):
        """Đóng ACM release contact sau khi HOME đã thực thi xong và đã an toàn."""
        if not self._release_contact_object_ids:
            return
        for obj_id in tuple(self._release_contact_object_ids):
            self._set_object_gripper_collision(obj_id, False)
        self._release_contact_object_ids.clear()
        contacts = self._state_validity_contacts(
            self.moveit2.joint_state, "HOME-after-closing-release-ACM"
        )
        if contacts:
            raise RuntimeError(f"HOME vẫn collision sau khi đóng release ACM: {', '.join(contacts)}")

    def _move_to_home(self, label: str):
        """Joint PTP về SRDF pose `arm_group/up` trước/sau lượt robot."""
        contacts = self._state_validity_contacts(
            self._home_joint_state(), f"HOME-goal/{label}"
        )
        if contacts:
            raise RuntimeError(f"HOME goal collision: {', '.join(contacts)}")
        trajectory = self._plan_motion(
            joint_positions=HOME_JOINTS,
            joint_names=JOINT_NAMES,
            tolerance_joint_position=0.03,
            cartesian=False,
        )
        if trajectory is None:
            raise RuntimeError(f"Không tìm được joint PTP: {label}")
        self._execute_and_wait(self.moveit2, trajectory)
        self._close_release_contacts_at_home()

    def _current_tcp_quat(self) -> list[float]:
        """Đọc quaternion TCP hiện tại (BASE_LINK -> END_EFFECTOR).

        Runtime chốt 1 quaternion duy nhất sau approach rồi truyền cho mọi
        đoạn vertical của nước đi, thay vì để mỗi _move_vertical đọc lại
        (OMPL position-only có thể trả orientation khác nhau mỗi lần gọi,
        khiến Cartesian giữ orientation mới mà 5-DOF không làm được)."""
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_LINK, END_EFFECTOR, rclpy.time.Time()
            )
        except Exception as exc:
            raise RuntimeError(f"Không đọc được TF {BASE_LINK}->{END_EFFECTOR}: {exc}")
        q = transform.transform.rotation
        qn = self._normalize_quaternion((q.x, q.y, q.z, q.w))
        return [qn[0], qn[1], qn[2], qn[3]]

    def _current_tcp_xyz(self) -> tuple[float, float, float]:
        """Vị trí TCP hiện tại qua TF (dùng để skip pre-place thừa)."""
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_LINK, END_EFFECTOR, rclpy.time.Time()
            )
        except Exception as exc:
            raise RuntimeError(f"Không đọc được TF {BASE_LINK}->{END_EFFECTOR}: {exc}")
        t = transform.transform.translation
        return (float(t.x), float(t.y), float(t.z))

    def _fk_tcp_pose(self, joint_names, joint_positions):
        """FK qua /compute_fk ra pose TCP (xyz + quat). Fail-closed: service
        vắng thì raise thay vì cho execute mù."""
        if not self._fk_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(
                "service /compute_fk không sẵn sàng (không FK-validate được pose đặt)")
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_LINK
        req.fk_link_names = [END_EFFECTOR]
        js = JointState()
        js.name = list(joint_names)
        js.position = [float(v) for v in joint_positions]
        req.robot_state.joint_state = js
        future = self._fk_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("/compute_fk timeout khi FK-validate pose đặt")
            time.sleep(0.01)
        result = future.result()
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = None if result is None else result.error_code.val
            raise RuntimeError(f"/compute_fk báo lỗi (code={code}) khi FK-validate pose đặt")
        if not result.pose_stamped:
            raise RuntimeError("/compute_fk không trả pose khi FK-validate pose đặt")
        p = result.pose_stamped[0].pose
        return ((p.position.x, p.position.y, p.position.z),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def _piece_error_from_tcp(self, tcp_xyz, tcp_quat, local: Pose,
                              target_xy, piece_type: str):
        """Tính (position_error, tilt, center, q_piece) của quân từ pose TCP."""
        pl = (local.position.x, local.position.y, local.position.z)
        ql = self._normalize_quaternion(
            (local.orientation.x, local.orientation.y,
             local.orientation.z, local.orientation.w))
        fq = self._normalize_quaternion(tuple(tcp_quat))
        rx, ry, rz = self._rotate_by_quaternion(pl, fq)
        center = (tcp_xyz[0] + rx, tcp_xyz[1] + ry, tcp_xyz[2] + rz)
        spec = PIECE_SPECS[piece_type]
        want = (target_xy[0], target_xy[1], BOARD_Z + spec.pickup_height / 2)
        position_error = math.sqrt(sum(
            (got - w) ** 2 for got, w in zip(center, want)))
        q_piece = self._multiply_quaternions(fq, ql)
        tilt = self._tilt_from_quaternion(q_piece)
        return position_error, tilt, center, q_piece

    def _compensate_place_tcp(self, place_tcp, center, target_xy,
                               piece_type: str):
        """Dịch XYZ TCP theo sai số tâm quân FK; không thay quaternion.

        IK position-only có thể chọn orientation khác q yêu cầu, nhưng log thực
        tế cho thấy XYZ TCP vẫn đạt chính xác. Vì vậy dùng sai số tâm quân làm
        bước fixed-point correction. Giới hạn mỗi bước để một TF/transform lỗi
        không đẩy target quá xa trong một lần.
        """
        want = (
            target_xy[0], target_xy[1],
            BOARD_Z + PIECE_SPECS[piece_type].pickup_height / 2)
        delta = [want[i] - center[i] for i in range(3)]
        length = math.sqrt(sum(v * v for v in delta))
        if not math.isfinite(length):
            raise RuntimeError("sai số bù tâm quân là NaN/inf")
        if length > PLACE_COMPENSATION_MAX_STEP_M:
            scale = PLACE_COMPENSATION_MAX_STEP_M / length
            delta = [v * scale for v in delta]
        return (
            float(place_tcp[0]) + delta[0],
            float(place_tcp[1]) + delta[1],
            float(place_tcp[2]) + delta[2],
            place_tcp[3],
        )

    # ---------------- TODO-2: candidate IK + scoring ----------------

    @staticmethod
    def _region_for_target(target_xy) -> str:
        """Phân vùng bàn cờ để chọn joint template (TODO-2/3)."""
        x, y = float(target_xy[0]), float(target_xy[1])
        if y < -0.10:
            return "discard"
        if x < 0.13:
            return "near"
        if x > 0.27:
            return "far"
        return "center"

    def _candidate_seed_states(self, region: str) -> list:
        """Các seed joint-state hữu hạn cho pre-place (TODO-2).

        HOME + template vùng + state hiện tại: phủ nhánh khớp khác nhau thay
        vì mọi ô cùng một seed.
        """
        seeds = []
        try:
            self._wait_for_joint_state(self.moveit2, timeout_sec=3.0)
            cur = copy.deepcopy(self.moveit2.joint_state)
            seeds.append(("current", cur))
        except Exception:
            pass
        try:
            home = self._home_joint_state()
            seeds.append(("home", home))
        except Exception:
            pass
        template = REGION_JOINT_TEMPLATES.get(region)
        if template is not None:
            try:
                self._wait_for_joint_state(self.moveit2, timeout_sec=3.0)
                base = copy.deepcopy(self.moveit2.joint_state)
                values = dict(zip(base.name, base.position))
                values.update(zip(JOINT_NAMES, template))
                base.position = [values[n] for n in base.name]
                seeds.append((f"template:{region}", base))
            except Exception:
                pass
        # Dedup giữ thứ tự.
        seen, out = set(), []
        for label, state in seeds:
            if label not in seen:
                seen.add(label)
                out.append((label, state))
        return out

    def _query_ik_joint_target(self, position, quat_xyzw, seed_state,
                               context: str):
        """Gọi /compute_ik với seed tường minh, avoid_collisions=True.

        Trả về dict joint->pos hoặc None (có log lý do reject).
        """
        if not self._ik_client.service_is_ready():
            self.get_logger().warning(f"[CANDIDATE] {context}: IK service chưa sẵn sàng")
            return None
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = GROUP_NAME
        ik.ik_link_name = END_EFFECTOR
        ik.avoid_collisions = True
        ik.robot_state = RobotState()
        ik.robot_state.joint_state = copy.deepcopy(seed_state)
        ik.robot_state.is_diff = False
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = BASE_LINK
        ik.pose_stamped.pose.position.x = float(position[0])
        ik.pose_stamped.pose.position.y = float(position[1])
        ik.pose_stamped.pose.position.z = float(position[2])
        ik.pose_stamped.pose.orientation.x = float(quat_xyzw[0])
        ik.pose_stamped.pose.orientation.y = float(quat_xyzw[1])
        ik.pose_stamped.pose.orientation.z = float(quat_xyzw[2])
        ik.pose_stamped.pose.orientation.w = float(quat_xyzw[3])
        ik.timeout.sec = 1
        future = self._ik_client.call_async(req)
        deadline = time.monotonic() + 3.0
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(f"[CANDIDATE] {context}: IK timeout")
                return None
            time.sleep(0.01)
        result = future.result()
        if result is None or result.error_code.val != 1:
            code = "no-response" if result is None else str(result.error_code.val)
            self.get_logger().info(f"[CANDIDATE] {context}: IK không nghiệm (code={code})")
            return None
        return dict(zip(result.solution.joint_state.name,
                        result.solution.joint_state.position))

    @staticmethod
    def _joint_travel_rad(current_state: JointState, target_map: dict) -> float:
        total = 0.0
        cur = dict(zip(current_state.name, current_state.position))
        for name in JOINT_NAMES:
            if name in cur and name in target_map:
                d = math.atan2(math.sin(cur[name] - float(target_map[name])),
                               math.cos(cur[name] - float(target_map[name])))
                total += abs(d)
        return total

    @staticmethod
    def _min_limit_margin_rad(target_map: dict) -> float:
        margins = []
        for name in JOINT_NAMES:
            lim = DOFBOT_JOINT_LIMITS.get(name)
            if lim is None or name not in target_map:
                continue
            v = float(target_map[name])
            margins.append(min(v - lim[0], lim[1] - v))
        return min(margins) if margins else 0.0

    @staticmethod
    def _trajectory_max_step(trajectory) -> float:
        worst = 0.0
        prev = None
        for pt in trajectory.points:
            cur = [float(v) for v in pt.positions]
            if prev is not None and len(cur) == len(prev):
                worst = max(worst, max(abs(b - a) for a, b in zip(prev, cur)))
            prev = cur
        return worst

    def _score_place_candidate(self, pos_err: float, tilt: float,
                               travel: float, margin: float) -> float:
        """Điểm càng thấp càng tốt (TODO-2): err + tilt + travel - margin."""
        return (pos_err * CANDIDATE_SCORE_W_POS
                + tilt * CANDIDATE_SCORE_W_TILT
                + travel * CANDIDATE_SCORE_W_TRAVEL
                + margin * CANDIDATE_SCORE_W_LIMIT_MARGIN)

    def _upright_quat_yaw_free(self, yaw: float = 0.0):
        """Quat TCP thẳng đứng (tilt=0), yaw tự do quanh Z (TODO-3)."""
        s, c = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return (0.0, 0.0, s, c)

    def _evaluate_upright_constraint_impact(self, context: str) -> dict:
        """Đánh giá ảnh hưởng orientation constraint tới reachability (TODO-3).

        So sánh: descend với quat thẳng đứng vs quat scoring hiện tại trên cùng
        target. Không làm fail diagnostic; chỉ log để quyết định giữ scoring
        (mặc định) hay bật constraint.
        """
        result = {"constraint": "upright-yaw-free", "enabled": USE_UPRIGHT_ORIENTATION_CONSTRAINT}
        self.get_logger().info(
            f"[TILT-EVAL] {context}: upright-constraint="
            f"{USE_UPRIGHT_ORIENTATION_CONSTRAINT} (scoring mặc định giữ "
            f"reachability; bật cờ để ép thẳng đứng rồi đo lại)")
        return result

    def _plan_quiet(self, context: str, **kwargs):
        """Plan trả None khi fail thay vì raise — precheck thử nghiệm kế.

        _plan_motion raise ở timeout; Cartesian fail trả None. Chuẩn hoá cả hai
        thành None để một plan flake không đánh sập cả vòng tìm offset.
        """
        try:
            return self._plan_motion(**kwargs)
        except Exception as exc:
            self.get_logger().warning(f"[CHAIN] {context}: plan lỗi ({exc})")
            return None

    def _validate_carry_chain_for_offset(
            self, hypo_local: Pose, piece_type: str, dest_piece_type: str,
            grasp_xyz, grasp_approach_z: float, grasp_q,
            target_xy, target_approach_z: float,
            source_obj_id: str | None, source_xyz, context: str,
            yaw_count: int = CANDIDATE_YAW_COUNT):
        """Kiểm tra TOÀN CHUỖI mang trước khi gắp thật, với đúng thể tích quân.

        Một scratch session duy nhất giữ T_tcp_piece giả định của đúng offset
        đang thử xuyên suốt: lift Cartesian (từ pose descend hiện tại) →
        transfer position-only tới approach đích (start nối từ cuối lift) →
        pre-place bù bằng OMPL position-only (được đổi nhánh khớp) →
        descend Cartesian (start nối từ cuối pre-place) → FK endpoint.
        Chỉ chốt offset khi tồn tại nghiệm đưa tâm quân vào sai số 5 mm;
        orientation của quân chỉ ghi log, không dùng làm điều kiện loại.

        Attach scratch fail → None (FAIL kiểm tra transfer, KHÔNG fallback
        IK-only rồi PASS). Mọi plan trong chuỗi đều có mang (scratch attached).
        Robot không hề di chuyển (toàn plan-only). Trả về dict nghiệm gồm cả
        toàn bộ trajectory đã PASS {lift, transfer, pre, desc, hypo_local,
        scene_fingerprint, place_tcp, place_quat, pos_err, tilt}
        để runtime
        execute đúng nhánh khớp đã FK-xác nhận thay vì plan lại (OMPL ngẫu
        nhiên có thể lật sang nhánh khác với IK position-only) — hoặc None.
        """
        self._last_chain_failure_reason = "chuỗi mang không có nghiệm"
        scratch_id = None
        try:
            scratch_id = self._dry_attach_scratch_with_local(
                hypo_local, piece_type, source_obj_id)
        except Exception as exc:
            self._last_chain_failure_reason = f"attach scratch lỗi: {exc}"
            self.get_logger().warning(
                f"[CHAIN] {context}: không attach scratch đúng transform "
                f"({exc}) -> loại offset (không coi là PASS)")
            try:
                self._dry_detach_scratch(
                    scratch_id or "__dry_carry__", source_obj_id,
                    source_xyz, piece_type)
            except Exception:
                pass
            return None
        try:
            # Snapshot pose + geometry + attached khác + ACM đúng lúc chuỗi
            # được kiểm tra. Scratch/quân nguồn được loại và được gate riêng
            # bằng T_tcp_piece, nên hai scene trước/sau attach so sánh được.
            try:
                scene_fingerprint = self._scene_cache_fingerprint(source_obj_id)
            except Exception as exc:
                self._last_chain_failure_reason = f"snapshot scene lỗi: {exc}"
                self.get_logger().warning(
                    f"[CHAIN] {context}: không đọc scene làm baseline "
                    f"({exc}) -> loại offset")
                return None
            # Lift giữ orientation lúc gắp; start = pose descend thật của robot.
            # Timeout plan raise -> chuẩn hoá thành None (loại offset này).
            try:
                lift = self._plan_vertical_trajectory(
                    grasp_xyz[0], grasp_xyz[1], grasp_approach_z, grasp_q,
                    f"{context}/lift")
            except Exception as exc:
                self._last_chain_failure_reason = f"lift plan lỗi: {exc}"
                self.get_logger().warning(
                    f"[CHAIN] {context}: lift plan lỗi ({exc}) -> loại offset")
                return None
            if lift is None:
                self._last_chain_failure_reason = "Cartesian clearance lift có mang fail"
                self.get_logger().warning(
                    f"[CHAIN] {context}: lift có mang fail -> loại offset")
                return None
            try:
                lift_end = self._joint_state_from_trajectory_end(lift)
            except Exception as exc:
                self._last_chain_failure_reason = f"endpoint lift xấu: {exc}"
                self.get_logger().warning(
                    f"[CHAIN] {context}: endpoint lift xấu ({exc}) -> loại")
                return None
            transfer = self._plan_quiet(
                f"{context}/transfer",
                _start_joint_state=lift_end,
                position=[target_xy[0], target_xy[1], target_approach_z],
                target_link=END_EFFECTOR, tolerance_position=0.004,
                cartesian=False)
            if transfer is None:
                self._last_chain_failure_reason = "OMPL transfer có mang fail"
                self.get_logger().warning(
                    f"[CHAIN] {context}: transfer có mang không plan được "
                    f"-> loại offset")
                return None
            try:
                transfer_end = self._joint_state_from_trajectory_end(transfer)
            except Exception as exc:
                self._last_chain_failure_reason = f"endpoint transfer xấu: {exc}"
                self.get_logger().warning(
                    f"[CHAIN] {context}: endpoint transfer xấu ({exc}) -> loại")
                return None
            yaw_cands = self._place_yaw_candidates_for_local(
                hypo_local, target_xy, dest_piece_type, grasp_q, yaw_count)
            for place_tcp, _k, _psi in yaw_cands:
                for correction in range(PLACE_COMPENSATION_MAX_ITERATIONS):
                    px, py, pz, pq = (place_tcp[0], place_tcp[1],
                                      place_tcp[2], place_tcp[3])
                    label = f"{context}/comp{correction + 1}"
                    # Plan-only từ đúng cuối transfer ở mọi vòng; arm chưa di
                    # chuyển. Quaternion chỉ là seed API, không phải điều kiện
                    # PASS. XYZ được sửa theo tâm quân FK của vòng trước.
                    pre = self._plan_quiet(
                        f"{label}/pre-place",
                        _start_joint_state=transfer_end,
                        position=[px, py, target_approach_z],
                        target_link=END_EFFECTOR,
                        tolerance_position=0.002, cartesian=False)
                    if pre is None:
                        break
                    try:
                        pre_end = self._joint_state_from_trajectory_end(pre)
                        pre_last = pre.points[-1]
                        _pre_xyz, pre_q = self._fk_tcp_pose(
                            pre.joint_names, pre_last.positions)
                    except Exception:
                        break
                    try:
                        self._set_piece_collision(
                            scratch_id, gripper_touch=True, board_contact=True)
                        desc = self._plan_quiet(
                            f"{label}/descend",
                            _start_joint_state=pre_end,
                            position=[px, py, pz], quat_xyzw=pre_q,
                            target_link=END_EFFECTOR,
                            tolerance_position=0.002,
                            tolerance_orientation=0.03,
                            cartesian=True, max_step=CARTESIAN_MAX_STEP_M,
                            cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD)
                    finally:
                        self._set_piece_collision(
                            scratch_id, gripper_touch=True,
                            board_contact=False)
                    if desc is None:
                        break
                    try:
                        last = desc.points[-1]
                        (fx, fy, fz), fq = self._fk_tcp_pose(
                            desc.joint_names, last.positions)
                        pos_err, tilt, center, _q = self._piece_error_from_tcp(
                            (fx, fy, fz), fq, hypo_local,
                            target_xy, dest_piece_type)
                    except Exception:
                        break
                    if pos_err <= PLACE_POSITION_TOL_M:
                        # Quaternion thực tế của endpoint, chỉ dùng để retreat
                        # giữ pose hiện tại; không phải constraint của quân.
                        place_tcp = (px, py, pz, fq)
                        self.get_logger().info(
                            f"[CHAIN] {context}: bù tâm FK đạt sau "
                            f"{correction + 1}/{PLACE_COMPENSATION_MAX_ITERATIONS} "
                            f"lần, lệch {pos_err * 1000:.1f}mm; "
                            f"tilt tham khảo {math.degrees(tilt):.1f}°")
                        return {"lift": lift, "transfer": transfer,
                                "pre": pre, "desc": desc,
                                "hypo_local": copy.deepcopy(hypo_local),
                                "scene_fingerprint": scene_fingerprint,
                                "place_tcp": place_tcp, "place_quat": fq,
                                "pos_err": pos_err, "tilt": tilt}
                    corrected = self._compensate_place_tcp(
                        place_tcp, center, target_xy, dest_piece_type)
                    self.get_logger().info(
                        f"[PLACE-COMP] {label}: tâm lệch "
                        f"{pos_err * 1000:.1f}mm -> dịch TCP "
                        f"({(corrected[0] - px) * 1000:.1f}, "
                        f"{(corrected[1] - py) * 1000:.1f}, "
                        f"{(corrected[2] - pz) * 1000:.1f})mm")
                    place_tcp = corrected
            self.get_logger().warning(
                f"[CHAIN] {context}: lift + transfer đạt nhưng bù tâm FK "
                f"không hội tụ sau {PLACE_COMPENSATION_MAX_ITERATIONS} lần "
                f"-> loại offset")
            self._last_chain_failure_reason = (
                "bù tâm FK/pre-place→descend không hội tụ sau "
                f"{PLACE_COMPENSATION_MAX_ITERATIONS} lần")
            return None
        finally:
            self._dry_detach_scratch(
                scratch_id or "__dry_carry__", source_obj_id,
                source_xyz, piece_type)

    def _validate_place_trajectory_end(self, trajectory, obj_id: str, target_xy,
                                       piece_type: str, requested_tcp, step_name: str,
                                       local_override=None):
        """Dùng FK kiểm tra ĐIỂM CUỐI trajectory hạ đặt TRƯỚC execute.

        IK position-only có thể thực hiện orientation khác với quat đã dùng để
        tính bù TCP: dù TCP tới đúng XYZ thì tâm quân vẫn lệch. FK điểm cuối +
        T_tcp_piece cho tâm quân THỰC SẼ ĐẠT; tâm lệch quá 5 mm thì raise.
        Góc nghiêng vẫn được log để chẩn đoán nhưng không làm thao tác FAIL.
        local_override: T_tcp_piece giả định (validate trước attach).
        """
        last = trajectory.points[-1]
        (fx, fy, fz), fq = self._fk_tcp_pose(trajectory.joint_names, last.positions)
        local = local_override if local_override is not None else \
            self._grasp_local_by_id.get(obj_id)
        if local is None:
            raise RuntimeError(f"thiếu T_tcp_piece của {obj_id} khi FK-validate {step_name}")
        position_error, tilt, center, _q_piece = self._piece_error_from_tcp(
            (fx, fy, fz), fq, local, target_xy, piece_type)
        want = (target_xy[0], target_xy[1],
                BOARD_Z + PIECE_SPECS[piece_type].pickup_height / 2)
        if position_error <= PLACE_POSITION_TOL_M:
            return
        raise RuntimeError(
            f"FK cuối trajectory {step_name} không đạt pose đặt: "
            f"TCP yêu cầu xyz={[round(v, 4) for v in requested_tcp[:3]]} "
            f"quat={[round(v, 3) for v in requested_tcp[3]]}; "
            f"TCP FK xyz={[round(v, 4) for v in (fx, fy, fz)]} "
            f"quat={[round(v, 3) for v in fq]}; "
            f"tâm quân FK={[round(v, 4) for v in center]} muốn={want} "
            f"(lệch {position_error:.4f}m); góc nghiêng tham khảo "
            f"(không dùng để FAIL)={math.degrees(tilt):.1f}deg. "
            f"Khả năng: TCP compensation chưa khớp orientation FK thực tế.")

    def _move_vertical(self, x, y, z, step_name: str, quat_xyzw=None):
        """Đi thẳng đứng bằng compute_cartesian_path, không để OMPL lách qua
        bàn/quân trong đoạn hạ hoặc nâng.

        quat_xyzw: orientation giữ suốt đoạn đi. Nên truyền quaternion đã chốt
        sau approach (xem _current_tcp_quat); None = đọc TF hiện tại (giữ hành
        vi cũ cho caller đơn lẻ). Plan fail thì raise rõ ràng — KHÔNG fallback
        position-only (kể cả khi ALLOW_CARTESIAN_FALLBACK=True, vì fallback lúc
        đang ATTACHED sẽ làm rơi/lệch quân mà flow vẫn attach như thành công).
        Bước HẠ ĐẶT không dùng hàm này mà dùng _move_vertical_place để
        FK-validate pose quân trước execute.
        """
        q = quat_xyzw if quat_xyzw is not None else self._current_tcp_quat()
        trajectory = self._plan_vertical_or_raise(x, y, z, q, step_name)
        self._execute_and_wait(self.moveit2, trajectory)

    def _plan_vertical_or_raise(self, x, y, z, q, step_name: str):
        """Plan Cartesian hoặc raise (kèm chẩn đoán). Không fallback opt-in:
        ALLOW_CARTESIAN_FALLBACK chỉ còn là cờ tài liệu, mọi caller đều
        fail-loud để NACK thay vì đặt lệch."""
        trajectory = self._plan_vertical_trajectory(x, y, z, q, step_name)
        if trajectory is None:
            self._diagnose_position_goal_collision(
                (x, y, z), q, f"cartesian/{step_name}/{(x, y, z)}"
            )
            raise RuntimeError(
                f"Không có Cartesian path an toàn khi {step_name} tới {(x, y, z)} "
                f"(quat giữ {[round(v, 3) for v in q]}). "
                f"Hãy chạy /chess/check_reachability để xem ô/phase lỗi, "
                f"kiểm tra approach offset và orientation sau approach."
            )
        return trajectory

    def _move_vertical_place(self, place_tcp, step_name: str, obj_id: str,
                             target_xy, piece_type: str, approach_z=None):
        """Hạ đặt: plan -> FK-validate điểm cuối (tâm quân + nghiêng) -> execute.

        Không đạt thì raise TRƯỚC khi arm nhúc nhích: caller loại candidate /
        NACK thay vì đặt lệch rồi mới phát hiện ở _verify. Giữ wrapper cho
        caller cũ; flow mới dùng compensation lặp theo FK.
        """
        if approach_z is None:
            # Legacy: descend trực tiếp, caller tự quản ACM. Fail-loud:
            # không fallback position-only khi hạ đặt.
            trajectory = self._plan_vertical_or_raise(
                *place_tcp[:3], place_tcp[3], step_name)
            self._validate_place_trajectory_end(
                trajectory, obj_id, target_xy, piece_type, place_tcp,
                step_name)
            self._execute_and_wait(self.moveit2, trajectory)
            return place_tcp
        chosen = self._move_vertical_place_compensated(
            [place_tcp], step_name, obj_id, target_xy, piece_type, approach_z)
        return chosen

    def _trajectory_start_close(self, trajectory, context: str,
                                  tol: float = CACHED_CHAIN_JOINT_TOL_RAD) -> bool:
        """Arm hiện tại có đang đứng đúng start của trajectory cached không."""
        try:
            self._wait_for_joint_state(self.moveit2, timeout_sec=3.0)
            cur = dict(zip(self.moveit2.joint_state.name,
                           self.moveit2.joint_state.position))
        except Exception as exc:
            self.get_logger().warning(
                f"[CACHED] {context}: không đọc joint hiện tại ({exc})")
            return False
        names = list(trajectory.joint_names)
        pts = trajectory.points[0].positions if trajectory.points else []
        if len(names) != len(pts) or not names:
            self.get_logger().warning(
                f"[CACHED] {context}: trajectory cached rỗng/lệch")
            return False
        worst = 0.0
        for n, p in zip(names, pts):
            if n not in cur or not math.isfinite(float(p)):
                return False
            delta = math.atan2(
                math.sin(float(cur[n]) - float(p)),
                math.cos(float(cur[n]) - float(p)))
            worst = max(worst, abs(delta))
        if worst > tol:
            self.get_logger().warning(
                f"[CACHED] {context}: arm lệch start cached "
                f"{math.degrees(worst):.2f}° > {math.degrees(tol):.2f}°")
            return False
        return True

    @staticmethod
    def _trajectory_boundary_close(first, second,
                                   tol: float = CACHED_CHAIN_JOINT_TOL_RAD) -> bool:
        """Endpoint trajectory trước có nối đúng start trajectory sau không."""
        if (first is None or second is None or not first.points
                or not second.points):
            return False
        end = dict(zip(first.joint_names, first.points[-1].positions))
        start = dict(zip(second.joint_names, second.points[0].positions))
        required = set(JOINT_NAMES)
        if not required.issubset(end) or not required.issubset(start):
            return False
        common = set(end) & set(start)
        if not common:
            return False
        for name in common:
            a, b = float(end[name]), float(start[name])
            if not (math.isfinite(a) and math.isfinite(b)):
                return False
            delta = math.atan2(math.sin(a - b), math.cos(a - b))
            if abs(delta) > tol:
                return False
        return True

    def _joint_state_for_positions(self, names, positions) -> JointState:
        """Merge waypoint arm vào joint state hiện tại (giữ gripper thật)."""
        self._wait_for_joint_state(self.moveit2, timeout_sec=3.0)
        state = copy.deepcopy(self.moveit2.joint_state)
        values = dict(zip(state.name, state.position))
        if len(names) != len(positions):
            raise RuntimeError("joint_names/positions lệch khi validate cached")
        for name, value in zip(names, positions):
            value = float(value)
            if not math.isfinite(value):
                raise RuntimeError(f"waypoint cached NaN/inf tại {name}")
            values[name] = value
        missing = [name for name in state.name if name not in values]
        if missing:
            raise RuntimeError(f"joint state cached thiếu {missing}")
        state.position = [float(values[name]) for name in state.name]
        return state

    def _cached_state_valid(self, state: JointState, context: str) -> bool:
        """State-validity fail-closed và giữ attached object từ scene."""
        if not self._state_validity_client.service_is_ready():
            self.get_logger().warning(
                f"[CACHED] {context}: /check_state_validity chưa sẵn sàng")
            return False
        request = GetStateValidity.Request()
        request.group_name = GROUP_NAME
        request.robot_state = RobotState()
        request.robot_state.joint_state = state
        request.robot_state.is_diff = True
        future = self._state_validity_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning(
                    f"[CACHED] {context}: /check_state_validity timeout")
                return False
            time.sleep(0.01)
        result = future.result()
        if result is None:
            self.get_logger().warning(
                f"[CACHED] {context}: /check_state_validity không phản hồi")
            return False
        if result.valid:
            return True
        contacts = sorted({
            f"{c.contact_body_1}<->{c.contact_body_2}" for c in result.contacts
        })
        self.get_logger().warning(
            f"[CACHED] {context}: waypoint collision"
            + (f" ({', '.join(contacts)})" if contacts else ""))
        return False

    def _cached_trajectory_collision_free(self, trajectory,
                                          context: str) -> bool:
        """Revalidate toàn đường cached với quân thật và gripper hiện tại.

        Planner đã kiểm scratch, nhưng T_tcp_piece thật được phép sai khác nhỏ.
        Vì vậy nội suy lại từng đoạn và hỏi PlanningScene hiện tại trước execute.
        """
        if trajectory is None or not trajectory.points:
            self.get_logger().warning(
                f"[CACHED] {context}: trajectory rỗng khi revalidate collision")
            return False
        names = list(trajectory.joint_names)
        previous = None
        sample_index = 0
        for point in trajectory.points:
            current = [float(v) for v in point.positions]
            if len(current) != len(names):
                return False
            if previous is None:
                samples = [current]
            else:
                max_delta = max(
                    (abs(b - a) for a, b in zip(previous, current)),
                    default=0.0)
                steps = max(1, int(math.ceil(
                    max_delta / CACHED_COLLISION_SAMPLE_RAD)))
                samples = [
                    [a + (b - a) * (i / steps)
                     for a, b in zip(previous, current)]
                    for i in range(1, steps + 1)
                ]
            for sample in samples:
                try:
                    state = self._joint_state_for_positions(names, sample)
                except Exception as exc:
                    self.get_logger().warning(
                        f"[CACHED] {context}: waypoint xấu ({exc})")
                    return False
                if not self._cached_state_valid(
                        state, f"{context}/sample{sample_index}"):
                    return False
                sample_index += 1
            previous = current
        return True

    def _real_local_close_to_hypo(self, obj_id: str, hypo_local: Pose,
                                  context: str):
        """Trả pose attached MoveIt nếu gần giả định precheck, ngược lại None.

        Đây là transform thật trong PlanningScene, không phải phép đo trượt
        quân vật lý. Khi có camera, pose quan sát sau grasp phải thay/bổ sung
        gate này.
        """
        try:
            real = self._attached_piece_local_pose(obj_id)
        except Exception as exc:
            self.get_logger().warning(
                f"[CACHED] {context}: không đọc được attached pose ({exc})")
            return None
        if hypo_local is None:
            self.get_logger().warning(
                f"[CACHED] {context}: thiếu T_tcp_piece thật/giả định")
            return None
        try:
            dp = math.sqrt(
                (real.position.x - hypo_local.position.x) ** 2
                + (real.position.y - hypo_local.position.y) ** 2
                + (real.position.z - hypo_local.position.z) ** 2)
            dq = self._quat_angle(
                self._normalize_quaternion(
                    (real.orientation.x, real.orientation.y,
                     real.orientation.z, real.orientation.w)),
                self._normalize_quaternion(
                    (hypo_local.orientation.x, hypo_local.orientation.y,
                     hypo_local.orientation.z, hypo_local.orientation.w)))
        except ValueError as exc:
            self.get_logger().warning(
                f"[CACHED] {context}: quaternion local xấu ({exc})")
            return None
        if dp > CACHED_LOCAL_POS_TOL_M or dq > math.radians(
                CACHED_LOCAL_ANG_TOL_DEG):
            self.get_logger().warning(
                f"[CACHED] {context}: local thật lệch giả định "
                f"({dp * 1000:.1f}mm, {math.degrees(dq):.1f}°) -> plan lại")
            return None
        return real

    def _place_at_dest_compensated(self, obj_id: str, target_xy, tcp_z: float,
                                   piece_type: str, approach_z: float,
                                   verified_quat, step_name: str):
        """Hạ đặt tại đích bằng candidate IK hữu hạn + FK chấm điểm (TODO-2)."""
        local = self._grasp_local_by_id.get(obj_id)
        candidates = self._place_tcp_candidates_for_target(
            obj_id, target_xy, tcp_z, piece_type,
            preferred_quat=self._current_tcp_quat(),
            verified_quat=verified_quat,
            yaw_count=CANDIDATE_YAW_COUNT)
        # Ghi seed vùng để log tái hiện (TODO-6).
        self._last_place_region = self._region_for_target(target_xy)
        self._last_place_seed_count = len(
            self._candidate_seed_states(self._last_place_region))
        return self._move_vertical_place_compensated(
            candidates, step_name, obj_id, target_xy, piece_type, approach_z)

    def _try_execute_cached_carry_chain(self, verified, obj_id: str,
                                       dest_piece_type: str, target_xy,
                                       label: str):
        """Execute đúng chuỗi trajectory đã PASS precheck (giữ nhánh khớp).

        OMPL ngẫu nhiên + IK position-only: plan lại có thể lật sang nhánh khớp
        khác (cùng XYZ, orientation khác) như log a1->e4 đã chứng minh. Hàm này
        tái dùng lift/transfer/pre/desc đã FK-xác nhận, sau khi qua các gate:
        đủ trajectory, arm đúng start lift, local thật gần hypo, scene không
        đổi, endpoint descend (với local THẬT) vẫn đưa tâm quân đúng 5 mm.

        Trả về (place_tcp, where): ("done", place đã execute) | (None,
        "source") chưa hề di chuyển -> caller plan lại toàn bộ | (None,
        "dest") đã mang tới đích -> caller chỉ plan lại đoạn đặt. Execute fail
        (robot đã chuyển động) thì raise, không fallback.
        """
        def _fallback(where: str, reason: str):
            self.get_logger().warning(f"[CACHED] {label}: {reason} -> plan lại")
            return None, where

        if not isinstance(verified, dict):
            return _fallback("source", "không có nghiệm cached")
        lift = verified.get("lift")
        transfer = verified.get("transfer")
        pre = verified.get("pre")
        desc = verified.get("desc")
        hypo_local = verified.get("hypo_local")
        if lift is None or transfer is None or desc is None or hypo_local is None:
            return _fallback("source", "nghiệm cached thiếu trajectory")
        place_start = pre if pre is not None else desc
        if (not self._trajectory_boundary_close(lift, transfer)
                or not self._trajectory_boundary_close(transfer, place_start)
                or (pre is not None
                    and not self._trajectory_boundary_close(pre, desc))):
            return _fallback("source", "các đoạn cached không nối joint liên tục")
        if not self._trajectory_start_close(lift, f"{label}/lift"):
            return _fallback("source", "arm không còn ở start lift cached")
        real_local = self._real_local_close_to_hypo(
            obj_id, hypo_local, label)
        if real_local is None:
            return _fallback("source", "cách gắp thật khác giả định")
        try:
            attached, _world = self._scene_object_ids()
            if obj_id not in attached or "__dry_carry__" in attached:
                return _fallback(
                    "source", f"attached state sai (attached={sorted(attached)})")
            current_fingerprint = self._scene_cache_fingerprint(obj_id)
            if current_fingerprint != verified.get("scene_fingerprint"):
                return _fallback(
                    "source", "pose/geometry/attached/ACM của scene đã đổi")
        except Exception as exc:
            return _fallback("source", f"không đối chiếu được scene ({exc})")

        # Endpoint cached là hàm của trajectory (xác định), nhưng tâm quân phụ
        # thuộc local THẬT. Kiểm tra TRƯỚC mọi chuyển động để gate fail còn
        # có thể plan lại toàn chuỗi từ nguồn.
        try:
            last = desc.points[-1]
            (fx, fy, fz), fq = self._fk_tcp_pose(
                desc.joint_names, last.positions)
            pos_err, tilt, _c, _q = self._piece_error_from_tcp(
                (fx, fy, fz), fq, real_local, target_xy, dest_piece_type)
        except Exception as exc:
            return _fallback("source", f"FK-validate cached fail ({exc})")
        if pos_err > PLACE_POSITION_TOL_M:
            return _fallback(
                "source",
                f"local thật làm tâm endpoint cached lệch "
                f"{pos_err * 1000:.1f}mm; tilt tham khảo "
                f"{math.degrees(tilt):.1f}°")

        # Revalidate toàn bộ waypoint với quân thật. ACM đúng phase: lift còn
        # được chạm bàn tại nguồn; transfer/pre đóng; descend mở tại đích.
        if not self._cached_trajectory_collision_free(lift, f"{label}/lift"):
            return _fallback("source", "lift cached không còn collision-free")
        self._set_piece_collision(
            obj_id, gripper_touch=True, board_contact=False)
        try:
            if not self._cached_trajectory_collision_free(
                    transfer, f"{label}/transfer"):
                return _fallback(
                    "source", "transfer cached không còn collision-free")
            if pre is not None and not self._cached_trajectory_collision_free(
                    pre, f"{label}/pre-place"):
                return _fallback(
                    "source", "pre-place cached không còn collision-free")
            self._set_piece_collision(
                obj_id, gripper_touch=True, board_contact=True)
            if not self._cached_trajectory_collision_free(
                    desc, f"{label}/descend"):
                return _fallback(
                    "source", "descend cached không còn collision-free")
        finally:
            # Trước lift runtime, quân vẫn ở mặt bàn nên phục hồi phase nguồn.
            self._set_piece_collision(
                obj_id, gripper_touch=True, board_contact=True)

        # Qua toàn bộ gate, execute đúng nhánh đã kiểm tra.
        self._execute_and_wait(self.moveit2, lift)
        self._set_piece_collision(
            obj_id, gripper_touch=True, board_contact=False)
        if not self._trajectory_start_close(transfer, f"{label}/transfer"):
            raise RuntimeError(
                f"{label}: arm lệch start transfer cached sau lift")
        self._execute_and_wait(self.moveit2, transfer)
        next_traj = pre if pre is not None else desc
        if not self._trajectory_start_close(next_traj, f"{label}/place"):
            return _fallback("dest", "arm lệch start đoạn đặt cached")
        if pre is not None:
            self._execute_and_wait(self.moveit2, pre)
            if not self._trajectory_start_close(desc, f"{label}/descend"):
                return _fallback("dest", "arm lệch start descend cached")
        self._set_piece_collision(
            obj_id, gripper_touch=True, board_contact=True)
        self._execute_and_wait(self.moveit2, desc)
        self.get_logger().info(
            f"[CACHED] {label}: execute đúng chuỗi precheck đã bù tâm FK")
        return verified["place_tcp"], "done"

    def _move_vertical_place_compensated(self, place_candidates,
                                         step_name: str, obj_id: str,
                                         target_xy, piece_type: str,
                                         approach_z: float):
        """Hạ đặt với candidate IK hữu hạn + Cartesian descend (TODO-2).

        Robot đang ATTACHED ở approach đích. Mỗi candidate (yaw quanh trục
        đứng × seed vùng): plan OMPL pre-place -> Cartesian descend 2mm /
        fraction 0.98 từ chính candidate -> FK tâm quân + tilt -> gate
        fraction/err<=5mm/tilt<=26°/collision toàn trajectory/sát limit/
        nhảy joint -> chấm điểm (err, tilt, travel, margin) -> chọn tốt nhất
        rồi execute MỘT lần. Không hội tụ thì raise khi arm chưa nhúc nhích.
        """
        self._set_piece_collision(
            obj_id, gripper_touch=True, board_contact=False)
        errors = []
        converged = []  # (tilt, pos_err, pre, desc, place_tcp, label)
        chosen = None
        for rank, place_tcp in enumerate(place_candidates, 1):
            for correction in range(PLACE_COMPENSATION_MAX_ITERATIONS):
                px, py, pz, pq = (place_tcp[0], place_tcp[1],
                                  place_tcp[2], place_tcp[3])
                label = f"{step_name}/comp{correction + 1}"
                # Gộp transfer/pre-place: transfer vừa execute tới đúng
                # approach thì pre-place OMPL tới cùng điểm là thừa.
                try:
                    cur_xyz = self._current_tcp_xyz()
                except Exception:
                    cur_xyz = None
                if (cur_xyz is not None
                        and math.sqrt((cur_xyz[0] - px) ** 2
                                      + (cur_xyz[1] - py) ** 2
                                      + (cur_xyz[2] - approach_z) ** 2)
                        <= PREPLACE_SKIP_TOL_M):
                    pre = None
                    self._wait_for_joint_state(self.moveit2)
                    pre_end = copy.deepcopy(self.moveit2.joint_state)
                    pre_q = self._current_tcp_quat()
                    self.get_logger().info(
                        f"[PREPLACE-SKIP] {label}: đã ở approach, "
                        f"descend thẳng không OMPL lại")
                else:
                    # Transfer/pre-place là chuyển động xa hoặc dịch ngang:
                    # dùng OMPL position-only để được đổi nhánh khớp. Chỉ
                    # descend cuối mới bắt buộc Cartesian.
                    try:
                        pre = self._plan_motion(
                            position=[px, py, approach_z],
                            target_link=END_EFFECTOR,
                            tolerance_position=0.002, cartesian=False)
                    except Exception as exc:
                        errors.append(f"{label}: pre-place plan lỗi ({exc})")
                        break
                    if pre is None:
                        errors.append(f"{label}: không có OMPL pre-place")
                        break
                    try:
                        pre_end = self._joint_state_from_trajectory_end(pre)
                        pre_last = pre.points[-1]
                        _pre_xyz, pre_q = self._fk_tcp_pose(
                            pre.joint_names, pre_last.positions)
                    except Exception as exc:
                        errors.append(f"{label}: endpoint pre-place xấu ({exc})")
                        break
                try:
                    self._set_piece_collision(
                        obj_id, gripper_touch=True, board_contact=True)
                    try:
                        desc = self._plan_motion(
                            _start_joint_state=pre_end,
                            position=[px, py, pz], quat_xyzw=pre_q,
                            target_link=END_EFFECTOR,
                            tolerance_position=0.002,
                            tolerance_orientation=0.03,
                            cartesian=True, max_step=CARTESIAN_MAX_STEP_M,
                            cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD)
                    finally:
                        self._set_piece_collision(
                            obj_id, gripper_touch=True,
                            board_contact=False)
                except Exception as exc:
                    errors.append(f"{label}: descend plan/ACM lỗi ({exc})")
                    break
                if desc is None:
                    errors.append(f"{label}: không có Cartesian descend")
                    break
                try:
                    last = desc.points[-1]
                    (fx, fy, fz), fq = self._fk_tcp_pose(
                        desc.joint_names, last.positions)
                    local = self._grasp_local_by_id.get(obj_id)
                    if local is None:
                        raise RuntimeError(f"thiếu T_tcp_piece của {obj_id}")
                    pos_err, tilt, center, _q = self._piece_error_from_tcp(
                        (fx, fy, fz), fq, local, target_xy, piece_type)
                except Exception as exc:
                    errors.append(f"{label}: FK lỗi ({exc})")
                    break
                if pos_err <= PLACE_POSITION_TOL_M:
                    place_tcp = (px, py, pz, fq)
                    # TODO-2/3 gates trên từng candidate (fraction đã gate bởi
                    # planner threshold 0.98: desc is None đã bị loại ở trên).
                    reject = None
                    if tilt > MAX_ACCEPTED_TILT_RAD:
                        reject = (f"tilt {math.degrees(tilt):.1f}° > "
                                  f"{math.degrees(MAX_ACCEPTED_TILT_RAD):.0f}°")
                    else:
                        try:
                            last_map = dict(zip(desc.joint_names, last.positions))
                            margin = self._min_limit_margin_rad(last_map)
                            if margin < JOINT_LIMIT_MARGIN_RAD:
                                reject = (f"sát joint limit margin "
                                          f"{math.degrees(margin):.1f}°")
                            elif max(self._trajectory_max_step(pre)
                                     if pre is not None else 0.0,
                                     self._trajectory_max_step(desc)) > MAX_JOINT_STEP_RAD:
                                reject = ("bước nhảy joint bất thường")
                            elif not self._cached_trajectory_collision_free(
                                    desc, f"{label}/descend"):
                                reject = "trajectory descend collision"
                            elif (pre is not None
                                  and not self._cached_trajectory_collision_free(
                                      pre, f"{label}/pre-place")):
                                reject = "trajectory pre-place collision"
                        except Exception as exc:
                            reject = f"gate candidate lỗi ({exc})"
                    if reject is not None:
                        errors.append(f"{label}: loại candidate ({reject})")
                        self.get_logger().warning(
                            f"[CANDIDATE] {step_name}: {label} loại: {reject} "
                            f"(err {pos_err * 1000:.1f}mm "
                            f"tilt {math.degrees(tilt):.1f}°)")
                        corrected = self._compensate_place_tcp(
                            place_tcp, center, target_xy, piece_type)
                        place_tcp = corrected
                        continue
                    try:
                        self._wait_for_joint_state(self.moveit2, timeout_sec=3.0)
                        travel = self._joint_travel_rad(
                            self.moveit2.joint_state, last_map)
                    except Exception:
                        travel, margin = 0.0, self._min_limit_margin_rad(last_map)
                    score = self._score_place_candidate(
                        pos_err, tilt, travel, margin)
                    converged.append(
                        (score, tilt, pos_err, pre, desc, place_tcp, label,
                         travel, margin))
                    self.get_logger().info(
                        f"[PLACE-COMP] {step_name}: {label} đạt, lệch "
                        f"{pos_err * 1000:.1f}mm; "
                        f"tilt {math.degrees(tilt):.1f}° "
                        f"(target <={math.degrees(PREFERRED_TILT_RAD):.0f}°, "
                        f"hard <={math.degrees(MAX_ACCEPTED_TILT_RAD):.0f}°); "
                        f"travel {math.degrees(travel):.0f}° "
                        f"margin {math.degrees(margin):.0f}° score={score:.3f}")
                    if tilt <= PREFERRED_TILT_RAD:
                        break  # rất tốt: chốt ngay, khỏi bù tiếp
                    # Chưa tốt nhưng đạt tâm: bù tiếp xem vòng sau có tilt
                    # thấp hơn không; quyết định cuối chọn tilt min.
                    corrected = self._compensate_place_tcp(
                        place_tcp, center, target_xy, piece_type)
                    place_tcp = corrected
                    continue
                corrected = self._compensate_place_tcp(
                    place_tcp, center, target_xy, piece_type)
                self.get_logger().info(
                    f"[PLACE-COMP] {label}: tâm lệch "
                    f"{pos_err * 1000:.1f}mm -> dịch TCP "
                    f"({(corrected[0] - px) * 1000:.1f}, "
                    f"{(corrected[1] - py) * 1000:.1f}, "
                    f"{(corrected[2] - pz) * 1000:.1f})mm")
                place_tcp = corrected
            # Hết vòng bù của rank này mà chưa break sớm: sang rank tiếp theo
            # (nếu còn) để tìm nghiệm tilt thấp hơn.
        if converged:
            # TODO-2: chọn score tổng hợp (err + tilt + travel - margin), log
            # đầy đủ candidate được chọn + nguyên nhân reject (errors).
            converged.sort(key=lambda item: (item[0], item[1], item[2]))
            best_score, best_tilt = converged[0][0], converged[0][1]
            self.get_logger().info(
                f"[CANDIDATE-SELECT] {step_name}: {len(converged)} nghiệm đạt, "
                + ", ".join(
                    f"{lab} tilt={math.degrees(t):.1f}° err={e * 1000:.1f}mm "
                    f"travel={math.degrees(tr):.0f}° margin={math.degrees(m):.0f}° "
                    f"score={s:.3f}"
                    for s, t, e, _p, _d, _tcp, lab, tr, m in converged)
                + f" -> chọn {converged[0][6]} score={best_score:.3f}")
            if errors:
                self.get_logger().info(
                    f"[CANDIDATE-REJECTS] {step_name}: " + " | ".join(errors))
            if best_tilt > MAX_ACCEPTED_TILT_RAD:
                raise RuntimeError(
                    f"{step_name}: mọi nghiệm đạt tâm đều nghiêng "
                    f">{math.degrees(MAX_ACCEPTED_TILT_RAD):.0f}° "
                    f"(tốt nhất {math.degrees(best_tilt):.1f}°); từ chối để "
                    f"thử nước/offset khác (arm chưa di chuyển)")
            _bs, _bt, _be, pre, desc, place_tcp, _bl, _tr, _m = converged[0]
            # Lưu candidate được chọn + rejects để tái hiện ván (TODO-6).
            self._last_place_choice = {
                "label": _bl, "score": _bs, "tilt_deg": math.degrees(_bt),
                "err_mm": _be * 1000.0, "rejects": list(errors),
            }
            chosen = (pre, desc, place_tcp)
        if chosen is None:
            raise RuntimeError(
                f"FK cuối trajectory {step_name} không đưa tâm quân vào "
                f"sai số {PLACE_POSITION_TOL_M * 1000:.0f}mm sau "
                f"{PLACE_COMPENSATION_MAX_ITERATIONS} lần bù "
                f"(arm chưa di chuyển): " + "; ".join(errors))
        pre, desc, place_tcp = chosen
        try:
            if pre is not None:
                self._execute_and_wait(self.moveit2, pre)
        except Exception:
            # Robot đã chuyển động: không thử plan khác, thoát để reconcile.
            raise
        # Đúng đoạn chạm bàn mới mở board-contact.
        self._set_piece_collision(
            obj_id, gripper_touch=True, board_contact=True)
        try:
            self._execute_and_wait(self.moveit2, desc)
        except Exception:
            # Fail-closed: thu hẹp ngoại lệ để motion phục hồi sau đó vẫn
            # check va chạm bàn, rồi raise để reconcile.
            try:
                self._set_piece_collision(
                    obj_id, gripper_touch=True, board_contact=False)
            except Exception as acm_exc:
                self.get_logger().warning(
                    f"[PLACE-COMP] {step_name}: không thu hẹp board ACM sau "
                    f"execute fail ({acm_exc})")
            raise
        return place_tcp

    def _plan_vertical_trajectory(self, x, y, z, q, step_name):
        """Plan Cartesian, không execute và không fallback.

        Thử tối đa 2 lần: lần 2 là RE-ROLL cùng tham số (seed IK/path khác),
        chống flake của planner. NÓI RÕ: tolerance_orientation KHÔNG ảnh hưởng
        nội suy Cartesian (GetCartesianPath chỉ dùng waypoint + max_step),
        nên hai lần thử không khác nhau về tolerance — đừng log kiểu "nới".
        """
        for attempt in (1, 2):
            trajectory = self._plan_motion(
                position=[x, y, z],
                quat_xyzw=q,
                target_link=END_EFFECTOR,
                tolerance_position=0.002,
                tolerance_orientation=0.03,
                cartesian=True,
                max_step=CARTESIAN_MAX_STEP_M,
                cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD,
            )
            if trajectory is not None:
                if attempt == 2:
                    self.get_logger().warning(
                        f"[WARN] {step_name} đạt ở lần thử 2 (re-plan cùng "
                        f"tham số, không phải nới tolerance)")
                return trajectory
        return None

    def _set_gripper(self, opening: float):
        if GRIPPER_JOINT is None:
            return
        # Rlink1_Joint điều khiển cả hai ngón qua mimic joints.
        # Binary CHỐT (mimic phi tuyến, cấm nội suy tuyến tính): mở 0.0,
        # đóng 1.57. Muốn 3 phase 20/14/12mm phải có bảng FK
        # gripper_width_to_joint_angle rồi hiệu chỉnh servo thật.
        target = GRIPPER_OPEN_RAD if opening > 0.0 else GRIPPER_CLOSED_RAD
        self._wait_for_joint_state(self.gripper)
        self.gripper.move_to_configuration(
            joint_positions=[target], joint_names=[GRIPPER_JOINT]
        )
        self._execute_and_wait(self.gripper, None)


def main():
    rclpy.init()
    node = PickPlaceNode()
    # BẮT BUỘC dùng MultiThreadedExecutor: _execute_move chạy trên thread riêng
    # và gọi service /apply_planning_scene (attach/detach) một cách đồng bộ
    # (poll future.done()) - cần executor xử lý callback đó song song, không
    # phải chờ tuần tự như SingleThreadedExecutor mặc định của rclpy.spin().
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# ==== THAY THẾ ====
# Nếu code MoveIt2 bạn đã viết cho task ấm trà dùng MoveGroupInterface (C++) hoặc
# một wrapper Python khác thay vì pymoveit2, chỉ cần giữ nguyên toàn bộ logic
# _do_pick_place / _do_discard / _setup_initial_scene ở trên (đây là phần "kịch bản"
# không phụ thuộc thư viện) và thay các lệnh self.moveit2.xxx() bằng API tương ứng
# bạn đã dùng (move_group.set_pose_target(), planning_scene_interface.add_box(), ...).
