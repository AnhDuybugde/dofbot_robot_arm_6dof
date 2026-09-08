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
    BOARD_Z,
    DISCARD_MAX_SLOTS,
    DISCARD_TCP_Z,
    GRASP_APPROACH_CANDIDATE_OFFSETS,
    GRASP_MAX_OFFSET,
    GRASP_SEARCH_TIMEOUT_SEC,
    PICK_TCP_Z,
    PIECE_SPECS,
    COLLISION_ENABLED,
    REACHABILITY_EXECUTE_ON_FAKESYSTEM,
    approach_tcp_z,
    discard_slot_pose,
    square_to_grasp_pose,
    square_to_place_pose,
    square_to_xy,
)

# ==== TODO: thay bằng thông số robot 6DOF thật của bạn (giống config đã dùng cho task ấm trà) ====
JOINT_NAMES = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]
BASE_LINK = "base_link"
END_EFFECTOR = "Gripping_point_Link"
GROUP_NAME = "arm_group"
GRIPPER_JOINT = "Rlink1_Joint"
GRIPPER_GROUP = "grip_group"
# Toàn bộ các link thực sự của ngón. Khi object đã attach, MoveIt được phép cho
# object chạm các link này, nhưng vẫn kiểm tra va chạm với tay/bàn/quân khác.
GRIPPER_TOUCH_LINKS = [
    END_EFFECTOR,
    # Palm/đế của cụm kẹp. Quân hậu/vua cao chạm phần này khi hai ngón kẹp
    # đúng thân quân; chỉ được phép với CHÍNH quân đang gắp qua ACM tạm thời.
    "arm5_Link",
    "Rlink1_Link", "Rlink2_Link", "Rlink3_Link",
    "Llink1_Link", "Llink2_Link", "Llink3_Link",
]
# SRDF `arm_group/up`: pose joint đã biết, dùng làm điểm đầu/cuối ổn định cho
# mỗi lượt robot. Đây là joint-goal (PTP), không phải Cartesian target.
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]


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
        # Guard race: on_move spawn thread mỗi message; 2 thread _execute_move
        # song song sẽ xé board/piece maps dùng chung. Flow chuẩn đã ACK-gated
        # nên cờ này chỉ chặn publish thủ công chồng lệnh.
        self._exec_lock = threading.Lock()
        self._executing = False
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

        self._setup_initial_scene()
        self.get_logger().info(
            f"Pick-place node sẵn sàng; collision={'ON' if COLLISION_ENABLED else 'OFF'}, "
            f"deep-check={'EXECUTE' if REACHABILITY_EXECUTE_ON_FAKESYSTEM else 'plan-only'}."
        )

    # ---------------- Planning scene ----------------

    def _new_piece_id(self) -> str:
        return f"piece_{next(self._id_counter):03d}"

    def _setup_initial_scene(self):
        """Thêm bàn cờ (1 box) + quân cờ (cylinder) vào PlanningScene theo đúng
        vị trí bắt đầu chuẩn của chess.Board()."""
        # d4 là tâm một ô; cộng nửa ô để thành đúng tâm bàn 8x8.
        board_center_x, board_center_y = square_to_xy("d4")
        if COLLISION_ENABLED:
            self.moveit2.add_collision_box(
                id="chessboard",
                position=[board_center_x + 0.5 * 0.027, board_center_y + 0.5 * 0.027, BOARD_Z - 0.01],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                size=[0.216, 0.216, 0.02],
            )

        self._publish_board_visual(publish=False)
        for square, piece in self.board.piece_map().items():
            self._add_piece_collision(chess.square_name(square), piece, publish=False)
        # FPS fix: startup trước đây publish 1 snapshot full (64 ô + N quân)
        # sau MỖI quân (33 lần) + mỗi add_collision_* trigger 1 planning-scene
        # diff -> RViz update storm. Giờ chỉ publish 1 lần duy nhất.
        self._publish_all_visual()

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
        spec = PIECE_SPECS[piece_type]
        self.moveit2.add_collision_cylinder(
            id=obj_id,
            position=[x, y, z + spec.pickup_height / 2],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            height=spec.pickup_height,
            radius=0.012,
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
            marker.scale.x = marker.scale.y = 0.027
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
        spec = PIECE_SPECS[piece_type]
        marker = Marker()
        marker.header.frame_id = BASE_LINK
        marker.ns = "chess_pieces"
        marker.id = int(obj_id.rsplit("_", 1)[1])
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + spec.pickup_height / 2
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.021
        marker.scale.z = spec.pickup_height
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
        spec = PIECE_SPECS[piece_type]
        marker = Marker()
        marker.header.frame_id = END_EFFECTOR
        marker.ns = "chess_pieces"
        marker.id = int(obj_id.rsplit("_", 1)[1])
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = local_pose
        marker.scale.x = marker.scale.y = 0.021
        marker.scale.z = spec.pickup_height
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
        """Xoay vector bằng quaternion (x, y, z, w) đã chuẩn hoá."""
        x, y, z, w = quaternion
        vx, vy, vz = vector
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
        trước). Trả về (x, y, z, quat_xyzw).
        """
        spec = PIECE_SPECS[piece_type]
        want = (target_xy[0], target_xy[1], BOARD_Z + spec.pickup_height / 2)
        local = self._grasp_local_by_id.get(obj_id)
        if local is None:
            q = self._current_tcp_quat()
            self.get_logger().warning(
                f"[WARN] {obj_id} không có offset attach -> đặt TCP đúng tâm, "
                f"quân có thể lệch nếu gắp không chuẩn tâm")
            return (*target_xy, tcp_z_fallback, q)
        lx, ly, lz = local.position.x, local.position.y, local.position.z
        ql = (local.orientation.x, local.orientation.y,
              local.orientation.z, local.orientation.w)
        if preferred_quat is not None:
            # Yaw tối ưu: Q_tcp(ψ) = Y(ψ) ⊗ conj(q_local); cực tiểu góc với
            # preferred khi tan(ψ/2) = N.z / N.w.
            n = self._multiply_quaternions(tuple(preferred_quat), ql)
            half = math.atan2(n[2], n[3])
            s, c = math.sin(half), math.cos(half)
            yaw = (0.0, 0.0, s, c)
            m = (-ql[0], -ql[1], -ql[2], ql[3])
            q_tcp = self._multiply_quaternions(yaw, m)
        else:
            # Quân đứng thẳng ở đích: Q_want = identity -> Q_tcp = conj(q_local).
            q_tcp = (-ql[0], -ql[1], -ql[2], ql[3])
        # P_tcp = P_want - R(Q_tcp) * p_local.
        rx, ry, rz = self._rotate_by_quaternion((lx, ly, lz), q_tcp)
        tcp = (want[0] - rx, want[1] - ry, want[2] - rz)
        return (*tcp, q_tcp)

    @staticmethod
    def _multiply_quaternions(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _tilt_from_quaternion(q_xyzw) -> float:
        """Góc nghiêng (rad) giữa trục Z của vật và trục Z thế giới, BỎ QUA yaw.

        Đo sai cũ (2·acos(|qw|)) so toàn bộ quaternion với identity nên yaw
        thuần 131° cũng bị tính thành 'nghiêng 131°'. Chỉ lấy thành phần z của
        R(q)·ẑ = 1−2(x²+y²): yaw-only cho đúng 0.
        """
        x, y, _z, _w = q_xyzw
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
            if position_error <= 0.005 and tilt <= math.radians(5.0):
                return
            if time.monotonic() >= deadline:
                req = ("không rõ" if requested_tcp is None else
                       f"xyz={[round(v, 4) for v in requested_tcp[:3]]} "
                       f"quat={[round(v, 3) for v in requested_tcp[3]]}")
                raise RuntimeError(
                    f"pose quân trước detach sai: tâm quân thực tế "
                    f"={[round(v, 4) for v in actual]} muốn={expected} "
                    f"(lệch {position_error:.4f}m), nghiêng "
                    f"(bỏ yaw)={math.degrees(tilt):.1f}deg; TCP yêu cầu {req}, "
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
        """Tính T_tcp_piece cho một quân đang đứng trên mặt bàn."""
        spec = PIECE_SPECS[piece_type]
        transform = self.tf_buffer.lookup_transform(
            BASE_LINK, END_EFFECTOR, rclpy.time.Time())
        tcp = transform.transform.translation
        q = transform.transform.rotation
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
        spec = PIECE_SPECS[piece_type]
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
        primitive.dimensions = [spec.pickup_height, 0.012]  # [height, radius]
        aco.object.primitives = [primitive]
        aco.object.primitive_poses = [pose]
        aco.touch_links = GRIPPER_TOUCH_LINKS

        self._apply_attached_object(aco)
        self._wait_for_scene_object(obj_id, attached=True)
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
            return
        else:
            self.get_logger().info(f"[OK] done {uci}")
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
                        max_step=0.002, cartesian_fraction_threshold=0.999))
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
                        max_step=0.002, cartesian_fraction_threshold=0.999))
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
        obj_id = "__dry_carry__"
        spec = PIECE_SPECS[piece_type]
        if piece_world_xy is not None:
            pose = self._piece_local_pose(piece_type, piece_world_xy)
        else:
            # Proxy discard bắt đầu đứng thẳng trong BASE_LINK và có tâm thấp
            # hơn TCP đúng bằng quan hệ tại pose gắp chuẩn. Nhờ lưu orientation
            # inverse của TCP, nó xoay theo arm giống một quân đã attach thật.
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
        primitive.dimensions = [spec.pickup_height, 0.012]
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
                        max_step=0.002, cartesian_fraction_threshold=0.999))
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
            return self._check_reachability_impl(request, response)
        finally:
            with self._exec_lock:
                self._executing = False

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
            aborted_case = None
            squares_done = 0
            for square in deep_squares:
                piece = self.board.piece_at(chess.parse_square(square))
                piece_type = piece.symbol().lower() if piece else "p"
                before = sum(len(items) for items in deep.values())
                self._dry_run_square(square, piece_type, scratch_square, deep, execute)
                squares_done += 1
                if sum(len(items) for items in deep.values()) > before:
                    aborted_case = square
                    break
            slots_done = 0
            if aborted_case is None:
                for slot in self.DEEP_CHECK_DISCARD_SLOTS:
                    before = sum(len(items) for items in deep.values())
                    self._dry_run_discard_slot(
                        slot, deep,
                        execute and slot in self.DEEP_CHECK_DISCARD_EXEC_SLOTS)
                    slots_done += 1
                    if sum(len(items) for items in deep.values()) > before:
                        aborted_case = f"slot{slot}"
                        break
            if aborted_case is not None:
                skipped_squares = len(deep_squares) - squares_done
                skipped_slots = (
                    len(self.DEEP_CHECK_DISCARD_SLOTS) - slots_done
                    if squares_done == len(deep_squares) else
                    len(self.DEEP_CHECK_DISCARD_SLOTS))
                self.get_logger().warning(
                    f"[FAIL-FAST] dừng deep-check sau root failure tại {aborted_case}; "
                    f"đã chạy {squares_done}/{len(deep_squares)} ô + "
                    f"{slots_done}/{len(self.DEEP_CHECK_DISCARD_SLOTS)} slot, "
                    f"bỏ qua (skipped) {skipped_squares} ô + {skipped_slots} slot."
                )
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
                    "[RECOVERY-REQUIRED] deep-check execute để lại scene không "
                    "nhất quán (hoặc carry_state="
                    f"{self._carry_state}). Restart launch trước khi chơi tiếp.")

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
                f"deep-check đã chạy {squares_done}/{len(deep_squares)} ô trắng + "
                f"{slots_done}/{len(self.DEEP_CHECK_DISCARD_SLOTS)} slot "
                f"(fail-fast bỏ qua phần còn lại khi có lỗi), "
                f"lỗi {deep_total} "
                f"({'EXECUTE' if execute else 'plan-only'}, collision={'ON' if COLLISION_ENABLED else 'OFF'}). "
                + ("PASS toàn bộ." if response.success
                   else "CÓ LỖI — xem terminal để biết ô/phase.")
            )
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
            x0, y0, z0, source_approach_z, grasp_q = self._approach_and_descend_for_grasp(
                from_sq, piece_type, "hạ gắp", obj_id
            )
            self._set_gripper(0.0)
            self._carry_state = "ATTACH_PENDING"
            self._attach_piece(obj_id, piece_type, square_to_xy(from_sq))
            self._carry_state = "ATTACHED"
            self._move_vertical(x0, y0, source_approach_z, "nâng sau gắp", quat_xyzw=grasp_q)
            # Quân đã thoát mặt bàn: ĐÓNG board-contact ngay (Fix 3). Giữ
            # gripper-touch suốt lúc mang. Nếu còn mở, quân attached xuyên bàn
            # trong transfer mà không bị chặn.
            self._set_piece_collision(obj_id, gripper_touch=True, board_contact=False)

            # Transit position-only tới tâm approach ô đích: KHÔNG ép orientation
            # (5 DOF + IK position-only; ép quat grasp lúc gắp qua cả bàn cờ là
            # vô nghiệm như case a1->e4). Orientation chỉ chốt ở bước hạ đặt
            # Cartesian với yaw tối ưu bên dưới.
            self._move_to(x1, y1, target_approach_z)
            # Đặt bù offset gắp (Fix 2): TCP tới pose đã bù để TÂM QUÂN (không
            # phải TCP) rơi đúng tâm ô đích; yaw quân ở đích được chọn để TCP
            # gần nhất với quat sau transfer (quân vẫn đứng thẳng đứng vì yaw
            # quanh trục đứng không gây nghiêng). Cartesian fail ở đây -> raise
            # để NACK, không đặt lệch âm thầm.
            place_tcp = self._place_tcp_for_target(
                obj_id, (x1, y1), z1, placed_piece_type or piece_type,
                preferred_quat=self._current_tcp_quat())
            # Sắp chạm mặt bàn ở điểm đặt: MỞ board-contact trước khi hạ.
            self._set_piece_collision(obj_id, gripper_touch=True, board_contact=True)
            self._move_vertical_place(
                place_tcp, "hạ đặt", obj_id, (x1, y1),
                placed_piece_type or piece_type)
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
            x0, y0, z0, source_approach_z, grasp_q = self._approach_and_descend_for_grasp(
                square, piece_type, "hạ gắp quân bị ăn", obj_id
            )
            self._set_gripper(0.0)
            self._carry_state = "ATTACH_PENDING"
            self._attach_piece(obj_id, piece_type, square_to_xy(square))
            self._carry_state = "ATTACHED"
            self._move_vertical(x0, y0, source_approach_z, "nâng quân bị ăn", quat_xyzw=grasp_q)
            self._set_piece_collision(obj_id, gripper_touch=True, board_contact=False)
            # Transit position-only tới khu discard (lý do như _do_pick_place:
            # không ép orientation lúc mang), rồi hạ bù với yaw tối ưu.
            self._move_to(xd, yd, discard_tcp_z + APPROACH_HEIGHT)
            drop_tcp = self._place_tcp_for_target(
                obj_id, (xd, yd), discard_tcp_z, piece_type,
                preferred_quat=self._current_tcp_quat())
            self._set_piece_collision(obj_id, gripper_touch=True, board_contact=True)
            self._move_vertical_place(
                drop_tcp, "hạ thả quân bị ăn", obj_id, (xd, yd), piece_type)
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
                                        source_obj_id=None):
        """Tìm offset gắp an toàn bằng chính pipeline OMPL -> Cartesian runtime.

        Mỗi candidate đều được collision-check. Candidate thất bại ở approach
        hoặc descend không làm attach quân, vì vậy có thể thử candidate kế.
        Fix 6: candidate chỉ được CHỌN sau khi lift cũng plan được VỚI thể tích
        mang (proxy scratch attached ở đúng TCP sau descend) — trước đây chọn
        ngay khi descend xong nên lift/place vẫn có thể rớt sau attach. Thêm
        điều kiện hình học offset tối đa và trần tổng thời gian tìm kiếm.
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
            # Validate lift CÓ mang trước khi kẹp thật: gắn proxy scratch đúng
            # TCP hiện tại, plan nâng, gỡ proxy. Kẹp vẫn mở, chưa attach gì.
            scratch = None
            source_xyz = (*square_to_xy(square), BOARD_Z)
            try:
                scratch = self._dry_attach_scratch(
                    piece_type, square_to_xy(square), source_obj_id)
                lift_trajectory = self._plan_vertical_trajectory(
                    x, y, approach_z, grasp_q, f"{step_name}/lift-validate")
            finally:
                self._dry_detach_scratch(
                    scratch or "__dry_carry__", source_obj_id, source_xyz, piece_type)
            if lift_trajectory is None:
                errors.append(f"{offset}: Cartesian lift fail (có mang)")
                self.get_logger().warning(
                    f"[GRASP-CANDIDATE] {square} thử {index} offset={offset}: "
                    f"descend được nhưng lift có mang fail -> loại"
                )
                continue
            self._grasp_offset_cache[square] = offset
            self.get_logger().info(
                f"[GRASP-CANDIDATE] {square} chọn offset={offset} sau {index} lần thử"
            )
            return x, y, z, approach_z, grasp_q
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

    def _plan_motion(self, **kwargs):
        """Phiên bản non-spinning của pymoveit2.plan().

        pymoveit2.plan() tự gọi rclpy.spin_once(), điều này không hợp lệ khi
        node đã chạy trong MultiThreadedExecutor và dẫn tới lỗi wait-set.
        """
        self._wait_for_joint_state(self.moveit2)
        cartesian = kwargs.get("cartesian", False)
        fraction_threshold = kwargs.pop("cartesian_fraction_threshold", 0.0)
        future = self.moveit2.plan_async(
            start_joint_state=self.moveit2.joint_state, **kwargs
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
        return [q.x, q.y, q.z, q.w]

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

    def _validate_place_trajectory_end(self, trajectory, obj_id: str, target_xy,
                                       piece_type: str, requested_tcp, step_name: str):
        """Dùng FK kiểm tra ĐIỂM CUỐI trajectory hạ đặt TRƯỚC execute.

        IK position-only có thể thực hiện orientation khác với quat đã dùng để
        tính bù TCP: dù TCP tới đúng XYZ thì tâm quân vẫn lệch. FK điểm cuối +
        T_tcp_piece cho tâm quân và độ nghiêng THỰC SẼ ĐẠT; không đạt ngưỡng
        (5 mm / 5°) thì raise kèm đủ pose để phân biệt lỗi IK, transform hay TF.
        """
        last = trajectory.points[-1]
        (fx, fy, fz), fq = self._fk_tcp_pose(trajectory.joint_names, last.positions)
        local = self._grasp_local_by_id.get(obj_id)
        if local is None:
            raise RuntimeError(f"thiếu T_tcp_piece của {obj_id} khi FK-validate {step_name}")
        pl = (local.position.x, local.position.y, local.position.z)
        ql = (local.orientation.x, local.orientation.y,
              local.orientation.z, local.orientation.w)
        rx, ry, rz = self._rotate_by_quaternion(pl, fq)
        center = (fx + rx, fy + ry, fz + rz)
        spec = PIECE_SPECS[piece_type]
        want = (target_xy[0], target_xy[1], BOARD_Z + spec.pickup_height / 2)
        position_error = math.sqrt(sum(
            (got - w) ** 2 for got, w in zip(center, want)))
        q_piece = self._multiply_quaternions(fq, ql)
        tilt = self._tilt_from_quaternion(q_piece)
        if position_error <= 0.005 and tilt <= math.radians(5.0):
            return
        raise RuntimeError(
            f"FK cuối trajectory {step_name} không đạt pose đặt: "
            f"TCP yêu cầu xyz={[round(v, 4) for v in requested_tcp[:3]]} "
            f"quat={[round(v, 3) for v in requested_tcp[3]]}; "
            f"TCP FK xyz={[round(v, 4) for v in (fx, fy, fz)]} "
            f"quat={[round(v, 3) for v in fq]}; "
            f"tâm quân FK={[round(v, 4) for v in center]} muốn={want} "
            f"(lệch {position_error:.4f}m), nghiêng (bỏ yaw)={math.degrees(tilt):.1f}deg. "
            f"Khả năng: IK position-only thực hiện orientation khác quat tính bù.")

    def _move_vertical(self, x, y, z, step_name: str, quat_xyzw=None):
        """Đi thẳng đứng bằng compute_cartesian_path, không để OMPL lách qua
        bàn/quân trong đoạn hạ hoặc nâng.

        quat_xyzw: orientation giữ suốt đoạn đi. Nên truyền quaternion đã chốt
        sau approach (xem _current_tcp_quat); None = đọc TF hiện tại (giữ hành
        vi cũ cho caller đơn lẻ). Plan fail mới raise rõ ràng thay vì fallback
        âm thầm (trừ khi ALLOW_CARTESIAN_FALLBACK=True được bật tường minh cho
        demo). Bước HẠ ĐẶT không dùng hàm này mà dùng _move_vertical_place để
        FK-validate pose quân trước execute.
        """
        q = quat_xyzw if quat_xyzw is not None else self._current_tcp_quat()
        trajectory = self._plan_vertical_or_raise(x, y, z, q, step_name)
        if trajectory is None:
            self.get_logger().warning(
                f"Cartesian không đủ tại {step_name}; dùng position-only fallback tới {(x, y, z)}"
            )
            self._move_to(x, y, z)
            return
        self._execute_and_wait(self.moveit2, trajectory)

    def _plan_vertical_or_raise(self, x, y, z, q, step_name: str):
        """Plan Cartesian hoặc raise (kèm chẩn đoán), giữ nguyên fallback opt-in."""
        trajectory = self._plan_vertical_trajectory(x, y, z, q, step_name)
        if trajectory is None:
            self._diagnose_position_goal_collision(
                (x, y, z), q, f"cartesian/{step_name}/{(x, y, z)}"
            )
            if not COLLISION_ENABLED and ALLOW_CARTESIAN_FALLBACK:
                return None
            raise RuntimeError(
                f"Không có Cartesian path an toàn khi {step_name} tới {(x, y, z)} "
                f"(quat giữ {[round(v, 3) for v in q]}). "
                f"Hãy chạy /chess/check_reachability để xem ô/phase lỗi, "
                f"kiểm tra approach offset và orientation sau approach."
            )
        return trajectory

    def _move_vertical_place(self, place_tcp, step_name: str, obj_id: str,
                             target_xy, piece_type: str):
        """Hạ đặt: plan -> FK-validate điểm cuối (tâm quân + nghiêng) -> execute.

        Không đạt thì raise TRƯỚC khi arm nhúc nhích: caller loại candidate /
        NACK thay vì đặt lệch rồi mới phát hiện ở _verify.
        """
        trajectory = self._plan_vertical_or_raise(
            *place_tcp[:3], place_tcp[3], step_name)
        if trajectory is None:
            self.get_logger().warning(
                f"Cartesian không đủ tại {step_name}; dùng position-only fallback tới {place_tcp[:3]}"
            )
            self._move_to(*place_tcp[:3])
            return
        self._validate_place_trajectory_end(
            trajectory, obj_id, target_xy, piece_type, place_tcp, step_name)
        self._execute_and_wait(self.moveit2, trajectory)

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
                max_step=0.002,
                cartesian_fraction_threshold=0.999,
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
        # Rlink1_Joint điều khiển cả hai ngón trong URDF Dofbot qua mimic joints.
        # Giá trị 0 là mở, 1.57 rad là đóng (named states open/close trong SRDF).
        target = 0.0 if opening > 0.0 else 1.57
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
