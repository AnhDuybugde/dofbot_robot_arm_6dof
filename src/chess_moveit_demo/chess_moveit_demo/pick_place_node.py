"""Node pick-place: nhận nước cờ UCI từ /chess/move, phân rã thành
Approach -> Pick -> Lift -> Move -> Place -> Return Home, đồng thời cập nhật
PlanningScene (add/remove/di chuyển collision object quân cờ) để MoveIt2 tính
toán tránh va chạm với các quân khác trên bàn.

Cài đặt:
    pip install pymoveit2
(hoặc thay lớp MoveIt2 này bằng MoveGroupInterface bạn đã dùng cho task ấm trà -
 xem ghi chú "THAY THẾ" ở cuối file).
"""

import itertools
import json
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

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

from pymoveit2 import MoveIt2

from .chess_utils import (
    ALLOW_CARTESIAN_FALLBACK,
    APPROACH_HEIGHT,
    BOARD_Z,
    DISCARD_MAX_SLOTS,
    DISCARD_TCP_Z,
    PIECE_SPECS,
    SIMULATION_IGNORE_COLLISIONS,
    approach_tcp_z,
    discard_slot_pose,
    square_to_grasp_pose,
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
        # Chế độ demo bỏ collision của bàn/quân để xác nhận pipeline end-to-end.
        # Self-collision của robot vẫn do MoveIt/URDF kiểm tra. Không dùng cờ
        # này với arm thật; đổi SIMULATION_IGNORE_COLLISIONS về False trước.
        self.moveit2.cartesian_avoid_collisions = not SIMULATION_IGNORE_COLLISIONS
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.board = chess.Board()
        self.discard_count = 0
        self.carried_piece_id: str | None = None
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
        self._apply_scene_client.wait_for_service(timeout_sec=10.0)

        self._setup_initial_scene()
        self.get_logger().info("Pick-place node sẵn sàng, đã dựng planning scene ban đầu.")

    # ---------------- Planning scene ----------------

    def _new_piece_id(self) -> str:
        return f"piece_{next(self._id_counter):03d}"

    def _setup_initial_scene(self):
        """Thêm bàn cờ (1 box) + quân cờ (cylinder) vào PlanningScene theo đúng
        vị trí bắt đầu chuẩn của chess.Board()."""
        board_center_x, board_center_y = square_to_xy("d4")  # ~ tâm bàn 8x8
        if not SIMULATION_IGNORE_COLLISIONS:
            self.moveit2.add_collision_box(
                id="chessboard",
                position=[board_center_x + 0.025, board_center_y + 0.025, BOARD_Z - 0.01],
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
        if SIMULATION_IGNORE_COLLISIONS:
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
        while not future.done():
            time.sleep(0.01)

    def _take_piece_from_world(self, square: str) -> str:
        """Bỏ collision của chính quân sắp gắp trước khi hạ gripper.

        Nếu vẫn để world object này trong scene, goal ở vị trí kẹp là va chạm
        hợp lệ về mặt vật lý nhưng MoveIt sẽ loại bỏ mọi IK.  Các quân khác và
        bàn cờ vẫn giữ collision bình thường.
        """
        obj_id = self.piece_id_by_square.pop(square)
        # Ở mode demo bỏ collision, object chưa từng được add nên skip remove
        # để khỏi spam warn "does not exist in this scene" mỗi nước đi.
        if not SIMULATION_IGNORE_COLLISIONS:
            self.moveit2.remove_collision_object(id=obj_id)
            # PlanningScene cập nhật bất đồng bộ; tránh lập plan ngay trong cùng tick.
            time.sleep(0.15)
        return obj_id

    def _restore_piece_to_world(self, square: str, obj_id: str, piece_type: str):
        """Rollback duy nhất an toàn trước khi object được attach vào gripper."""
        x, y = square_to_xy(square)
        self._add_piece_collision_at(obj_id, (x, y, BOARD_Z), piece_type)
        self.piece_id_by_square[square] = obj_id
        self._publish_piece_visual(obj_id, (x, y, BOARD_Z))

    def _restore_piece_collision(self, square: str, obj_id: str, piece_type: str):
        """Hoàn trả collision tạm bỏ bởi reachability check.

        Khác ``_restore_piece_to_world``: mapping/visual không đổi, vì service
        chỉ mô phỏng planning scene chứ không hề nhấc quân khỏi bàn.
        """
        x, y = square_to_xy(square)
        self._add_piece_collision_at(obj_id, (x, y, BOARD_Z), piece_type)
        # add/remove collision object là asynchronous; tránh để lần kiểm tra ô
        # sau nhìn vào scene trung gian.
        time.sleep(0.15)

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

    def _attach_piece(self, obj_id: str, piece_type: str, pick_xyz) -> str:
        """Gọi NGAY SAU KHI gripper đã đóng ở vị trí gắp: chuyển collision object
        từ 'world object' đứng yên trên bàn sang 'attached object' dính vào
        END_EFFECTOR, để nó trôi theo cánh tay trong suốt Lift->Move->Place.
        Trả về obj_id để hàm gọi truyền tiếp cho _detach_piece."""
        spec = PIECE_SPECS[piece_type]
        # Pose tương đối so với END_EFFECTOR lấy từ TF hiện tại. Nhờ vậy object
        # giữ đúng pose world lúc kẹp, kể cả TCP không song song trục Z của bàn.
        transform = self.tf_buffer.lookup_transform(
            BASE_LINK, END_EFFECTOR, rclpy.time.Time()
        )
        tcp = transform.transform.translation
        q = transform.transform.rotation
        center_world = (pick_xyz[0], pick_xyz[1], BOARD_Z + spec.pickup_height / 2)
        local = self._rotate_by_inverse_quaternion(
            (center_world[0] - tcp.x, center_world[1] - tcp.y, center_world[2] - tcp.z), q
        )
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = local
        # Object đứng thẳng theo BASE_LINK trước khi attach, nên orientation
        # tương đối là inverse(T_base_tcp).
        pose.orientation.x = -q.x
        pose.orientation.y = -q.y
        pose.orientation.z = -q.z
        pose.orientation.w = q.w
        self._publish_carried_piece_visual(obj_id, pose)

        # Demo vẫn có carry visual ở trên, chỉ bỏ attached collision để nhẹ.
        if SIMULATION_IGNORE_COLLISIONS:
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
        return obj_id

    def _detach_piece(self, obj_id: str, to_square: str, world_xyz, piece_type: str):
        """Gọi NGAY SAU KHI gripper đã mở ở vị trí đặt: gỡ object khỏi
        END_EFFECTOR và thêm lại thành world object tại toạ độ mới.
        to_square=None nếu thả vào khu 'nghĩa địa' (quân bị ăn), không thuộc bàn cờ."""
        if not SIMULATION_IGNORE_COLLISIONS:
            aco = AttachedCollisionObject()
            aco.link_name = END_EFFECTOR
            aco.object.id = obj_id
            aco.object.operation = CollisionObject.REMOVE
            self._apply_attached_object(aco)

        self._add_piece_collision_at(obj_id, world_xyz, piece_type)
        old_type, is_white = self.piece_info_by_id[obj_id]
        if old_type != piece_type:  # phong cấp: tốt thành hậu/xe/tượng/mã
            self.piece_info_by_id[obj_id] = (piece_type, is_white)
        self._publish_piece_visual(obj_id, world_xyz)
        if self.carried_piece_id == obj_id:
            self.carried_piece_id = None
        if to_square is not None:
            self.piece_id_by_square[to_square] = obj_id

    # ---------------- Move execution ----------------

    def _fail(self, uci: str, reason: str):
        """Gửi NACK để brain dừng chờ ACK và dừng game một cách tường minh."""
        msg = String()
        msg.data = f"{uci}: {reason}"
        self.fail_pub.publish(msg)
        self.get_logger().error(f"[FAIL] {uci}: {reason} (đã gửi NACK)")

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
                self._fail(uci, "overlapped with previous move")
                return
            self._executing = True
        # chạy trên thread riêng để không block callback ROS trong lúc chờ MoveIt2 thực thi
        threading.Thread(target=self._execute_move_guarded, args=(payload,), daemon=True).start()

    def _execute_move_guarded(self, payload: dict):
        try:
            self._execute_move(payload)
        finally:
            with self._exec_lock:
                self._executing = False

    def _execute_move(self, payload: dict):
        uci = payload["uci"]
        from_sq, to_sq = uci[:2], uci[2:4]
        move = chess.Move.from_uci(uci)
        if move not in self.board.legal_moves:
            self._fail(uci, "nước đi không hợp lệ theo python-chess")
            return
        moving_piece = self.board.piece_at(move.from_square)
        piece_type = moving_piece.symbol().lower()
        expected_execution = "robot" if moving_piece.color == chess.WHITE else "virtual"
        execution = payload.get("execution", expected_execution)
        if execution != expected_execution:
            self._fail(uci, f"execution sai: nhận {execution}, phải là {expected_execution}")
            return

        try:
            if execution == "virtual":
                self._do_virtual_move(move, piece_type, payload)
                self.board.push(move)
                self.get_logger().info(f"[OK] virtual done {uci}")
                done = String()
                done.data = uci
                self.done_pub.publish(done)
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
                # object được đổi sang loại quân phong cấp.
                final_piece_type = payload.get("promotion") or piece_type
                self._do_pick_place(from_sq, to_sq, piece_type, final_piece_type)

            self._move_to_home("HOME sau lượt Trắng")

            # cập nhật board nội bộ SAU KHI đã dùng vị trí cũ để tính toán ở trên
            self.board.push(chess.Move.from_uci(uci))
        except Exception as exc:
            self._fail(uci, f"không thực thi: {exc}")
            return
        else:
            self.get_logger().info(f"[OK] done {uci}")
            done = String()
            done.data = uci
            self.done_pub.publish(done)

    def _remove_virtual_piece(self, square: str):
        """Bỏ quân bị ăn khỏi scene/visual trong lượt Đen tự đi."""
        obj_id = self.piece_id_by_square.pop(square)
        if not SIMULATION_IGNORE_COLLISIONS:
            self.moveit2.remove_collision_object(id=obj_id)
        self._delete_piece_visual(obj_id)

    def _move_virtual_piece(self, from_sq: str, to_sq: str, piece_type: str):
        obj_id = self.piece_id_by_square.pop(from_sq)
        if not SIMULATION_IGNORE_COLLISIONS:
            self.moveit2.remove_collision_object(id=obj_id)
        old_type, is_white = self.piece_info_by_id[obj_id]
        if old_type != piece_type:
            self.piece_info_by_id[obj_id] = (piece_type, is_white)
        target_xyz = (*square_to_xy(to_sq), BOARD_Z)
        self._add_piece_collision_at(obj_id, target_xyz, piece_type)
        self.piece_id_by_square[to_sq] = obj_id
        self._publish_piece_visual(obj_id, target_xyz)

    def _do_virtual_move(self, move: chess.Move, piece_type: str, payload: dict):
        """Cập nhật atomically scene + visual cho nước Đen, không chạy arm."""
        from_sq = chess.square_name(move.from_square)
        to_sq = chess.square_name(move.to_square)
        if self.board.is_capture(move):
            captured_sq = (
                to_sq[0] + from_sq[1]
                if self.board.is_en_passant(move) else to_sq
            )
            self._remove_virtual_piece(captured_sq)

        final_piece_type = payload.get("promotion") or piece_type
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

    # Ô đại diện cho deep-check: góc bàn, mép gần đế (c1/d1/e1/f1 có
    # approach offset riêng), hàng tốt, trung tâm. Full-chain trên toàn bộ
    # 64 ô quá lâu (mỗi ô 6 đoạn), subset này phủ mọi vùng hình học đặc biệt.
    DEEP_CHECK_SQUARES = (
        "a1", "c1", "d1", "e1", "f1", "h1", "b2", "g7",
        "d4", "e4", "a8", "h8",
    )
    # Quét plan-only (nhanh) toàn bộ 16 slot nghĩa địa — bản cũ chỉ check 64 ô
    # cờ nên slot discard thứ 6 mới lòi lỗi khi game đang chạy. Chỉ execute
    # 3 slot đầu/giữa/cuối để giữ thời gian check hợp lý.
    DEEP_CHECK_DISCARD_SLOTS = tuple(range(DISCARD_MAX_SLOTS))
    DEEP_CHECK_DISCARD_EXEC_SLOTS = (0, 7, DISCARD_MAX_SLOTS - 1)

    def _dry_segment(self, failures: dict, key: str, label: str, fn) -> bool:
        """Chạy 1 đoạn của dry-run, ghi nhận lỗi theo phase thay vì raise."""
        try:
            fn()
            return True
        except Exception as exc:
            failures.setdefault(key, []).append(label)
            self.get_logger().error(f"[FAIL] dry-run {label}: {exc}")
            return False

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
        tx, ty, tz = square_to_grasp_pose(target_square, piece_type)
        tgt_approach = approach_tcp_z(target_square, tz)
        obj_id = self.piece_id_by_square.get(square)
        if obj_id and not SIMULATION_IGNORE_COLLISIONS:
            self.moveit2.remove_collision_object(id=obj_id)
            time.sleep(0.15)
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
                ok = self._dry_segment(
                    failures, "approach", f"{square}/approach",
                    lambda: self._move_to(x0, y0, src_approach))
                if not ok:
                    return
                grasp_q = self._current_tcp_quat()
                if not self._dry_segment(
                    failures, "descend_pick", f"{square}/descend",
                    lambda: self._move_vertical(x0, y0, z0, f"dry hạ gắp {square}", quat_xyzw=grasp_q)):
                    return
                if not self._dry_segment(
                    failures, "lift_pick", f"{square}/lift",
                    lambda: self._move_vertical(x0, y0, src_approach, f"dry nâng {square}", quat_xyzw=grasp_q)):
                    return
                if not self._dry_segment(
                    failures, "transfer", f"{square}->{target_square}",
                    lambda: self._move_to(tx, ty, tgt_approach)):
                    return
                place_q = self._current_tcp_quat()
                if not self._dry_segment(
                    failures, "descend_place", f"{target_square}/descend",
                    lambda: self._move_vertical(tx, ty, tz, f"dry hạ đặt {target_square}", quat_xyzw=place_q)):
                    return
                self._dry_segment(
                    failures, "lift_place", f"{target_square}/lift",
                    lambda: self._move_vertical(tx, ty, tgt_approach, f"dry nâng {target_square}", quat_xyzw=place_q))
            else:
                grasp_q = self._current_tcp_quat()
                self._dry_segment(
                    failures, "approach", f"{square}/approach",
                    lambda: self._plan_motion(
                        position=[x0, y0, src_approach],
                        target_link=END_EFFECTOR, tolerance_position=0.004))
                self._dry_segment(
                    failures, "descend_pick", f"{square}/descend",
                    lambda: self._plan_motion(
                        position=[x0, y0, z0], quat_xyzw=grasp_q,
                        target_link=END_EFFECTOR, tolerance_position=0.002,
                        tolerance_orientation=0.03, cartesian=True,
                        max_step=0.002, cartesian_fraction_threshold=0.999))
                self._dry_segment(
                    failures, "transfer", f"{square}->{target_square}",
                    lambda: self._plan_motion(
                        position=[tx, ty, tgt_approach],
                        target_link=END_EFFECTOR, tolerance_position=0.004))
                self._dry_segment(
                    failures, "descend_place", f"{target_square}/descend",
                    lambda: self._plan_motion(
                        position=[tx, ty, tz], quat_xyzw=grasp_q,
                        target_link=END_EFFECTOR, tolerance_position=0.002,
                        tolerance_orientation=0.03, cartesian=True,
                        max_step=0.002, cartesian_fraction_threshold=0.999))
        finally:
            if obj_id and not SIMULATION_IGNORE_COLLISIONS:
                self._restore_piece_collision(
                    square, obj_id,
                    self.piece_info_by_id[obj_id][0] if obj_id in self.piece_info_by_id else piece_type)

    def _dry_run_discard_slot(self, slot: int, failures: dict, execute: bool):
        """Mirror _do_discard: tới khu nghĩa địa + hạ/thả thẳng đứng."""
        xd, yd, _zd = discard_slot_pose(slot)
        dz = DISCARD_TCP_Z
        try:
            if execute:
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
            else:
                q = self._current_tcp_quat()
                self._dry_segment(
                    failures, "discard", f"slot{slot}/transfer",
                    lambda: self._plan_motion(
                        position=[xd, yd, dz + APPROACH_HEIGHT],
                        target_link=END_EFFECTOR, tolerance_position=0.004))
                self._dry_segment(
                    failures, "discard", f"slot{slot}/descend",
                    lambda: self._plan_motion(
                        position=[xd, yd, dz], quat_xyzw=q,
                        target_link=END_EFFECTOR, tolerance_position=0.002,
                        tolerance_orientation=0.03, cartesian=True,
                        max_step=0.002, cartesian_fraction_threshold=0.999))
        except Exception as exc:
            failures.setdefault("discard", []).append(f"slot{slot}: {exc}")

    def _check_reachability(self, _request, response):
        """Kiểm tra khả năng gắp thật theo 2 tầng.

        Tầng 1 (nhanh): position-only tới approach + pick của cả 64 ô từ
        state hiện tại — tín hiệu hồi quy nhanh, KHÔNG chứng minh pick được.
        Tầng 2 (deep): dry-run đúng chuỗi runtime (approach -> hạ/nâng
        Cartesian giữ quaternion đã chốt -> chuyển ô -> hạ/đặt) trên
        DEEP_CHECK_SQUARES + khu discard. Ở sim thì execute thật trên
        FakeSystem; ở robot thật chỉ plan-only. response.success=False nếu
        BẤT KỲ phase nào có lỗi (trước đây luôn True).
        """
        try:
            self._wait_for_motion_planner(timeout_sec=20.0)
            failures = {"approach": [], "pick": []}
            for square in chess.SQUARES:
                name = chess.square_name(square)
                piece = self.board.piece_at(square)
                x, y, pick_z = square_to_grasp_pose(
                    name, piece.symbol().lower() if piece else "p")
                # Pipeline gắp thật bỏ collision của CHÍNH quân mục tiêu trước
                # khi plan pre-grasp. Nếu để quân đó trong scene, vua/hậu ở d1/e1
                # làm goal bị coi là va chạm và cho ra false negative.
                obj_id = self.piece_id_by_square.get(name)
                piece_type = self.piece_info_by_id[obj_id][0] if obj_id else None
                if obj_id and not SIMULATION_IGNORE_COLLISIONS:
                    self.moveit2.remove_collision_object(id=obj_id)
                    time.sleep(0.15)
                try:
                    for phase, z in (("approach", approach_tcp_z(name, pick_z)), ("pick", pick_z)):
                        trajectory = self._plan_motion(
                            position=[x, y, z],
                            target_link=END_EFFECTOR,
                            tolerance_position=0.004,
                            cartesian=False,
                        )
                        if trajectory is None:
                            failures[phase].append(name)
                finally:
                    if obj_id and not SIMULATION_IGNORE_COLLISIONS:
                        self._restore_piece_collision(name, obj_id, piece_type)

            for phase, squares in failures.items():
                if squares:
                    self.get_logger().error(
                        f"[FAIL] reach nhanh {phase}: {64 - len(squares)}/64, "
                        f"không có plan: {', '.join(squares)}"
                    )
            if not failures["approach"] and not failures["pick"]:
                self.get_logger().info("[OK] reach nhanh: 64/64 approach, 64/64 pick")

            execute = SIMULATION_IGNORE_COLLISIONS
            self.get_logger().info(
                f"Deep-check chuỗi runtime trên {len(self.DEEP_CHECK_SQUARES)} ô "
                f"({'EXECUTE trên FakeSystem' if execute else 'plan-only, robot thật không di chuyển'})..."
            )
            deep: dict = {}
            for square in self.DEEP_CHECK_SQUARES:
                piece = self.board.piece_at(chess.parse_square(square))
                piece_type = piece.symbol().lower() if piece else "p"
                target = "e4" if square != "e4" else "d5"
                self._dry_run_square(square, piece_type, target, deep, execute)
            for slot in self.DEEP_CHECK_DISCARD_SLOTS:
                self._dry_run_discard_slot(
                    slot, deep,
                    execute and slot in self.DEEP_CHECK_DISCARD_EXEC_SLOTS)

            deep_total = sum(len(v) for v in deep.values())
            for phase, items in deep.items():
                if items:
                    self.get_logger().error(
                        f"[FAIL] deep-check {phase}: {len(items)} lỗi: "
                        f"{', '.join(items[:12])}"
                    )
            fast_fail = sum(len(v) for v in failures.values())
            if fast_fail == 0 and deep_total == 0:
                response.success = True
                self.get_logger().info("[OK] reachability PASS toàn bộ")
            else:
                response.success = False
                self.get_logger().error(
                    "[FAIL] reachability CÓ LỖI — xem các dòng [FAIL] trên để biết ô/phase"
                )
            response.message = (
                f"nhanh approach {64-len(failures['approach'])}/64, "
                f"pick {64-len(failures['pick'])}/64; "
                f"deep-check lỗi {deep_total} "
                f"({'EXECUTE' if execute else 'plan-only'}). "
                + ("PASS toàn bộ." if response.success
                   else "CÓ LỖI — xem terminal để biết ô/phase.")
            )
        except Exception as exc:
            response.success = False
            response.message = f"Reachability check thất bại: {exc}"
        return response

    def _do_pick_place(self, from_sq: str, to_sq: str, piece_type: str,
                       placed_piece_type: str | None = None):
        x0, y0, z0 = square_to_grasp_pose(from_sq, piece_type)
        x1, y1, z1 = square_to_grasp_pose(to_sq, placed_piece_type or piece_type)
        gripper_open = PIECE_SPECS[piece_type].gripper_open
        # Ở c1/d1/e1/f1, nâng thẳng đứng (Cartesian) lên cao độ vận chuyển
        # chuẩn 0.125 m là vô nghiệm IK. Vì vậy arm chỉ nâng thẳng đến
        # approach riêng của ô nguồn (xem approach_tcp_z), rồi dùng
        # position-only OMPL để rời vùng gần đế robot sang approach của ô đích.
        source_approach_z = approach_tcp_z(from_sq, z0)
        target_approach_z = approach_tcp_z(to_sq, z1)
        obj_id = None
        attached = False

        try:
            # Bỏ collision của quân mục tiêu TRƯỚC KHI plan pre-grasp. Với quân
            # cao (nhất là vua/hậu), để object trong scene sẽ khiến goal phía
            # trên nó bị MoveIt loại là collision, dù đó là quân sắp được gắp.
            self._set_gripper(gripper_open)
            obj_id = self._take_piece_from_world(from_sq)
            self._move_to(x0, y0, source_approach_z)
            # Chốt orientation NGAY sau approach: mọi đoạn vertical của lượt
            # gắp giữ cùng 1 quaternion. Không để mỗi _move_vertical tự đọc
            # TF (OMPL có thể đã chọn orientation khác ở lần plan trước).
            grasp_q = self._current_tcp_quat()
            self._move_vertical(x0, y0, z0, "hạ gắp", quat_xyzw=grasp_q)
            self._set_gripper(0.0)
            self._attach_piece(obj_id, piece_type, (x0, y0, z0))
            attached = True
            self._move_vertical(x0, y0, source_approach_z, "nâng sau gắp", quat_xyzw=grasp_q)

            self._move_to(x1, y1, target_approach_z)
            place_q = self._current_tcp_quat()
            self._move_vertical(x1, y1, z1, "hạ đặt", quat_xyzw=place_q)
            self._set_gripper(gripper_open)
            self._detach_piece(
                obj_id, to_sq, (x1, y1, BOARD_Z), placed_piece_type or piece_type
            )
            attached = False
            self._move_vertical(x1, y1, target_approach_z, "nâng sau đặt", quat_xyzw=place_q)
        except Exception:
            if obj_id is not None and not attached:
                # Failure trước attach: collision và mapping quay về nguyên trạng.
                self._restore_piece_to_world(from_sq, obj_id, piece_type)
            elif attached:
                self.get_logger().error(
                    "[FAIL] quân đang attached vào gripper sau lỗi; không gửi ACK. "
                    "Hãy mở gripper/đưa arm về safe pose rồi chạy lại nước này."
                )
            raise

    def _do_discard(self, square: str):
        """Quân bị ăn: pick tại chỗ, mang sang khu 'nghĩa địa'. to_square=None vì
        quân này không còn thuộc bàn cờ (không tham gia mapping ô -> id nữa)."""
        piece = self.board.piece_at(chess.parse_square(square))
        piece_type = piece.symbol().lower()
        x0, y0, z0 = square_to_grasp_pose(square, piece_type)
        xd, yd, zd = discard_slot_pose(self.discard_count)
        self.discard_count += 1
        discard_tcp_z = DISCARD_TCP_Z
        source_approach_z = approach_tcp_z(square, z0)
        obj_id = None
        attached = False
        try:
            self._set_gripper(PIECE_SPECS[piece_type].gripper_open)
            obj_id = self._take_piece_from_world(square)
            self._move_to(x0, y0, source_approach_z)
            grasp_q = self._current_tcp_quat()
            self._move_vertical(x0, y0, z0, "hạ gắp quân bị ăn", quat_xyzw=grasp_q)
            self._set_gripper(0.0)
            self._attach_piece(obj_id, piece_type, (x0, y0, z0))
            attached = True
            self._move_vertical(x0, y0, source_approach_z, "nâng quân bị ăn", quat_xyzw=grasp_q)
            # Move position-only tới khu discard để có thể vừa rời c1--f1 vừa
            # đổi cao độ; nâng thẳng tới 0.125 m ngay tại mép gần là vô nghiệm.
            self._move_to(xd, yd, discard_tcp_z + APPROACH_HEIGHT)
            drop_q = self._current_tcp_quat()
            self._move_vertical(xd, yd, discard_tcp_z, "hạ thả quân bị ăn", quat_xyzw=drop_q)
            self._set_gripper(PIECE_SPECS[piece_type].gripper_open)
            self._detach_piece(obj_id, None, (xd, yd, zd), piece_type)
            attached = False
            self._move_vertical(xd, yd, discard_tcp_z + APPROACH_HEIGHT, "nâng sau thả quân bị ăn", quat_xyzw=drop_q)
        except Exception:
            if obj_id is not None and not attached:
                self._restore_piece_to_world(square, obj_id, piece_type)
            elif attached:
                self.get_logger().error("[FAIL] quân bị ăn vẫn attached sau lỗi; không gửi ACK.")
            raise

    def _castling_rook_squares(self, uci: str):
        mapping = {
            "e1g1": ("h1", "f1"), "e1c1": ("a1", "d1"),
            "e8g8": ("h8", "f8"), "e8c8": ("a8", "d8"),
        }
        return mapping[uci]

    # ---------------- Robot helpers ----------------

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
                raise RuntimeError("MoveIt execution timeout")
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
            raise RuntimeError(f"Không tìm được position-only plan tới {(x, y, z)}")
        self._execute_and_wait(self.moveit2, trajectory)

    def _move_to_home(self, label: str):
        """Joint PTP về SRDF pose `arm_group/up` trước/sau lượt robot."""
        trajectory = self._plan_motion(
            joint_positions=HOME_JOINTS,
            joint_names=JOINT_NAMES,
            tolerance_joint_position=0.03,
            cartesian=False,
        )
        if trajectory is None:
            raise RuntimeError(f"Không tìm được joint PTP: {label}")
        self._execute_and_wait(self.moveit2, trajectory)

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

    def _move_vertical(self, x, y, z, step_name: str, quat_xyzw=None):
        """Đi thẳng đứng bằng compute_cartesian_path, không để OMPL lách qua
        bàn/quân trong đoạn hạ hoặc nâng.

        quat_xyzw: orientation giữ suốt đoạn đi. Nên truyền quaternion đã chốt
        sau approach (xem _current_tcp_quat); None = đọc TF hiện tại (giữ hành
        vi cũ cho caller đơn lẻ). Nếu Cartesian path không hoàn thành 100%,
        raise rõ ràng thay vì fallback âm thầm (trừ khi
        ALLOW_CARTESIAN_FALLBACK=True được bật tường minh cho demo).
        """
        q = quat_xyzw if quat_xyzw is not None else self._current_tcp_quat()
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
        if trajectory is None:
            if SIMULATION_IGNORE_COLLISIONS and ALLOW_CARTESIAN_FALLBACK:
                # Opt-in tường minh cho demo: cho phép planner position-only
                # chọn orientation trung gian để hoàn tất ván. Pick lúc này
                # KHÔNG đi thẳng đứng thật — chỉ dùng để demo flow, không
                # dùng với robot thật.
                self.get_logger().warning(
                    f"Cartesian không đủ tại {step_name}; dùng position-only fallback tới {(x, y, z)}"
                )
                self._move_to(x, y, z)
                return
            raise RuntimeError(
                f"Không có Cartesian path an toàn khi {step_name} tới {(x, y, z)} "
                f"(quat giữ {[round(v, 3) for v in q]}). "
                f"Hãy chạy /chess/check_reachability để xem ô/phase lỗi, "
                f"kiểm tra approach offset và orientation sau approach."
            )
        self._execute_and_wait(self.moveit2, trajectory)

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
