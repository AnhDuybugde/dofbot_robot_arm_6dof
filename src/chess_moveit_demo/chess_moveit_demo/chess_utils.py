"""Tiện ích chuyển đổi giữa toạ độ bàn cờ (a1-h8) và toạ độ Cartesian của robot,
cùng thông số vật lý (chiều cao gắp, độ mở gripper) cho từng loại quân cờ.

Toạ độ được nội suy tuyến tính từ 1 điểm gốc (tâm ô a1) + kích thước ô.
CHỈNH các hằng số bên dưới theo bàn cờ ảo bạn dựng trong RViz / scene thật của bạn.
"""

from dataclasses import dataclass

# ==== THAM SỐ CALIBRATION (chỉnh theo bàn cờ mô phỏng của bạn) ====
# Bàn được tịnh tiến xa theo +X để các ô hàng 1--2 không chồng lên đế robot.
# a1 = (0.095, -0.095); h8 = (0.284, 0.094); riêng e2 = (0.122, 0.013).
# Z giữ nguyên. Khi dùng bàn thật, calibrate lại theo vị trí vật lý.
BOARD_ORIGIN = (0.095, -0.095, 0.005)  # toạ độ tâm ô a1 so với base_link (m)
SQUARE_SIZE = 0.027                     # cạnh ô (m)
BOARD_Z = 0.005                         # cao độ mặt bàn cờ so với base_link (m)
APPROACH_HEIGHT = 0.070                 # độ cao approach/lift phía trên quân cờ (m)
# Bốn ô giữa của hàng 1 nằm gần đế Dofbot nhất (X nhỏ nhất).  TCP ở
# PICK_TCP_Z + 70 mm rơi vào vùng không có IK tại các ô này, trong khi TCP ở
# 100 mm vẫn có IK.  Chỉ hạ *điểm approach* tại đây; khi mang quân theo phương
# ngang, executor vẫn nâng về độ cao chuẩn APPROACH_HEIGHT để tránh các quân
# còn lại.
NEAR_EDGE_APPROACH_HEIGHT = 0.045
LOW_APPROACH_SQUARES = frozenset({"c1", "d1", "e1", "f1"})
# Đo IK thực tế (KDL position-only qua /compute_ik, 06/2026): hai ô file giữa
# ở rank 1 (x=0.095, gần đế robot nhất, gần như chính diện) KHÔNG có nghiệm
# ở vùng cao:
#   e1 (0.095, 0.013): OK ở z<=0.070, FAIL mọi z từ 0.080 tới 0.135
#   d1 (0.095, -0.014): OK ở z<=0.080, FAIL từ 0.090 trở lên
# c1/f1 (lệch tâm) OK ở 0.100. Vì vậy e1/d1 dùng offset approach riêng thấp
# hơn (giá trị = cao độ TCP tuyệt đối đã verify có IK):
#   e1 -> 0.070, d1 -> 0.080. Quân mục tiêu đã được gỡ khỏi scene trước khi
# plan pre-grasp nên approach thấp không gây va chạm giả với chính nó.
SQUARE_APPROACH_OFFSET = {"e1": 0.015, "d1": 0.025}
# Cao độ TCP (Gripping_point_Link) tại lúc ngón kẹp đúng quân. Đây là tham số
# calibration của ROBOT, không phải chiều cao của quân.  Bắt đầu ở 55 mm để
# tránh TCP chạm bàn; hãy tune +/- 2--3 mm trên RViz rồi mới dùng robot thật.
PICK_TCP_Z = 0.055
DISCARD_TCP_Z = PICK_TCP_Z

# Chế độ tạm thời chỉ dành cho RViz + ros2_control FakeSystem: không đưa bàn
# và quân vào collision checking. Dùng để xác nhận toàn bộ chuỗi game -> arm
# -> ACK trước khi tinh chỉnh hình học gripper. PHẢI đặt lại False trước khi
# điều khiển robot thật.
SIMULATION_IGNORE_COLLISIONS = True

FILES = "abcdefgh"
RANKS = "12345678"


@dataclass
class PieceSpec:
    pickup_height: float   # chiều cao collision/visual của quân từ mặt bàn (m)
    gripper_open: float    # độ mở gripper (m hoặc rad, tuỳ cấu hình gripper của bạn)


# Chiều cao/gripper xấp xỉ cho bộ cờ Staunton chuẩn - chỉnh theo bộ cờ thật/mô phỏng của bạn
PIECE_SPECS = {
    "p": PieceSpec(pickup_height=0.035, gripper_open=0.018),  # pawn
    "n": PieceSpec(pickup_height=0.045, gripper_open=0.020),  # knight
    "b": PieceSpec(pickup_height=0.055, gripper_open=0.018),  # bishop
    "r": PieceSpec(pickup_height=0.045, gripper_open=0.022),  # rook
    "q": PieceSpec(pickup_height=0.070, gripper_open=0.020),  # queen
    "k": PieceSpec(pickup_height=0.075, gripper_open=0.020),  # king
}

DISCARD_ZONE = (0.16, 0.14, 0.005)  # vị trí "nghĩa địa" quân bị ăn, so với base_link
DISCARD_SPACING = 0.025


def square_to_xy(square: str):
    """'e4' -> (x, y) tâm ô, chưa cộng offset lệch tâm quân."""
    file_idx = FILES.index(square[0])
    rank_idx = RANKS.index(square[1])
    x = BOARD_ORIGIN[0] + rank_idx * SQUARE_SIZE
    y = BOARD_ORIGIN[1] + file_idx * SQUARE_SIZE
    return x, y


def square_to_grasp_pose(square: str, piece_type: str, offset_xy=(0.0, 0.0)):
    """Trả về (x, y, z) TCP khi gắp, cộng offset lệch tâm nếu có.

    `z` là PICK_TCP_Z đã hiệu chuẩn cho Gripping_point_Link; không suy ra từ
    chiều cao quân.  Chiều cao quân chỉ phục vụ visual/collision và offset khi
    attach object vào gripper.

    Cộng offset lệch tâm nếu có
    (offset_xy mô phỏng vai trò của position-regression model trong bản gốc;
    ở chế độ giả lập không có camera thì để mặc định (0, 0))."""
    x, y = square_to_xy(square)
    x += offset_xy[0]
    y += offset_xy[1]
    return x, y, PICK_TCP_Z


def approach_tcp_z(square: str, pick_tcp_z: float = PICK_TCP_Z) -> float:
    """Cao độ TCP tại pre-grasp/pre-place của một ô.

    Hàng 1 là mép gần robot vì rank tăng theo +X.  Hàm này là calibration
    theo ô, tách bạch với ``APPROACH_HEIGHT`` (cao độ an toàn để vận chuyển).

    e1/d1 có override riêng trong ``SQUARE_APPROACH_OFFSET`` vì KDL không giải
    được pose cao ở hai ô này (xem chú thích ở hằng số).
    """
    if square in SQUARE_APPROACH_OFFSET:
        return pick_tcp_z + SQUARE_APPROACH_OFFSET[square]
    height = (
        NEAR_EDGE_APPROACH_HEIGHT
        if square in LOW_APPROACH_SQUARES
        else APPROACH_HEIGHT
    )
    return pick_tcp_z + height


def discard_slot_pose(index: int):
    x, y, z = DISCARD_ZONE
    x += (index // 8) * DISCARD_SPACING
    y += (index % 8) * DISCARD_SPACING
    return x, y, z
