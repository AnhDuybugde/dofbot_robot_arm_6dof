"""Tiện ích chuyển đổi giữa toạ độ bàn cờ (a1-h8) và toạ độ Cartesian của robot,
cùng thông số vật lý (chiều cao gắp, độ mở gripper) cho từng loại quân cờ.

Toạ độ được nội suy tuyến tính từ 1 điểm gốc (tâm ô a1) + kích thước ô.
CHỈNH các hằng số bên dưới theo bàn cờ ảo bạn dựng trong RViz / scene thật của bạn.
"""

from dataclasses import dataclass
import math

# ==== BÀN CỜ THẬT 24 cm (đơn vị mét trong MoveIt) — CHỐT ====
# Mapping: X=rank, Y=file (giữ hàng 1 gần robot đã verify IK, không xoay).
BOARD_SIZE_X = 0.240
BOARD_SIZE_Y = 0.240
BOARD_THICKNESS = 0.020
BOARD_TOP_Z = 0.005
BOARD_CENTER_X = 0.2015
BOARD_CENTER_Y = -0.0005
BOARD_CENTER_Z = -0.005
BOARD_CENTER = (0.2015, -0.0005)
BOARD_COLLISION_CENTER = (0.2015, -0.0005, -0.005)
BOARD_COLLISION_SIZE = (0.240, 0.240, 0.020)
BOARD_ORIGIN = (0.1105, -0.0915, 0.005)  # tâm ô a1
SQUARE_SIZE = 0.026
BOARD_BORDER = 0.016
# Alias tương thích code cũ dùng BOARD_Z làm mặt bàn.
BOARD_Z = BOARD_TOP_Z
# Clearance chung 65mm: approach = lift = retreat = grasp + 0.065.
VERTICAL_CLEARANCE = 0.065
APPROACH_HEIGHT = VERTICAL_CLEARANCE
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
#   e1 -> 0.070, d1 -> 0.080. Khi gắp, ACM chỉ mở contact tạm thời giữa quân
# mục tiêu với link kẹp/mặt bàn; các collision khác vẫn được kiểm tra.
SQUARE_APPROACH_OFFSET = {"e1": 0.015, "d1": 0.025}
# PICK_TCP_Z / DISCARD_TCP_Z / PIECE_GRIP_Z định nghĩa ở cụm bàn thật bên dưới
# (TCP = TOP + GRASP_H + OFFSET), không dùng giá trị demo 0.055 nữa.

# Hai cờ này CỐ Ý độc lập. Bật collision không được làm deep-check đổi từ
# EXECUTE sang plan-only, vì cần kiểm tra đúng state chaining trên FakeSystem.
# Khi chuyển sang arm thật, đặt REACHABILITY_EXECUTE_ON_FAKESYSTEM=False.
COLLISION_ENABLED = True
REACHABILITY_EXECUTE_ON_FAKESYSTEM = False

# DEPRECATED (giữ để tương thích import): mọi Cartesian fail đều raise loud,
# không fallback position-only kể cả khi cờ này True — fallback lúc ATTACHED
# làm rơi/lệch quân mà flow vẫn attach như thành công.
ALLOW_CARTESIAN_FALLBACK = False

# ==== BUDGET PLANNING CỨNG (P1-P4 review) ====
# Một timeout ngoài (vd. GRASP_SEARCH_TIMEOUT_SEC) không đủ vì candidate ×
# yaw × seed × comp × timeout lồng nhau vẫn kéo dài hàng chục phút. Mọi service
# planning (IK/OMPL/Cartesian/FK/scene/ACM) chỉ được chờ
# timeout = min(default_timeout, remaining_của_deadline_chung).
class PlanningBudgetExceeded(RuntimeError):
    """Hết budget planning của một nước — dừng tìm candidate, KHÔNG phải lỗi
    robot. Caller chuẩn hoá thành NACK/PLACE_PRECHECK_FAILED với lý do budget."""


# Budget planning cho phase TÌM candidate một nước. ĐO THỰC trên sim
# (10/09/2026, máy VM + 32 collision objects):
#   - budget 25s: e2->e4 FAIL oan ở offset 8/21 (planning cần ~43s).
#   - budget 50s: PASS, planning ~43s, offset (0.006,0.0), err 1.2mm/tilt 8.7°.
#   - budget 45s: PASS nhưng planning ~45.5s (11 offset, OMPL ngẫu nhiên đổi
#     offset thắng sang (0.006,-0.006)) -> margin ~0, flaky. CHỐT 50s.
# Chỉ giới hạn PLANNING (plan-only), không giới hạn EXECUTION vật lý.
MOVE_PLANNING_BUDGET_SEC = 90.0
# Budget planning cho replan runtime khi cache invalid (đã ATTACHED hoặc sắp
# execute: ít candidate hơn vì đi từ current state, không tìm offset).
# Chưa đo case này -> giữ 30s gốc, không siết theo.
RUNTIME_REPLAN_BUDGET_SEC = 30.0
# Timeout từng request: OMPL/Cartesian/IK về giá trị gốc (5/5/1.5s).
# ĐO 10/09/2026: siết 3/3/1.0s làm transfer/place rớt vào nhánh tilt xấu dưới
# scene strict deny-all (không còn legacy entry lenient) — comp loop phân kỳ
# err 8->17mm + branch-jump. Vài giây/request rẻ hơn nước fail. FK/scene/ACM
# là roundtrip thuần, giữ siết (1.5/1.5/3.0) vì không ảnh hưởng chất lượng.
OMPL_PLANNING_TIMEOUT_SEC = 5.0
CARTESIAN_PLANNING_TIMEOUT_SEC = 5.0
IK_WAIT_TIMEOUT_SEC = 1.5
FK_SERVICE_TIMEOUT_SEC = 1.5
SCENE_SERVICE_TIMEOUT_SEC = 1.5
ACM_APPLY_TIMEOUT_SEC = 3.0

# ==== DENY-ALL: không còn ngoại lệ tiếp xúc ACM (đo 10/09/2026) ====
# MoveIt Humble ngó lơ entry ACM MỚI qua mọi đường apply (diff/full/topic/
# attach/monitored-topic đều trả success nhưng không land; chỉ flip giá trị
# cũ land — cùng triệu chứng upstream moveit#3527). Entry một khi mất thì
# không thể tạo lại cho tới khi restart move_group. Vì vậy pipeline KHÔNG
# được phụ thuộc bất kỳ exception tiếp xúc nào; mọi goal/state phải
# geometrically-valid dưới default-deny, an toàn do gate hình học strict gánh.
# - CONTACT_HOVER_M: gắp/đặt hover bấy nhiêu trên điểm chạm danh nghĩa để
#   goal descend và start lift/start validity không dính contact (attached-vs-
#   world CÓ check; world-vs-world không check). Trong dung sai 5mm; attach
#   đo T_tcp_piece từ TF sống nên hover tự nhất quán precheck/runtime.
# - GRASP_PROUD_MARGIN_RAD: siết cuối lùi lại bấy nhiêu so với close_rad để
#   goal close (planned qua MoveIt) không chạm mặt quân. Sim grasp là
#   bookkeeping (attach + visual chaining, FakeSystem không có vật lý) nên
#   proud không ảnh hưởng kết quả; ROBOT THẬT cần thêm stage siết direct
#   (GripperCommand thẳng, unplanned -> không cần ACM bao giờ) với mapping
#   đo thật (P8), chưa làm ở đây.
CONTACT_HOVER_M = 0.003
GRASP_PROUD_MARGIN_RAD = 0.0

# ==== ORIENTATION DISCIPLINE (step-3, đo thực strict deny-all) ====
# TCP convention: FK tại HOME cho tilt ĐÚNG 90° (Z TCP nằm ngang khi tay ở
# pose up). Mọi motion mang (approach/lift/transfer/pre-place/descend) phải
# giữ TCP quanh mốc này; lệch nhiều = KDL/OMPL whim nhánh lật (đo thực IK
# nghiệm tilt 133° dù seed thẳng đứng, transfer cuối 122-134° rồi place chết
# ở gate 26°). Lọc tilt NGAY tại IK target (rẻ, ms) thay vì để tilt mang
# suốt chuỗi rồi comp phân kỳ:
# - SEED_IK_TILT_TOL_RAD: IK target lệch quá mốc này -> bỏ seed (thử seed kế).
# - TRANSFER_TILT_TOL_RAD: transfer cuối lệch quá mốc này -> loại offset ngay
#   (place không cứu được — đã chứng minh 3 runs), khỏi đốt budget comp.
# Nằm giữa 11° desired và 26° hard của gate đặt.
TCP_GRASP_TILT_RAD = math.pi / 2
SEED_IK_TILT_TOL_RAD = 0.30
TRANSFER_TILT_TOL_RAD = 0.35
# Gate tilt gắp (đo thực compounding: grasp 17° + seed 17° -> piece 35° chết
# gate 26°; comp dịch tịnh không sửa được xoay). Grasp lệch quá mốc này ->
# loại offset ngay (rẻ, trước sweep/chain đắt). 12° + seed 17° = worst 29°,
# typical ~10° ≈ 11° desired; gate đặt 26° hard chốt cuối.
GRASP_TILT_TOL_RAD = 0.21

# Tổng thời gian tối đa cho tìm candidate gắp 1 ô (Fix 6): thử offset mà
# không trần thời gian có thể treo lượt đi khi scene khó. Hết trần -> raise
# để NACK thay vì thử mãi.
GRASP_SEARCH_TIMEOUT_SEC = 120.0
# Bán kính collision nhỏ nhất (tốt p = 8.5mm). Candidate offset vượt quá
# bán kính quân mục tiêu thì ngón kẹp chắc chắn trượt tâm (điều kiện hình học
# tối thiểu; FakeSystem không kiểm chứng tiếp xúc vật lý thật). Giữ 8.5mm để
# mọi loại quân đều an toàn.
GRASP_MAX_OFFSET = 0.0085

FILES = "abcdefgh"
RANKS = "12345678"


@dataclass
class PieceSpec:
    pickup_height: float   # chiều cao collision/visual của quân từ mặt bàn (m)
    gripper_open_rad: float = 0.0   # góc mở kẹp khi tiếp cận/gắp loại quân này (rad)
    gripper_close_rad: float = 1.57  # góc khép kẹp khi mang loại quân này (rad)
    # Hai giai đoạn kẹp (SIM default, chờ bảng FK đo thật P8 mới thành số vật
    # lý): khép SƠ BỘ ở cao độ approach trước descend + mở VỪA ĐỦ khi release
    # (không mở hết cỡ). Phải thỏa open <= preclose <= close và
    # open <= release <= close (validate ở dưới, fail-loud lúc import).
    # Override của user 10/09/2026: preclose 75° (1.309) và close 80° (1.396)
    # đồng nhất mọi loại quân. LƯU Ý: descend chạy ở độ rộng preclose — 75° hẹp
    # hơn 40° cũ nhiều, phải lọt quân to nhất (vua Ø18-20mm) + sai số vị trí,
    # không thì Cartesian descend rớt fraction; close đồng nhất có thể ôm hờ vua
    # (sim ok, bookkeeping) hoặc xuyên tốt (goal invalid). Sim test sẽ trả lời
    # vì chưa có bảng map rad->mm (P8).
    gripper_preclose_rad: float = math.radians(75.0)
    gripper_release_rad: float = 0.7


# Kích thước danh nghĩa quân thật (visual/RViz). Collision dùng bảng riêng
# PIECE_COLLISION bên dưới (có margin +3mm cao, +0.5-1mm radius).
PIECE_PHYSICAL = {
    "p": {"height": 0.023, "diameter": 0.015},
    "r": {"height": 0.027, "diameter": 0.016},
    "n": {"height": 0.031, "diameter": 0.017},
    "b": {"height": 0.035, "diameter": 0.017},
    "q": {"height": 0.041, "diameter": 0.018},
    "k": {"height": 0.047, "diameter": 0.018},
}
# Collision từng loại (margin an toàn so với physical).
PIECE_COLLISION = {
    "p": {"radius": 0.0085, "height": 0.026},
    "r": {"radius": 0.0090, "height": 0.030},
    "n": {"radius": 0.0095, "height": 0.034},
    "b": {"radius": 0.0095, "height": 0.038},
    "q": {"radius": 0.0100, "height": 0.044},
    "k": {"radius": 0.0100, "height": 0.050},
}
# Góc kẹp liên tục theo rad (KHÔNG binary): mở 0.0 = xòe hết cỡ cho hành
# lang approach/descend rộng nhất; khép tới đúng mặt quân, không khép mù
# 1.57 mọi loại (xuyên quân trong sim, bóp méo/mất lực trên robot thật).
# Giá trị close là default SIM (chưa phải calibration vật lý): tốt nhỏ nhất
# nên khép ít nhất; các quân lớn giữ 1.57 đã chứng minh plan/execute được.
# Đừng suy mm->rad tuyến tính (khớp mimic phi tuyến); bảng FK đo thật là P8.
# Override user 10/09/2026: close 80° (1.396) đồng nhất mọi loại (thử nghiệm;
# xem lưu ý ở PieceSpec.preclose về descend/siết).
PIECE_SPECS = {
    "p": PieceSpec(pickup_height=0.026, gripper_open_rad=0.0, gripper_close_rad=1.396),
    "r": PieceSpec(pickup_height=0.030, gripper_open_rad=0.0, gripper_close_rad=1.396),
    "n": PieceSpec(pickup_height=0.034, gripper_open_rad=0.0, gripper_close_rad=1.396),
    "b": PieceSpec(pickup_height=0.038, gripper_open_rad=0.0, gripper_close_rad=1.396),
    "q": PieceSpec(pickup_height=0.044, gripper_open_rad=0.0, gripper_close_rad=1.396),
    "k": PieceSpec(pickup_height=0.050, gripper_open_rad=0.0, gripper_close_rad=1.396),
}

# "Nghĩa địa" quân bị ăn: lưới 4x4=16 slot cạnh bàn phía -Y (bên file a),
# x nằm trong tầm file bàn cờ đã test, y chỉ ngoài mép bàn ~1 ô. Bán kính lớn
# nhất (slot 15) ~0.30 m, tương đương góc h8 đã pass 64/64.
# Bản cũ xếp 1 hàng dọc +Y vô hạn (slot thứ 6 đã y=0.265, ngoài tầm với thật:
# Cartesian chỉ được 40% rồi OMPL cũng bó tay) — lỗi này làm treo game vì quân
# đang attached mà không có ACK.
DISCARD_ORIGIN = (0.09, -0.13, BOARD_Z)
DISCARD_COLS = 4
DISCARD_ROWS = 4
DISCARD_DX = 0.033
DISCARD_DY = 0.033
DISCARD_MAX_SLOTS = DISCARD_COLS * DISCARD_ROWS  # 16

# Điểm kẹp so với mặt bàn (chưa gồm offset TCP->điểm tiếp xúc ngón).
PIECE_GRASP_HEIGHT = {
    "p": 0.011, "r": 0.014, "n": 0.015, "b": 0.017, "q": 0.020, "k": 0.022,
}
# Offset TCP->điểm tiếp xúc: 30mm CHƯA đủ bằng chứng (mesh/joint/orientation
# khi đóng đều ảnh hưởng) nên chỉ là estimate, không phải thông số chính thức.
TCP_TO_CONTACT_OFFSET_Z_ESTIMATE = 0.030
TCP_OFFSET_CALIBRATED = False
# Mốc RViz neo theo TCP tốt cũ đã chạy được (55mm): offset tương thích
# 55-5-11 = 39mm. Dùng tạm cho tới khi đo trên robot thật:
#   OFFSET = measured_tcp_z - BOARD_TOP_Z - PIECE_GRASP_HEIGHT[type].
TCP_TO_CONTACT_OFFSET_Z_SIM = 0.039
PAWN_TCP_Z_REFERENCE = 0.055
PICK_TCP_Z = PAWN_TCP_Z_REFERENCE
DISCARD_TCP_Z = PICK_TCP_Z
# Bảng SIM nhất quán với pawn 55mm (không phải calibration vật lý):
# p=55, r=58, n=59, b=61, q=64, k=66mm.
PIECE_GRIP_Z_SIM = {
    "p": 0.055, "r": 0.058, "n": 0.059, "b": 0.061, "q": 0.064, "k": 0.066,
}
PIECE_GRIP_Z = dict(PIECE_GRIP_Z_SIM)
# Giới hạn khớp kẹp Rlink1_Joint (URDF: 0..1.57). _set_gripper nhận MỌI giá
# trị rad liên tục trong đoạn này (fail-loud ngoài đoạn, không clamp câm).
# Đừng suy mm->rad tuyến tính (khớp mimic phi tuyến); muốn đặt theo mm phải
# có bảng FK gripper_width_to_joint_angle đo thật (P8).
GRIPPER_OPEN_RAD = 0.0
GRIPPER_CLOSED_RAD = 1.57
# Release HẸP fallback khi mở chuẩn (0.7) quẹt quân lân cận ở ô đích đông
# (đo thực f3: mở tới 1.1 đã chạm tốt f2/g2, 1.2 còn sạch; ô cờ 26mm).
# Ngón ở hold đã PROUD (không chạm quân) nên mở thêm ~11° rồi detach
# bookkeeping là đủ trong sim (FakeSystem không vật lý). Chỉ dùng khi mở
# chuẩn plan-fail; mở rộng được thì vẫn ưu tiên mở rộng.
GRIPPER_RELEASE_NARROW_RAD = 1.2
# Trần tốc độ cho fast_demo TRONG SIM (FakeSystem không tải). Robot thật
# không bao giờ vượt 0.25 (gate cứng). Chỉ có hiệu lực khi đồng thời
# sim_allow_execute=True và fast_demo=True; mặc định tắt (=0.25 như cũ).
SIM_FAST_VELOCITY_CAP = 0.6
# 3 phase widths tương lai: cần bảng FK/calibration joint->khoảng cách mặt
# trong finger trước (đo trong RViz rồi hiệu chỉnh servo thật).
HIGH_APPROACH_INNER_WIDTH = 0.020
NARROW_DESCENT_INNER_WIDTH = 0.014
FINAL_GRASP_INNER_WIDTH = 0.012
HIGH_APPROACH_WIDTH = HIGH_APPROACH_INNER_WIDTH
NARROW_DESCENT_WIDTH = NARROW_DESCENT_INNER_WIDTH
FINAL_GRASP_WIDTH = FINAL_GRASP_INNER_WIDTH
FINGER_THICKNESS = 0.006
# Fail-loud lúc import: trình tự 2 giai đoạn chỉ đúng khi góc giữa (sơ bộ /
# release) nằm trong đoạn [mở, khép cuối]. Đảo thứ tự (vd. preclose > close)
# sẽ thành "sơ bộ khép chặt hơn cả siết cuối" mà không ai biết.
for _pt, _spec in PIECE_SPECS.items():
    if not (GRIPPER_OPEN_RAD <= _spec.gripper_open_rad <= GRIPPER_CLOSED_RAD
            and GRIPPER_OPEN_RAD <= _spec.gripper_close_rad <= GRIPPER_CLOSED_RAD
            and _spec.gripper_open_rad <= _spec.gripper_preclose_rad <= _spec.gripper_close_rad
            and _spec.gripper_open_rad <= _spec.gripper_release_rad <= _spec.gripper_close_rad):
        raise ValueError(
            f"PIECE_SPECS[{_pt!r}] sai thứ tự góc kẹp 2 giai đoạn: "
            f"open={_spec.gripper_open_rad} preclose={_spec.gripper_preclose_rad} "
            f"release={_spec.gripper_release_rad} close={_spec.gripper_close_rad}")
del _pt, _spec
# Motion: bỏ RETREAT 40mm, dùng chung clearance 65mm cho cả 3 bước.
# Cartesian step 1mm (thay vi 2mm): finger hep preclose 75 do suot ngang
# quan cao lang gieng (vd. vua e1 cach e2 26mm); step 2mm co the nhay qua
# diem suot giua 2 waypoint trong khi revalidate joint-interp bat duoc.
# Planner thay duoc thi precheck loai offset do ngay, khoi fail o execution.
CARTESIAN_EEF_STEP = 0.001
MIN_CARTESIAN_FRACTION = 0.98

# ==== HẠ TẦNG / READY GATE (TODO-1) ====
# PlanningScene chuẩn: 1 board + 32 quân = 33 world objects, attached rỗng.
EXPECTED_WORLD_OBJECTS = 33
EXPECTED_PIECE_OBJECTS = 32
SYSTEM_READY_TOPIC = "/chess/system_ready"
# /joint_states coi là stale nếu không có mẫu mới trong cửa sổ này.
JOINT_STATE_MAX_AGE_SEC = 1.0
# Thời gian tối đa chờ hạ tầng READY sau khi dựng scene (log rõ điều kiện fail).
# move_group trên máy yếu cần vài phút để load xong pipeline mới serve service
# (advertise sớm nhưng request tới sớm sẽ timeout), nên trần phải rộng và init
# retry vòng lặp thay vì thử một lần.
SYSTEM_READY_TIMEOUT_SEC = 600.0
SYSTEM_READY_RETRY_SEC = 10.0
# Sai số cho phép khi verify pose scene đọc lại (tâm cylinder so với kỳ vọng).
SCENE_VERIFY_POS_TOL_M = 0.005
# Joint limits Dofbot 5-DOF (rad) để gate candidate quá sát limit (TODO-2).
# Lấy từ mô tả URDF/SRDF; margin an toàn áp khi chấm candidate.
DOFBOT_JOINT_LIMITS = {
    "arm1_Joint": (-2.61799, 2.61799),
    "arm2_Joint": (-1.57080, 1.57080),
    "arm3_Joint": (-1.57080, 1.57080),
    "arm4_Joint": (-1.57080, 1.57080),
    "arm5_Joint": (-2.09440, 2.09440),
}
# Planner domain uses the physical five-joint limits.  Arm5 around zero is a
# scoring/seed preference only; constraining it in the model made valid
# vertical pick trajectories unreachable.
ARM5_CAGE_RAD = math.radians(20.0)  # +-0.349
CHESS_JOINT_LIMITS = {
    "arm1_Joint": DOFBOT_JOINT_LIMITS["arm1_Joint"],
    "arm2_Joint": DOFBOT_JOINT_LIMITS["arm2_Joint"],
    "arm3_Joint": DOFBOT_JOINT_LIMITS["arm3_Joint"],
    "arm4_Joint": DOFBOT_JOINT_LIMITS["arm4_Joint"],
    "arm5_Joint": DOFBOT_JOINT_LIMITS["arm5_Joint"],
}
LOCKED_JOINT_TOL_RAD = 0.001
TCP_ORIENTATION_ERROR_RAD = math.radians(5.0)
COMMAND_TIMEOUT_SEC = 480.0
ACK_TIMEOUT_SEC = COMMAND_TIMEOUT_SEC + 15.0
# Simulation only. Hardware continues to require calibrated gripper widths.
# (Vong lap override close=80 toan cuc da don: gia tri close nam truc tiep
# trong PIECE_SPECS, mot moi duy nhat.)
# Margin an toan so voi URDF limit (CHOT theo spec: 0.02 rad). Dung chung cho
# optimizer bounds release-chain va gate margin toan duong: khong co 2 chuan
# (0.05/0.02) song song gay loai oan nghiem nam tren bound.
JOINT_LIMIT_MARGIN_RAD = 0.02
# Dung sai chạm-đích khi đặt quân: đáy quân mang trong ±tol so với mặt bàn
# thì contact attached<->board là chạm đặt hợp lệ (không phải va chạm).
# Mọi contact khác (tay<->bàn, quân<->quân, chạm khi còn ở cao) vẫn veto.
RELEASE_TOUCH_TOL_M = 0.004
# Gate margin cung (buoc C): candidate co margin < MARGIN_MIN_RAD so voi URDF
# limit thi bi loai, pipeline thu seed ke (fail-loud neu het seed). Warn khi
# duoi MARGIN_WARN_RAD de theo doi doan sat gioi han.
MARGIN_MIN_RAD = 0.02
MARGIN_WARN_RAD = 0.10
# Far-rank grasp_z tam thoi (buoc b: a8/h8 UNREACHABLE o grasp-z chuan 0.059;
# cua so kha thi do duoc >=0.077 (a8) / >=0.075 (h8), lay +1mm an toan).
# TAM THOI cho sim (attach la bookkeeping); y nghia gap vat ly o cao do nay
# CHO hardware calibration (co HW_GRASP_Z_PENDING).
FAR_RANK_GRASP_Z = {"a8": 0.078, "h8": 0.076}
HW_GRASP_Z_PENDING = True
# Bước nhảy joint bất thường trong một trajectory (rad giữa 2 waypoint kề).
MAX_JOINT_STEP_RAD = 0.6
# Scoring candidate (TODO-2): trọng số cho err (m), tilt (rad), travel (rad),
# margin tới limit (rad, càng xa càng tốt nên trừ điểm), |arm5| (rad, cang
# nho cang tot — uu tien nghiem gan 0 trong cage, huong B).
CANDIDATE_SCORE_W_POS = 1.0 / 0.005
CANDIDATE_SCORE_W_TILT = 1.0 / 0.45
CANDIDATE_SCORE_W_TRAVEL = 0.15
CANDIDATE_SCORE_W_LIMIT_MARGIN = -0.5
CANDIDATE_SCORE_W_ARM5 = 0.5
# Template joint theo vùng bàn cờ (TODO-2/3): seed IK ưu tiên theo vùng để
# phủ nhánh khớp khác nhau thay vì mọi ô cùng một seed HOME.
# Huong B: 5 phan tu (arm1-5); arm5 seed = 0.0 (giua cage, uu tien |arm5| min).
REGION_JOINT_TEMPLATES = {
    # rank 1-2 gần đế: gập gọn tránh tự va.
    "near": [0.0, -0.5, 1.0, -0.5, 0.0],
    # trung tâm bàn: tư thế trung tính.
    "center": [0.0, -0.3, 0.6, -0.3, 0.0],
    # rank 7-8 xa đế: vươn dài.
    "far": [0.0, -0.2, 0.4, -0.2, 0.0],
    # khu discard (-Y): xoay đế sang bên.
    "discard": [-0.5, -0.4, 0.8, -0.4, 0.0],
}

# ==== TILT 5-DOF (TODO-3) ====
# Quality target <=11° (KPI, chưa blocker MVP), hard reject >26°.
TILT_QUALITY_TARGET_RAD = 0.191986  # 11 deg
TILT_HARD_LIMIT_RAD = 0.453786  # 26 deg
# Thử orientation constraint giữ TCP gần thẳng đứng, yaw tự do (TODO-3).
# False = dùng candidate scoring (mặc định, reachability cao hơn trên 5-DOF
# position-only); True = ép descend theo quat thẳng đứng trước, rớt mới
# fallback scoring và log ảnh hưởng reachability.
USE_UPRIGHT_ORIENTATION_CONSTRAINT = False

# ==== DIAGNOSTIC (TODO-4) ====
DEEP_SQUARE_COUNT = 10
DISCARD_SLOT_COUNT = 16

# ==== CALIBRATION ROBOT THẬT (TODO-7, chưa đo -> fail-loud) ====
# TCP_OFFSET_CALIBRATED (định nghĩa ở cụm bàn thật phía trên): False cho tới
# khi đo OFFSET = measured_tcp_z - BOARD_TOP_Z - PIECE_GRASP_HEIGHT[type] trên
# phần cứng. FakeSystem/sim chạy được với False; hardware execute bị chặn.
# Tốc độ an toàn khi test phần cứng: scale 0..1, test không tải/tốc độ thấp.
HARDWARE_SAFE_VELOCITY_SCALE = 0.25
# Bảng gripper joint->inner width (m) khi đã đo FK/servo thật.
# Format: [(joint_rad, inner_width_m), ...] sorted theo joint.
# Trống = chưa calibration -> gripper_width_to_joint_angle raise.
GRIPPER_CALIBRATION_TABLE: list = []


def gripper_width_to_joint_angle(width_m: float) -> float:
    """Map khoảng cách mặt trong finger -> joint angle (TODO-7).

    Khi GRIPPER_CALIBRATION_TABLE đã đo (FK RViz + hiệu chỉnh servo thật, gồm
    các mốc 20/14/12mm) thì nội suy tuyến tính từng đoạn. Bảng trống ->
    raise để giữ binary GRIPPER_OPEN/CLOSED_RAD, cấm nội suy angle =
    width/max*1.57 vì mimic phi tuyến.
    """
    table = sorted(GRIPPER_CALIBRATION_TABLE)
    if not table:
        raise NotImplementedError(
            "Chưa có bảng calibration gripper_width_to_joint_angle; "
            "giữ binary GRIPPER_OPEN/CLOSED_RAD. Đo joint->width bằng FK RViz, "
            "hiệu chỉnh servo thật tại 20/14/12mm rồi điền "
            "GRIPPER_CALIBRATION_TABLE."
        )
    if width_m <= table[0][1]:
        return float(table[0][0])
    for (j0, w0), (j1, w1) in zip(table, table[1:]):
        if w0 <= width_m <= w1 or w1 <= width_m <= w0:
            t = (width_m - w0) / (w1 - w0) if w1 != w0 else 0.0
            return float(j0 + t * (j1 - j0))
    return float(table[-1][0])

# Candidate lệch tâm cho GẮP, tính bằng mét. Pipeline luôn thử tâm trước, rồi
# mở rộng hữu hạn 3 -> 6 -> 8 mm; không tìm vô hạn và không mở collision với
# quân lân cận. 8 mm vẫn nằm trong nửa ô 13.5 mm của bàn hiện tại. Điểm đặt,
# collision object và visual của quân luôn ở tâm ô.
GRASP_APPROACH_CANDIDATE_OFFSETS = (
    (0.0, 0.0),
    (0.003, 0.0), (-0.003, 0.0), (0.0, 0.003), (0.0, -0.003),
)


def square_to_xy(square: str):
    """'e4' -> (x, y) tâm ô, chưa cộng offset lệch tâm quân.
    CHỐT: X=rank, Y=file (hàng 1 gần robot)."""
    file_idx = FILES.index(square[0].lower())
    rank_idx = int(square[1]) - 1
    x = BOARD_ORIGIN[0] + rank_idx * SQUARE_SIZE
    y = BOARD_ORIGIN[1] + file_idx * SQUARE_SIZE
    return x, y


def square_center(square: str):
    """Tâm ô dạng (x, y, z) theo quy ước CHỐT X=rank/Y=file."""
    x, y = square_to_xy(square)
    return x, y, BOARD_ORIGIN[2]


def square_to_grasp_pose(square: str, piece_type: str, offset_xy=(0.0, 0.0)):
    """Trả về (x, y, z) TCP khi gắp, cộng offset lệch tâm nếu có.

    `z` tra từ PIECE_GRIP_Z theo loại quân (mặc định = PICK_TCP_Z cho mọi
    loại cho tới khi tune vật lý). Chiều cao quân (PIECE_SPECS) chỉ phục vụ
    visual/collision và offset khi attach object vào gripper.

    Cộng offset lệch tâm nếu có
    (offset_xy mô phỏng vai trò của position-regression model trong bản gốc;
    ở chế độ giả lập không có camera thì để mặc định (0, 0)).

    Deny-all: +CONTACT_HOVER_M để goal descend và start lift không dính
    contact hình học (không còn exception ACM nào land được).

    Branch exp/arm5-fixed: o far-rank a8/h8 dung FAR_RANK_GRASP_Z tam thoi
    (UNREACHABLE o grasp-z chuan; HW_GRASP_Z_PENDING=True)."""
    x, y = square_to_xy(square)
    x += offset_xy[0]
    y += offset_xy[1]
    if square in FAR_RANK_GRASP_Z:
        return x, y, FAR_RANK_GRASP_Z[square] + CONTACT_HOVER_M
    return x, y, PIECE_GRIP_Z.get(piece_type, PICK_TCP_Z) + CONTACT_HOVER_M


def square_to_place_pose(square: str, piece_type: str):
    """TCP khi đặt quân tại tâm ô; không áp dụng offset clearance của pick.

    Deny-all: +CONTACT_HOVER_M như grasp (quân attached chạm bàn ở goal
    descend là attached-vs-world, BỊ check). Detach visual vẫn ở BOARD_Z
    (đường detach dùng BOARD_Z/zd trực tiếp, không qua pose này)."""
    x, y = square_to_xy(square)
    return x, y, PIECE_GRIP_Z.get(piece_type, PICK_TCP_Z) + CONTACT_HOVER_M


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
    if index < 0 or index >= DISCARD_MAX_SLOTS:
        raise ValueError(
            f"Slot nghĩa địa {index} vượt lưới {DISCARD_COLS}x{DISCARD_ROWS} "
            f"({DISCARD_MAX_SLOTS} slot). Ván cờ đã ăn quá nhiều quân cho layout này."
        )
    col = index % DISCARD_COLS
    row = index // DISCARD_COLS
    x = DISCARD_ORIGIN[0] + col * DISCARD_DX
    y = DISCARD_ORIGIN[1] - row * DISCARD_DY
    return x, y, DISCARD_ORIGIN[2]


# Mô hình chess dùng đầy đủ arm1..arm5. Không lọc arm5 khỏi RobotState: nó là
# bậc tự do cần thiết để giữ hướng TCP. Miền CHESS_JOINT_LIMITS là gate cứng
# cho arm5 trong ±20 độ.
UNMODELED_JOINTS = frozenset()


# ==== BRANCH exp/arm5-fixed-pipeline: margin + occupancy helpers ====
def joint_margins(values: dict, limits: dict | None = None) -> dict:
    """Margin den URDF limit theo tung joint (rad, am = vuot gioi han)."""
    lim = CHESS_JOINT_LIMITS if limits is None else limits
    out = {}
    for name, (lo, hi) in lim.items():
        v = values.get(name)
        out[name] = min(float(v) - lo, hi - float(v)) if v is not None else float("inf")
    return out


def min_margin(values: dict, limits: dict | None = None):
    """(ten_joint, margin_min). Dung cho gate candidate buoc C."""
    m = joint_margins(values, limits)
    name = min(m, key=lambda k: m[k])
    return name, m[name]


# Quan cao co the cham ngon kep khi descend o ke ben (bai hoc d1/e1 buoc b:
# Llink2 vs vua e1). Tinh huong PHU THUOC occupancy hien tai, KHONG hard-code
# o co dinh nao la unreachable.
TALL_PIECE_HEIGHT_M = 0.038


def tall_neighbor_situation(square: str, piece_type_by_square: dict) -> list:
    """Liet ke (o_ke, loai_quan) cao dang dung quanh `square`."""
    out = []
    try:
        fi = FILES.index(square[0].lower())
        ri = int(square[1]) - 1
    except (ValueError, IndexError):
        return out
    for df in (-1, 0, 1):
        for dr in (-1, 0, 1):
            if df == 0 and dr == 0:
                continue
            f2, r2 = fi + df, ri + dr
            if 0 <= f2 < 8 and 0 <= r2 < 8:
                nb = f"{FILES[f2]}{r2 + 1}"
                pt = piece_type_by_square.get(nb)
                spec = PIECE_SPECS.get(pt) if pt else None
                if spec is not None and spec.pickup_height >= TALL_PIECE_HEIGHT_M:
                    out.append((nb, pt))
    return out
