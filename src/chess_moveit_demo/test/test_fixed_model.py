"""Unit tests for the five-joint constrained chess model: chay khong can ROS.
    python3 -m pytest src/chess_moveit_demo/test/test_fixed_model.py -q
"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from chess_moveit_demo import chess_utils as C


def test_templates_5elem():
    for name, tpl in C.REGION_JOINT_TEMPLATES.items():
        assert len(tpl) == 5, name
        assert all(math.isfinite(v) for v in tpl)


def test_arm5_cage():
    # Kien truc moi (f6b997a): arm5 dung full URDF range, ±20 deg chi la
    # preference/scoring (ARM5_CAGE_RAD, ARM5_PREFERENCE_SEEDS), KHONG phai
    # hard gate trong CHESS_JOINT_LIMITS (cage cung lam pick vo nghiem).
    assert "arm5_Joint" in C.CHESS_JOINT_LIMITS
    assert set(C.CHESS_JOINT_LIMITS) == {
        "arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"}
    lo, _ = C.CHESS_JOINT_LIMITS["arm2_Joint"]
    assert lo < 0.0  # quy tac duong da bo
    lo5, hi5 = C.CHESS_JOINT_LIMITS["arm5_Joint"]
    assert abs(lo5 - C.DOFBOT_JOINT_LIMITS["arm5_Joint"][0]) < 1e-9
    assert abs(hi5 - C.DOFBOT_JOINT_LIMITS["arm5_Joint"][1]) < 1e-9
    assert abs(C.ARM5_CAGE_RAD - math.radians(20.0)) < 1e-9


def test_release_gates():
    # Spec giu: margin 0.02, fraction 0.98; cham-dat trong tol khong phai collision.
    assert abs(C.JOINT_LIMIT_MARGIN_RAD - 0.02) < 1e-9
    assert abs(C.MARGIN_MIN_RAD - 0.02) < 1e-9
    assert abs(C.MIN_CARTESIAN_FRACTION - 0.98) < 1e-9
    assert C.RELEASE_TOUCH_TOL_M > 0.0


def test_margins():
    vals = {"arm1_Joint": 0.1, "arm2_Joint": -0.3,
            "arm3_Joint": 1.45, "arm4_Joint": 1.39, "arm5_Joint": 0.0}
    m = C.joint_margins(vals)
    assert abs(m["arm2_Joint"] - 1.2708) < 1e-6
    name, mm = C.min_margin(vals)
    assert name == "arm3_Joint" and abs(mm - 0.1208) < 1e-6


def test_tall_neighbor_d1():
    nb = C.tall_neighbor_situation(
        "d1", {"e1": "k", "d2": "p", "c1": "b", "c2": "p", "e2": "p"})
    assert ("e1", "k") in nb
    assert all(sq != "d1" for sq, _ in nb)
    assert C.tall_neighbor_situation("e4", {}) == []


def test_far_grasp_z_temp():
    assert C.HW_GRASP_Z_PENDING is True
    assert set(C.FAR_RANK_GRASP_Z) == {"a8", "h8"}
    a8 = C.square_to_grasp_pose("a8", "r")
    assert abs(a8[2] - (0.078 + C.CONTACT_HOVER_M)) < 1e-9
    e2 = C.square_to_grasp_pose("e2", "p")
    assert abs(e2[2] - (C.PICK_TCP_Z + C.CONTACT_HOVER_M)) < 1e-9


def test_gripper_stages_valid():
    for t, spec in C.PIECE_SPECS.items():
        assert spec.gripper_open_rad <= spec.gripper_preclose_rad <= spec.gripper_close_rad, t
        assert spec.gripper_open_rad <= spec.gripper_release_rad <= spec.gripper_close_rad, t
    assert abs(math.degrees(C.PIECE_SPECS["p"].gripper_preclose_rad) - 75.0) < 0.1
    assert abs(math.degrees(C.PIECE_SPECS["p"].gripper_close_rad) - 80.0) < 0.1


def test_pymoveit2_init_structure():
    """Hoi quy bug D: helper chen giua __init__ nuot phan con lai (py_compile
    van pass nhung instance thieu attrs). Quet AST, khong can ROS."""
    import ast
    import os
    repo = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    src = open(os.path.join(repo, "src", "pymoveit2", "pymoveit2", "moveit2.py")).read()
    tree = ast.parse(src)
    cls = next(n for n in ast.walk(tree)
               if isinstance(n, ast.ClassDef) and n.name == "MoveIt2")
    init = next(n for n in cls.body
                if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    nested = [n.name for n in ast.walk(init) if isinstance(n, ast.FunctionDef) and n is not init]
    assert nested == [], f"def trong __init__: {nested}"
    params = [a.arg for a in init.args.args] + [a.arg for a in init.args.kwonlyargs]
    assert "exclude_joints" in params
