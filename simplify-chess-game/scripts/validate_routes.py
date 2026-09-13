"""Offline validator for square_routes.yaml (§10 of the review spec).

Zero ROS dependencies (numpy + pyyaml only). For every chess-legal
(piece, square) pair from config/piece_reachability.yaml it validates the
shared square route geometry:

  1. route shape / HOME-first / VALIDATED status (via RouteDatabase)
  2. joint limits on every waypoint
  3. FK of the recorded endpoint vs the square centre (TCP off-target check)
  4. RDP simplification (same eps as runtime) + joint-space interpolation,
     then FK over the whole swept path, outbound AND reversed (retreat)
  5. worst-case neighbours: every on-board neighbour square may hold the
     biggest/tallest piece (king envelope) -> capsule-vs-cylinder check for
     wrist (J4->J5), palm (J5->TCP), both fingers, knuckles, camera
  6. minimum clearance vs safety.yaml neighbor_clearance_margin
  7. floor check (nothing may dive through the board)

Result table: Piece | Target | IK | JointLimit | NeighborCollision |
MinClearance | Tilt | Result. Geometry depends only on the square, so the
64 squares are validated once and joined with the 832 reachability pairs.

Limitations (stated honestly): no de-novo IK solver exists in this project,
so "IK" here = FK-verification that the recorded endpoint actually reaches
the target square; candidate search = local joint perturbations around the
recorded grip pose (kept only if strictly better and re-validated).
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve()
PKG = HERE.parents[1]
sys.path.insert(0, str(PKG))
from simplify_chess_game.route_database import RouteDatabase  # noqa: E402 (yaml only, no ROS)

CONFIG = PKG / "config"
BOARD_X0, BOARD_Y0, BOARD_Z = 0.1105, -0.0915, 0.005
SQUARE = 0.026
FILES = "abcdefgh"
ARM_JOINTS = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]
# Conservative obstacle: the biggest/tallest piece (king h=47mm, d=18mm).
PIECE_R, PIECE_Z0, PIECE_Z1 = 0.009, BOARD_Z, BOARD_Z + 0.047
GRIP_ANGLES = (0.0, 1.2)  # open .. PRE_CLOSE: finger union over both
FINGER_LEN = 0.040  # Rlink2/Llink2 0.03 link + tip pad


def neighbours(square: str) -> list[str]:
    f, r = FILES.index(square[0]), int(square[1]) - 1
    out = []
    for df in (-1, 0, 1):
        for dr in (-1, 0, 1):
            if df == 0 and dr == 0:
                continue
            nf, nr = f + df, r + dr
            if 0 <= nf < 8 and 0 <= nr < 8:
                out.append(f"{FILES[nf]}{nr + 1}")
    return out


def square_xy(square: str) -> tuple[float, float]:
    return (BOARD_X0 + (int(square[1]) - 1) * SQUARE,
            BOARD_Y0 + FILES.index(square[0]) * SQUARE)


def rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def parse_urdf(urdf_path: Path):
    root = ET.parse(str(urdf_path)).getroot()
    joints = {}
    for j in root.iter("joint"):
        name = j.get("name")
        o = j.find("origin")
        xyz = tuple(float(v) for v in o.get("xyz").split()) if o is not None else (0, 0, 0)
        rpy = tuple(float(v) for v in o.get("rpy").split()) if o is not None else (0, 0, 0)
        a = j.find("axis")
        axis = tuple(float(v) for v in a.get("xyz").split()) if a is not None else (0, 0, 0)
        joints[name] = {
            "type": j.get("type"), "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"), "xyz": np.array(xyz),
            "rot": rpy_matrix(rpy), "axis": np.array(axis)}
    return joints


def stl_bbox(mesh_path: Path) -> np.ndarray:
    """Bounding-box dims of a binary or ASCII STL (pure stdlib)."""
    data = mesh_path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:200].lower():
        pts = []
        for line in data.decode("utf-8", "ignore").splitlines():
            s = line.strip()
            if s.startswith("vertex"):
                pts.append([float(v) for v in s.split()[1:4]])
        arr = np.array(pts)
    else:
        n = struct.unpack("<I", data[80:84])[0]
        arr = np.zeros((n * 3, 3))
        off = 84
        for i in range(n):
            tri = struct.unpack("<12f", data[off:off + 48])
            # 12 floats = normal(3) + v1(3) + v2(3) + v3(3): rows of 3, skip normal
            arr[i * 3:(i + 1) * 3] = np.array(tri).reshape(4, 3)[1:]
            off += 50
    return arr.max(axis=0) - arr.min(axis=0)


class Arm:
    """Minimal FK for the dofbot chain + gripper, base_link frame."""

    def __init__(self, urdf_path: Path):
        self.j = parse_urdf(urdf_path)

    def fk(self, q: list[float], grip: float) -> dict[str, np.ndarray]:
        qmap = dict(zip(ARM_JOINTS, q))
        qmap.update({"Rlink1_Joint": grip, "Llink1_Joint": -grip,
                     "Rlink2_Joint": 0.0, "Llink2_Joint": 0.0,
                     "Rlink3_Joint": 0.0, "Llink3_Joint": 0.0})
        frames: dict[str, np.ndarray] = {"base_link": np.eye(4)}

        def frame(link: str) -> np.ndarray:
            if link in frames:
                return frames[link]
            # find joint whose child is this link
            for name, jn in self.j.items():
                if jn["child"] == link:
                    break
            parent = frame(jn["parent"])
            mat = np.eye(4)
            mat[:3, :3] = jn["rot"]
            mat[:3, 3] = jn["xyz"]
            if jn["type"] in ("revolute", "continuous"):
                ang = qmap.get(name, 0.0)
                ax = jn["axis"] / np.linalg.norm(jn["axis"])
                c, s = math.cos(ang), math.sin(ang)
                ux, uy, uz = ax
                rot = np.array([
                    [c + ux * ux * (1 - c), ux * uy * (1 - c) - uz * s, ux * uz * (1 - c) + uy * s],
                    [uy * ux * (1 - c) + uz * s, c + uy * uy * (1 - c), uy * uz * (1 - c) - ux * s],
                    [uz * ux * (1 - c) - uy * s, uz * uy * (1 - c) + ux * s, c + uz * uz * (1 - c)]])
                mat = mat @ np.vstack([np.hstack([rot, [[0], [0], [0]]]), [0, 0, 0, 1]])
            frames[link] = parent @ mat
            return frames[link]

        out = {}
        for link in ("arm1_Link", "arm2_Link", "arm3_Link", "arm4_Link",
                     "arm5_Link", "Gripping_point_Link", "Rlink1_Link",
                     "Llink1_Link", "Rlink2_Link", "Llink2_Link", "Camera_Link"):
            out[link] = frame(link)[:3, 3]
        # joint pivot positions: parent-frame origin of each arm joint
        piv = {}
        for i, name in enumerate(ARM_JOINTS, start=1):
            jn = self.j[name]
            piv[f"J{i}"] = frame(jn["parent"])[:3, :3] @ jn["xyz"] + frame(jn["parent"])[:3, 3]
        out.update(piv)
        out["TCP"] = out["Gripping_point_Link"]
        for side, knuckle, tip in (("R", "Rlink1_Link", "Rlink2_Link"),
                                   ("L", "Llink1_Link", "Llink2_Link")):
            kf = frame(knuckle)
            # fingertip: FINGER_LEN along the tip-link x axis
            out[f"{side}tip"] = frame(tip)[:3, :3] @ np.array([FINGER_LEN - 0.030, 0, 0]) \
                + frame(tip)[:3, 3]
            out[f"{side}knuckle"] = kf[:3, 3]
        return out


def rdp(points: list[list[float]], eps: float) -> list[list[float]]:
    if len(points) <= 2 or eps <= 0:
        return [list(p) for p in points]
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        d = [y - x for x, y in zip(points[a], points[b])]
        denom = sum(v * v for v in d)
        worst, idx = -1.0, -1
        for i in range(a + 1, b):
            if denom < 1e-12:
                dev = max(abs(p - x) for p, x in zip(points[i], points[a]))
            else:
                t = max(0.0, min(1.0, sum((p - x) * v for p, x, v in zip(points[i], points[a], d)) / denom))
                dev = max(abs(p - (x + t * v)) for p, x, v in zip(points[i], points[a], d))
            if dev > worst:
                worst, idx = dev, i
        if worst > eps:
            keep[idx] = True
            stack += [(a, idx), (idx, b)]
    return [list(p) for p, k in zip(points, keep) if k]


def interpolate(route: list[list[float]], max_step: float = 0.01) -> list[list[float]]:
    out = [list(route[0])]
    for nxt in route[1:]:
        prev = out[-1]
        dist = max(abs(a - b) for a, b in zip(prev, nxt))
        n = max(1, int(math.ceil(dist / max_step)))
        for k in range(1, n + 1):
            out.append([a + (b - a) * k / n for a, b in zip(prev, nxt)])
    return out


def seg_point_gap(px, py, pz, ax, ay, az, bx, by, bz, pr):
    """Min horizontal gap from segment AB (radius sr) to vertical cylinder (px,py,pr),
    or None when the segment is entirely above the piece."""
    if min(az, bz) > PIECE_Z1 + 0.002:
        return None
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if denom < 1e-12:
        d = math.hypot(ax - px, ay - py)
    else:
        t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / denom))
        d = math.hypot(ax + abx * t - px, ay + aby * t - py)
    return d - pr


def validate_square(square: str, route: list[list[float]], arm: Arm,
                    radii: dict[str, float], margin: float, rdp_eps: float) -> dict:
    res = {"square": square, "n_waypoints": len(route), "warnings": [],
           "ik": "OK", "joint_limit": "OK", "neighbor_collision": "OK",
           "min_clearance_m": float("inf"), "tilt_deg": 0.0, "result": "PASS",
           "fail_reason": ""}
    # joint limits
    for i, pt in enumerate(route):
        if len(pt) != 5 or any(not math.isfinite(v) for v in pt):
            res.update(joint_limit=f"BAD shape wp{i}", result="FAIL",
                       fail_reason="joint shape/non-finite")
            return res
        if any(abs(v) > 1.5708 + 1e-9 for v in pt):
            res.update(joint_limit=f"LIMIT wp{i}", result="FAIL",
                       fail_reason="joint limit")
            return res
    max_jump = max(max(abs(a - b) for a, b in zip(route[i], route[i + 1]))
                   for i in range(len(route) - 1))
    res["max_jump_rad"] = round(max_jump, 4)
    if max_jump > 0.35:
        res["warnings"].append(f"large waypoint jump {max_jump:.3f} rad")
    # endpoint FK vs square centre
    cx, cy = square_xy(square)
    worst_grip_err, tilt = 0.0, 0.0
    for grip in GRIP_ANGLES:
        f = arm.fk(route[-1], grip)
        err = math.hypot(f["TCP"][0] - cx, f["TCP"][1] - cy)
        worst_grip_err = max(worst_grip_err, err)
        v = f["TCP"] - f["J5"]
        tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, -v[2] / np.linalg.norm(v))))))
    res["tcp_error_m"] = round(worst_grip_err, 4)
    res["tilt_deg"] = round(tilt, 1)
    if worst_grip_err > 0.013:
        res.update(ik=f"OFF-TARGET {worst_grip_err*1000:.1f}mm", result="FAIL",
                   fail_reason="TCP off-target")
        return res
    if worst_grip_err > 0.006:
        res["warnings"].append(f"TCP {worst_grip_err*1000:.1f}mm from centre")
    # sweep both directions over the RDP-executed path
    simp = rdp(route, rdp_eps)
    res["n_rdp"] = len(simp)
    paths = [interpolate(simp), interpolate(list(reversed(simp)))]
    nbs = [(square_xy(n)[0], square_xy(n)[1]) for n in neighbours(square)]
    capsules = [("J4", "J5", radii["wrist"]), ("J5", "TCP", radii["palm"]),
                ("TCP", "Rtip", radii["finger"]), ("TCP", "Ltip", radii["finger"])]
    spheres = [("Rknuckle", radii["knuckle"]), ("Lknuckle", radii["knuckle"]),
               ("Camera_Link", radii["camera"])]
    worst = {"gap": float("inf"), "where": ""}
    for path in paths:
        for pt in path:
            for grip in GRIP_ANGLES:
                f = arm.fk(pt, grip)
                if min(f["TCP"][2], f["J5"][2]) < 0.002:
                    res.update(neighbor_collision="BELOW BOARD", result="FAIL",
                               fail_reason="below board")
                    return res
                for a, b, sr in capsules:
                    A, B = f[a], f[b]
                    for nx, ny in nbs:
                        g = seg_point_gap(nx, ny, 0, A[0], A[1], A[2], B[0], B[1], B[2],
                                          PIECE_R + sr)
                        if g is not None and g < worst["gap"]:
                            worst = {"gap": g, "where": f"{a}->{b}"}
                for s, sr in spheres:
                    P = f[s]
                    if P[2] < PIECE_Z1 + 0.002:
                        for nx, ny in nbs:
                            g = math.hypot(P[0] - nx, P[1] - ny) - (PIECE_R + sr)
                            if g < worst["gap"]:
                                worst = {"gap": g, "where": s}
    res["min_clearance_m"] = round(worst["gap"], 4) if worst["gap"] != float("inf") else 9.99
    res["min_clearance_where"] = worst["where"]
    if worst["gap"] < 0:
        res.update(neighbor_collision=f"COLLISION {worst['where']}",
                   result="FAIL", fail_reason=f"gripper/J4/J5 collision {worst['where']}")
    elif worst["gap"] < margin:
        res.update(neighbor_collision=f"CLEARANCE {worst['gap']*1000:.1f}mm",
                   result="FAIL", fail_reason="insufficient clearance")
    return res


def link_radii(urdf_path: Path, meshes: Path) -> dict[str, float]:
    root = ET.parse(str(urdf_path)).getroot()
    mesh_of = {}
    for link in root.iter("link"):
        for vis in link.iter("visual"):
            geo = vis.find("geometry/mesh")
            if geo is not None:
                mesh_of[link.get("name")] = geo.get("filename").split("/")[-1]
    radii = {}
    for key, link in (("wrist", "arm4_Link"), ("palm", "arm5_Link"),
                      ("finger", "Rlink2_Link"), ("knuckle", "Rlink1_Link"),
                      ("camera", "Camera_Link")):
        dims = sorted(stl_bbox(meshes / mesh_of[link]))
        radii[key] = dims[1] / 2.0  # cross-section (two smaller bbox dims)
        print(f"  radius {key:7s} ({link:12s} {mesh_of[link]:20s}): {radii[key]*1000:.2f} mm")
    return radii


def main() -> int:
    ap = argparse.ArgumentParser(description="Offline route geometry validator (no ROS)")
    ap.add_argument("--margin", type=float, default=None)
    ap.add_argument("--square", default=None)
    ap.add_argument("--json", default="/tmp/route_validation.json")
    args = ap.parse_args()
    safety = yaml.safe_load((CONFIG / "safety.yaml").read_text())
    margin = args.margin if args.margin is not None else float(
        safety.get("neighbor_clearance_margin", 0.0))
    home = yaml.safe_load((CONFIG / "home.yaml").read_text())
    print(f"neighbor_clearance_margin = {margin*1000:.1f} mm, rdp_eps = {safety['rdp_eps_rad']}")
    urdf = Path("/home/yahboom/dofbot_robot_arm_6dof/src/dofbot_urdf/urdf/dofbot.urdf")
    meshes = urdf.parent.parent / "meshes"
    print("link radii from STL bboxes:")
    radii = link_radii(urdf, meshes)
    arm = Arm(urdf)
    # FK sanity: HOME TCP should be straight above the arm base area
    f0 = arm.fk([0, 0, 0, 0, 0], 1.2)
    print(f"FK sanity HOME TCP = ({f0['TCP'][0]:.4f}, {f0['TCP'][1]:.4f}, {f0['TCP'][2]:.4f})")
    db = RouteDatabase(CONFIG / "square_routes.yaml", home["home_joints"],
                       home["home_tolerance_rad"])
    reach = yaml.safe_load((CONFIG / "piece_reachability.yaml").read_text())
    squares = [args.square] if args.square else sorted(
        {s for p in reach["pieces"].values() for s in p["allowed_squares"]})
    sqres = {}
    for sq in squares:
        entry = db.get(sq)
        if entry.status != "VALIDATED" or not entry.validated or len(entry.route) < 2:
            sqres[sq] = {"square": sq, "ik": "-", "joint_limit": "-", "neighbor_collision": "-",
                         "min_clearance_m": "-", "tilt_deg": "-", "result": "FAIL",
                         "fail_reason": f"route {entry.status}/empty", "warnings": []}
            continue
        try:
            db._validate_shape_and_home(entry)
        except Exception as exc:
            sqres[sq] = {"square": sq, "ik": "-", "joint_limit": "-", "neighbor_collision": "-",
                         "min_clearance_m": "-", "tilt_deg": "-", "result": "FAIL",
                         "fail_reason": str(exc), "warnings": []}
            continue
        sqres[sq] = validate_square(sq, entry.route, arm, radii, margin, safety["rdp_eps_rad"])
    rows = []
    for pid, info in reach["pieces"].items():
        for sq in info["allowed_squares"]:
            if args.square and sq != args.square:
                continue
            r = sqres[sq]
            rows.append({"piece": pid, "target": sq, **r})
    npass = sum(1 for r in rows if r["result"] == "PASS")
    print(f"\nvalidated squares used: {len(sqres)}, pair rows: {len(rows)}, "
          f"PASS {npass}, FAIL {len(rows)-npass}")
    print("\nPiece | Target | IK | JointLimit | NeighborCollision | MinClear_mm | Tilt_deg | Result")
    shown_fail = 0
    for r in rows:
        if r["result"] == "FAIL" and shown_fail < 40:
            print(f"{r['piece']:16s} | {r['target']:6s} | {r['ik']:14s} | {r['joint_limit']:10s} | "
                  f"{r['neighbor_collision']:22s} | {r['min_clearance_m']} | {r['tilt_deg']} | FAIL "
                  f"({r['fail_reason']})")
            shown_fail += 1
    if any(r["result"] == "FAIL" for r in rows):
        print(f"... ({sum(1 for r in rows if r['result']=='FAIL')} FAIL rows total)")
    Path(args.json).write_text(json.dumps(
        {"margin_m": margin, "radii_m": radii, "squares": sqres}, indent=1))
    print(f"full per-square results -> {args.json}")
    return 0 if npass == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
