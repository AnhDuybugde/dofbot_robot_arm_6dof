"""Sinh model planning arm5-fixed tu xacro da resolve (branch exp/arm5-fixed).

- Input: src/dofbot_moveit/config/dofbot.urdf.xacro (gom ca FakeSystem ros2_control)
- Output: src/dofbot_moveit/config/dofbot_fixed.{urdf,srdf}
- arm5_Joint: revolute -> fixed, GIU NGUYEN origin/parent/child (= transform q5=0).
- SRDF: group arm_group -> arm_fixed; group_state bo dong arm5; end_effector
  parent_group -> arm_fixed; giu grip_group/virtual/disables.
- kinematics.yaml phai co them block arm_fixed (sua tay 1 lan, xem cuoi file).

Chay (moi truong ROS da source):
    python3 src/dofbot_moveit/tools/generate_fixed_model.py
Kiem chung: check_urdf + assert origin arm5 khong doi + FK-equivalence bang
/tmp/dbg_ik_F42.py (do branch, khong phai test tu dong o day).
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.normpath(os.path.join(HERE, "..", "config"))
XACRO = os.path.join(CONFIG, "dofbot.urdf.xacro")
SRDF_SRC = os.path.join(CONFIG, "dofbot.srdf")
URDF_OUT = os.path.join(CONFIG, "dofbot_fixed.urdf")
SRDF_OUT = os.path.join(CONFIG, "dofbot_fixed.srdf")

ARM5_ORIGIN_XYZ = "-0.00215 -4.5E-05 0.078149"
ARM5_ORIGIN_RPY = "0 0 0"


def main():
    proc = subprocess.run(["xacro", XACRO], capture_output=True, text=True)
    if proc.returncode != 0:
        print(proc.stderr[-2000:])
        raise SystemExit("xacro resolve that bai")
    urdf = proc.stdout
    pat = re.compile(r'<joint\s+name="arm5_Joint"\s+type="revolute">(.*?)</joint>', re.S)
    m = pat.search(urdf)
    assert m, "khong thay arm5 revolute"
    assert ARM5_ORIGIN_XYZ in m.group(1) and ARM5_ORIGIN_RPY in m.group(1), \
        "origin arm5 DOI so voi ky vong - DUNG LAI, doi chieu!"
    fixed_joint = (
        '<joint name="arm5_Joint" type="fixed">\n'
        f'    <origin xyz="{ARM5_ORIGIN_XYZ}" rpy="{ARM5_ORIGIN_RPY}" />\n'
        '    <parent link="arm4_Link" />\n'
        '    <child link="arm5_Link" />\n  </joint>')
    urdf = pat.sub(lambda _: fixed_joint, urdf, count=1)
    assert 'name="arm5_Joint" type="revolute"' not in urdf
    with open(URDF_OUT, "w") as f:
        f.write(urdf)

    s = open(SRDF_SRC).read()
    s = s.replace('<group name="arm_group">', '<group name="arm_fixed">')
    s = s.replace('group="arm_group"', 'group="arm_fixed"')
    s = s.replace('parent_group="arm_group"', 'parent_group="arm_fixed"')
    s = re.sub(r'\s*<joint name="arm5_Joint" value="[^"]*"/>\n', '\n', s)
    with open(SRDF_OUT, "w") as f:
        f.write(s)
    print(f"OK: {URDF_OUT}\nOK: {SRDF_OUT}")
    print("NHO: them block arm_fixed vao kinematics.yaml (xem huong dan cuoi file nay).")


if __name__ == "__main__":
    sys.exit(main())

# Block can them vao src/dofbot_moveit/config/kinematics.yaml:
# arm_fixed:
#   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
#   kinematics_solver_search_resolution: 0.005
#   kinematics_solver_timeout: 0.05
#   position_only_ik: true
