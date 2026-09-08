from setuptools import find_packages, setup

package_name = "chess_moveit_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/chess_sim.launch.py", "launch/chess_base.launch.py"
        ]),
        ("share/" + package_name + "/config", [
            "config/chess.rviz", "config/chess_lite.rviz", "config/chess_base.rviz"
        ]),
    ],
    # python-chess được cài ở môi trường Python; pymoveit2 là ROS package trong workspace.
    # Không khai báo chúng ở đây để colcon không gọi pip trong lúc build offline.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leyud",
    maintainer_email="you@example.com",
    description="Demo mô phỏng robot 6DOF chơi cờ với MoveIt2 (Stockfish self-play)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chess_brain_node = chess_moveit_demo.chess_brain_node:main",
            "pick_place_node = chess_moveit_demo.pick_place_node:main",
            "chess_base_node = chess_moveit_demo.chess_base_node:main",
        ],
    },
)
