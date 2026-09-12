from setuptools import find_packages, setup

package_name = "simplify_chess_game"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/home.yaml", "config/gripper.yaml", "config/safety.yaml",
            "config/square_routes.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "chess_cli = simplify_chess_game.chess_cli:main",
        "calibrate_square = simplify_chess_game.calibration:main",
        "audit_routes = simplify_chess_game.route_audit:main",
    ]},
)
