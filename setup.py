from setuptools import find_packages, setup

package_name = "sphero_rvr_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sphero_rvr_driver"]),
        ("share/sphero_rvr_driver", ["package.xml"]),
        (
            "share/sphero_rvr_driver/launch",
            [
                "launch/rvr.launch.py",
                "launch/supervised_rvr.launch.py",
                "launch/lidar.launch.py",
                "launch/mapping.launch.py",
                "launch/explore.launch.py",
            ],
        ),
        (
            "share/sphero_rvr_driver/config",
            [
                "config/rvr.yaml",
                "config/collision_stop.yaml",
                "config/range_motion.yaml",
                "config/live_route_runner.yaml",
                "config/lidar.yaml",
                "config/slam_toolbox.yaml",
                "config/lean_rvr_tank_si.yaml",
                "config/lean_nav2.yaml",
                "config/lean_explore_lite.yaml",
                "config/coverage_explorer.yaml",
                "config/ekf.yaml",
            ],
        ),
        (
            "share/sphero_rvr_driver/behavior_trees",
            [
                "behavior_trees/navigate_to_pose_decisive.xml",
            ],
        ),
        (
            "share/sphero_rvr_driver/scripts",
            [
                "scripts/install-rvr-pi",
            ],
        ),
        (
            "share/sphero_rvr_driver/docs",
            [
                "docs/architecture_map.md",
                "docs/rvr_capability_matrix.md",
                "docs/lean_explore_run_guide.md",
                "docs/mapping.md",
                "docs/lidar_collision_stop_supervisor.md",
                "docs/range_motion_controller.md",
                "docs/decisive_controller.md",
                "docs/coverage_explorer.md",
                "docs/camera_low_obstacle_design.md",
            ],
        ),
        ("share/sphero_rvr_driver/docs/udev", ["docs/udev/99-rplidar.rules"]),
    ],
    install_requires=["setuptools", "pyserial"],
    extras_require={
        "dev": [
            "pytest>=8.0",
            "pytest-asyncio>=0.23",
            "tomli>=2; python_version < '3.11'",
        ]
    },
    zip_safe=True,
    maintainer="Jason Scott Person",
    maintainer_email="jsperson@users.noreply.github.com",
    description="ROS 2 driver for Sphero RVR",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rvr_node = sphero_rvr_driver.rvr_node:main",
            "lidar_collision_stop_supervisor = sphero_rvr_driver.collision_stop_node:main",
            "range_motion_controller = sphero_rvr_driver.range_motion_node:main",
            "live_route_runner = sphero_rvr_driver.live_route_runner_node:main",
            "decisive_controller = sphero_rvr_driver.decisive_controller_node:main",
            "coverage_explorer = sphero_rvr_driver.coverage_explorer_node:main",
        ],
    },
)
