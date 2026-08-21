from setuptools import find_packages, setup

package_name = "sphero_rvr_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    package_data={"sphero_rvr_driver": ["web_static/*"]},
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
                "launch/camera.launch.py",
                "launch/bringup_stationary_test.launch.py",
                "launch/sim_closed_loop.launch.py",
            ],
        ),
        (
            "share/sphero_rvr_driver/config",
            [
                "config/rvr.yaml",
                "config/collision_stop.yaml",
                "config/lidar.yaml",
                "config/slam_toolbox.yaml",
                "config/lean_rvr_tank_si.yaml",
                "config/lean_nav2.yaml",
                "config/lean_explore_lite.yaml",
                "config/coverage_explorer.yaml",
                "config/camera.yaml",
                "config/ekf.yaml",
                "config/lean_nav2_stock.yaml",
            ],
        ),
        (
            "share/sphero_rvr_driver/behavior_trees",
            [
                "behavior_trees/navigate_to_pose_decisive.xml",
                "behavior_trees/navigate_to_pose_stock_precise_turn.xml",
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
            "sim_clear_scan = sphero_rvr_driver.sim_clear_scan:main",
            "sim_raycast_scan = sphero_rvr_driver.sim_raycast_scan:main",
            "sim_laggy_map_tf = sphero_rvr_driver.sim_laggy_map_tf:main",
            "refusal_watcher = sphero_rvr_driver.refusal_watcher_node:main",
            "lidar_collision_stop_supervisor = sphero_rvr_driver.collision_stop_node:main",
            "decisive_controller = sphero_rvr_driver.decisive_controller_node:main",
            "coverage_explorer = sphero_rvr_driver.coverage_explorer_node:main",
            "contact_marker = sphero_rvr_driver.contact_marker_node:main",
            "vlm_scene = sphero_rvr_driver.vlm_scene_node:main",
            "tof = sphero_rvr_driver.tof_node:main",
            "vlm_explorer = sphero_rvr_driver.vlm_explorer_node:main",
            "semantic_map = sphero_rvr_driver.semantic_map_node:main",
            "task_node = sphero_rvr_driver.task_node:main",
            "task_client = sphero_rvr_driver.task_client:main",
            "web_console = sphero_rvr_driver.web_console_node:main",
            "recognition = sphero_rvr_driver.recognition_node:main",
        ],
    },
)
