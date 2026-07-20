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
                "launch/camera.launch.py",
            ],
        ),
        (
            "share/sphero_rvr_driver/config",
            [
                "config/rvr.yaml",
                "config/collision_stop.yaml",
                "config/range_motion.yaml",
                "config/lidar.yaml",
                "config/slam_toolbox.yaml",
                "config/camera.yaml",
                "config/mission_planner.yaml",
                "config/live_route_runner.yaml",
            ],
        ),
        (
            "share/sphero_rvr_driver/scripts",
            [
                "scripts/install-rvr-pi",
                "scripts/rvr-camera-node",
                "scripts/rvr-console",
                "scripts/rvr-shoe-detector-eval",
                "scripts/rvr-slam-replay-plan",
                "scripts/rvr_motion_calibration.py",
            ],
        ),
        (
            "share/sphero_rvr_driver/docs",
            [
                "docs/mapping.md",
                "docs/motion_calibration.md",
                "docs/rosbag_capture_replay.md",
                "docs/camera_lidar_calibration.md",
                "docs/lidar_collision_stop_supervisor.md",
                "docs/range_motion_controller.md",
                "docs/mission_api.md",
                "docs/mission_api_v2.md",
                "docs/mission_controls.md",
                "docs/mission_language.md",
                "docs/mission_planner.md",
                "docs/rvr_mcp_server.md",
                "docs/mission_observability.md",
                "docs/semantic_map_artifacts.md",
                "docs/supervised_coordinator.md",
                "docs/slam_replay.md",
                "docs/vertical_slice_capability_matrix.md",
                "docs/shoe_detector_replay.md",
                "docs/shoe_map_projection.md",
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
            "rvr_tui = sphero_rvr_driver.tui:main",
            "lidar_collision_stop_supervisor = sphero_rvr_driver.collision_stop_node:main",
            "range_motion_controller = sphero_rvr_driver.range_motion_node:main",
            "live_route_runner = sphero_rvr_driver.live_route_runner_node:main",
            "rvr_rosbag_capture = sphero_rvr_driver.rosbag_workflow:capture_main",
            "rvr_rosbag_replay = sphero_rvr_driver.rosbag_workflow:replay_main",
            "rvr_rosbag_inspect = sphero_rvr_driver.rosbag_workflow:inspect_main",
            "rvr_slam_replay_plan = sphero_rvr_driver.slam_replay_workflow:main",
            "rvr_shoe_detector_eval = sphero_rvr_driver.shoe_detector:main",
            "rvr_shoe_map_project = sphero_rvr_driver.shoe_map_projection:main",
            "rvr_semantic_map_artifacts = sphero_rvr_driver.semantic_map_artifacts:main",
            "rvr_mcp_server = sphero_rvr_driver.rvr_mcp_server:main",
        ],
    },
)
