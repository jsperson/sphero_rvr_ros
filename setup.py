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
        ("share/sphero_rvr_driver/launch", ["launch/rvr.launch.py", "launch/lidar.launch.py", "launch/mapping.launch.py"]),
        ("share/sphero_rvr_driver/config", ["config/rvr.yaml", "config/lidar.yaml", "config/slam_toolbox.yaml"]),
        (
            "share/sphero_rvr_driver/scripts",
            [
                "scripts/install-rvr-pi",
                "scripts/rvr-camera-node",
                "scripts/rvr-console",
                "scripts/rvr_motion_calibration.py",
            ],
        ),
        (
            "share/sphero_rvr_driver/docs",
            [
                "docs/mapping.md",
                "docs/motion_calibration.md",
                "docs/rosbag_capture_replay.md",
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
            "rvr_rosbag_capture = sphero_rvr_driver.rosbag_workflow:capture_main",
            "rvr_rosbag_replay = sphero_rvr_driver.rosbag_workflow:replay_main",
            "rvr_rosbag_inspect = sphero_rvr_driver.rosbag_workflow:inspect_main",
        ],
    },
)
