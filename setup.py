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
        ("share/sphero_rvr_driver/launch", ["launch/rvr.launch.py"]),
        ("share/sphero_rvr_driver/config", ["config/rvr.yaml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Jason Scott Person",
    maintainer_email="jsperson@users.noreply.github.com",
    description="ROS 2 driver for Sphero RVR",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rvr_node = sphero_rvr_driver.rvr_node:main",
        ],
    },
)
