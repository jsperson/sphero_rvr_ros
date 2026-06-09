from pathlib import Path


def test_rvr_console_script_sources_ros_and_workspace():
    script = Path("scripts/rvr-console").read_text()

    assert "source /opt/ros/jazzy/setup.bash" in script
    assert "source $WORKSPACE/install/setup.bash" in script
    assert "ros2 launch sphero_rvr_driver rvr.launch.py" in script
    assert "ros2 run sphero_rvr_driver rvr_tui" in script
    assert "ros2 service call /stop" in script


def test_rvr_console_script_logs_startup_and_driver_details():
    script = Path("scripts/rvr-console").read_text()

    assert "LOG_FILE" in script
    assert "rvr-console starting" in script
    assert "driver_log" in script
    assert "verified topic" in script
    assert "cleanup started" in script
