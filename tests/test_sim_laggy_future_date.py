"""The laggy map->odom publisher's falsifier shape must stay the default."""
def test_the_laggy_publishers_falsifier_shape_is_the_default():
    """future_date_ms (added after cert attempt 1, 2026-08-18: RPP aborted 5/5
    goals on the stamps-trail-now shape that real slam_toolbox's future-dating
    never presents) must DEFAULT to 0 in both the node and the launch -- every
    pre-existing falsifier use stays byte-identical, and mission arms opt in to
    the flight-faithful 200 explicitly."""
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    node_src = (root / "src" / "sphero_rvr_driver" / "sim_laggy_map_tf.py").read_text()
    launch_src = (root / "launch" / "sim_closed_loop.launch.py").read_text()
    assert 'declare_parameter("future_date_ms", 0.0)' in node_src, (
        "the node's falsifier-shape default moved off 0 -- past-stamp lookup "
        "falsifiers would silently stop falsifying")
    assert '"laggy_future_date_ms",\n            default_value="0.0"' in launch_src, (
        "the launch default moved off 0.0")

