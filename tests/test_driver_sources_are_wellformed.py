"""ROS-free source guards for the driver package — they must run EVERYWHERE.

Born 2026-08-22 from a real escape: a duplicated keyword argument in
coverage_explorer_node.py passed the whole Mac suite and failed at colcon build
on the Pi. Every test that could have caught it lived in a module that begins
`pytest.importorskip("rclpy")`, so on a host without ROS the file was skipped
entirely and the syntax error travelled. Parsing needs no ROS; so does checking
that every report call site carries the same counters.
"""


def test_every_driver_module_at_least_PARSES_on_a_host_without_rclpy():
    """The gap that let a syntax error reach the Pi (2026-08-22): driver modules
    import rclpy, so the Mac suite skips them entirely — a duplicated keyword
    argument passed every test here and failed only at colcon build. Parsing is
    ROS-free, so it can be checked everywhere the suite runs.
    """
    import ast
    from pathlib import Path
    driver = Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
    for path in sorted(driver.glob("*.py")):
        ast.parse(path.read_text(), filename=str(path))


def test_the_report_call_sites_all_carry_the_ledger_counters():
    """Every build_report call site must pass the SAME counter set. The
    START_BLOCKED site omitted the forensic fields until 2026-08-16 and shipped
    a report that lied by omission; this pins the shape so a new counter cannot
    reach one call site and miss another."""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
           / "coverage_explorer_node.py").read_text()
    call_sites = src.count("build_report(")
    for kwarg in ("goals_stall_killed=", "goals_cancelled_at_end=",
                  "planner_rejections=", "standoff_skips="):
        assert src.count(kwarg) == call_sites, (
            f"{kwarg} appears {src.count(kwarg)} times against {call_sites} "
            f"build_report call sites — a counter reaches some reports and not others")
