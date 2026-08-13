"""The ToF node's state line and the recorder's parser must speak the SAME vocabulary.

This exists because they did not. The node was renamed to publish `rule_a_zones=` and
`rule_b_zones=` with the 9.x amendment; the recorder went on scraping `rule_i_zones=`
and `rule_ii_zones=`. Nothing failed. The recorder would have written a healthy-looking
CSV with two empty columns for an entire mission, and the emptiness would have been
discovered during analysis -- after the flight it was supposed to measure.

ASSERT, DON'T INFER, AT SEAMS: the recorder INFERS the node's vocabulary by string
matching. That inference is now checked against the node's own source rather than
against a memory of it, so a rename breaks a test instead of a mission.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "src" / "sphero_rvr_driver" / "tof_node.py"
RECORDER = ROOT / "diagnostics" / "run_recorder.py"


def _published_tokens():
    """Every `name=` token the node writes into its state string."""
    src = NODE.read_text()
    block = src[src.index("state.data = ("):]
    block = block[:block.index("self._state_pub.publish(state)")]
    return set(re.findall(r"([a-z_]+)=\{?", block))


def _scraped_tokens():
    """Every `name=` prefix the recorder looks for in that string."""
    src = RECORDER.read_text()
    block = src[src.index("def _on_tof"):]
    block = block[:block.index("def _on_cmd")]
    return set(re.findall(r'startswith\("([a-z_]+)="\)', block))


def test_every_token_the_recorder_scrapes_is_one_the_node_publishes():
    published, scraped = _published_tokens(), _scraped_tokens()
    missing = scraped - published
    assert not missing, (
        f"the recorder scrapes {sorted(missing)} but the ToF node publishes "
        f"{sorted(published)} -- those columns will be silently EMPTY for a whole "
        "mission, which is how a recorder looks healthy while recording nothing")


def test_the_columns_that_matter_are_actually_scraped():
    """The reverse direction is deliberately NOT symmetric -- the node may publish more
    than the CSV carries. But these four are the flight's data product and a recorder
    that drops one of them has failed at its only job."""
    scraped = _scraped_tokens()
    for token in ("obstacle_zones", "rule_a_zones", "rule_b_zones", "background"):
        assert token in scraped, (
            f"the recorder does not scrape {token!r}; without it a mission CSV cannot "
            "say which rule concluded what, or whether rule B had a lidar at all")


def test_the_csv_header_matches_the_row_it_writes():
    """A header and a row built in two different places is a column-shift waiting to
    happen -- and a shifted column reads as data, not as an error."""
    src = RECORDER.read_text()
    header = re.search(r'"tof_state",\s*"tof_rate",\s*"tof_obstacles",\s*\n\s*(.*?),\n', src)
    row = re.search(r'self\.tof_state,\s*self\.tof_rate,\s*self\.tof_obstacles,\s*\n\s*(.*?),\n', src)
    assert header and row, "could not locate the ToF header/row pair in the recorder"
    header_cols = [c.strip().strip('"') for c in header.group(1).split(",") if c.strip()]
    row_cols = [c.strip().replace("self.", "") for c in row.group(1).split(",") if c.strip()]
    assert header_cols == row_cols, (
        f"ToF header {header_cols} does not match the row {row_cols}")
