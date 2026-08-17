# BANKED: `IsBlindContact` as an actual BT node — a packaging decision, not a coding one

**Status: the DECISION is built and proven (`src/sphero_rvr_core/blind_contact.py`,
`tests/test_blind_contact.py`, 13 tests against mission 2's recorded chair-leg contact).
The BT PLUGIN is not, and stopping here was deliberate.**

## Why it stopped

Nav2's behaviour-tree conditions are **C++ plugins** (BehaviorTree.CPP, loaded by
`bt_navigator` from a shared library). `sphero_rvr_driver` is a **pure Python package** —
`setup.py`, no `CMakeLists.txt`, no ament_cmake target. There is no place in this repo to
put a `.so` today.

That is a **packaging and build-system decision bigger than the node**, and it has at least
three shapes, each with consequences someone should choose deliberately rather than
discover:

1. **A second package** (`sphero_rvr_bt_nodes`, ament_cmake) holding C++ BT plugins. Clean
   separation, but adds a C++ build to a repo that has been Python-only, and the Pi's
   build time and toolchain become part of every deploy.
2. **Convert `sphero_rvr_driver` to `ament_cmake_python` + C++**. One package, but touches
   every existing build and install path — including the `setup.py` data_files manifest
   whose gaps already cost this project a config that could not be loaded.
3. **Don't write a BT node at all.** Have a Python node consume `/diagnostics`, evaluate
   `blind_contact.evaluate`, and publish a latched `/blind_contact` topic that a stock
   `IsTopicTrue`-style condition (or the LLM layer) reads. **No C++, no new package**, and
   the decision stays in the layer that owns the facts.

**Option 3 is worth serious consideration and is not obviously worse.** The BT node's only
real advantage is that the tree can branch on it directly; a latched topic gets the same
information into the tree through a stock condition, and keeps the logic where its tests
already live. The reason not to decide it tonight is that the answer depends on how much
else moves into the BT under Option A — if the cause-conditioned recovery frame and the 2c
chooser both become BT nodes, a C++ package pays for itself; if they don't, option 3 is
strictly cheaper.

## What is already done and needs no revisiting

* **The driver counts stall transitions** (`motor_stall_events`) and records the last one's
  timestamp, and `/diagnostics` publishes the counter. This is D48's close criterion: a
  level sampled at 1 Hz cannot see a stall that starts and clears inside a second; a
  monotonic counter cannot miss one however brief. The single stall this project has
  recorded survived only because it happened to last 2 s.
* **The decision is written and tested**, including the three failure modes it exists to
  prevent: an uncommanded robot, a dead command path (mission 1's trap — calling that
  contact would convict the room for the driver's silence), and a robot that actually
  moved.
* **It fires on the real contact and nowhere else** across all 654 samples of mission 2.

## What the next session needs

1. Scott's or the PM's ruling between the three shapes above — informed by §3a's result,
   since that determines how much of the middle is BT-shaped at all.
2. Whichever shape wins, the consumer needs a **live** proof: mission 2's bag predates the
   counter, so `test_blind_contact.py` reconstructs it from flag transitions and therefore
   *undercounts*. The next mission's bag will carry `motor_stall_events` directly, and that
   is the recording to re-prove against — at which point the reconstruction in the test
   fixture can be deleted.
