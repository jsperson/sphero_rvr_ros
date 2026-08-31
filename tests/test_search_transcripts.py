"""Canned-transcript tests for SEARCH — the two-stage loop as a system prompt.

Search round 2's ratified shape (bridge consensus item 2): search is a SYSTEM
PROMPT plus the existing nine tools — no choreography code. What must therefore
be proven here is that the CONTRACT LAYER carries every shape the search loop
takes: candidate→approach→confirm, refusal recovery, honest not-found,
close-range mismatch pruning, and the hallucination/reprompt path — each as a
canned transcript through `run_instruction`, with no network and no robot.

The model replies are canned strings and the tool results are the REAL wire
shapes (tool_result JSON from the bridge; the recognition verb's own result
JSON or REFUSED text forwarded verbatim for look_and_recognize). The tests pin
the plumbing, and the transcripts double as the documented intended behavior
the live model is prompted toward.
"""

import json

import pytest

from sphero_rvr_core.task_agent import (
    SYSTEM_PROMPT,
    Budget,
    run_instruction,
)


class ScriptedModel:
    """Plays back canned replies; records every prompt it was shown."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.prompts = []

    def __call__(self, system, prompt):
        assert system == SYSTEM_PROMPT
        self.prompts.append(prompt)
        return self.replies.pop(0)


class ScriptedRunner:
    """Plays back canned tool results; records every call made."""

    def __init__(self, results):
        self.results = list(results)
        self.calls = []

    def run(self, tool, args):
        self.calls.append((tool, args))
        return self.results.pop(0)


def drive(replies, results, budget=None,
          instruction="search the room for the dr pepper bottle"):
    model = ScriptedModel(replies)
    runner = ScriptedRunner(results)
    lines = []
    run_instruction(instruction, model, runner,
                    budget or Budget(), out=lines.append)
    assert not model.replies, "transcript ended with unplayed model replies"
    assert not runner.results, "transcript ended with unplayed tool results"
    return model, runner, lines


# The verb's real result shape (forwarded verbatim by the bridge), abridged to
# the fields the search loop consumes.
def look_result(match, identity, where=None, bearing=None, conf=0.8, desc="…"):
    return json.dumps({
        "target": "dr pepper bottle", "match": match, "identity": identity,
        "where_in_frame": where, "confidence": conf, "description": desc,
        "photo_path": "/home/pi/recognitions/x.jpg",
        "map_pose": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        "bearing_deg": bearing, "stamp": "s", "model": "syn:large:vision"})


OK_GOTO = '{"ok": true, "tool": "goto", "outcome": "SUCCEEDED"}'
OK_STOP = '{"ok": true, "tool": "stop"}'
OK_TURN = '{"ok": true, "tool": "turn", "message": "turned -40 degrees (firmware-settled)"}'


# --- the canonical two-stage transcript --------------------------------------

def test_candidate_approach_confirm_is_carried_end_to_end():
    """Standoff look yields unverified at a bearing → goto TOWARD it (short of
    it) → close look confirms → the find is reported. The contract layer must
    carry each hop and replay every result into the next prompt."""
    model, runner, lines = drive(
        replies=[
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"tool": "goto", "args": {"x": 0.75, "y": 0.3}}',
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"say": "Found the Dr Pepper bottle about a metre ahead-left of where I started."}',
        ],
        results=[
            look_result(True, "unverified", "left", 22.0,
                        desc="a bottle with a reddish label, brand unreadable"),
            OK_GOTO,
            look_result(True, "confirmed", "center", 20.0, conf=0.93,
                        desc="Dr Pepper branding clearly visible"),
        ])
    assert [c[0] for c in runner.calls] == \
        ["look_and_recognize", "goto", "look_and_recognize"]
    assert lines[-1].startswith("robot> Found the Dr Pepper bottle")
    # history plumbing: the approach prompt saw the candidate's bearing, and
    # the final turn saw the confirmed identity.
    assert "unverified" in model.prompts[1] and "22.0" in model.prompts[1]
    assert "confirmed" in model.prompts[3]


# --- refusal recovery ---------------------------------------------------------

def test_a_moving_refusal_recovers_via_stop_then_look():
    """The verb's stationarity refusal (forwarded verbatim) must flow back so
    the model can stop first and re-look — the prompt's stated recovery."""
    _, runner, lines = drive(
        replies=[
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"tool": "stop", "args": {}}',
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"say": "Found it just right of centre, about a metre out."}',
        ],
        results=[
            "REFUSED: motion is commanded — the camera never runs concurrent "
            "with driving (the charter)",
            OK_STOP,
            look_result(True, "confirmed", "right", -20.0, conf=0.9),
        ])
    assert [c[0] for c in runner.calls] == ["look_and_recognize", "stop",
                                            "look_and_recognize"]
    assert lines[-1].startswith("robot> Found")


def test_a_disabled_tool_is_reported_not_guessed_around():
    """The bridge's DISABLED text ends the search with an honest report — the
    transcript spends nothing else."""
    _, runner, lines = drive(
        replies=[
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"say": "I cannot search by camera right now: the recognition tool '
            'is disabled pending its bench certification."}',
        ],
        results=[
            '{"ok": false, "tool": "look_and_recognize", "message": "this tool '
            'is DISABLED until its bench certification passes '
            '(docs/bench_card_recognition_2026-08-19.md). Report that honestly; '
            'do not guess at what the camera would see."}',
        ])
    assert len(runner.calls) == 1
    assert "disabled" in lines[-1]


# --- the honest endings -------------------------------------------------------

def test_close_range_mismatch_prunes_and_the_search_ends_honestly():
    """PIN 1's close-range rule in transcript form: the approach look says
    mismatch → the candidate is pruned, the search continues elsewhere, and a
    covered room without a confirm ends in not-found WITH where it looked."""
    _, runner, lines = drive(
        replies=[
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"tool": "goto", "args": {"x": 0.7, "y": -0.3}}',
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"tool": "turn", "args": {"degrees": -40}}',
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"say": "Not found. The bottle-like object by the couch is a Coke '
            'bottle, and the rest of the room I could see from here has no '
            'bottle at all."}',
        ],
        results=[
            look_result(True, "unverified", "right", -22.0),
            OK_GOTO,
            look_result(True, "mismatch", "center", -2.0, conf=0.9,
                        desc="the label clearly reads Coca-Cola"),
            OK_TURN,
            look_result(False, None, desc="no bottle of any kind in view"),
        ])
    assert [c[0] for c in runner.calls] == [
        "look_and_recognize", "goto", "look_and_recognize", "turn",
        "look_and_recognize"]
    assert lines[-1].startswith("robot> Not found")
    assert "Coke" in lines[-1] or "Coca" in lines[-1]


def test_unverified_at_close_range_is_itself_the_answer():
    _, runner, lines = drive(
        replies=[
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"tool": "goto", "args": {"x": 0.75, "y": 0.3}}',
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"say": "There is a bottle right where I looked, but I could not '
            'verify it is a Dr Pepper even up close — the label is not '
            'readable."}',
        ],
        results=[
            look_result(True, "unverified", "left", 22.0),
            OK_GOTO,
            look_result(True, "unverified", "center", 1.0, conf=0.7,
                        desc="a bottle with an illegible reddish label"),
        ])
    assert lines[-1].startswith("robot> There is a bottle")
    assert "could not" in lines[-1] and "verify" in lines[-1]


# --- contract violations mid-search -------------------------------------------

def test_a_hallucinated_approach_tool_is_reprompted_once_and_corrected():
    """The obvious search hallucination is a bespoke 'approach' verb. It must
    cost a reprompt, not a crash and not a silent drop — and the correction
    must reach the model so the transcript can recover with goto."""
    model, runner, lines = drive(
        replies=[
            '{"tool": "approach", "args": {"bearing_deg": 22.0}}',
            '{"tool": "goto", "args": {"x": 0.75, "y": 0.3}}',
            '{"say": "Moved toward the candidate."}',
        ],
        results=[OK_GOTO])
    assert [c[0] for c in runner.calls] == ["goto"], \
        "the hallucinated tool must never reach the runner"
    assert any(line.startswith("[reprompt]") for line in lines)
    assert "approach" in model.prompts[1] and "rejected" in model.prompts[1]


def test_the_model_is_told_its_remaining_budget_every_turn():
    """Flights 1+4 ended wordless because the model cannot count a budget it is
    never shown — the calls-remaining line must reach every prompt."""
    model, _, _ = drive(
        replies=[
            '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}',
            '{"say": "Nothing seen."}',
        ],
        results=[look_result(False, None)],
        budget=Budget(max_tool_calls=5))
    assert "You have 5 tool call(s) remaining" in model.prompts[0]
    assert "You have 4 tool call(s) remaining" in model.prompts[1]
    assert "MUST finish with say" in model.prompts[0]


def test_budget_exhaustion_demands_a_final_say_and_gets_one():
    """THE BELT (flight 4: the last call went to a redundant query and the
    mission ended wordless): after the last tool call, the model gets ONE
    no-tool turn demanding say — the honest partial answer."""
    looks = '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}'
    model, runner, lines = drive(
        replies=[looks, looks,
                 '{"say": "Not found in the two views I checked."}'],
        results=[look_result(False, None)] * 2,
        budget=Budget(max_tool_calls=2))
    assert len(runner.calls) == 2
    assert "NO tool calls left" in model.prompts[2]
    assert lines[-1].startswith("robot> Not found")


def test_a_model_that_ignores_the_final_say_demand_ends_in_the_loops_words():
    looks = '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}'
    _, runner, lines = drive(
        replies=[looks, looks],
        results=[look_result(False, None)],
        budget=Budget(max_tool_calls=1))
    assert len(runner.calls) == 1, "the post-budget tool reply must not run"
    assert lines[-1].startswith("[budget] stopping after 1")
    assert "did not give one" in lines[-1]


# --- the configuration preamble (§5) -------------------------------------------

def test_availability_note_names_only_the_missing_tools():
    from sphero_rvr_core.task_agent import availability_note
    note = availability_note({"explore": False, "observe": False,
                              "status": False, "goto": True,
                              "look_and_recognize": True})
    assert "explore" in note and "observe" in note and "status" in note
    assert "goto" not in note and "look_and_recognize" not in note
    assert "do not call" in note
    assert availability_note({"goto": True, "turn": True}) == ""


def test_a_model_call_failure_ends_the_instruction_in_words():
    """Flight 2's crash (2026-08-20): query_text raised after empty retries and
    the client died holding a live search candidate. The loop must end HONESTLY
    on a failed model call — transcript line, clean return, no propagation, and
    the runner is never touched again."""
    calls = {"n": 0}

    def failing_model(system, prompt):
        calls["n"] += 1
        if calls["n"] == 1:
            return '{"tool": "look_and_recognize", "args": {"target": "dr pepper bottle"}}'
        raise RuntimeError("model returned no usable text after retries (last: '')")

    runner = ScriptedRunner([look_result(True, "unverified", "right", 159.8)])
    lines = []
    run_instruction("search the room for the dr pepper bottle",
                    failing_model, runner, Budget(), out=lines.append)  # must NOT raise
    assert len(runner.calls) == 1, "no tool may run after the model call fails"
    assert any(line.startswith("[model-failure]") for line in lines)
    assert "no usable" in " ".join(lines)


def test_the_default_transcript_writer_flushes_per_line():
    """Flights 2+3: plain print block-buffered the transcript to its file, so a
    LIVE flight looked dead from outside ("nothing is running") and events
    arrived in one burst at exit. The loop's default writer must flush."""
    import inspect
    from sphero_rvr_core.task_agent import _say, run_instruction
    assert inspect.signature(run_instruction).parameters["out"].default is _say
    assert "flush=True" in inspect.getsource(_say)


def test_the_client_default_token_headroom_carries_the_truncation_lesson():
    """Source pin: task_client's --max-tokens default is 1500 (json_mode models
    reason before the JSON; 500 truncated to empty at flight 2's history-heavy
    call 11). The comment must cite the trap so the next model path checks
    BEFORE flying."""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" /
           "task_client.py").read_text()
    assert '"--max-tokens", type=int, default=1500' in src
    assert "truncates to nothing" in src
    assert "default=500)" not in src, \
        "the old default must not linger anywhere in the client"


# --- the stanza itself ---------------------------------------------------------

def test_the_system_prompt_carries_the_search_loop():
    """The searcher IS the prompt (no choreography code), so the prompt must
    state the loop: two honest endings, candidate-not-find, the approach step,
    the close-range rules, and the range-scaled mismatch authority (PIN 1)."""
    assert "two-stage loop" in SYSTEM_PROMPT
    assert "CANDIDATE, not a find" in SYSTEM_PROMPT
    assert "after searching" in SYSTEM_PROMPT and "where you looked" in SYSTEM_PROMPT
    assert "about 0.8 m" in SYSTEM_PROMPT          # min confirm range (flight 4's miss)
    assert "range minus about a metre" in SYSTEM_PROMPT   # range-aware stop
    assert "use it to place your approach stop" in SYSTEM_PROMPT
    # ruling C: an ambiguous range is a MINIMUM, never the certified distance
    assert "range_ambiguous" in SYSTEM_PROMPT
    assert "treat range_m as a MINIMUM" in SYSTEM_PROMPT
    # flight 5's wrong turn: frames are STATED, and the spendable number named
    # (assert-don't-infer applies to prose too). Collapse whitespace so line
    # wrapping can't hide a phrase.
    flat = " ".join(SYSTEM_PROMPT.split())
    assert "FRAMES, stated exactly" in flat
    assert "bearing_deg is in the MAP frame" in flat
    assert "never hand it to turn" in flat
    assert "bearing_relative_deg is how far to turn RIGHT NOW" in flat
    assert "turn by bearing_relative_deg to face it" in flat
    assert "demotes a candidate" in SYSTEM_PROMPT
    # the reasoning discount (flight 3's root cause: the model burned its cap
    # on bearing trigonometry) — approximate coordinates are LICENSED:
    assert "Roughly toward" in SYSTEM_PROMPT
    assert "precision comes" in SYSTEM_PROMPT
    # round 2 (three flights of receipts): the discovery-tax and re-query
    # rules, and the endgame say rule keyed to the now-visible budget:
    assert "will not become available" in SYSTEM_PROMPT
    assert "Do not repeat a query that answered empty" in SYSTEM_PROMPT
    assert "ends without say wastes everything" in SYSTEM_PROMPT
    assert "when 1 remains you MUST reply" in SYSTEM_PROMPT
    assert "misread at range" in SYSTEM_PROMPT
    assert "could not verify" in SYSTEM_PROMPT


def test_the_stanza_added_no_tool_and_loosened_no_count():
    """Search round 2's ratified shape: a prompt and the EXISTING tools. The
    closed set stays counted and the prompt still says so. (Widened 9 -> 10 on
    2026-08-21 for `clear_map`, Scott's move-the-rover order — a deliberate
    edit here, exactly as this guard demands.)"""
    from sphero_rvr_core.task_agent import TOOL_SCHEMAS
    assert len(TOOL_SCHEMAS) == 11   # 2026-08-31: move_relative, PM-ratified
    assert "eleven tools" in SYSTEM_PROMPT
