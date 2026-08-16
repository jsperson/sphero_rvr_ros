"""Stage three: try the ways out, one at a time, and look again after every failure.

Stage one MEASURED (`escape_survey`), stage two RANKED (`escape_plan`). This stage
SEQUENCES, and it still commands nothing itself -- it hands out one attempt at a time
and is told what happened. The caller owns the transport; the supervisor remains the
sole arbiter. Keeping the sequencing pure is what lets the whole escalation be proved
against the three recorded wedges without a robot.

THE DEFECT THIS EXISTS TO KILL is the shape of 2026-08-15's run 1 and mission 2 alike:
the rover tried the SAME move repeatedly against a world it never re-examined. Four
arc attempts, four refusals, 21 s, one pose. The give-up escape did not lack a
recovery -- it lacked a SECOND IDEA and a fresh look.

FOUR RULES, each with a pose behind it:

1. **A FAILURE OBLIGES A FRESH SURVEY, and this module refuses to continue without
   one.** "The world after a failed attempt is not the world the first survey saw"
   is a design sentence that a `survey=None` default would quietly turn into a lie,
   so `report()` RAISES rather than reusing the stale one. The rover has moved, or
   tried to; a mark may have been planted; the thing behind it may have shifted.
   Re-ranking a stale survey is how a ladder becomes a loop.

2. **A DIRECTION THAT FROZE IS RETIRED FOR THE REST OF THIS ESCAPE.** A freeze is the
   robot's touch sense reporting an obstacle no sensor on it can see (all five freezes
   on 2026-08-15 were exactly that). Re-proposing that heading after a fresh survey is
   guaranteed, because the survey CANNOT see what caused the freeze -- the sensors
   will keep calling it open. This is the one place where a belief must outlive the
   evidence, and it is the same principle as the D39 hold one layer up: silence from
   a sensor that could not have seen is not clearance.

3. **A REFUSED SHAPE IS RETIRED, BUT ONLY THAT SHAPE AT THAT CLOCK.** The arbiter
   refused this command here; it may well grant a different kind toward the same
   opening, and it may grant this one after the rover has rotated. Retiring the whole
   direction on a refusal would throw away most of the escape vocabulary for a reason
   the arbiter never gave.

4. **`declined` ABORTS THE ESCAPE. It is never retried.** It means two nodes disagree
   about which of them is driving (`escape_outcome`), and a quiet retry there rebuilds
   the give-up livelock from the other side -- the exact failure the outcome word was
   invented to expose.

EXHAUSTION IS AN ESCALATION, NOT A VERDICT. When the candidates run out, the result
is `ASK_HUMAN` carrying the last survey, because the survey is already the content of
the human plea (one artifact, three consumers). The honest blocked ending sits behind
the human, never in front of them.

D34 COMPOSITION lives in the caller and this module makes it checkable: `in_progress`
is true from the first attempt until a terminal result, so a node can refuse to start
a goal mid-escape and a test can assert it did. No goal starts mid-escape; that rule
cost 26 s of pushing a bench leg on 2026-08-11 and is not re-litigated here.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

from sphero_rvr_core.escape_outcome import CLEARED, DECLINED, FROZEN, OUTCOMES, REFUSED
from sphero_rvr_core.escape_plan import ExitCandidate, PlanConfig, PlanGates, rank_candidates
from sphero_rvr_core.escape_survey import Survey

#: Terminal results.
ESCAPED = "escaped"          # a candidate cleared; the pose changed
ASK_HUMAN = "ask_human"      # candidates exhausted -- the next rung is a person
ABORTED = "aborted"          # `declined`: the nodes disagree about who is driving
RESULTS = (ESCAPED, ASK_HUMAN, ABORTED)


class StaleSurvey(RuntimeError):
    """Raised when a failure is reported without a fresh survey.

    RAISED, NOT DEFAULTED, and the distinction is the whole rule. A `survey=None`
    default would silently re-rank the previous snapshot, which reads as adaptive and
    is a fixed ladder -- the exact behaviour that produced four identical attempts at
    one pose. A caller that cannot survey has not failed to pass an argument; it has
    lost the ability to escape, and that must surface here rather than downstream.
    """


@dataclass(frozen=True)
class Attempt:
    """One candidate, handed out to be executed through the supervisor."""

    candidate: ExitCandidate
    attempt_index: int
    survey: Survey
    #: Why this one and not the one before it -- for the log and the human plea.
    reason: str


@dataclass(frozen=True)
class Result:
    """A terminal outcome for the whole escape."""

    result: str
    survey: Survey
    attempts: Tuple[ExitCandidate, ...]
    detail: str = ""


@dataclass
class EscapeExecution:
    """Sequences one escape. Commands nothing; hands out attempts and is told results."""

    gates: PlanGates
    config: PlanConfig = field(default_factory=PlanConfig)

    _survey: Optional[Survey] = field(default=None, init=False)
    _attempted: list = field(default_factory=list, init=False)
    _frozen_clocks: set = field(default_factory=set, init=False)
    _refused_shapes: set = field(default_factory=set, init=False)
    _done: bool = field(default=False, init=False)
    _started: bool = field(default=False, init=False)

    @property
    def in_progress(self) -> bool:
        """True from the first attempt until a terminal result. D34's hook: a node
        asks THIS rather than inferring from whether a controller looks busy, because
        inferring another component's state by proxy is how three defects got in."""
        return self._started and not self._done

    @property
    def frozen_clocks(self) -> frozenset:
        """Directions retired by a freeze. Exposed so the caller can plant marks and a
        test can assert the retirement happened, rather than infer it from a choice."""
        return frozenset(self._frozen_clocks)

    def begin(self, survey: Survey, cause: str):
        """Start an escape from this pose. Returns an `Attempt` or a terminal `Result`."""
        if self._started:
            raise RuntimeError("this execution has already begun; build a new one per escape")
        self._started = True
        self._survey = survey
        return self._next(cause, "first_candidate")

    def report(self, outcome: str, survey: Optional[Survey] = None, detail: str = ""):
        """Report what the supervisor and the world did with the last attempt.

        `survey` is REQUIRED for every non-clearing outcome -- see `StaleSurvey`.
        """
        if outcome not in OUTCOMES:
            raise ValueError(f"unknown escape outcome {outcome!r}; expected {OUTCOMES}")
        if not self._started or self._done:
            raise RuntimeError("no attempt is outstanding")

        last = self._attempted[-1]

        if outcome == CLEARED:
            return self._finish(ESCAPED, detail or "the pose changed")

        if outcome == DECLINED:
            # Never retried, and deliberately not escalated to a human either: a human
            # cannot fix two nodes disagreeing about which is driving, and asking them
            # to would dress a wiring bug up as a rescue.
            return self._finish(ABORTED, detail or "controller was not idle")

        if survey is None:
            raise StaleSurvey(
                f"outcome {outcome!r} is a failure, and a failure obliges a fresh "
                f"survey before the next candidate is chosen. The world after a failed "
                f"attempt is not the world the first survey saw."
            )

        if outcome == FROZEN:
            # RULE 2. The direction is retired for the rest of this escape, because the
            # fresh survey cannot see what froze us and will keep calling it open.
            self._frozen_clocks.add(last.clock)
        else:                                   # REFUSED
            # RULE 3. Only this shape at this clock; the arbiter said nothing about the
            # rest of the vocabulary.
            self._refused_shapes.add((last.kind, last.clock))

        self._survey = survey
        return self._next(survey.cause, f"after_{outcome}")

    def _next(self, cause: str, reason: str):
        survey = self._survey
        for candidate in rank_candidates(survey, cause, self.gates, self.config):
            if candidate.clock in self._frozen_clocks:
                continue
            if (candidate.kind, candidate.clock) in self._refused_shapes:
                continue
            if not candidate.grantable:
                # Plan-time grantability already sorts these last and carries the gate
                # that would refuse them. Executing one anyway would spend an attempt
                # to learn what the plan already knew (the un-grantable-by-construction
                # family, form 3) -- so they are logged by the plan and skipped here.
                continue
            self._attempted.append(candidate)
            return Attempt(candidate=candidate, attempt_index=len(self._attempted),
                           survey=survey, reason=reason)
        return self._finish(ASK_HUMAN, "candidates exhausted")

    def _finish(self, result: str, detail: str) -> Result:
        self._done = True
        return Result(result=result, survey=self._survey,
                      attempts=tuple(self._attempted), detail=detail)
