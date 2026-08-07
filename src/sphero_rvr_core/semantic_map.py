"""Semantic object memory: what was seen, where it is on the map (Stage C).

The exploration stack knows *geometry* (occupancy, frontiers) but forgets *what*
things are. This is the missing piece the LLM goal layer is supposed to reason
over: a persistent, queryable list of objects with map-frame positions, built up
from repeated observations.

Design notes:
* Observations are noisy (a monocular range guess especially), so an object's
  position is a confidence-weighted running mean, not the latest reading.
* The same physical object is seen many times from different poses. Merging is by
  (label, proximity): a new observation of the same label within `merge_radius_m`
  updates the existing object instead of creating a duplicate.
* Everything here is pure and JSON-serialisable so it can be unit-tested without
  ROS and persisted across runs.
"""

import json
import math


# Words that carry no identity — a VLM sprinkles these differently every call
# ("closed window blinds" vs "window with closed dark blinds"), so they must not be
# what decides whether two sightings are the same object.
_LABEL_STOPWORDS = frozenset(
    """a an the of with and or on in at to for its his her their this that these those
    is are was were be been being small large big little tall short wide narrow
    dark light bright dim pale deep open closed empty full new old plain
    left right front back near far upper lower top bottom side corner
    coloured colored color colour looking looks maybe possibly probably some
    piece object thing item unknown partial visible partially""".split()
)


def _label_tokens(label):
    """Meaningful lowercase word-stems in a label, for identity comparison."""
    out = set()
    for raw in str(label).lower().replace("/", " ").replace("-", " ").split():
        word = "".join(c for c in raw if c.isalnum())
        if len(word) < 3 or word in _LABEL_STOPWORDS:
            continue
        out.add(word[:-1] if word.endswith("s") and len(word) > 4 else word)
    return out


class SemanticObject:
    """One remembered thing: a label, a map position, and its observation history."""

    __slots__ = ("label", "x", "y", "weight", "count", "confidence", "first_seen", "last_seen")

    def __init__(self, label, x, y, confidence=0.5, stamp=0.0):
        self.label = str(label)
        self.x = float(x)
        self.y = float(y)
        self.weight = max(1e-6, float(confidence))
        self.count = 1
        self.confidence = float(confidence)
        self.first_seen = float(stamp)
        self.last_seen = float(stamp)

    def reinforce(self, x, y, confidence, stamp):
        """Fold in another sighting: confidence-weighted mean position."""
        w = max(1e-6, float(confidence))
        total = self.weight + w
        self.x = (self.x * self.weight + float(x) * w) / total
        self.y = (self.y * self.weight + float(y) * w) / total
        self.weight = total
        self.count += 1
        # Repeated sightings raise confidence, but never to certainty.
        self.confidence = min(0.99, max(self.confidence, float(confidence)) + 0.05 * (self.count - 1))
        self.last_seen = max(self.last_seen, float(stamp))

    def distance_to(self, x, y):
        return math.hypot(self.x - float(x), self.y - float(y))

    def to_dict(self):
        return {
            "label": self.label,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "confidence": round(self.confidence, 3),
            "count": self.count,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
        }

    @classmethod
    def from_dict(cls, d):
        o = cls(d["label"], d["x"], d["y"], d.get("confidence", 0.5), d.get("first_seen", 0.0))
        o.count = int(d.get("count", 1))
        o.last_seen = float(d.get("last_seen", o.first_seen))
        o.weight = max(1e-6, float(d.get("confidence", 0.5))) * o.count
        return o


class SemanticMap:
    """Map-frame object memory with merge-on-proximity and simple queries."""

    def __init__(self, merge_radius_m=0.6, min_confidence=0.0):
        self.merge_radius_m = float(merge_radius_m)
        self.min_confidence = float(min_confidence)
        self._objects = []

    def observe(self, label, x, y, confidence=0.5, stamp=0.0):
        """Record a sighting; returns the object it created or reinforced (None if
        the observation is below `min_confidence`)."""
        if float(confidence) < self.min_confidence:
            return None
        match = self._nearest_same_label(label, x, y, self.merge_radius_m)
        if match is not None:
            match.reinforce(x, y, confidence, stamp)
            return match
        obj = SemanticObject(label, x, y, confidence, stamp)
        self._objects.append(obj)
        return obj

    def _nearest_same_label(self, label, x, y, within):
        """Nearest object whose label refers to the same thing, within `within`.

        Matching is on meaningful-token overlap, not string equality: a VLM renames
        the same object every call. One live run produced "closed window blinds" and
        "window with closed dark blinds" at IDENTICAL coordinates as two entries, plus
        "window with frame"/"bright window" and "dark tabletop"/"dark tabletop or
        counter" — 7 entries for ~3 real objects. Proximity still gates the merge, so
        sharing a word is not enough on its own.
        """
        want = _label_tokens(label)
        best, best_d = None, within
        for o in self._objects:
            if not (want & _label_tokens(o.label)):
                continue
            d = o.distance_to(x, y)
            if d <= best_d:
                best, best_d = o, d
        return best

    def objects(self):
        return list(self._objects)

    def query(self, label=None, near=None, radius_m=None, min_confidence=None, min_count=None):
        """Objects filtered by label (case-insensitive substring), proximity to a
        (x, y) point, confidence and sighting count -- nearest first when `near`
        is given, else strongest first."""
        out = []
        for o in self._objects:
            if label is not None and str(label).lower() not in o.label.lower():
                continue
            if min_confidence is not None and o.confidence < float(min_confidence):
                continue
            if min_count is not None and o.count < int(min_count):
                continue
            if near is not None and radius_m is not None:
                if o.distance_to(near[0], near[1]) > float(radius_m):
                    continue
            out.append(o)
        if near is not None:
            out.sort(key=lambda o: o.distance_to(near[0], near[1]))
        else:
            out.sort(key=lambda o: (-o.confidence, -o.count))
        return out

    def forget_stale(self, older_than_stamp):
        """Drop objects last seen before `older_than_stamp`. Returns how many went."""
        before = len(self._objects)
        self._objects = [o for o in self._objects if o.last_seen >= float(older_than_stamp)]
        return before - len(self._objects)

    def to_json(self, indent=None):
        return json.dumps({"objects": [o.to_dict() for o in self._objects]}, indent=indent)

    @classmethod
    def from_json(cls, text, merge_radius_m=0.6):
        m = cls(merge_radius_m=merge_radius_m)
        for d in json.loads(text).get("objects", []):
            m._objects.append(SemanticObject.from_dict(d))
        return m

    def __len__(self):
        return len(self._objects)
