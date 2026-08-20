"""Minimal client for an OpenAI-compatible vision API (Synthetic), plus helpers to
turn a VLM reply into a structured exploration decision.

`query_vlm` posts an image + prompt and returns the text (retrying past the
occasional garbage chat-template token). `extract_json` pulls the first JSON object
out of a reply (models often wrap it in prose). `direction_to_goal` converts a
relative turn into a map-frame goal point. The parsing/geometry helpers are pure and
unit-tested; the HTTP call needs the network.
"""

import base64
import json
import math
import re

import requests


def query_vlm(base_url, api_key, model, prompt, jpeg_bytes, max_tokens=300, timeout=30.0, retries=3, json_mode=False):
    """Return the text reply for an image+prompt, retrying past empty/garbage
    (`<|...|>` template) responses. `json_mode=True` sets response_format json_object
    so the reply ends in a JSON object (essential for syn:large:vision, which reasons
    first and otherwise never reaches the JSON). It still reasons, so give it token
    headroom (~1500) — too small a max_tokens truncates before the closing brace."""
    b64 = base64.b64encode(jpeg_bytes).decode("ascii")
    payload = {
        "model": model,
        "max_tokens": max_tokens,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                ],
            }
        ],
    }
    if json_mode:
        payload["response_format"] = {"type": "json_object"}
    last = ""
    for _ in range(retries):
        r = requests.post(
            f"{base_url.rstrip('/')}/chat/completions",
            headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
            json=payload,
            timeout=timeout,
        )
        r.raise_for_status()
        content = _reply_text(r)
        if len(content) >= 5 and "<|" not in content:
            return content
        last = content
    raise RuntimeError(f"VLM returned no usable text after retries (last: {last!r})")


def _reply_text(r):
    """The reply body's content field — or RuntimeError on a 200 whose body has
    the wrong shape. A misbehaving endpoint answering 200-with-garbage raised a
    bare KeyError here, the same refuse-don't-die family as the transport fix
    (bench card R6a, consensus 2026-08-20)."""
    try:
        return r.json()["choices"][0]["message"]["content"].strip()
    except (KeyError, IndexError, TypeError, AttributeError, ValueError) as exc:
        raise RuntimeError(
            f"VLM returned an unexpected body shape: {type(exc).__name__}: {exc}"
        ) from exc


def query_text(base_url, api_key, model, prompt, system=None, max_tokens=500,
               timeout=30.0, retries=3, json_mode=False):
    """Same endpoint, key and retry discipline as `query_vlm`, without an image.

    Exists because the task client reasons over text (a tool contract and tool
    results), not pictures -- so this adds an argument shape, not a dependency or a
    provider. `json_mode` sets response_format json_object, which is what makes a
    model that likes to think out loud still land on a parseable object; see the
    vision path's note about needing token headroom for the same reason.
    """
    messages = []
    if system:
        messages.append({"role": "system", "content": system})
    messages.append({"role": "user", "content": prompt})
    payload = {"model": model, "max_tokens": max_tokens, "messages": messages}
    if json_mode:
        payload["response_format"] = {"type": "json_object"}
    last = ""
    for _ in range(retries):
        r = requests.post(
            f"{base_url.rstrip('/')}/chat/completions",
            headers={"Authorization": f"Bearer {api_key}",
                     "Content-Type": "application/json"},
            json=payload,
            timeout=timeout,
        )
        r.raise_for_status()
        content = _reply_text(r)
        if len(content) >= 2 and "<|" not in content:
            return content
        last = content
    raise RuntimeError(f"model returned no usable text after retries (last: {last!r})")


def extract_json(text):
    """Parse the first COMPLETE JSON object in `text` (models wrap JSON in prose or
    code fences). Raises ValueError if none parses.

    Scans for balanced braces rather than regex-matching. A non-greedy
    ``\\{.*?\\}`` returns an INNER object when the JSON is nested: for
    ``{"objects": [{"label": ...}]}`` the outer span is unbalanced and fails to
    parse, so the first thing that *does* parse is an inner element and a caller
    doing ``.get("objects")`` silently gets nothing. That made the semantic map
    report "0 objects" against a reply listing three (2026-08-07). Flat schemas
    like vlm_explorer's were unaffected, which is why it went unnoticed.
    """
    for start in range(len(text)):
        if text[start] != "{":
            continue
        depth = 0
        in_string = False
        escaped = False
        for i in range(start, len(text)):
            ch = text[i]
            if in_string:
                if escaped:
                    escaped = False
                elif ch == "\\":
                    escaped = True
                elif ch == '"':
                    in_string = False
                continue
            if ch == '"':
                in_string = True
            elif ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    try:
                        return json.loads(text[start : i + 1])
                    except json.JSONDecodeError:
                        break  # not valid JSON -> try the next opening brace
    raise ValueError(f"no JSON object in VLM reply: {text!r}")


def direction_to_goal(robot_x, robot_y, robot_yaw, turn_deg, distance_m):
    """Map-frame goal point `distance_m` ahead of the robot, rotated `turn_deg`
    (negative = left, positive = right) from its current heading."""
    goal_yaw = robot_yaw - math.radians(turn_deg)  # +deg = right = clockwise = -yaw
    return (robot_x + distance_m * math.cos(goal_yaw), robot_y + distance_m * math.sin(goal_yaw), goal_yaw)
