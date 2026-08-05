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


def query_vlm(base_url, api_key, model, prompt, jpeg_bytes, max_tokens=300, timeout=30.0, retries=3):
    """Return the text reply for an image+prompt, retrying past empty/garbage
    (`<|...|>` template) responses."""
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
    last = ""
    for _ in range(retries):
        r = requests.post(
            f"{base_url.rstrip('/')}/chat/completions",
            headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
            json=payload,
            timeout=timeout,
        )
        r.raise_for_status()
        content = r.json()["choices"][0]["message"]["content"].strip()
        if len(content) >= 5 and "<|" not in content:
            return content
        last = content
    raise RuntimeError(f"VLM returned no usable text after retries (last: {last!r})")


def extract_json(text):
    """Parse the first {...} JSON object found in `text` (models wrap JSON in prose
    or code fences). Raises ValueError if none parses."""
    for match in re.finditer(r"\{.*?\}", text, re.DOTALL):
        try:
            return json.loads(match.group(0))
        except json.JSONDecodeError:
            continue
    # Fall back to the greediest span (nested braces).
    m = re.search(r"\{.*\}", text, re.DOTALL)
    if m:
        try:
            return json.loads(m.group(0))
        except json.JSONDecodeError:
            pass
    raise ValueError(f"no JSON object in VLM reply: {text!r}")


def direction_to_goal(robot_x, robot_y, robot_yaw, turn_deg, distance_m):
    """Map-frame goal point `distance_m` ahead of the robot, rotated `turn_deg`
    (negative = left, positive = right) from its current heading."""
    goal_yaw = robot_yaw - math.radians(turn_deg)  # +deg = right = clockwise = -yaw
    return (robot_x + distance_m * math.cos(goal_yaw), robot_y + distance_m * math.sin(goal_yaw), goal_yaw)
