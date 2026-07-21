# Constrained Mission language translator

`src/sphero_rvr_driver/mission_language.py` is the ROS-free plain-English translator for the canonical shoe-mapping fixture:

```text
Map the room and identify every shoe. Put it on a map.
```

The translator is deliberately narrow and deterministic. It does not call an LLM, open a browser, publish ROS topics, call `/cmd_vel`, or expose a generic ROS bridge. Its only outputs are:

- a validated canonical `mission_api.v2` `MissionPlan`; or
- a structured rejection with a reason, message, `requires_confirmation: true`, and evidence about any detected unsafe surface.

## Accepted language

Accepted requests must express the same intent as the canonical command:

- map/build/create a room/space map;
- identify/find/detect/mark/label/locate shoes/footwear;
- keep the object label constrained to shoes.

Examples accepted by the test suite include:

```text
Please map this room and mark all the shoes on the map.
Create a room map, find every shoe, and put the shoes on the map.
Could you identify shoes while mapping the room?
Find all shoes in the room and build a map.
```

## Rejection categories

The translator returns a structured rejection instead of a partial or unsafe schema for:

- unsupported objects such as backpacks, people, pets, keys, cups, or chairs;
- unsupported actions such as picking up, bringing, carrying, pushing, following, chasing, or navigating to targets;
- direct movement requests such as driving forward, turning, spinning, reversing, speed/velocity, or distance/angle commands;
- direct ROS requests mentioning `/cmd_vel`, `/cmd_vel_motor`, `cmd_vel`, raw motor, teleop, publishing, ROS topics, or ROS bridge surfaces;
- prompt-injection-like attempts to ignore/bypass safety rules or expose a ROS bridge;
- Mission API schema validation failures such as an unavailable required registry tool.

Every rejection keeps `unsafe_surfaces_exposed` empty. Detected unsafe text is reported as evidence only; it is never converted into a runnable ROS surface.

## Schema validation

For accepted language, `translate_plain_english_mission()` builds the canonical Mission API plan with `build_canonical_shoe_mapping_plan()` and immediately checks that every invocation is available in the supplied `CapabilityRegistry` or the default registry.

If validation fails, the translator emits `schema_validation_failed` and no schema. This preserves the downstream contract: consumers receive either a validated mission plan or an explicit rejection that requires confirmation.
