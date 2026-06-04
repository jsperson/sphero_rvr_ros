# sphero_rvr_ros

Concurrency-safe Sphero RVR core driver plus a ROS 2 adapter package.

This project is intentionally starting fresh from the older MCP implementation. The MCP repo remains useful as a protocol reference, but this repo is built around ROS 2 needs: one serial owner, request/response dispatching, safety preemption, and continuous velocity control.

## Initial scope

- `sphero_rvr_core`: async Python core driver and transport abstractions
- `sphero_rvr_driver`: ROS 2-facing adapter/node package

First hardware milestone: `/cmd_vel` teleop with timeout stop, emergency stop, diagnostics, and battery publishing.
