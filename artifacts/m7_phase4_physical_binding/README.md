# M7 Phase 4 physical binding evidence

This directory records the no-motion Pi audit for executable candidate
`5b7c96f52b5b21a59655d2d9c102d0d3fee2f4cc` on 2026-07-28.

The candidate was built on `sphero-pi-2` and its bounded binding, WFD, and M6
semantic-goal suites passed 45/45. The ROS graph contained only the installed
read-only Mission Service and the static base-to-lidar transform. No command,
motor, private Nav2 request, hierarchical authority/dispatch, lidar, or camera
topic existed; there were no action servers, hardware Python processes, serial
device owners, or recent camera/rosbag files.

The physical launch was not started. The chassis remained off and no sensor,
driver, serial, Nav2, authority, semantic-controller, perception, bridge, or
supervisor process was activated.

The first combined Pi run exposed a pre-existing one-ULP libm difference in a
reconstructed Next-Best-View clearance between Darwin/arm64 and Linux/aarch64.
The historical OAuth decision still carried its exact captured snapshot ID.
The regression now validates that recorded ID directly and separately proves
that a different snapshot ID is rejected; it does not alter the historical
artifact or weaken live snapshot revalidation.

