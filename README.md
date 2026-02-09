This project was used to demonstrate algorithms on LEGO Mindstorms. The vision pipeline
tracks the system state (e.g., trailer positions/angles), and the control logic computes actions to reverse the last trailer into the target region.

# Reverse_n_Trailers_ai
Given a vehicle with **n trailers attached**, reverse the last trailer into a target “parking space”.
Built as a robotics/AI demonstration project (LEGO Mindstorms + external vision/control pipeline).

## Highlights
- **Multi-trailer reversing** using predictive/control logic (recursive trajectory reasoning).
- **Computer vision tracking** (color-based tracking + calibration) to estimate pose/state.
- **Geometry utilities** for intersections/constraints used by planning/control.
- **Client/server split** to interface between control logic and hardware/simulator.

## Repository Structure
- `real_reverse.py` — main reversing/control routine (entrypoint for demo runs)
- `color_tracking.py`, `color_tracking_hsv.py` — vision-based tracking utilities
- `camera_caliberation/` — camera calibration helpers/data
- `client.py`, `server.py` — communication layer (robot/hardware interface)
- Geometry helpers:
  - `point.py`
  - `circle_intersect.py`
  - `perpendicular_intersect_circle.py`
  - `calc_exit.py`
- `video_test.py` — camera/vision test harness

## Requirements
- Python 3.x
- Common scientific/vision stack (typical):
  - `numpy`
  - `opencv-python`
  - LEGO Mindstorms / robotics comms libraries

