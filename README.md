A collection of ROS 2 Humble packages for autonomous mobile robot (AMR) control. This project implements proportional controllers, waypoint navigation, and state-machine logic to drive a TurtleBot3 through complex paths in Gazebo.

## Project Structure
* **`amr_control/`**: Custom C++ package for navigation logic.
    * `waypoint_patrol.cpp`: Main node for multi-point navigation.
    * `go_to_goal.cpp`: Basic P-Controller for single target.
* **`cpp_turtle_control/`**: Introductory TurtleSim controllers.
* **`media/`**: Demo videos and logs.

* ## Key Features
* **Proportional Control (P-Controller):** Smooth acceleration and steering based on distance/angle errors.
* **Pivot-Turn Logic:** Handles "overshoot" by rotating in place if the heading error is too large.
* **Waypoint Navigation:** Accepts a list of (x, y) coordinates to navigate complex shapes.
* **Patrol Mode:** Can loop through waypoints indefinitely for surveillance tasks.
