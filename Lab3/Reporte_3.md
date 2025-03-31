# TurtleSim Lab Report 3
## Introduction

In this laboratory session, we explored the basic concepts of position control using the ROS (Robot Operating System) environment with the Turtlesim simulator. The focus was on implementing two main functionalities:

1. Calculating and displaying the **Distance To Goal (DTG)** and **Angle To Goal (ATG)**.
2. Spawning the turtle directly at a target position and orientation without any motion.
3. Mapping and applying velocity commands appropriately using control principles.
4. Using a (custom) **Proportional controller** in an infinite loop to move the turtle to a desired position and orientation.

This experiment provided hands-on experience with real-time control and coordinate transformations in robotics.

---

## Theoretical Framework

### Euclidean Coordinates

Euclidean coordinates allow us to determine the spatial relationship between two points in a 2D plane. To compute the **Distance To Goal (DTG)**, we use the Euclidean distance formula:

$$
DTG = \sqrt{(x_{goal} - x_{current})^2 + (y_{goal} - y_{current})^2}
$$

To find the **Angle To Goal (ATG)**, which represents the angle between the current heading and the line toward the goal, we use:

$$
ATG = \arctan2(y_{goal} - y_{current}, x_{goal} - x_{current})
$$

This angle must be adjusted to match the orientation system used in Turtlesim (measured in radians or degrees) (Siciliano & Khatib, 2016).

### Proportional Controller (P Controller)

A proportional controller adjusts the control effort proportionally to the error. In this context:

- The **linear velocity** is proportional to the distance to the goal.
- The **angular velocity** is proportional to the difference between the current orientation and the angle to the goal.

$$
v = K_p \cdot DTG,\quad \omega = K_p \cdot (ATG - \theta_{current})
$$

Where:
$$
- $v$ is the linear velocity.
- $\omega$ is the angular velocity.
- $K_p$ is the proportional gain (tuned manually).
$$


This controller is a fundamental feedback control strategy, widely used in mobile robotics for trajectory following and pose regulation (Corke, 2017).

### ROS Topics and Messages

ROS (Robot Operating System) provides communication tools to exchange information between nodes. In this case:

- `/turtle1/pose` provides real-time position and orientation of the turtle.
- `/turtle1/cmd_vel` is used to publish linear and angular velocity commands.
- Services like `/spawn` and `/kill` are used to create or remove turtles in the simulation.

The standard message types `geometry_msgs/Twist` and `turtlesim/Pose` facilitate the communication between controller logic and the robot’s simulation (Quigley et al., 2009).

---

## Problems to Solve

### Problem 1: Calculate and Show DTG & ATG, Spawn Turtle at Goal

#### Problem Description

The goal of this problem is to receive user input for a target position (x, y) and orientation (theta), and then:

- Kill the default turtle if it exists.
- Spawn a new turtle directly at the given coordinates.
- Calculate and print the DTG and ATG based on the current and goal positions.

Since the turtle is spawned directly at the goal, both DTG and ATG are expected to be zero.

#### Code Explanation

- The user is prompted to input the goal position and angle (in degrees).
- The angle is converted from degrees to radians.
- The `/kill` service removes the original turtle.
- The `/spawn` service creates a new turtle at the specified position and orientation.
- The DTG and ATG are calculated using Euclidean formulas.
- The ATG is then converted back to degrees and both values are printed.

This code does not move the turtle—it simply performs geometric calculations and prints them to the console.

---

### Problem 2: Move Turtle to Goal using Proportional Controller

#### Problem Description

In this problem, the turtle is not spawned at the goal. Instead, it starts at an arbitrary position and must move to the user-defined goal using a custom proportional controller.

The system must:
- Continuously read the current pose.
- Compute the DTG and ATG in real-time.
- Send velocity commands to guide the turtle to the goal.

This is done in an **infinite loop**, allowing repeated control sessions.

#### Code Explanation

- The node subscribes to `/turtle1/pose` to obtain real-time pose updates.
- The user is asked to input the target x, y, and theta (in degrees).
- The controller continuously calculates:
  - The Euclidean distance to the goal.
  - The desired heading angle.
  - The angular difference between the current heading and the desired heading.
- A linear velocity proportional to the distance, and an angular velocity proportional to the heading error are published to `/turtle1/cmd_vel`.
- Once the turtle is within a small threshold of the goal, it stops.
- Then, a separate rotation phase aligns the turtle with the desired final orientation using angular velocity only.

This is wrapped inside a loop, allowing the user to input multiple targets without restarting the program.

---

## Conclusions

This laboratory provided practical experience with coordinate geometry and control systems using ROS and Turtlesim. We successfully implemented two control strategies:

1. Direct positioning by spawning at the goal and computing DTG and ATG.
2. Real-time movement to a goal using a proportional controller with Euclidean distance and angle calculation.

Key libraries and tools used:
- `rospy`: ROS client library for Python.
- `geometry_msgs/Twist`: For sending velocity commands.
- `turtlesim/Pose`: To receive turtle position and orientation.
- `math`: For trigonometric and distance calculations.

These exercises reinforce essential robotics concepts such as frame transformations, velocity control, and feedback loops.

---

## References

Corke, P. (2017). *Robotics, Vision and Control: Fundamental Algorithms in MATLAB® (2nd ed.)*. Springer.

Quigley, M., Gerkey, B., Conley, K., Faust, J., Foote, T., Leibs, J., ... & Ng, A. Y. (2009). ROS: an open-source Robot Operating System. In *ICRA workshop on open source software* (Vol. 3, No. 3.2, p. 5).

Siciliano, B., & Khatib, O. (2016). *Springer Handbook of Robotics* (2nd ed.). Springer.

ROS Wiki. (n.d.). Retrieved from http://wiki.ros.org

Turtlesim Package. (n.d.). Retrieved from http://wiki.ros.org/turtlesim