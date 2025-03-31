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

```python
#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
import math
```
- #!/usr/bin/env python3: Specifies that the script should be run with Python 3.

- rospy: ROS client library for Python; used to create nodes and interact with topics/services.

- Spawn, Kill: Service definitions from the turtlesim package used to create or remove turtles.

- math: Python module for mathematical functions (e.g., radians, degrees, sqrt, atan2).

```python
def kill_turtle(name):
    rospy.wait_for_service('/kill')
    try:
        kill = rospy.ServiceProxy('/kill', Kill)
        kill(name)
    except rospy.ServiceException:
        rospy.logwarn(f"No se pudo eliminar {name}, probablemente ya fue eliminada.")
```
- Waits for the /kill service to become available.

- Creates a service proxy to call /kill and attempts to remove a turtle with the given name.

- If the turtle doesn't exist, it logs a warning.

```python
def spawn_turtle(x, y, theta_deg, name):
    rospy.wait_for_service('/spawn')
    try:
        theta_rad = math.radians(theta_deg)  # Convert degrees to radians
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta_rad, name)
        return x, y, theta_rad
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al crear la tortuga: {e}")
        return None
```
- Waits for the /spawn service.

- Converts the input angle theta_deg from degrees to radians, since ROS expects radians.

- Calls the /spawn service with the provided coordinates and name.

- Returns the coordinates and orientation of the new turtle.

```python
def main():
    rospy.init_node('turtle_spawn_goal', anonymous=True)
```
- Initializes a new ROS node named turtle_spawn_goal.

```python
x_goal = float(input("Ingresa la coordenada x del goal: "))
    y_goal = float(input("Ingresa la coordenada y del goal: "))
    theta_goal_deg = float(input("Ingresa el ángulo theta del goal (en grados): "))
```
- Prompts the user to input the goal position (x, y) and orientation theta in degrees.

```python
kill_turtle("turtle1")
```
- Removes the default turtle (turtle1) if it's present.

```python
result = spawn_turtle(x_goal, y_goal, theta_goal_deg, "turtle1")
```
- Creates a new turtle at the user-specified position and orientation.

```python
if result:
        x_current, y_current, theta_current = result

        # Calculate Distance to Goal (DTG)
        dtg = math.sqrt((x_goal - x_current)**2 + (y_goal - y_current)**2)

        # Calculate Angle to Goal (ATG)
        atg_rad = math.atan2((y_goal - y_current), (x_goal - x_current))
        atg_deg = math.degrees(atg_rad)

        print(f"\nDistance to Goal (DTG): {dtg:.4f}")
        print(f"Angle to Goal (ATG): {atg_deg:.4f}°")
```
- Because the turtle is spawned at the goal, the position difference is 0, resulting in:

    DTG = 0.0

    ATG = 0.0°

- Nevertheless, the code calculates and prints these values using the standard Euclidean formulas:

$DTG = \sqrt{(x_{goal} - x_{current})^2 + (y_{goal} - y_{current})^2}$

$ATG = \arctan2(y_{goal} - y_{current}, x_{goal} - x_{current})$

```python
if __name__ == '__main__':
    main()
```
- Ensures the script runs the main() function when executed directly (not when imported as a module).

This script is useful for initializing a turtle's position at a specific goal and for verifying positional logic by computing DTG and ATG. It's especially helpful as a debugging tool before implementing motion control systems.
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