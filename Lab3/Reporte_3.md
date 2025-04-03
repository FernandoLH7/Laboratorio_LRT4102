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

- $v$ is the linear velocity.
- $\omega$ is the angular velocity.
- $K_p$ is the proportional gain (tuned manually).

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
- Ensures the script runs the main() function when executed directly (not when imported as a module). This script is useful for initializing a turtle's position at a specific goal and for verifying positional logic by computing DTG and ATG. It's especially helpful as a debugging tool before implementing motion control systems.
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

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pow, radians, degrees
```
- #!/usr/bin/env python3: Tells the system to run the script with Python 3.

- rospy: Python client library for ROS.

- Twist: ROS message type used to send linear and angular velocities.

- Pose: ROS message type from Turtlesim providing current position and orientation.

- math: Standard math functions used for trigonometry and distance calculations.

```python
def __init__(self):
    rospy.init_node('turtle_proportional_controller', anonymous=True)
    
    self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    self.rate = rospy.Rate(10)  # 10 Hz

    self.x = 0
    self.y = 0
    self.theta = 0
```
- Initializes the ROS node.

- Subscribes to /turtle1/pose to receive real-time position and heading data.

- Publishes velocity commands to /turtle1/cmd_vel.

- Sets the loop rate to 10 Hz.

- Initializes pose variables (x, y, theta).

```python
def pose_callback(self, data):
    self.x = data.x
    self.y = data.y
    self.theta = data.theta
```
- Callback function for the /turtle1/pose subscriber.

- Updates the current pose of the turtle whenever a new message is received.

```python
def get_user_input(self):
    print("\nIngresar nueva posición objetivo")
    x = float(input("Coordenada x del objetivo: "))
    y = float(input("Coordenada y del objetivo: "))
    theta_deg = float(input("Ángulo deseado (grados): "))
    return x, y, radians(theta_deg)
```
- Prompts the user to enter a target x, y position and orientation theta in degrees.

- Converts theta to radians before returning (since ROS uses radians for angles).

```python
def move_to_goal(self, goal_x, goal_y):
    vel_msg = Twist()
    Kp_linear = 1.5
    Kp_angular = 6.0
```
- Initializes a Twist message to send velocity commands.

- Defines proportional gains for linear and angular velocity.

```python
while not rospy.is_shutdown():
        dtg = sqrt(pow(goal_x - self.x, 2) + pow(goal_y - self.y, 2))
        atg = atan2(goal_y - self.y, goal_x - self.x)
        angle_diff = atg - self.theta
        angle_diff = (angle_diff + 3.14159) % (2 * 3.14159) - 3.14159
```
- Calculates Distance To Goal (DTG) using the Euclidean distance formula:

    $DTG = \sqrt{(x_{goal} - x_{current})^2 + (y_{goal} - y_{current})^2}$

- Calculates Angle To Goal (ATG):

    $ATG = \arctan2(y_{goal} - y_{current}, x_{goal} - x_{current})$

- Computes the difference between the current orientation and the ATG.

- Normalizes the angle to the range [−π,π].

```python
vel_msg.linear.x = Kp_linear * dtg
        vel_msg.angular.z = Kp_angular * angle_diff

        self.cmd_vel_pub.publish(vel_msg)

        rospy.loginfo("DTG: %.4f | ATG: %.4f°", dtg, degrees(angle_diff))
```
- Sets the linear and angular velocities proportionally.

- Publishes the velocity to the turtle.

- Logs DTG and ATG in the console for debugging.

```python
if dtg < 0.1:
            break

        self.rate.sleep()
```
- If DTG is very small (close to target), the loop exits.

- Sleeps to maintain the desired rate.

```python
vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    self.cmd_vel_pub.publish(vel_msg)
    rospy.loginfo("Objetivo alcanzado.\n")
```
- Stops the turtle once it reaches the goal.

```python
def rotate_to_theta(self, desired_theta):
    vel_msg = Twist()
    Kp_theta = 4.0
```
- Initializes angular velocity control.

- Sets proportional gain for rotation only.

```python
while not rospy.is_shutdown():
        error_theta = desired_theta - self.theta
        error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159

        vel_msg.angular.z = Kp_theta * error_theta
        self.cmd_vel_pub.publish(vel_msg)

        rospy.loginfo("Error ángulo final: %.4f°", degrees(error_theta))

        if abs(error_theta) < 0.05:
            break

        self.rate.sleep()
```
- Calculates the angular error and normalizes it.

- Publishes angular velocity.

- Stops when the orientation is close enough to the desired heading.

```python
vel_msg.angular.z = 0
    self.cmd_vel_pub.publish(vel_msg)
```
- Fully stops the rotation.

```python
def run(self):
    while not rospy.is_shutdown():
        goal_x, goal_y, goal_theta = self.get_user_input()
        self.move_to_goal(goal_x, goal_y)
        self.rotate_to_theta(goal_theta)
```
- Infinite loop that:

    - Gets new goal position from the user.

    - Moves the turtle to the position.

    - Rotates the turtle to the final desired orientation.

```python
if __name__ == '__main__':
    try:
        controller = MoveTurtleProportionalControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
```
- Initializes the class and starts the control loop.

- Handles ROS shutdown gracefully.

- This code demonstrates a fully functional position and orientation control system for a Turtlesim robot using proportional control. It combines real-time pose tracking, user interaction, Euclidean distance calculation, and angle normalization to achieve precise turtle movement in a simulation.
---

## Velocity Mapping and Error-to-Command Translation

To ensure the turtle moves effectively toward the target, it is necessary to map the position errors both in distance and orientation into velocity commands. This mapping transforms the spatial differences into actionable motion through a proportional control law. The linear velocity v is assigned based on the Distance to Goal (DTG), causing the turtle to move faster when it's farther from the goal and to slow down smoothly as it approaches. Similarly, the angular velocity ω depends on the difference between the turtle's current heading and the Angle to Goal (ATG), allowing it to rotate toward the target direction. This mapping ensures that motion remains stable and responsive, avoiding abrupt movements. The proportional constants Kp and Kθ serve as gains that control how aggressively the turtle reacts to these errors. Fine tuning these gains is crucial to achieving both precision and smoothness in the trajectory.

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