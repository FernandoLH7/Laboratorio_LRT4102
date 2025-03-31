# Introduction
In this lab, we will explore fundamental concepts of ROS (Robot Operating System), specifically communication between nodes through talker and listener. Additionally, we will implement basic and advanced controls in turtlesim, ranging from manual keyboard control to proportional (P), proportional-integral (PI), and proportional-integral-derivative (PID) controllers to regulate the turtle's position. These concepts are essential in the development of autonomous robotic systems and the implementation of control algorithms in ROS (Quigley et al., 2015; Siciliano & Khatib, 2016).

## *Key concepts*
- ROS Nodes: In ROS, a node is a single process that performs computation. Nodes communicate with each other by publishing or subscribing to topics (Quigley et al., 2015).

- Talker and Listener: A talker node is responsible for publishing messages to a topic, while a listener node subscribes to that topic and receives the messages (Quigley et al., 2015). These are basic forms of communication in ROS using the Publisher-Subscriber model.

- Topics: In ROS, a topic is a bus over which data is exchanged. A node can publish data to a topic, and other nodes can subscribe to it to receive the data. Topics are used to implement one-to-many or many-to-many communication (Quigley et al., 2015).

- Control Systems: Control systems are used to manage, command, direct, or regulate the behavior of other devices or systems using control loops. In the context of ROS, control systems are used to regulate robot motion.

- Proportional (P) Control: The P controller applies a correction proportional to the current error (Khalil, 2002). It tries to minimize the error by making the system respond to it directly.

- Proportional-Derivative (PD) Control: The PD controller not only applies a correction proportional to the error, but also a correction based on the rate of change of the error. This helps to reduce overshooting (Siciliano & Khatib, 2016).

- Proportional-Integral-Derivative (PID) Control: The PID controller combines the proportional, derivative, and integral terms to correct the error, accounting for past, current, and future error behavior (Khalil, 2002).

- Turtlesim: turtlesim is a simple simulation environment provided by ROS, used primarily for learning and testing purposes. It allows the user to control a virtual turtle using commands sent from ROS nodes (Quigley et al., 2015).

# Problems to Solve

Below are the problems solved in this lab. Each one addresses key Python programming concepts such as the use of control structures, list manipulation, random number generation, and the application of the Object-Oriented Programming (OOP) paradigm.

This lab consists of three levels of complexity:

- Lab2_Basic: Implementation of a ROS package with talker and listener nodes.

- Lab2_Medium: Creation of a keyboard control for turtlesim and drawing basic geometric figures.

- Lab2_Advanced: Implementation of P, PI, and PID controllers for turtlesim and comparison of their performance.

Each problem will be detailed below.

## *Lab2_Basic: Creating a ROS Package*
### *Problem Description*

The goal is to create a ROS package named Practicas_lab with dependencies on rospy, roscpp, and std_msgs. The scripts listener.py and talker.py will be included to test communication between nodes. Finally, the package will be compiled and executed to verify its functionality.

### *Code Explanation*
#### *talker.py*
This node publishes a message on a specified topic at a given frequency.
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
#### *Explanation of talker.py*
The talker.py script is responsible for publishing messages on the chatter topic, which will be received by the listener.py node.

- The rospy.Publisher('chatter', String, queue_size=10) line creates a publisher object for sending messages to the chatter topic.

- The rospy.init_node('talker', anonymous=True) function initializes the ROS node named talker.

- The rate = rospy.Rate(10) sets the publishing rate to 10Hz.

- Inside the loop, the message "hello world" along with the ROS time is logged and published to the topic.

- The rate.sleep() function ensures that the loop maintains the specified publishing rate.

- The script handles interruptions using try-except to avoid errors when shutting down.

#### *listener.py*
This node subscribes to the talker topic and displays the received messages.
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```
#### *Explanation of listener.py*
The listener.py script is responsible for subscribing to the chatter topic and processing messages sent by the talker node.

- The function callback(data) is triggered whenever a new message is received on the chatter topic. It logs the message using rospy.loginfo(), including the caller ID and the received data.

- The listener() function initializes a ROS node named listener with anonymous=True, ensuring that multiple instances can run simultaneously without name conflicts.

- The function rospy.Subscriber("chatter", String, callback) subscribes the node to the chatter topic and links incoming messages to the callback function.

- Finally, rospy.spin() keeps the node running, allowing it to continuously listen for messages until it is manually stopped.

## *Lab2_Medium: Keyboard Control and Drawing in Turtlesim*
### *Keyboard Control*
In this part of the lab, we implement keyboard control for the turtle in turtlesim using ROS. The goal is to allow the user to control the turtle's movement by pressing specific keys, showcasing the practical use of teleoperation with keyboard inputs.

Key Concepts Addressed:
- Teleoperation: This is the main concept, where the user sends control commands (via keyboard input) to the turtle to manipulate its movement in the 2D simulation environment. Teleoperation in ROS often involves sending velocity commands (e.g., Twist messages) to the robot's control system (Cacace & Meli, 2021).

- Twist Messages: These messages are published to the /turtle1/cmd_vel topic to control the linear and angular velocities of the turtle. The Twist message type allows us to specify velocities in both x, y (linear velocities) and z (angular velocity), making it useful for simple motion control (ROS Wiki, 2025).

- Topics in ROS: We use a topic (/turtle1/cmd_vel) to publish velocity commands that will control the movement of the turtle. The concept of topics in ROS facilitates communication between nodes through publish-subscribe communication (Quigley et al., 2009).

#### *Code: keyboard_control.py*
```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Reads a key press without requiring the user to press Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Reads a single character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Control the turtle with the following keys:")
    print("  x -> Move in +X direction")
    print("  y -> Move in +Y direction")
    print("  w -> Move in -X direction")
    print("  z -> Move in -Y direction")
    print("  → (Right Arrow) -> Rotate right")
    print("  ← (Left Arrow) -> Rotate left")
    print("  q -> Exit")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()

        if key.lower() == 'x':  # Move in +X direction
            msg.linear.x = 2.0
            print("Moving in +X direction")
        elif key.lower() == 'y':  # Move in +Y direction
            msg.linear.y = 2.0
            print("Moving in +Y direction")
        elif key.lower() == 'w':  # Move in -X direction
            msg.linear.x = -2.0
            print("Moving in -X direction")
        elif key.lower() == 'z':  # Move in -Y direction
            msg.linear.y = -2.0
            print("Moving in -Y direction")

        elif key == '\x1b':  # Special keys (arrow keys)
            key = sys.stdin.read(2)  # Read next character

            if key == '[C':  # Right arrow
                msg.angular.z = -1.0
                print("Rotating right")
            elif key == '[D':  # Left arrow
                msg.angular.z = 1.0
                print("Rotating left")

        elif key.lower() == 'q':  # Exit
            print("Exiting...")
            break  

        pub.publish(msg)

if __name__ == '__main__':
    main()
```
#### *Explanation: keyboard_control.py*
- get_key():

    This function reads a key press from the user without needing to press the "Enter" key. It uses the termios and tty libraries to configure the terminal for raw input, ensuring that we can capture a single character.

- Twist Message and pub.publish(msg):

    The Twist message is used to control the turtle's movement. Depending on the key pressed, we set the values of msg.linear.x, msg.linear.y, and msg.angular.z to move the turtle forward, backward, or rotate. For example:

        Pressing 'x' makes the turtle move forward in the X direction (positive X-axis).

        Pressing the right arrow key rotates the turtle to the right.

    After setting the velocities, the pub.publish(msg) sends the Twist message to the topic /turtle1/cmd_vel.

- Key Control:

    The user controls the turtle using keyboard inputs:

        'x': Move forward along the X-axis.

        'y': Move forward along the Y-axis.

        'w': Move backward along the X-axis.

        'z': Move backward along the Y-axis.

        Arrow keys: Rotate the turtle left or right.

        'q': Exit the program.

- ROS Publisher:

    The pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) line creates a publisher that sends Twist messages to the /turtle1/cmd_vel topic. This is the topic that controls the turtle's movement.

The script provides a simple interface for teleoperation. By publishing velocity commands to a topic, the user can move and rotate the turtle based on real-time inputs. This technique can be expanded to more complex robotic systems, where sensors and feedback loops are used for autonomous control (Vasudevan, 2017).

In this case, using the Twist message type allows the user to control both the linear and angular velocities of the turtle, which is a straightforward way to handle movement in 2D space.

### *Drawing shapes*
In this section, we will focus on drawing basic shapes with turtlesim and creating a keyboard control for a turtle. This practice allows us to demonstrate the usage of ROS topics, services, and control commands, while learning how to interact with the turtlesim environment.

Key Concepts Addressed:

- Services in ROS: Services in ROS are used for synchronous communication between nodes, where one node requests an operation and waits for a response. In the code, services like /spawn and /kill are used to create and delete turtles in the simulator.

#### *Code: dibujar.py*
```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
import sys
import termios
import tty
import time

def get_key():
    """Reads a keypress without needing to press Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Read a single character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def kill_turtle(name):
    """Kills a turtle if it exists."""
    rospy.wait_for_service('/kill')
    try:
        kill = rospy.ServiceProxy('/kill', Kill)
        kill(name)
    except rospy.ServiceException as e:
        rospy.logwarn(f"Could not kill {name}. It might not exist.")

def spawn_turtle(x, y, theta, name):
    """Spawns a new turtle at the given position."""
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta, name)
    except rospy.ServiceException as e:
        rospy.logerr(f"Error spawning {name}: {e}")

def draw_square(pub):
    """Draws a square with the turtle."""
    msg = Twist()
    for _ in range(4):
        msg.linear.x = 0.0
        msg.angular.z = 1.57  # 90 degrees
        pub.publish(msg)
        time.sleep(2)

        msg.linear.x = 2.0
        msg.angular.z = 0.0  # Move forward
        pub.publish(msg)
        time.sleep(2)

def draw_triangle(pub):
    """Draws an equilateral triangle with the turtle."""
    msg = Twist()
    for _ in range(3):
        msg.linear.x = 0.0
        msg.angular.z = 2.094  # 120 degrees
        pub.publish(msg)
        time.sleep(2)

        msg.linear.x = 2.0
        msg.angular.z = 0.0  # Move forward
        pub.publish(msg)
        time.sleep(2)

def print_menu():
    """Displays the menu options."""
    print("\nOptions:")
    print("C -> Square")
    print("T -> Triangle")
    print("Q -> Quit")

def main():
    rospy.init_node('turtle_shapes', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    print_menu()
    kill_turtle("turtle1")  # Kill the current turtle if it exists

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'q':  
            print("Exiting...")
            break  

        elif key == 'c':
            spawn_turtle(4.0, 6.0, 0, "turtle1")  # Create at new position
            draw_square(pub)
            kill_turtle("turtle1")  # Kill after drawing
            print_menu()

        elif key == 't':
            spawn_turtle(6.0, 2.0, 0, "turtle1")  # New turtle at another position
            draw_triangle(pub)
            kill_turtle("turtle1")  # Kill after drawing
            print_menu()

if __name__ == '__main__':
    main()
```
#### *Explanation: dibujar.py*
- get_key():

    This function reads a key press from the user without requiring the user to press "Enter". It is implemented using termios to handle low-level terminal input.

- kill_turtle(name):

    This function sends a service request to /kill to delete a turtle from the simulator. It waits for the service to be available and then sends the Kill request with the turtle’s name.

- spawn_turtle(x, y, theta, name):

    This function creates a new turtle at the specified coordinates (x, y) and orientation theta. It uses the /spawn service, which requires the service to be available before sending the request.

- draw_square(pub):

    This function commands the turtle to draw a square. The turtle performs movements by publishing Twist messages to the /turtle1/cmd_vel topic. A Twist message controls both linear and angular velocities, where msg.linear.x controls forward speed, and msg.angular.z controls rotation. The turtle moves forward and then rotates by 90 degrees to form the square.

- draw_triangle(pub):

    Similar to the square, this function draws an equilateral triangle. It makes the turtle rotate 120 degrees and move forward to complete each side.

- print_menu():

    A simple menu system is provided, where the user can press keys to draw different shapes (square or triangle) or quit the program.

This section focuses on teleoperation for controlling the turtle via keyboard inputs. The user can press different keys to control the turtle’s actions, which is an essential aspect of interacting with robots in real-time systems (Vasudevan, 2017). The main operations include:

    Pressing 'C': This spawns a turtle and commands it to draw a square.

    Pressing 'T': This spawns a turtle and commands it to draw a triangle.

    Pressing 'Q': This quits the program.

## *Lab2_Advanced: P, PD, and PID Controllers*
In this section, the implementation of Proportional (P), Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID) controllers in ROS is discussed. These controllers are used to control the movement of a virtual turtle in Turtlesim, adjusting its position and orientation dynamically.

### *Proportional (P) Controller*
The Proportional (P) controller is the simplest form of feedback control. It calculates an error between the current and desired positions, then applies a proportional correction. The control law is given by:

u = Kp*e

where:
- u is the control signal (velocity),

- Kp​ is the proportional gain,

- e is the error (difference between desired and current position).

#### *Code Breakdown: P Controller Implementation*
The turtle_pc.py script implements a Proportional (P) controller to move the turtle to a user-specified position and orientation.

##### 1. Node Initialization and ROS Communication Setup
```python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians
```
The script imports necessary ROS libraries:

- rospy for ROS node handling,

- geometry_msgs.msg.Twist to send movement commands,

- turtlesim.msg.Pose to receive the turtle's position,

- math.radians to convert degrees to radians.

```python
rospy.init_node('control_tortuga_x')
```
This initializes a ROS node named control_tortuga_x, which will control the turtle's movement.

```python
self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
```
A subscriber is created to listen to /turtle1/pose, which provides the turtle’s real-time position.
```python
self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
```
A publisher is created to send velocity commands to /turtle1/cmd_vel.

##### 2. Retrieving the Turtle’s Position
```python
def pose_callback(self, pose):
    self.current_x = pose.x
    self.current_y = pose.y
    self.current_theta = pose.theta
```
The pose_callback function updates the current x, y, and θ (orientation) of the turtle whenever a new position is received from Turtlesim.

##### 3. Moving the Turtle to the Desired (x, y) Position
```python
def move_turtle_to_desired_x_y(self, desired_x, desired_y):
    Kp = 1  # Proportional gain
```
A proportional gain Kp​ is defined. Increasing this value results in more aggressive movements.
```python
error_x = desired_x - self.current_x
error_y = desired_y - self.current_y
distancia = (error_x**2 + error_y**2)**0.5
```
The script calculates the error (difference between desired and current positions) and the Euclidean distance to the target point.
```python
vel_x = Kp * error_x
vel_y = Kp * error_y
```
Velocity commands are proportional to the error.
```python
twist_msg = Twist()
twist_msg.linear.x = vel_x
twist_msg.linear.y = vel_y
self.velocity_publisher.publish(twist_msg)
```
A Twist message is created and published to move the turtle.
```python
if distancia < 0.1:
    rospy.loginfo("Target position reached")
    break
```
Once the turtle is close enough (error < 0.1), movement stops.
```python
twist_msg = Twist()
self.velocity_publisher.publish(twist_msg)
```
Finally, the velocity is set to zero to halt the turtle.

##### 4. Rotating the Turtle to a Desired Orientation (θ)
```python
def rotate_turtle_to_desired_theta(self, desired_theta):
    Kp_theta = 4  # Proportional gain for rotation
```
A separate proportional gain is used for angular movement.
```python
error_theta = desired_theta - self.current_theta
error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159
```
This ensures the angle error remains between -π and π (avoiding large rotations).
```python
angular_z = Kp_theta * error_theta
twist_msg = Twist()
twist_msg.angular.z = angular_z
self.velocity_publisher.publish(twist_msg)
```
The turtle rotates with a speed proportional to the error.
```python
if abs(error_theta) < 0.05:
    rospy.loginfo("Target orientation reached")
    break
```
Once the angular error is small enough, the turtle stops rotating.

##### 5. User Interaction and Execution
```python
def obtener_posicion(self):
    x = float(input("Enter X coordinate: "))
    y = float(input("Enter Y coordinate: "))
    theta_grados = float(input("Enter Theta (degrees): "))
    theta_radianes = radians(theta_grados)
    return x, y, theta_radianes
```
The program prompts the user to input the desired (x, y, θ) position.
```python
def move_turtle_interactively(self):
    while not rospy.is_shutdown():
        desired_x, desired_y, desired_theta = self.obtener_posicion()
        self.move_turtle_to_desired_x_y(desired_x, desired_y)
        self.rotate_turtle_to_desired_theta(desired_theta)
```
This function continuously receives user inputs and moves the turtle accordingly.

### *Proportional-Derivative (PD) Controller and Proportional-Derivative-Integrative (PID) Controller*
The P controller is functional but has limitations:

- It can lead to oscillations and overshoot when approaching the target.

- It does not account for sudden changes in the system.

To improve performance, the PD and PID controllers are modifications of the same control strategy by adding more terms:

1. PD Controller (Proportional-Derivative)

    - Introduces a derivative (D) term, which reacts to the rate of change of the error

    - The derivative gain Kd helps reduce overshoot and provides smoother convergence.

    - It stabilizes the system by anticipating future errors based on the current rate of change.

2. PID Controller (Proportional-Integral-Derivative)

    - Adds an integral (I) term to eliminate steady-state error.

    - The integral gain Ki accumulates the error over time and ensures that even small errors eventually disappear.
    
    - Helps correct persistent offsets that the P or PD controller might leave.

Since the PD and PID controllers only add additional gains (derivative and integral), their implementation follows the same structure as the P controller, with extra terms for error rate and error accumulation. Therefore, their code will not be analyzed separately, as it is fundamentally an extension of the Proportional (P) controller with additional adjustments.

## Comparison of Controllers: P, PD, and PID

To evaluate the performance of different controllers, a step response comparison was made between Proportional (P), Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID) control strategies. The simulation was designed to mimic the behavior of the turtlebot used in the ROS environment. A second-order underdamped system was used to better visualize transient characteristics such as overshoot and oscillations.

### Reference Input

A unit step input was applied as the reference trajectory for all three controllers. The goal was to assess how each controller reacts to sudden changes in desired position and how accurately and quickly it reaches the target.

### Controller P (Proportional)

- **Behavior:** The P controller shows a moderately fast response with slight oscillations around the reference. However, it does not fully eliminate the steady-state error.
- **Performance:** Due to the lack of integral action, it fails to reach the desired position precisely.
- **Conclusion:** It is simple and reactive but lacks accuracy and exhibits steady-state error.

### Controller PD (Proportional-Derivative)

- **Behavior:** The PD controller responds more aggressively, with a higher overshoot and faster convergence than P. It reduces oscillations over time but still shows some steady-state error.
- **Performance:** The derivative action helps predict future error trends and improves transient response.
- **Conclusion:** Suitable when a faster response is needed, but still not ideal for steady-state accuracy.

### Controller PID (Proportional-Integral-Derivative)

- **Behavior:** The PID controller provides a smooth response with minimal overshoot and oscillation. It reaches the reference quickly and eliminates the steady-state error.
- **Performance:** Combines fast reaction, error correction over time, and damping, resulting in the most balanced performance.
- **Conclusion:** The PID controller delivers the best performance overall, making it ideal for precise and stable control in dynamic systems.

### Final Observation

Based on the graphical analysis, the PID controller is the most efficient in tracking the reference input accurately while maintaining system stability. This aligns with the practical behavior observed in the ROS simulations of the turtle’s motion using different control strategies.

# Conclusion
This report outlines a lab focused on learning ROS (Robot Operating System) and the implementation of basic to advanced control techniques for controlling a turtle in the turtlesim simulation environment. The key concepts discussed, such as ROS nodes, topics, publishers, subscribers, and control loops (P, PI, and PID), are fundamental to building autonomous robotic systems.

The Introduction gives a solid overview of ROS, introducing concepts like nodes, topics, and control systems, which are essential for communication and control in a robotic context. The report presents these concepts clearly and ties them together with references to well-established sources, making it a valuable resource for both beginners and those looking to strengthen their ROS skills.

The Lab Problems are presented in a structured way, progressing from basic tasks (creating nodes and establishing communication) to more complex ones (implementing control algorithms and drawing shapes in turtlesim). Each level is introduced with clear explanations, making the process easy to follow. The use of Python code snippets and detailed commentary on each step adds clarity to the overall understanding.

## *Libraries Used*
- rospy: Essential for interacting with ROS nodes in Python, it facilitates creating nodes, publishing, and subscribing to topics. It's widely used in ROS-based Python applications for communication.

- std_msgs.msg: Specifically used for message types like String to exchange simple data types (e.g., strings) between nodes in ROS. This is a standard library used for common message types in ROS.

- geometry_msgs.msg: Used to handle the Twist message type, which is crucial for controlling the robot’s linear and angular velocities. This is key to enabling movement and teleoperation of robots in ROS.

- turtlesim.srv: This service package is unique to ROS's turtlesim simulator, offering services like spawning and killing turtles, as well as teleporting them. It’s a useful package for testing and visualizing robotic control in a 2D simulation.

- termios and tty: These libraries are used for handling raw keyboard input, allowing real-time keypress detection without requiring the user to press "Enter." This is ideal for interactive control in ROS applications.

- time: A simple but effective library for controlling timing in the execution of robot movements, ensuring that each action has the correct timing for control loops or sequences of commands.

# References
- Quigley, M., Gerkey, B., & Smart, W. D. (2015). Programming Robots with ROS: A Practical Introduction to the Robot Operating System. O'Reilly Media.

- Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.

- Khalil, H. K. (2002). Nonlinear Systems (3rd ed.). Prentice Hall.

- ROS Wiki. (2025). Twist message. Retrieved from http://wiki.ros.org/geometry_msgs/msg/Twist

- Cacace, J., & Meli, M. (2021). ROS for Robot Programming: Teleoperation and Navigation. Springer.

- Vasudevan, R. (2017). Robotics: Teleoperation and Navigation in ROS. Springer.

