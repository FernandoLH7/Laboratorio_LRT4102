# Introduction
In this lab, we will explore fundamental concepts of ROS (Robot Operating System), specifically communication between nodes through talker and listener. Additionally, we will implement basic and advanced controls in turtlesim, ranging from manual keyboard control to proportional (P), proportional-integral (PI), and proportional-integral-derivative (PID) controllers to regulate the turtle's position. These concepts are essential in the development of autonomous robotic systems and the implementation of control algorithms in ROS (Quigley et al., 2015; Siciliano & Khatib, 2016).

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

# Libraries Used
# References
- Lutz, M. (2013). Learning Python. O'Reilly Media.
- Van Rossum, G. (1991). Python programming language. Python Software Foundation.
- Python Software Foundation. (2023). Python documentation. https://docs.python.org/3/
- Booch, G. (1994). Object-Oriented Analysis and Design with Applications. Addison-Wesley.