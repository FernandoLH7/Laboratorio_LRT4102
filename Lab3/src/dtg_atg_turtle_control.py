#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pow, radians, degrees

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('turtle_proportional_controller', anonymous=True)
        
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.x = 0
        self.y = 0
        self.theta = 0

    def pose_callback(self, data):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta

    def get_user_input(self):
        print("\nIngresar nueva posición objetivo")
        x = float(input("Coordenada x del objetivo: "))
        y = float(input("Coordenada y del objetivo: "))
        theta_deg = float(input("Ángulo deseado (grados): "))
        return x, y, radians(theta_deg)

    def move_to_goal(self, goal_x, goal_y):
        vel_msg = Twist()
        Kp_linear = 1.5
        Kp_angular = 6.0

        while not rospy.is_shutdown():
            # Calcular DTG y ATG usando coordenadas euclidianas
            dtg = sqrt(pow(goal_x - self.x, 2) + pow(goal_y - self.y, 2))
            atg = atan2(goal_y - self.y, goal_x - self.x)
            angle_diff = atg - self.theta

            # Normalizar el ángulo a [-pi, pi]
            angle_diff = (angle_diff + 3.14159) % (2 * 3.14159) - 3.14159

            # Velocidades proporcional al error
            vel_msg.linear.x = Kp_linear * dtg
            vel_msg.angular.z = Kp_angular * angle_diff

            self.cmd_vel_pub.publish(vel_msg)

            rospy.loginfo("DTG: %.4f | ATG: %.4f°", dtg, degrees(angle_diff))

            # Cuando se está muy cerca del objetivo, detener
            if dtg < 0.1:
                break

            self.rate.sleep()

        # Detener completamente
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)
        rospy.loginfo("Objetivo alcanzado.\n")

    def rotate_to_theta(self, desired_theta):
        vel_msg = Twist()
        Kp_theta = 4.0

        while not rospy.is_shutdown():
            error_theta = desired_theta - self.theta
            error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159

            vel_msg.angular.z = Kp_theta * error_theta
            self.cmd_vel_pub.publish(vel_msg)

            rospy.loginfo("Error ángulo final: %.4f°", degrees(error_theta))

            if abs(error_theta) < 0.05:
                break

            self.rate.sleep()

        # Detener rotación
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)

    def run(self):
        while not rospy.is_shutdown():
            goal_x, goal_y, goal_theta = self.get_user_input()
            self.move_to_goal(goal_x, goal_y)
            self.rotate_to_theta(goal_theta)

if __name__ == '__main__':
    try:
        controller = MoveTurtleProportionalControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
