#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        self.rate = rospy.Rate(10)
        
        self.posicion_x = 0
        self.posicion_y = 0
        self.orientacion_theta = 0
    
    def pose_callback(self, pose):
        self.posicion_x = pose.x
        self.posicion_y = pose.y
        self.orientacion_theta = pose.theta

    def mover_tortuga_posicion_deseada(self, posicion_deseada_x, posicion_deseada_y, theta_deseado):
        Kp_pos = 1
        Kp_theta = 4

        while not rospy.is_shutdown():
            error_x = posicion_deseada_x - self.posicion_x
            error_y = posicion_deseada_y - self.posicion_y
            distancia = (error_x**2 + error_y**2)**0.5

            if distancia < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            angle_to_goal = atan2(error_y, error_x)
            twist_msg = Twist()
            twist_msg.linear.x = Kp_pos * distancia
            twist_msg.angular.z = 0  # Sin velocidad angular en esta fase
            
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Moviendo a (%f, %f) con velocidad: %f", posicion_deseada_x, posicion_deseada_y, twist_msg.linear.x)
            
            self.rate.sleep()

        twist_msg = Twist()
        self.velocity_publisher.publish(twist_msg)
        rospy.sleep(1)
        
        while not rospy.is_shutdown():
            error_theta = theta_deseado - self.orientacion_theta
            if abs(error_theta) < 0.05:
                rospy.loginfo("Orientación deseada alcanzada")
                break
            
            twist_msg = Twist()
            twist_msg.linear.x = 0  # Sin velocidad lineal en esta fase
            twist_msg.angular.z = Kp_theta * error_theta
            
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Corrigiendo orientación con velocidad angular: %f", twist_msg.angular.z)
            
            self.rate.sleep()
        
        twist_msg = Twist()
        self.velocity_publisher.publish(twist_msg)
    
    def obtener_posicion(self):
        x = float(input("Coordenada x: "))
        y = float(input("Coordenada y: "))
        theta_grados = float(input("Theta (grados): "))
        return x, y, radians(theta_grados)

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            posicion_deseada_x, posicion_deseada_y, theta_deseado = self.obtener_posicion()
            self.mover_tortuga_posicion_deseada(posicion_deseada_x, posicion_deseada_y, theta_deseado)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass