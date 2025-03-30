#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_x_y(self, desired_x, desired_y):
        # Constante de proporcionalidad del controlador (ajustable)
        Kp = 1

        while not rospy.is_shutdown():
            # Calcular el error de posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Calcular la distancia al objetivo
            distancia = (error_x**2 + error_y**2)**0.5
            
            # Calcular la velocidad lineal del movimiento
            vel_x = Kp * error_x
            vel_y = Kp * error_y
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y las velocidades en la terminal
            rospy.loginfo("Posición actual: (%f, %f), Error: (%f, %f), Velocidad lineal: (%f, %f)", 
                          self.current_x, self.current_y, error_x, error_y, vel_x, vel_y)

            # Verificar si se alcanza la posición deseada
            if distancia < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

        # Detener el movimiento lineal
        twist_msg = Twist()
        self.velocity_publisher.publish(twist_msg)

    def rotate_turtle_to_desired_theta(self, desired_theta):
        # Constante de proporcionalidad del controlador (ajustable)
        Kp_theta = 4

        while not rospy.is_shutdown():
            # Calcular el error de orientación
            error_theta = desired_theta - self.current_theta
            
            # Normalizar el error_theta para que esté en el rango [-pi, pi]
            error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159
            
            # Calcular la velocidad angular
            angular_z = Kp_theta * error_theta
            
            # Crear un mensaje de Twist para enviar el comando de rotación
            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir el error de orientación y la velocidad angular
            rospy.loginfo("Error de orientación: %f, Velocidad angular: %f", error_theta, angular_z)

            # Verificar si se alcanza la orientación deseada
            if abs(error_theta) < 0.05:
                rospy.loginfo("Orientación deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

        # Detener el movimiento angular
        twist_msg = Twist()
        self.velocity_publisher.publish(twist_msg)

    def obtener_posicion(self):
        print("Ingrese la posición deseada en el eje x:")
        x = float(input("Coordenada x: "))
        print("Ingrese la posición deseada en el eje y:")
        y = float(input("Coordenada y: "))
        print("Ingrese el ángulo deseado (en grados):")
        theta_grados = float(input("Theta (grados): "))
        theta_radianes = radians(theta_grados)  # Convertir a radianes
        return x, y, theta_radianes

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición y orientación deseada del usuario
            desired_x, desired_y, desired_theta = self.obtener_posicion()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_x_y(desired_x, desired_y)

            # Rotar la tortuga al ángulo deseado
            self.rotate_turtle_to_desired_theta(desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
