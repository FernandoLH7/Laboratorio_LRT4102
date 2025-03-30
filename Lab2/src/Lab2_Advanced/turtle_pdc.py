#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians, sqrt, pi

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pd')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables para almacenar la posición y orientación actuales
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        
        # Variables para el control derivativo
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def stop_turtle(self):
        # Detener completamente la tortuga
        twist_msg = Twist()
        self.velocity_publisher.publish(twist_msg)
        rospy.sleep(0.5)  # Esperar un momento para asegurarse de que se detenga

    def move_turtle_to_desired_x_y(self, desired_x, desired_y):
        # Constantes de proporcionalidad y derivativa del controlador (ajustables)
        Kp = 1
        Kd = 0.1

        while not rospy.is_shutdown():
            # Calcular el error de posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Calcular la distancia al objetivo
            distancia = sqrt(error_x**2 + error_y**2)
            
            # Calcular la velocidad lineal del movimiento
            vel_x = Kp * error_x + Kd * (error_x - self.last_error_x)
            vel_y = Kp * error_y + Kd * (error_y - self.last_error_y)
            
            # Guardar los errores actuales para usar en la próxima iteración
            self.last_error_x = error_x
            self.last_error_y = error_y
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            twist_msg.angular.z = 0  # Asegurarse de que no haya velocidad angular
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual y los errores
            rospy.loginfo("Posición actual: (%f, %f), Error: (%f, %f), Distancia: %f", 
                          self.current_x, self.current_y, error_x, error_y, distancia)

            # Verificar si se alcanza la posición deseada
            if distancia < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

        # Detener el movimiento lineal
        self.stop_turtle()

    def rotate_turtle_to_desired_theta(self, desired_theta):
        # Constantes de proporcionalidad y derivativa del controlador (ajustables)
        Kp_theta = 4
        Kd_theta = 0.1

        while not rospy.is_shutdown():
            # Calcular el error de orientación
            error_theta = desired_theta - self.current_theta
            
            # Normalizar el error_theta para que esté en el rango [-pi, pi]
            error_theta = (error_theta + pi) % (2 * pi) - pi
            
            # Calcular la velocidad angular
            angular_z = Kp_theta * error_theta + Kd_theta * (error_theta - self.last_error_theta)
            
            # Guardar el error actual para usar en la próxima iteración
            self.last_error_theta = error_theta
            
            # Crear un mensaje de Twist para enviar el comando de rotación
            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            twist_msg.linear.x = 0  # Asegurarse de que no haya velocidad lineal
            
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
        self.stop_turtle()

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
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
