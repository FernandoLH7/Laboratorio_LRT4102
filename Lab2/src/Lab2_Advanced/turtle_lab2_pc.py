#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.posicion_x = 0
        self.posicion_y = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.posicion_x = pose.x
        self.posicion_y = pose.y

    def mover_tortuga_posicion_deseada(self, posicion_deseada):
        # Constante de proporcionalidad del controlador (ajustable)
        Kp = 1

        while not rospy.is_shutdown():
            # Calcular el error de posición
            error_x = posicion_deseada - self.posicion_x
            error_y = posicion_deseada - self.posicion_y
            
            # Calcular la velocidad lineal del movimiento
            vel_x = Kp * error_x
            vel_y = Kp * error_y
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y la variable vel_x en la terminal
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad lineal: %f", self.posicion_x, error_x, vel_x)
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad lineal: %f", self.posicion_y, error_x, vel_x)


            # Verificar si se alcanza la posición deseada
            if abs(error_x and error_y) < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def obtener_posicion(self):
        print("Ingrese la posición deseada en el eje x:")
        x = input("Coordenada x: ")
        print("Ingrese la posición deseada en el eje y:")
        y = input("Coordenada y: ")
        return float(x)
        return float(y)       

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            posicion_deseada = self.obtener_posicion()

            # Mover la tortuga a la posición deseada
            self.mover_tortuga_posicion_deseada(posicion_deseada)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
