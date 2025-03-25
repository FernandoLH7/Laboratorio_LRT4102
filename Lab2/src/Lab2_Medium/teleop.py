#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
import sys
import termios
import tty
import time

def get_key():
    """Lee una tecla sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def kill_turtle(name):
    """Elimina una tortuga si existe."""
    rospy.wait_for_service('/kill')
    try:
        kill = rospy.ServiceProxy('/kill', Kill)
        kill(name)
    except rospy.ServiceException as e:
        rospy.logwarn(f"No se pudo eliminar {name}. Puede que ya no exista.")

def spawn_turtle(x, y, theta, name):
    """Crea una nueva tortuga en la posición especificada."""
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta, name)
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al crear {name}: {e}")

def draw_square(pub):
    """Dibuja un cuadrado con la tortuga."""
    msg = Twist()
    for _ in range(4):
        msg.linear.x = 0.0
        msg.angular.z = 1.57 
        pub.publish(msg)
        time.sleep(2)

        msg.linear.x = 2.0
        msg.angular.z = 0.0    # 90 grados
        pub.publish(msg)
        time.sleep(2)

def draw_triangle(pub):
    """Dibuja un triángulo equilátero con la tortuga."""
    msg = Twist()
    for _ in range(3):
        msg.linear.x = 0.0
        msg.angular.z = 2.094  
        pub.publish(msg)
        time.sleep(2)

        msg.linear.x = 2.0
        msg.angular.z = 0.0  # 120 grados
        pub.publish(msg)
        time.sleep(2)

def print_menu():
    """Imprime el menú de opciones."""
    print("\nOpciones:")
    print("C -> Cuadrado")
    print("T -> Triángulo")
    print("Q -> Salir")

def main():
    rospy.init_node('turtle_shapes', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    print_menu()
    kill_turtle("turtle1")  # Eliminar tortuga actual si existe

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'q':  
            print("Saliendo...")
            break  

        elif key == 'c':
            spawn_turtle(4.0, 6.0, 0, "turtle1")  # Crear en nueva posición
            draw_square(pub)
            kill_turtle("turtle1")  # Eliminar después de dibujar
            print_menu()

        elif key == 't':
            spawn_turtle(6.0, 2.0, 0, "turtle1")  # Nueva tortuga en otra posición
            draw_triangle(pub)
            kill_turtle("turtle1")  # Eliminar después de dibujar
            print_menu()

if __name__ == '__main__':
    main()

