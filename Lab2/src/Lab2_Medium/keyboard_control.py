#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controla la tortuga con las siguientes teclas:")
    print("  x -> Mover en +X (Avanzando en X)")
    print("  y -> Mover en +Y (Avanzando en Y)")
    print("  w -> Mover en -X (Avanzando en -X)")
    print("  z -> Mover en -Y (Avanzando en -Y)")
    print("  → (Flecha derecha) -> Rotar a la derecha")
    print("  ← (Flecha izquierda) -> Rotar a la izquierda")
    print("  q -> Salir")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()

        if key.lower() == 'x':  # Mover en +X
            msg.linear.x = 2.0
            print("Avanzando en X")
        elif key.lower() == 'y':  # Mover en +Y
            msg.linear.y = 2.0
            print("Avanzando en Y")
        elif key.lower() == 'w':  # Mover en -X
            msg.linear.x = -2.0
            print("Avanzando en -X")
        elif key.lower() == 'z':  # Mover en -Y
            msg.linear.y = -2.0
            print("Avanzando en -Y")

        elif key == '\x1b':  # Si es una tecla especial (como una flecha)
            key = sys.stdin.read(2)  # Leer el siguiente carácter

            if key == '[C':  # Flecha derecha
                msg.angular.z = -1.0
                print("Rotando hacia la derecha")
            elif key == '[D':  # Flecha izquierda
                msg.angular.z = 1.0
                print("Rotando hacia la izquierda")

        elif key.lower() == 'q':  # Salir
            print("Saliendo...")
            break  

        pub.publish(msg)

if __name__ == '__main__':
    main()

