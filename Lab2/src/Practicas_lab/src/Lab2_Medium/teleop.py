#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import time

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
    
    print("Controla la tortuga con las teclas:")
    print("  x -> Mover en X")
    print("  y -> Mover en Y")
    print("  i -> Mover en -X")
    print("  o -> Mover en -Y")
    print("  u -> Rotar en Z")
    print("  s -> Detenerse")
    print("Presiona 'q' para salir.")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        velocidad_linear = 1
        velocidad_angular = 1.57
        #if key == 'x':
            #msg.linear.x = 2.0  # Avanza en X
        #elif key == 'y':
            #msg.linear.y = 2.0  # Avanza en Y
        #elif key == 's':
            #msg.linear.x = 0.0
            #msg.linear.y = 0.0  # Detiene el movimiento
        #elif key == 'i':
            #msg.linear.x = -2.0
        #elif key == 'o':
            #msg.linear.y = -2.0
        #elif key == 'u': 
            #msg.angular.z = 2.0	
        if key == 'q':  
            print("Saliendo...")         
            break  # Sale del loop
        
        # Hacer Cuadrado
        elif key == 'c':
            for _ in range(4):
                msg.linear.x = velocidad_linear
                msg.angular.z = 0.0
                pub.publish(msg)
                time.sleep(2)
                
                msg.linear.x = 0.0
                msg.angular.z = velocidad_angular
                pub.publish(msg)
                time.sleep(2)
        
        elif key == 't':
            #rospy.wait_for_service(/kill)
            #rospy.wait_for_service(/spawn)
            for _ in range(3):
                msg.linear.x = velocidad_linear
                msg.angular.z = 0.0
                pub.publish(msg)
                time.sleep(2)
                
                msg.linear.x = 0.0
                msg.angular.z = 2.094
                pub.publish(msg)
                time.sleep(2)  
                     
                
                

if __name__ == '__main__':
    main()
