from random import *

class AdivinarNumero:
    def __init__(self):
        self.numero_aleatorio = randint(1,10)

    def jugar(self):
        while True:
            try:
                numero_usuario = int(input("¿Cual es el número secreto entre 0 y 10?: "))
                if numero_usuario > self.numero_aleatorio:
                    print("El numero esta por debajo del que diste")
                elif numero_usuario < self.numero_aleatorio:
                    print("El numero esta por arriba del que diste")
                else:
                    print("¡Felicidades! Adivinaste el número secreto.")
                    break
            except ValueError:
                print("Por favor ingrese un número válido")

juego = AdivinarNumero()
juego.jugar()