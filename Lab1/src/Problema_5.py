from random import *

class AdivinarNumero:
    def __init__(self):
        # Genera un número aleatorio entre 1 y 10
        self.numero_aleatorio = randint(1, 10)

    def jugar(self):
        # Inicializa el contador de intentos
        intentos = 0
        while True:
            try:
                # Incrementa el contador de intentos en cada iteración
                intentos += 1
                # Solicita al usuario que ingrese un número y lo convierte a entero
                numero_usuario = int(input("¿Cual es el número secreto entre 0 y 10?: "))
                # Compara el número ingresado por el usuario con el número aleatorio
                if numero_usuario > self.numero_aleatorio:
                    # Si el número ingresado es mayor, informa al usuario
                    print("El número está por debajo del que diste")
                elif numero_usuario < self.numero_aleatorio:
                    # Si el número ingresado es menor, informa al usuario
                    print("El número está por arriba del que diste")
                else:
                    # Si el número ingresado es igual al número aleatorio, felicita al usuario
                    # e informa la cantidad de intentos realizados
                    print(f"¡Felicidades! Adivinaste el número secreto en {intentos} intentos.")
                    break
            except ValueError:
                # Si el usuario ingresa un valor no válido, muestra un mensaje de error
                print("Por favor ingrese un número válido")

# Crea una instancia de la clase AdivinarNumero y llama al método jugar
juego = AdivinarNumero()
juego.jugar()