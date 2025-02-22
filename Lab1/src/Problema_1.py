class Suma:
    def __init__(self):
        self.numero_aleatorio = int(input("¿Cual es el número?: "))

    def sumar(self):
        suma = (self.numero_aleatorio * (self.numero_aleatorio + 1))/2
        print("La suma de todos los números es: ", suma)

resultado = Suma()
resultado.sumar()