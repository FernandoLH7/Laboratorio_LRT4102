class Suma:
    def __init__(self):
        # El constructor __init__ se llama automáticamente al crear una instancia de la clase Suma.
        # Aquí pedimos al usuario que ingrese un número y lo convertimos a entero.
        self.numero_aleatorio = int(input("¿Cual es el número?: "))

    def sumar(self):
        # El método sumar calcula la suma de todos los números desde 1 hasta numero_aleatorio.
        # Utiliza la fórmula de la suma de los primeros n números naturales: n(n + 1)/2.
        suma = (self.numero_aleatorio * (self.numero_aleatorio + 1)) / 2
        # Imprime el resultado de la suma.
        print("La suma de todos los números es: ", suma)

# Creamos una instancia de la clase Suma, lo que llama al constructor __init__.
resultado = Suma()
# Llamamos al método sumar de la instancia resultado para realizar el cálculo e imprimir el resultado.
resultado.sumar()