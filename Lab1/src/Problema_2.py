class Numer:
    def __init__(self):
        # El constructor __init__ se llama automáticamente al crear una instancia de la clase Numer.
        # Aquí pedimos al usuario que ingrese el número de horas trabajadas y el costo por hora.
        # Convertimos estas entradas a enteros y las guardamos en las variables de instancia self.horas y self.costo.
        self.horas = int(input("Ingresa el número de horas que has trabajado: "))
        self.costo = int(input("Ingrese el costo de cada hora trabajada: "))

    def calcular(self):
        # El método calcular multiplica el número de horas trabajadas por el costo por hora.
        # El resultado de esta multiplicación se guarda en la variable salario.
        salario = self.horas * self.costo
        # Imprime el salario calculado con un mensaje descriptivo.
        print("El salario que te corresponde es de: ", salario, "$")

# Creamos una instancia de la clase Numer, lo que llama al constructor __init__.
paga = Numer()
# Llamamos al método calcular de la instancia paga para realizar el cálculo e imprimir el resultado.
paga.calcular()