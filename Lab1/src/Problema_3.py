class Operador:
    def __init__(self, nombre, sueldo_por_hora, horas_trabajadas):
        """Inicializa los atributos del operador."""
        self.nombre = nombre
        self.sueldo_por_hora = sueldo_por_hora
        self.horas_trabajadas = horas_trabajadas

    def calcular_sueldo(self):
        """Calcula el sueldo total del operador."""
        return self.sueldo_por_hora * self.horas_trabajadas
    
    def mostrar_info(self):
        """Muestra el nombre y sueldo total del operador."""
        print(f"Operador: {self.nombre}, Sueldo total: ${self.calcular_sueldo():.2f}")


class Empresa:
    def __init__(self):
        """Inicializa la lista de operadores."""
        self.operadores = []

    def agregar_operador(self, nombre, sueldo_por_hora, horas_trabajadas):
        """Añade un operador a la empresa."""
        operador = Operador(nombre, sueldo_por_hora, horas_trabajadas)
        self.operadores.append(operador)

    def mostrar_sueldos(self):
        """Muestra el sueldo de cada operador."""
        print("\nLista de sueldos de operadores:")
        for operador in self.operadores:
            operador.mostrar_info()

# Crear empresa y agregar operadores
empresa = Empresa()
empresa.agregar_operador("Juan", 10, 40)
empresa.agregar_operador("María", 12, 35)
empresa.agregar_operador("Carlos", 15, 42)
empresa.agregar_operador("Sofía", 9, 38)
empresa.agregar_operador("Pedro", 11, 45)
empresa.agregar_operador("Ana", 14, 30)

# Mostrar sueldos
empresa.mostrar_sueldos()