class Operador:
    def __init__(self, nombre, sueldo_por_hora, horas_trabajadas):
        """Inicializa los atributos del operador."""
        # Inicializa el nombre del operador
        self.nombre = nombre
        # Inicializa el sueldo por hora del operador
        self.sueldo_por_hora = sueldo_por_hora
        # Inicializa las horas trabajadas por el operador
        self.horas_trabajadas = horas_trabajadas

    def calcular_sueldo(self):
        """Calcula el sueldo total del operador."""
        # Calcula el sueldo total multiplicando el sueldo por hora por las horas trabajadas
        return self.sueldo_por_hora * self.horas_trabajadas
    
    def mostrar_info(self):
        """Muestra el nombre y sueldo total del operador."""
        # Imprime el nombre del operador y su sueldo total formateado a dos decimales
        print(f"Operador: {self.nombre}, Sueldo total: ${self.calcular_sueldo():.2f}")


class Empresa:
    def __init__(self):
        """Inicializa la lista de operadores."""
        # Inicializa una lista vacía para almacenar los operadores
        self.operadores = []

    def agregar_operador(self, nombre, sueldo_por_hora, horas_trabajadas):
        """Añade un operador a la empresa."""
        # Crea una instancia de la clase Operador con los datos proporcionados
        operador = Operador(nombre, sueldo_por_hora, horas_trabajadas)
        # Añade el operador a la lista de operadores de la empresa
        self.operadores.append(operador)

    def mostrar_sueldos(self):
        """Muestra el sueldo de cada operador."""
        # Imprime un encabezado para la lista de sueldos
        print("\nLista de sueldos de operadores:")
        # Itera sobre cada operador en la lista de operadores
        for operador in self.operadores:
            # Llama al método mostrar_info de cada operador para imprimir su información
            operador.mostrar_info()

# Crear empresa y agregar operadores
# Crea una instancia de la clase Empresa
empresa = Empresa()
# Añade varios operadores a la empresa con sus respectivos nombres, sueldos por hora y horas trabajadas
empresa.agregar_operador("Juan", 10, 40)
empresa.agregar_operador("María", 12, 35)
empresa.agregar_operador("Carlos", 15, 42)
empresa.agregar_operador("Sofía", 9, 38)
empresa.agregar_operador("Pedro", 11, 45)
empresa.agregar_operador("Ana", 14, 30)

# Mostrar sueldos
# Llama al método mostrar_sueldos de la empresa para imprimir la lista de sueldos de los operadores
empresa.mostrar_sueldos()