import math

# Definimos la clase Numeros
class Numeros:
    def __init__(self, lista):
        # Inicializamos el constructor con la lista proporcionada
        self.lista = lista

    def obtener_numeros_pares_impares(self):
        # Inicializamos las listas de pares e impares
        pares = []
        impares = []
        # Recorremos cada número en la lista
        for num in self.lista:
            # Verificamos si el número es par
            if num % 2 == 0:
                # Si es par, lo agregamos a la lista de pares
                pares.append(num)
            else:
                # Si es impar, lo agregamos a la lista de impares
                impares.append(num)
        # Retornamos las listas de pares e impares
        return pares, impares

    def calcular_promedio(self):
        # Obtenemos los números pares usando el método anterior
        pares, _ = self.obtener_numeros_pares_impares()
        # Verificamos si la lista de pares está vacía para evitar división por cero
        if len(pares) == 0:
            return 0
        # Calculamos el promedio de los números pares
        prom = sum(pares) / len(pares)
        # Retornamos el promedio calculado
        return prom

    def calcular_producto(self):
        # Obtenemos los números impares usando el método anterior
        _, impares = self.obtener_numeros_pares_impares()
        # Verificamos si la lista de impares está vacía para evitar errores
        if len(impares) == 0:
            return 0
        # Calculamos el producto de los números impares usando math.prod
        prod = math.prod(impares)
        # Retornamos el producto calculado
        return prod

# Definimos una lista de números del 1 al 10
numeros_lista = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
# Creamos una instancia de la clase Numeros con la lista proporcionada
numeros_objeto = Numeros(numeros_lista)

# Obtenemos los números pares e impares
pares, impares = numeros_objeto.obtener_numeros_pares_impares()
# Calculamos el promedio de los números pares
prom = numeros_objeto.calcular_promedio()
# Calculamos el producto de los números impares
prod = numeros_objeto.calcular_producto()

# Imprimimos la lista original y los resultados obtenidos
print("Lista original:", numeros_objeto.lista)
print("Números pares:", pares)
print("Números impares:", impares)
print("Promedio pares:", prom)
print("Producto impares:", prod)