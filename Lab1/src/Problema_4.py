import math

# Como trabajaremos con programación orientada a objetos, primero debemos definir nuestra clase
class Numeros:                #Llamamos nuestra clase Numeros
    def __init__(self, lista):      #Inicializamos el constructor usando self y lista como atributos
        self.lista = lista      #Asignamos el valor de lista a self de forma que con esto indicamos que self almacenará nuestra lista

    def obtener_numeros_pares_impares(self):   #Creamos un nuevo metodo para difereciar entre numeros pares e impares
        pares = []  # Inicializamos la lista de pares
        impares = []     #Inicializamos la lista de impares
        for num in self.lista:      #Creamos un ciclo for donde num será nuestro contador que incrementará de 1 en 1 para recorrer todos los valores en la lista
            if num % 2 == 0:      #Establecemos que si el valor que tiene num, su residuo es 0, entonces será par
                pares.append(num)   #Si el valor es par entonces se agregará a la lista de pares con append
            else:
                impares.append(num)   #Sino son pares, entonces se mandarán a la lista de impares
        return pares, impares     #Regresamos pares e impares para usarlos adelante

    def calcular_promedio(self):
        pares, _ = self.obtener_numeros_pares_impares()     #Como nuestro metodo obtener_numeros_pares_impares retorna dos valores y solo queremos pares, dejamos con _ a impares
        # usamos sum() para sumar los elementos de una lista
        # usamos len() para saber cuantos elementos tiene una lista
        # en caso de que nuestra lista este vacia, para no dividir una cantidad entre cero que da infinito, retornamos cero
        if len(pares) == 0:   #Nos aseguramos de que no haya una división entre cero de esta forma si la dimesión de la lista pares es de 0 se retornará cero
            return 0
        prom = sum(pares) / len(pares)    #Se obtiene el promedio de la suma de los valores en la lista entre el numero de elementos
        return prom    #Retornamos el promedio

    def calcular_producto(self):
        _, impares = self.obtener_numeros_pares_impares()    #Como antes, ahora usamos impares y no pares
        if len(impares) == 0: # Nuevamente evitamos dividir en cero
            return 0
        prod = math.prod(impares) #Usamos la biblioteca math con la función prod para sacar el producto de los componentes de la lista
        return prod

numeros_lista = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]    #Esta es nuestra lista del 1 al 10
numeros_objeto = Numeros(numeros_lista)      # Aignamos esta lista a nuestra clase Numeros y posteriormente esta se asignara a self

pares, impares = numeros_objeto.obtener_numeros_pares_impares()   #Generamos nuestras variables con el reusltadoa  imprimir
prom = numeros_objeto.calcular_promedio()
prod = numeros_objeto.calcular_producto()

print("Lista original:", numeros_objeto.lista) #Impirmimos los resultados
print("Números pares:", pares)

print("Números impares:", impares)
print("Promedio pares:", prom)
print("Producto impares:", prod)