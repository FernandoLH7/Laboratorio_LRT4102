# Introducción a Python
Python es un lenguaje de programación interpretado, de alto nivel y de propósito general, diseñado con un enfoque en la simplicidad y legibilidad del código (Van Rossum, 1991). Su sintaxis clara permite que sea ampliamente utilizado en múltiples áreas como desarrollo web, ciencia de datos, automatización, inteligencia artificial y sistemas embebidos.

Python es un *lenguaje multiparadigma, lo que significa que admite diferentes estilos de programación, como **programación estructurada, funcional y orientada a objetos (POO)*. Además, cuenta con una gran cantidad de bibliotecas y módulos que facilitan su uso en diversas aplicaciones.

---

## *1. Tipos de Variables en Python*
En Python, las variables no requieren una declaración de tipo previa, ya que el lenguaje usa *tipado dinámico*. Esto significa que el tipo de una variable se determina automáticamente según el valor asignado (Lutz, 2013).

Los tipos de datos más utilizados en Python incluyen:

- *Enteros (int)*: Representan números enteros, como 5, -10, 1000.
- *Flotantes (float)*: Números con decimales, como 3.14, -0.5, 2.718.
- *Cadenas (str)*: Texto entre comillas, como "Hola mundo", 'Python es genial'.
- *Booleanos (bool)*: Representan valores de verdad: True o False.
- *Listas (list)*: Colecciones ordenadas y mutables, como [1, 2, 3, "Python"].
- *Tuplas (tuple)*: Colecciones ordenadas e inmutables, como (10, 20, "robot").
- *Diccionarios (dict)*: Colecciones de pares clave-valor, como {"nombre": "Juan", "edad": 22}.

### *Ejemplo de Variables en Python*
```python
# Declaración de variables en Python
entero = 10
flotante = 3.14
cadena = "Hola, Python"
booleano = True
lista = [1, 2, 3, "robot"]
tupla = (10, 20, 30)
diccionario = {"nombre": "Juan", "edad": 22}

# Imprimir variables
print(entero, flotante, cadena, booleano, lista, tupla, diccionario)
```
### *Estructuras de Control en Python*
Las estructuras de control permiten ejecutar bloques de código según condiciones específicas o repetir instrucciones.

#### *Condiciones: if, elif, else*
Las estructuras condicionales permiten ejecutar código en función de una evaluación lógica.

```python
edad = 18
if edad >= 18:
    print("Eres mayor de edad")
elif edad > 12:
    print("Eres un adolescente")
else:
    print("Eres un niño")
```
En este caso, el programa evalúa la variable edad y ejecuta el bloque correspondiente según el valor.

### *Bucles en Python*
Python ofrece dos estructuras principales de repetición: for y while.

### *Bucle for*
El bucle for se usa para iterar sobre secuencias como listas, tuplas o cadenas.

```python
# Iterar sobre una lista
frutas = ["Manzana", "Banana", "Cereza"]
for fruta in frutas:
    print(fruta)

# Iterar usando range()
for i in range(5):  # Itera de 0 a 4
    print("Número:", i)
```
### *Bucle while*
El bucle while se ejecuta mientras una condición sea verdadera.
```python
contador = 0
while contador < 5:
    print("Contador:", contador)
    contador += 1
```
## *2. Funciones en python*
Las funciones permiten encapsular código reutilizable. En Python, se definen con la palabra clave def.

```python
def saludar(nombre):
    return f"Hola, {nombre}!"

print(saludar("Juan"))  # Salida: Hola, Juan!
```
Como se puede observar anteriormente, python es un lenguaje versátil y fácil de aprender, lo que lo hace ideal para principiantes y profesionales. Sus estructuras de control, tipos de datos y paradigmas de programación permiten desarrollar soluciones eficientes para diferentes problemas.

# Programación Orientada a Objetos (POO) en Python

La *Programación Orientada a Objetos (POO)* es un paradigma de programación que se basa en la construcción de *objetos* que contienen datos (atributos) y comportamientos (métodos). Este enfoque permite modelar problemas del mundo real de una manera más estructurada y modular (Booch, 1994).

Python es un lenguaje que soporta POO de manera nativa, permitiendo a los desarrolladores estructurar su código de forma eficiente y reutilizable (Van Rossum, 1991). En este paradigma, el código se organiza en *clases y objetos, aplicando principios como **encapsulamiento, herencia y polimorfismo* para mejorar la reutilización y mantenimiento del software (Lutz, 2013).

---

## *1. Clases y Objetos*
Una *clase* es una plantilla para crear objetos, y un *objeto* es una instancia de una clase con atributos y métodos propios.

```python
class Persona:
    def __init__(self, nombre, edad):
        self.nombre = nombre  # Atributo
        self.edad = edad      # Atributo

    def saludar(self):
        """Método para saludar."""
        return f"Hola, soy {self.nombre} y tengo {self.edad} años."

# Crear un objeto de la clase Persona
persona1 = Persona("Carlos", 22)
print(persona1.saludar())  # Salida: Hola, soy Carlos y tengo 22 años.
```
- El método __init__() actúa como un constructor que inicializa los atributos del objeto.
- self permite acceder a los atributos y métodos de la instancia.

## *2. Encapsulamiento*
El encapsulamiento protege los datos de acceso externo, evitando modificaciones no controladas. Se logra mediante la definición de atributos privados y métodos de acceso.

```python
class CuentaBancaria:
    def __init__(self, saldo):
        self.__saldo = saldo  # Atributo privado

    def depositar(self, cantidad):
        """Método para depositar dinero."""
        self.__saldo += cantidad

    def obtener_saldo(self):
        """Método para obtener el saldo actual."""
        return self.__saldo
# Uso de la clase
cuenta = CuentaBancaria(1000)
cuenta.depositar(500)
print(cuenta.obtener_saldo())  # Salida: 1500
```
- Los atributos privados (__saldo) solo pueden ser modificados dentro de la clase.
- El acceso a datos se realiza mediante métodos específicos (obtener_saldo()).

## *3. Herencia*
La herencia permite que una clase (subclase) reutilice atributos y métodos de otra clase (superclase). Esto evita la repetición de código y mejora la organización.

```python
class Animal:
    def __init__(self, nombre):
        self.nombre = nombre

    def hacer_sonido(self):
        return "Hace un sonido"

# Clase Perro hereda de Animal
class Perro(Animal):
    def hacer_sonido(self):
        return "Ladra"

perro1 = Perro("Rex")
print(perro1.nombre)          # Salida: Rex
print(perro1.hacer_sonido())  # Salida: Ladra
```
- La clase Perro hereda de Animal, reutilizando su estructura y sobrescribiendo el método hacer_sonido().

## *4. Polimorfismo*
El polimorfismo permite que diferentes clases usen el mismo método con diferentes implementaciones. Esto mejora la flexibilidad y reutilización del código.

```python
class Gato:
    def hacer_sonido(self):
        return "Maulla"

class Vaca:
    def hacer_sonido(self):
        return "Muge"

# Uso de polimorfismo
animales = [Gato(), Vaca()]
for animal in animales:
    print(animal.hacer_sonido())
```
- Cada clase define su propia versión de hacer_sonido().
- El polimorfismo permite tratar objetos de distintas clases de manera uniforme.

El paradigma de Programación Orientada a Objetos en Python ofrece una forma modular y eficiente de estructurar el código. Aplicando principios como encapsulamiento, herencia y polimorfismo, los programas son más organizados, reutilizables y mantenibles.

# Problemas a Resolver

A continuación, se presentan los problemas resueltos en este laboratorio. Cada uno de ellos aborda conceptos clave de programación en Python, como el uso de estructuras de control, manipulación de listas, generación de números aleatorios y la aplicación del paradigma de Programación Orientada a Objetos (POO).

---

## *Problema 1: Suma de los primeros números enteros positivos*
Escribir un programa que lea un entero positivo “n” introducido por el usuario y después muestre en pantalla la suma de todos los enteros desde 1 hasta n. La suma de los primeros enteros positivos puede ser calculada de la siguiente forma:

$$
\text{suma} = \frac{n(n+1)}{2}
$$

### *Código*

```python
class Suma:
    def __init__(self):
        # El constructor __init__ se llama automáticamente al crear una instancia de la clase Suma.
        # Aquí pedimos al usuario que ingrese un número y lo convertimos a entero.
        self.numero_aleatorio = int(input("¿Cual es el número?: "))
la la suma de todos los números desde 1 hasta numero_aleatorio.
        # Utiliza la fórmula de la suma de los primeros n números naturales: n(n + 1)/2.
        suma = (self.numero_aleatorio * (self.numero_aleatorio + 1)) / 2
        # Imprime el resultado de la suma.
        print("La suma de todos los números es: ", suma)

# Creamos una instancia de la clase Suma, lo que llama al constructor __init__.
resultado = Suma()
# Llamamos al método sumar de 
    def sumar(self):
        # El método sumar calcula instancia resultado para realizar el cálculo e imprimir el resultado.
resultado.sumar()
```
### *Descripción código*

Este programa implementa la suma de los primeros n números enteros positivos utilizando Programación Orientada a Objetos (POO) en Python. Para ello, se define una clase llamada Suma que encapsula la funcionalidad.

- Entrada de datos: Al crear una instancia de la clase, el constructor _init_ solicita al usuario un número entero (n).
- Cálculo de la suma: La operación se realiza dentro del método sumar(), utilizando la fórmula matemática: 

$$
\text{suma} = \frac{n(n+1)}{2}
$$

- Salida: El resultado se imprime en pantalla mostrando la suma de todos los números desde 1 hasta n.

---

## *Problema 2: Cálculo de salario basado en horas trabajadas*
Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora. Después debe mostrar por pantalla la paga que le corresponde.

### *Código*

```python

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
```
### *Descripción código*

Este programa calcula el salario de un trabajador en función de las horas trabajadas y el costo por hora, utilizando Programación Orientada a Objetos (POO) en Python.

- Entrada de datos: La clase Numer solicita al usuario el número de horas trabajadas y el costo por hora mediante el constructor _init_(), convirtiéndolos a enteros.

- Cálculo del salario: El método calcular() multiplica las horas trabajadas por el costo por hora para obtener el salario total.

- Salida: El programa imprime el salario calculado en pantalla con un mensaje descriptivo.

---

## *Problema 3: Cálculo de salario de operadores*
Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores. Imprime el nombre y el sueldo a pagar de cada operador.

### *Código*

```python
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
```
### *Descripción código*

Este programa implementa un sistema de gestión de sueldos para operadores utilizando Programación Orientada a Objetos (POO) en Python.

- Clase Operador: Modela a un empleado con los atributos nombre, sueldo por hora y horas trabajadas. Contiene el método calcular_sueldo(), que multiplica el sueldo por hora por las horas trabajadas, y mostrar_info(), que imprime el nombre del operador y su sueldo total.

- Clase Empresa: Administra una lista de operadores y permite:

    - Agregar operadores con agregar_operador().
    - Mostrar el sueldo de todos los operadores con mostrar_sueldos().

- Ejecución del programa: Se crea una instancia de Empresa, se añaden varios operadores con sus datos y se imprimen los sueldos de todos los empleados.
---

## *Problema 4: Promedio de números pares y producto de impares*
- Crea una lista llamada numeros que contenga al menos 10 números.  
- Calcula el *promedio de los números pares* y el *producto de los números impares*.  
- Imprime los resultados.

### *Código*

``` python
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
```
### *Descripción código*

Este programa permite analizar una lista de números enteros y obtener información relevante sobre ellos, como la separación en números pares e impares, el cálculo del promedio de los pares y el producto de los impares. Se implementa utilizando Programación Orientada a Objetos (POO) para encapsular la funcionalidad en una clase llamada Numeros.

- Inicialización de la clase Numeros:

    - Se define un constructor _init_() que recibe una lista de números y la almacena en una variable de instancia.

- Método obtener_numeros_pares_impares()

    - Recorre la lista original y clasifica los números en pares e impares.
    - Retorna dos listas separadas: una con los pares y otra con los impares.

- Método calcular_promedio()

    - Obtiene la lista de números pares llamando a obtener_numeros_pares_impares().
    - Calcula el promedio sumando todos los pares y dividiéndolos por la cantidad de elementos en la lista.
    - Si la lista de pares está vacía, retorna 0 para evitar errores de división por cero.

- Método calcular_producto()

    - Obtiene la lista de números impares llamando a obtener_numeros_pares_impares().
    - Utiliza math.prod() para calcular el producto de los números impares.
    - Si no hay impares, retorna 0 para evitar errores.

- Ejecución del programa

    - Se define una lista con los números del 1 al 10.
    - Se crea una instancia de la clase Numeros con esa lista.
    - Se llaman los métodos para obtener los pares, impares, el promedio de los pares y el producto de los impares.
    - Finalmente, los resultados se imprimen en pantalla.
---

## *Problema 5: Adivinanza de un número secreto*
Crea un programa que solicite al usuario adivinar un número secreto.  
- El programa debe generar un número aleatorio entre *1 y 10*, y el usuario debe intentar adivinarlo.  
- El programa debe proporcionar pistas si el número ingresado por el usuario es *demasiado alto o bajo*.  
- El bucle while debe continuar hasta que el usuario adivine correctamente.  
- Al final, se debe imprimir en cuántos intentos el usuario logró adivinar el número.

### *Código*

```python
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
```
### *Descripción código*
Este programa implementa un juego de adivinanza de un número secreto utilizando Programación Orientada a Objetos (POO) en Python. A través de la clase AdivinarNumero, se genera un número aleatorio entre 1 y 10 y se le solicita al usuario que intente adivinarlo, proporcionando pistas hasta que lo logre.

- Generación del número aleatorio:

    - Al crear una instancia de la clase AdivinarNumero, el constructor _init_() genera un número secreto utilizando randint(1,10).
    - Este número se almacena en el atributo self.numero_aleatorio.

- Método jugar()

    - Inicia un bucle while infinito, en el que el usuario debe ingresar un número.
    - Se lleva un contador de intentos (intentos) que se incrementa en cada intento.
    - El número ingresado por el usuario se convierte en entero (int(input())).

- Verificación del número ingresado

    - Si el número del usuario es mayor al número secreto, se muestra "El número está por debajo del que diste".
    - Si es menor, se muestra "El número está por arriba del que diste".
    - Si el usuario adivina el número, el programa muestra "¡Felicidades! Adivinaste el número secreto en X intentos." y el bucle se detiene con break.
    - Si el usuario ingresa un valor no numérico, el programa muestra "Por favor ingrese un número válido" gracias al bloque try-except.
---

## *Problema 6: Robot explorador en una matriz*
El programa debe generar una matriz de al menos *5x5*. El robot inicia su camino en la *posición (0,0)* de la matriz y debe salir en la *posición (4,4)* o en la posición máxima si el tamaño de la matriz cambia. El número y la posición de los obstáculos es aleatoria.  

El robot solo puede:
- Avanzar.
- Girar a la izquierda.
- Girar a la derecha para buscar un camino libre.

Si el robot *no puede salir*, debe imprimir en pantalla: "Imposible llegar al destino"  

Si el robot *llega a su destino*, deberá imprimir el mapa con los espacios libres y obstáculos de la siguiente forma (X para obstáculos y o para espacios libres). Deberá imprimir también la ruta que siguió y deberá mostrar un segundo mapa con el "camino" seguido por el robot mediante flechas

### *Código*

```python
import random

class RobotExplorador:
    def __init__(self, n=5):
        # Inicializa el tamaño del mapa
        self.n = n
        # Crea una matriz n x n llena de 'o' (espacios libres)
        self.matriz = [['o' for _ in range(n)] for _ in range(n)]
        # Posición inicial del robot
        self.robot_pos = (0, 0)
        # Meta en la esquina inferior derecha
        self.destino = (n - 1, n - 1)
        # Lista para guardar el camino recorrido
        self.camino = []
        # El robot inicia moviéndose a la derecha ('D' = Derecha)
        self.direccion = 'D'

        # Genera obstáculos aleatorios en el mapa
        self.generar_obstaculos()
        # Inicia la búsqueda del camino hacia la meta
        self.encontrar_camino()

    def generar_obstaculos(self):
        """Genera obstáculos aleatorios sin bloquear el inicio ni la meta."""
        # Número aleatorio de obstáculos
        num_obstaculos = random.randint(self.n, self.n * 2)
        for _ in range(num_obstaculos):
            while True:
                # Genera coordenadas aleatorias para los obstáculos
                x, y = random.randint(0, self.n - 1), random.randint(0, self.n - 1)
                # Asegura que los obstáculos no se coloquen en el inicio ni en la meta
                if (x, y) not in [(0, 0), self.destino]:
                    self.matriz[x][y] = 'X'
                    break

    def encontrar_camino(self):
        """Ejecuta la lógica de exploración del robot para encontrar la salida."""
        # Posición inicial del robot
        x, y = self.robot_pos
        # Guarda la posición inicial en el camino
        self.camino.append((x, y))

        # Movimientos posibles con sus direcciones
        movimientos = {
            'D': (0, 1),  # Derecha →
            'I': (0, -1),  # Izquierda ←
            'A': (-1, 0),  # Arriba ↑
            'B': (1, 0)  # Abajo ↓
        }

        while (x, y) != self.destino:
            movido = False  # Bandera para saber si se movió en este turno

            # Intentar moverse en la dirección actual
            dx, dy = movimientos[self.direccion]
            nuevo_x, nuevo_y = x + dx, y + dy

            if self.movimiento_valido(nuevo_x, nuevo_y):
                # Si el movimiento es válido, actualiza la posición del robot
                x, y = nuevo_x, nuevo_y
                # Guarda la nueva posición en el camino
                self.camino.append((x, y))
                movido = True
            else:
                # Si no puede moverse, intenta girar en sentido horario
                self.direccion = self.girar_derecha()

            # Si el robot no logra moverse en ningún turno, está atrapado
            if not movido and not any(self.movimiento_valido(x + dx, y + dy) for dx, dy in movimientos.values()):
                print("\nImposible llegar al destino.")
                self.imprimir_mapa()
                return

        print("\nEl robot llegó al destino.")
        self.imprimir_mapa()
        self.mostrar_ruta()

    def movimiento_valido(self, x, y):
        """Verifica si la posición es válida y no es un obstáculo."""
        return 0 <= x < self.n and 0 <= y < self.n and self.matriz[x][y] != 'X'

    def girar_derecha(self):
        """Cambia la dirección en sentido horario."""
        direcciones = ['D', 'B', 'I', 'A']  # Orden: Derecha, Abajo, Izquierda, Arriba
        return direcciones[(direcciones.index(self.direccion) + 1) % 4]

    def imprimir_mapa(self):
        """Imprime el mapa con obstáculos y espacios libres."""
        print("\nMapa del Terreno:")
        for i in range(self.n):
            for j in range(self.n):
                if (i, j) in self.camino:
                    print("*", end=" ")  # Marca el camino con *
                else:
                    print(self.matriz[i][j], end=" ")
            print()

    def mostrar_ruta(self):
        """Muestra el mapa con las flechas indicando la ruta seguida."""
        mapa_ruta = [['o' for _ in range(self.n)] for _ in range(self.n)]

        # Diccionario para representar las direcciones con flechas
        flechas = {
            (0, 1): '→',  # Derecha
            (0, -1): '←',  # Izquierda
            (-1, 0): '↑',  # Arriba
            (1, 0): '↓'  # Abajo
        }

        for i in range(len(self.camino) - 1):
            x1, y1 = self.camino[i]
            x2, y2 = self.camino[i + 1]
            dx, dy = x2 - x1, y2 - y1
            mapa_ruta[x1][y1] = flechas[(dx, dy)]

        mapa_ruta[self.destino[0]][self.destino[1]] = 'F'  # 'F' indica la meta

        print("\nMapa con la Ruta Seguida:")
        for fila in mapa_ruta:
            print(" ".join(fila))


# Ejecutar el programa
RobotExplorador()
```
### *Descripción código*
Este programa implementa un robot explorador en una matriz, que debe encontrar un camino desde la posición inicial (0,0) hasta la meta en (n-1, n-1), esquivando obstáculos generados aleatoriamente. Se utiliza Programación Orientada a Objetos (POO) en Python para estructurar la solución, encapsulando toda la lógica dentro de la clase RobotExplorador.

- Inicialización del robot y el mapa:

    - Se define una matriz n x n con espacios vacíos ('o'), donde n es el tamaño del mapa (por defecto 5x5).
    - Se establece la posición inicial del robot en (0,0).
    - Se define la posición de la meta en (n-1, n-1).
    - Se inicializa una lista camino para registrar las posiciones visitadas por el robot.

- Generación de obstáculos aleatorios

    - Se colocan de n a 2n obstáculos ('X') en posiciones aleatorias dentro de la matriz.
    - Se asegura que ni la posición inicial ni la meta sean bloqueadas.

- Búsqueda del camino

    - El robot comienza en (0,0) y trata de moverse en la dirección predeterminada (derecha).
    - Puede desplazarse en cuatro direcciones (→ ↓ ← ↑) según la disponibilidad del camino.
    - Si encuentra un obstáculo, gira en sentido horario hasta encontrar un camino libre.
    - Si queda atrapado sin salida, el programa imprime "Imposible llegar al destino" y finaliza.
    - Si alcanza la meta, se imprimen:

        - El mapa del terreno con los obstáculos y los espacios libres.
        - El mapa con la ruta seguida, representada con flechas (→ ↓ ← ↑).

Ahora se explica brevemente cada uno de los métodos que se implementaron en el código para entender su función dentro de la solución al problema dado.

- Método generar_obstaculos()

    - Coloca obstáculos aleatorios sin bloquear la entrada o la salida.

- Método encontrar_camino()

    - Aplica la lógica de navegación del robot, moviéndolo hasta la meta o detectando si queda atrapado.

- Método movimiento_valido(x, y)

    - Verifica si la posición (x, y) es válida y no contiene un obstáculo.

- Método girar_derecha()

    - Cambia la dirección de movimiento del robot en sentido horario.

- Método imprimir_mapa()

    - Imprime el mapa con los obstáculos ('X') y los espacios libres ('o').

- Método mostrar_ruta()

    - Imprime el camino recorrido por el robot con flechas (→ ↓ ← ↑) indicando la trayectoria.
---

## *Problema 7: Gestión de inventario de una tienda*
Una tienda quiere gestionar su inventario de productos. Para ello, debes implementar un sistema 
en Python que permita: 
- Crear productos, cada uno con un nombre, precio y cantidad en stock. 
- Actualizar la cantidad en stock cuando se venden productos. 
- Mostrar la información de un producto con su disponibilidad. 
- Calcular el valor total del inventario (precio × cantidad de cada producto).

### *Código y descripción*

Este programa implementa un sistema de gestión de inventario para un supermercado, permitiendo al usuario administrar productos mediante Programación Orientada a Objetos (POO) en Python. Se utiliza una clase Supermercado, la cual encapsula todas las funcionalidades dentro de diferentes métodos.

- Inicialización de la Clase Supermercado

```python
class Supermercado:
    def __init__(self):
        """Inicializa el supermercado con un diccionario para almacenar los productos y ejecuta el menú."""
        self.productos = {}  # Diccionario para almacenar los productos
        self.ejecutar_menu()  # Ejecuta el menú al iniciar la clase
```
- Explicación:
    - Se define la clase Supermercado, que representa el sistema de inventario.
    - self.productos = {}: Se inicializa un diccionario vacío para almacenar los productos. Cada producto tendrá su nombre como clave y un diccionario interno con su precio y cantidad en stock.
    - self.ejecutar_menu(): Llama automáticamente al método ejecutar_menu(), mostrando el menú de opciones cuando se inicia el programa.
    
```python
    def mostrar_menu(self):
        """Muestra las opciones disponibles en el menú."""
        print("\nMenú"
              "\n 1. Añadir productos"
              "\n 2. Vender productos"
              "\n 3. Información de un producto"
              "\n 4. Valor total del inventario"
              "\n 5. Salir")

    def ejecutar_menu(self):
        """Ejecuta el menú en un bucle hasta que el usuario elija salir."""
        while True:
            self.mostrar_menu()  # Muestra el menú de opciones
            try:
                opcion = int(input("¿Cuál es la opción que eliges?: "))  # Solicita al usuario que elija una opción
                if opcion == 1:
                    self.agregar_producto()  # Llama al método para agregar un producto
                elif opcion == 2:
                    self.vender_producto()  # Llama al método para vender un producto
                elif opcion == 3:
                    self.mostrar_producto()  # Llama al método para mostrar información de un producto
                elif opcion == 4:
                    self.valor_inventario()  # Llama al método para calcular el valor total del inventario
                elif opcion == 5:
                    print("Saliendo del programa...")  # Mensaje de salida
                    break  # Sale del bucle y termina el programa
                else:
                    print("Opción no válida, intenta de nuevo.")  # Mensaje de opción no válida
            except ValueError:
                print("Error: Ingresa un número válido.")  # Mensaje de error si la entrada no es un número

    def agregar_producto(self):
        """Añade un nuevo producto con nombre, precio y cantidad al inventario."""
        nombre = input("Nombre del producto: ").lower()  # Solicita el nombre del producto y lo convierte a minúsculas
        if nombre in self.productos:
            print(f"El producto '{nombre}' ya existe. Usa la opción de venta para reducir stock.")
            return  # Si el producto ya existe, muestra un mensaje y termina el método
        try:
            precio = float(input("Precio del producto: "))  # Solicita el precio del producto
            cantidad = int(input("Cantidad en stock: "))  # Solicita la cantidad en stock
            if precio < 0 or cantidad < 0:
                print("Error: Precio y cantidad deben ser valores positivos.")
                return  # Si el precio o la cantidad son negativos, muestra un mensaje y termina el método
        except ValueError:
            print("Error: Ingresa valores numéricos válidos.")  # Mensaje de error si la entrada no es numérica
            return

        self.productos[nombre] = {"precio": precio, "stock": cantidad}  # Añade el producto al diccionario
        print(f"El producto '{nombre}' fue añadido correctamente.")  # Mensaje de confirmación

    def vender_producto(self):
        """Reduce la cantidad en stock cuando se venden productos."""
        nombre = input("Nombre del producto a vender: ").lower()  # Solicita el nombre del producto y lo convierte a minúsculas
        if nombre not in self.productos:
            print(f"El producto '{nombre}' no existe en el inventario.")
            return  # Si el producto no existe, muestra un mensaje y termina el método
        try:
            cantidad = int(input("Cantidad a vender: "))  # Solicita la cantidad a vender
            if cantidad <= 0:
                print("Error: La cantidad debe ser mayor a cero.")
                return  # Si la cantidad es menor o igual a cero, muestra un mensaje y termina el método
        except ValueError:
            print("Error: Ingresa un número entero válido.")  # Mensaje de error si la entrada no es numérica
            return

        if cantidad > self.productos[nombre]['stock']:
            print(f"No hay suficiente stock disponible. Solo hay {self.productos[nombre]['stock']} unidades.")
        else:
            self.productos[nombre]['stock'] -= cantidad  # Reduce la cantidad en stock
            print(f"Venta realizada. Stock restante de '{nombre}': {self.productos[nombre]['stock']}")

            if self.productos[nombre]['stock'] == 0:
                print(f"El producto '{nombre}' se ha agotado y será eliminado del inventario.")
                del self.productos[nombre]  # Elimina el producto si su stock llega a 0

    def mostrar_producto(self):
        """Muestra la información de un producto específico."""
        nombre = input("Nombre del producto a consultar: ").lower()  # Solicita el nombre del producto y lo convierte a minúsculas
        if nombre in self.productos:
            info = self.productos[nombre]
            disponibilidad = "Disponible" if info['stock'] > 0 else "Agotado"
            print(f"Producto: {nombre}, Precio: ${info['precio']:.2f}, Stock: {info['stock']} ({disponibilidad})")
        else:
            print(f"El producto '{nombre}' no existe en el inventario.")

    def valor_inventario(self):
        """Calcula el valor total del inventario (precio * cantidad de cada producto)."""
        if not self.productos:
            print("El inventario está vacío.")
            return

        total = sum(info["precio"] * info["stock"] for info in self.productos.values())  # Calcula el valor total del inventario
        print(f"Valor total del inventario: ${total:.2f}")  # Muestra el valor total del inventario

# Ejecutar el programa
supermercado = Supermercado()  # Crea una instancia de la clase Supermercado y ejecuta el menú
```
### *Descripción código*


---

# Referencias
- Lutz, M. (2013). Learning Python. O'Reilly Media.
- Van Rossum, G. (1991). Python programming language. Python Software Foundation.
- Python Software Foundation. (2023). Python documentation. https://docs.python.org/3/
- Booch, G. (1994). Object-Oriented Analysis and Design with Applications. Addison-Wesley.