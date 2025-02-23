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

---

## *Problema 5: Adivinanza de un número secreto*
Crea un programa que solicite al usuario adivinar un número secreto.  
- El programa debe generar un número aleatorio entre *1 y 10*, y el usuario debe intentar adivinarlo.  
- El programa debe proporcionar pistas si el número ingresado por el usuario es *demasiado alto o bajo*.  
- El bucle while debe continuar hasta que el usuario adivine correctamente.  
- Al final, se debe imprimir en cuántos intentos el usuario logró adivinar el número.

---

## *Problema 6: Robot explorador en una matriz*
El programa debe generar una matriz de al menos *5x5*. El robot inicia su camino en la *posición (0,0)* de la matriz y debe salir en la *posición (4,4)* o en la posición máxima si el tamaño de la matriz cambia. El número y la posición de los obstáculos es aleatoria.  

El robot solo puede:
- Avanzar.
- Girar a la izquierda.
- Girar a la derecha para buscar un camino libre.

Si el robot *no puede salir*, debe imprimir en pantalla: "Imposible llegar al destino"  

Si el robot *llega a su destino*, deberá imprimir el mapa con los espacios libres y obstáculos de la siguiente forma (X para obstáculos y o para espacios libres). Deberá imprimir también la ruta que siguió y deberá mostrar un segundo mapa con el "camino" seguido por el robot mediante flechas

---

## *Problema 7: Gestión de inventario de una tienda*
Una tienda quiere gestionar su inventario de productos. Para ello, debes implementar un sistema 
en Python que permita: 
- Crear productos, cada uno con un nombre, precio y cantidad en stock. 
- Actualizar la cantidad en stock cuando se venden productos. 
- Mostrar la información de un producto con su disponibilidad. 
- Calcular el valor total del inventario (precio × cantidad de cada producto).

---

# Referencias
- Lutz, M. (2013). Learning Python. O'Reilly Media.
- Van Rossum, G. (1991). Python programming language. Python Software Foundation.
- Python Software Foundation. (2023). Python documentation. https://docs.python.org/3/
- Booch, G. (1994). Object-Oriented Analysis and Design with Applications. Addison-Wesley.