# Introducción a Python
Python es un lenguaje de programación interpretado, de alto nivel y de propósito general, diseñado con un enfoque en la simplicidad y legibilidad del código (Van Rossum, 1991). Su sintaxis clara permite que sea ampliamente utilizado en múltiples áreas como desarrollo web, ciencia de datos, automatización, inteligencia artificial y sistemas embebidos.

Python es un *lenguaje multiparadigma, lo que significa que admite diferentes estilos de programación, como **programación estructurada, funcional y orientada a objetos (POO)*. Además, cuenta con una gran cantidad de bibliotecas y módulos que facilitan su uso en diversas aplicaciones.

---

## *Tipos de Variables en Python*
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
## *Funciones en python*
Las funciones permiten encapsular código reutilizable. En Python, se definen con la palabra clave def.

```python
def saludar(nombre):
    return f"Hola, {nombre}!"

print(saludar("Juan"))  # Salida: Hola, Juan!
```
Como se puede observar anteriormente, python es un lenguaje versátil y fácil de aprender, lo que lo hace ideal para principiantes y profesionales. Sus estructuras de control, tipos de datos y paradigmas de programación permiten desarrollar soluciones eficientes para diferentes problemas.