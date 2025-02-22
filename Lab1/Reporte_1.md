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
´´´

Estructuras de Control en Python
Las estructuras de control permiten ejecutar bloques de código según condiciones específicas o repetir instrucciones.

Condiciones: if, elif, else
Las estructuras condicionales permiten ejecutar código en función de una evaluación lógica.

python
Copiar
Editar
edad = 18
if edad >= 18:
    print("Eres mayor de edad")
elif edad > 12:
    print("Eres un adolescente")
else:
    print("Eres un niño")
En este caso, el programa evalúa la variable edad y ejecuta el bloque correspondiente según el valor.

Bucles en Python
Los bucles permiten ejecutar repetidamente un bloque de código mientras se cumpla una condición o sobre una secuencia de elementos.

Bucle for
El bucle for se usa para recorrer elementos dentro de una secuencia, como listas o rangos.

python
Copiar
Editar
# Iterar sobre una lista de elementos
frutas = ["Manzana", "Banana", "Cereza"]
for fruta in frutas:
    print(fruta)

# Iterar un rango de números
for i in range(5):  # Itera de 0 a 4
    print("Número:", i)
En este caso:

Se recorre una lista de frutas y se imprime cada elemento.
Se usa range(5), lo que genera los valores 0, 1, 2, 3, 4.
Bucle while
El bucle while ejecuta un bloque de código mientras la condición especificada sea True.

python
Copiar
Editar
contador = 0
while contador < 5:
    print("Contador:", contador)
    contador += 1
Aquí:

Se inicia la variable contador en 0.
Mientras contador < 5, se imprime su valor y se incrementa en 1.
El bucle termina cuando contador alcanza 5.
Funciones en Python
Las funciones permiten reutilizar código y modularizar programas. Se definen con la palabra clave def.

python
Copiar
Editar
def saludar(nombre):
    return f"Hola, {nombre}!"

print(saludar("Juan"))  # Salida: Hola, Juan!
Se define la función saludar() que recibe un parámetro nombre.
Se devuelve un mensaje con el nombre proporcionado.
Se imprime el resultado de llamar a la función.
Conclusión
Python es un lenguaje versátil y potente con una sintaxis sencilla, lo que lo hace ideal para principiantes y desarrolladores avanzados. Sus estructuras de control (if, for, while) y la posibilidad de definir funciones permiten crear programas eficientes y organizados. Gracias a su flexibilidad y soporte para distintos paradigmas de programación, Python se ha convertido en uno de los lenguajes más utilizados en el mundo.

Referencias
Lutz, M. (2013). Learning Python. O'Reilly Media.
Van Rossum, G. (1991). Python programming language. Python Software Foundation.
Python Software Foundation. (2023). Python documentation. https://docs.python.org/3/