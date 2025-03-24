# Introduction to Python
Python is an interpreted, high-level, general-purpose programming language designed with a focus on simplicity and code readability (Van Rossum, 1991). Its clear syntax allows it to be widely used in multiple fields such as web development, data science, automation, artificial intelligence, and embedded systems.

Python is a multi-paradigm language, which means that it supports different programming styles, such as **structured, functional and object-oriented programming (OOP). In addition, it has a large number of libraries and modules that facilitate its use in various applications.

---

## *1. Types of Variables in Python*
In Python, variables do not require a prior type declaration, as the language uses dynamic typing. This means that the type of a variable is determined automatically based on the assigned value (Lutz, 2013).

The most commonly used data types in Python include:

- *Integers  (int)*: Represent whole numbers, such as 5, -10, 1000.
- *Floats  (float)*: Numbers with decimals, such as 3.14, -0.5, 2.718.
- *Strings  (str)*: Text enclosed in quotes, such as "Hello world", 'Python is great'.
- *Booleans  (bool)*: Represent truth values: True or False.
- *Lists  (list)*: Ordered and mutable collections, such as [1, 2, 3, "Python"].
- *Tuples  (tuple)*: Ordered and immutable collections, such as (10, 20, "robot").
- *Dictionaries  (dict)*: Collections of key-value pairs, such as {"name": "Juan", "age": 22}.

### *Example of Variables in Python*
```python
# Declaration of variables in Python
entero = 10
flotante = 3.14
cadena = "Hello, Python"
booleano = True
lista = [1, 2, 3, "robot"]
tupla = (10, 20, 30)
diccionario = {"name": "Juan", "age": 22}

# Print variables
print(entero, flotante, cadena, booleano, lista, tupla, diccionario)
```
### *Control Structures in Python*
Control structures allow you to execute blocks of code based on specific conditions or to repeat instructions.

#### *Conditions: if, elif, else*
Conditional structures allow you to execute code according to a logical evaluation.

```python
age = 18
if age >= 18:
    print("You are an adult")
elif age > 12:
    print("You are a teenager")
else:
    print("You are a child")
```
In this case, the program evaluates the variable age and executes the corresponding block based on its value.

### *Loops in Python*
Python offers two main looping structures: for and while.

### *For Loop*
The for loop is used to iterate over sequences such as lists, tuples, or strings.

```python
# Iterate over a list
fruits = ["Apple", "Banana", "Cherry"]
for fruit in fruits:
    print(fruit)

# Iterate using range()
for i in range(5):  # Iterates from 0 to 4
    print("Number:", i)
```
### *While Loop*
The while loop executes as long as a condition remains true.
```python
counter = 0
while counter < 5:
    print("Counter:", counter)
    counter += 1
```
## *2. Functions in Python*
Functions allow you to encapsulate reusable code. In Python, they are defined using the def keyword.

```python
def greet(name):
    return f"Hello, {name}!"

print(greet("Juan"))  # Output: Hello, Juan!
```
As seen above, Python is a versatile and easy-to-learn language, making it ideal for both beginners and professionals. Its control structures, data types, and programming paradigms enable the development of efficient solutions for various problems.

# Object-Oriented Programming (OOP) in Python

Object-Oriented Programming (OOP) is a programming paradigm based on the construction of objects that contain data (attributes) and behaviors (methods). This approach allows real-world problems to be modeled in a more structured and modular way (Booch, 1994).

Python is a language that natively supports OOP, enabling developers to structure their code in an efficient and reusable manner (Van Rossum, 1991). In this paradigm, the code is organized into classes and objects, applying principles such as encapsulation, inheritance, and polymorphism to enhance code reusability and maintainability (Lutz, 2013).

---

## *1. Classes and Objects*
A class is a template for creating objects, and an object is an instance of a class with its own attributes and methods.

```python
class Person:
    def __init__(self, name, age):
        self.name = name  # Attribute
        self.age = age    # Attribute

    def greet(self):
        """Method to greet."""
        return f"Hello, I am {self.name} and I am {self.age} years old."

# Create an object of the Person class
person1 = Person("Carlos", 22)
print(person1.greet())  # Output: Hello, I am Carlos and I am 22 years old.
```
- The __init__() method acts as a constructor that initializes the object's attributes.
- self allows access to the instance's attributes and methods.

## *2. Encapsulation*
Encapsulation protects data from external access, preventing uncontrolled modifications. This is achieved by defining private attributes and accessor methods.

```python
class BankAccount:
    def __init__(self, balance):
        self.__balance = balance  # Private attribute

    def deposit(self, amount):
        """Method to deposit money."""
        self.__balance += amount

    def get_balance(self):
        """Method to obtain the current balance."""
        return self.__balance

# Using the class
account = BankAccount(1000)
account.deposit(500)
print(account.get_balance())  # Output: 1500
```
- Private attributes (e.g., __balance) can only be modified within the class.
- Data is accessed through specific methods (e.g., get_balance()).

## *3. Inheritance*
Inheritance allows a class (subclass) to reuse attributes and methods from another class (superclass). This avoids code repetition and improves organization.

```python
class Animal:
    def __init__(self, name):
        self.name = name

    def make_sound(self):
        return "Makes a sound"

# Dog class inherits from Animal
class Dog(Animal):
    def make_sound(self):
        return "Barks"

dog1 = Dog("Rex")
print(dog1.name)          # Output: Rex
print(dog1.make_sound())  # Output: Barks
```
- The Dog class inherits from Animal, reusing its structure and overriding the make_sound() method.

## *4. Polymorphism*
Polymorphism allows different classes to use the same method with different implementations. This enhances flexibility and code reusability.

```python
class Cat:
    def make_sound(self):
        return "Meows"

class Cow:
    def make_sound(self):
        return "Moos"

# Using polymorphism
animals = [Cat(), Cow()]
for animal in animals:
    print(animal.make_sound())
```
- Each class defines its own version of the make_sound() method.
- Polymorphism allows objects of different classes to be treated uniformly.

The Object-Oriented Programming paradigm in Python offers a modular and efficient way to structure code. By applying principles such as encapsulation, inheritance, and polymorphism, programs become more organized, reusable, and maintainable.

# Problems to Solve

Below are the problems solved in this lab. Each one addresses key Python programming concepts such as the use of control structures, list manipulation, random number generation, and the application of the Object-Oriented Programming (OOP) paradigm.

---

## *Problem 1: Sum of the First Positive Integers*
Write a program that reads a positive integer “n” entered by the user and then displays on screen the sum of all integers from 1 to n. The sum of the first positive integers can be calculated as follows:

$$
\text{suma} = \frac{n(n+1)}{2}
$$

### *Code*

```python
class Suma:
    def __init__(self):
        # The __init__ constructor is automatically called when creating an instance of the Suma class.
        # Here we ask the user to input a number and convert it to an integer.
        self.numero_aleatorio = int(input("What is the number?: "))
la la sum of all the numbers from 1 to numero_aleatorio.
        # It uses the formula for the sum of the first n natural numbers: n(n + 1)/2.
        suma = (self.numero_aleatorio * (self.numero_aleatorio + 1)) / 2
        # Prints the result of the sum.
        print("The sum of all the numbers is: ", suma)

# We create an instance of the Suma class, which calls the __init__ constructor.
resultado = Suma()
# We call the sumar method of 
    def sumar(self):
        # The sumar method calculates (using) the result instance to perform the calculation and print the result.
resultado.sumar()
```
### *Code Description*

This program implements the sum of the first n positive integers using Object-Oriented Programming (OOP) in Python. For this, a class named Suma is defined to encapsulate the functionality.

- Input: When an instance of the class is created, the init constructor asks the user for an integer (n).
- Sum Calculation: The operation is performed within the sumar() method, using the mathematical formula:

$$
\text{suma} = \frac{n(n+1)}{2}
$$

- Output: The result is printed on the screen, showing the sum of all numbers from 1 to n.

---

## *Problem 2: Salary Calculation Based on Hours Worked*
Write a program that asks the user for the number of hours worked and the cost per hour. Then it should display on screen the corresponding pay.

### *Code*

```python
class Numer:
    def __init__(self):
        # The __init__ constructor is automatically called when creating an instance of the Numer class.
        # Here we ask the user to enter the number of hours worked and the cost per hour.
        # We convert these inputs to integers and store them in the instance variables self.horas and self.costo.
        self.horas = int(input("Enter the number of hours you have worked: "))
        self.costo = int(input("Enter the cost per hour worked: "))

    def calcular(self):
        # The calcular method multiplies the number of hours worked by the cost per hour.
        # The result of this multiplication is stored in the variable salario.
        salario = self.horas * self.costo
        # Prints the calculated salary with a descriptive message.
        print("The salary you are entitled to is: ", salario, "$")

# We create an instance of the Numer class, which calls the __init__ constructor.
paga = Numer()
# We call the calcular method of the paga instance to perform the calculation and print the result.
paga.calcular()
```
### *Code Description*

This program calculates a worker's salary based on the number of hours worked and the cost per hour, using Object-Oriented Programming (OOP) in Python.

- Input: The Numer class asks the user for the number of hours worked and the cost per hour through the init constructor, converting them to integers.
- Salary Calculation: The calcular() method multiplies the hours worked by the cost per hour to obtain the total salary.
- Output: The program prints the calculated salary on the screen with a descriptive message.

---

## *Problem 3: Calculation of Operators' Salary*
Create a list containing the name, hourly wage, and hours worked for at least six operators. Print the name and the salary to be paid for each operator.

### *Code*

```python
class Operator:
    def __init__(self, name, hourly_wage, hours_worked):
        """Initializes the operator's attributes."""
        # Initialize the operator's name
        self.name = name
        # Initialize the operator's hourly wage
        self.hourly_wage = hourly_wage
        # Initialize the operator's hours worked
        self.hours_worked = hours_worked

    def calculate_salary(self):
        """Calculates the operator's total salary."""
        # Calculate the total salary by multiplying the hourly wage by the hours worked
        return self.hourly_wage * self.hours_worked
    
    def show_info(self):
        """Displays the operator's name and total salary."""
        # Print the operator's name and total salary formatted to two decimal places
        print(f"Operator: {self.name}, Total Salary: ${self.calculate_salary():.2f}")


class Company:
    def __init__(self):
        """Initializes the list of operators."""
        # Initialize an empty list to store the operators
        self.operators = []

    def add_operator(self, name, hourly_wage, hours_worked):
        """Adds an operator to the company."""
        # Create an instance of the Operator class with the provided data
        operator = Operator(name, hourly_wage, hours_worked)
        # Add the operator to the company's list of operators
        self.operators.append(operator)

    def show_salaries(self):
        """Displays the salary of each operator."""
        # Print a header for the list of salaries
        print("\nList of operator salaries:")
        # Iterate over each operator in the list
        for operator in self.operators:
            # Call the show_info method of each operator to print their information
            operator.show_info()

# Create a company and add operators
# Create an instance of the Company class
company = Company()
# Add several operators to the company with their respective names, hourly wages, and hours worked
company.add_operator("Juan", 10, 40)
company.add_operator("María", 12, 35)
company.add_operator("Carlos", 15, 42)
company.add_operator("Sofía", 9, 38)
company.add_operator("Pedro", 11, 45)
company.add_operator("Ana", 14, 30)

# Show salaries
# Call the show_salaries method of the company to print the list of operator salaries
company.show_salaries()
```
### *Code Description*

This program implements a salary management system for operators using Object-Oriented Programming (OOP) in Python.

- Operator Class: Models an employee with the attributes name, hourly wage, and hours worked. It contains the method calculate_salary(), which multiplies the hourly wage by the hours worked, and show_info(), which prints the operator's name and total salary.

- Company Class: Manages a list of operators and allows:

    - Adding operators with add_operator().
    - Displaying the salary of all operators with show_salaries().

- Program Execution: An instance of Company is created, several operators are added with their data, and the salaries of all employees are printed.

---

## *Problem 4: Average of Even Numbers and Product of Odd Numbers*
- Create a list called numbers that contains at least 10 numbers.
- Calculate the average of the even numbers and the product of the odd numbers.
- Print the results.

### *Code*

``` python
import math

# Define the Numbers class
class Numbers:
    def __init__(self, numbers_list):
        # Initialize the constructor with the provided list
        self.numbers_list = numbers_list

    def get_even_odd_numbers(self):
        # Initialize lists for evens and odds
        evens = []
        odds = []
        # Iterate over each number in the list
        for num in self.numbers_list:
            # Check if the number is even
            if num % 2 == 0:
                # If it is even, add it to the evens list
                evens.append(num)
            else:
                # If it is odd, add it to the odds list
                odds.append(num)
        # Return the lists of evens and odds
        return evens, odds

    def calculate_average(self):
        # Obtain the even numbers using the previous method
        evens, _ = self.get_even_odd_numbers()
        # Check if the evens list is empty to avoid division by zero
        if len(evens) == 0:
            return 0
        # Calculate the average of the even numbers
        avg = sum(evens) / len(evens)
        # Return the calculated average
        return avg

    def calculate_product(self):
        # Obtain the odd numbers using the previous method
        _, odds = self.get_even_odd_numbers()
        # Check if the odds list is empty to avoid errors
        if len(odds) == 0:
            return 0
        # Calculate the product of the odd numbers using math.prod
        prod = math.prod(odds)
        # Return the calculated product
        return prod

# Define a list of numbers from 1 to 10
numbers_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
# Create an instance of the Numbers class with the provided list
numbers_object = Numbers(numbers_list)

# Get the even and odd numbers
evens, odds = numbers_object.get_even_odd_numbers()
# Calculate the average of the even numbers
avg = numbers_object.calculate_average()
# Calculate the product of the odd numbers
prod = numbers_object.calculate_product()

# Print the original list and the obtained results
print("Original list:", numbers_object.numbers_list)
print("Even numbers:", evens)
print("Odd numbers:", odds)
print("Average of evens:", avg)
print("Product of odds:", prod)
```
### *Code Description*

This program analyzes a list of integers and extracts relevant information such as separating the numbers into evens and odds, calculating the average of the even numbers, and computing the product of the odd numbers. It is implemented using Object-Oriented Programming (OOP) to encapsulate functionality within a class called Numbers.

- Initialization of the Numbers Class:

    - An init() constructor is defined that receives a list of numbers and stores it in an instance variable.

- Method get_even_odd_numbers():

    - Iterates over the original list and classifies the numbers into evens and odds.
    - Returns two separate lists: one with the even numbers and another with the odd numbers.

- Method calculate_average():

    - Obtains the list of even numbers by calling get_even_odd_numbers().
    - Calculates the average by summing all the even numbers and dividing by the number of elements.
    - If the even numbers list is empty, it returns 0 to avoid division by zero errors.

- Method calculate_product():

    - Obtains the list of odd numbers by calling get_even_odd_numbers().
    - Uses math.prod() to calculate the product of the odd numbers.
    - If there are no odd numbers, it returns 0 to avoid errors.

- Program Execution:

    - A list containing the numbers from 1 to 10 is defined.
    - An instance of the Numbers class is created with that list.
    - The methods are called to obtain the even numbers, odd numbers, the average of the even numbers, and the product of the odd numbers.
    - Finally, the results are printed on the screen.
---

## *Problem 5: Guessing the Secret Number*
Create a program that asks the user to guess a secret number.
- The program must generate a random number between 1 and 10, and the user must try to guess it.
- The program must provide hints if the number entered by the user is too high or too low.
- The while loop must continue until the user guesses correctly.
- At the end, it must print how many attempts the user took to guess the number.

### *Code*
```python
from random import *

class AdivinarNumero:
    def __init__(self):
        # Generates a random number between 1 and 10
        self.numero_aleatorio = randint(1, 10)

    def jugar(self):
        # Initializes the attempt counter
        intentos = 0
        while True:
            try:
                # Increment the attempt counter on each iteration
                intentos += 1
                # Ask the user to enter a number and convert it to an integer
                numero_usuario = int(input("What is the secret number between 0 and 10?: "))
                # Compare the user's number with the random number
                if numero_usuario > self.numero_aleatorio:
                    # If the entered number is greater, inform the user
                    print("The number is lower than the one you provided")
                elif numero_usuario < self.numero_aleatorio:
                    # If the entered number is lower, inform the user
                    print("The number is higher than the one you provided")
                else:
                    # If the entered number equals the random number, congratulate the user
                    # and inform them of the number of attempts made
                    print(f"Congratulations! You guessed the secret number in {intentos} attempts.")
                    break
            except ValueError:
                # If the user enters an invalid value, display an error message
                print("Please enter a valid number")

# Create an instance of the AdivinarNumero class and call the jugar method
juego = AdivinarNumero()
juego.jugar()
```
### *Code Description*
This program implements a secret number guessing game using Object-Oriented Programming (OOP) in Python. Through the AdivinarNumero class, a random number between 1 and 10 is generated, and the user is prompted to guess it, receiving hints until they succeed.

-Random Number Generation:

    - When an instance of the AdivinarNumero class is created, the __init__() constructor generates a secret number using randint(1,10).
    - This number is stored in the attribute self.numero_aleatorio.

- Method jugar():

    - Starts an infinite while loop in which the user must input a number.
    - An attempt counter (intentos) is maintained, incrementing with each guess.
    - The number entered by the user is converted to an integer using int(input()).

- Verification of the Entered Number:

    - If the user's number is greater than the secret number, the program prints "The number is lower than the one you provided".
    - If it is lower, the program prints "The number is higher than the one you provided".
    - If the user guesses correctly, the program prints "Congratulations! You guessed the secret number in X attempts." and exits the loop with break.
    - If the user enters a non-numeric value, the program prints "Please enter a valid number" using a try-except block.
---

## *Problem 6: Robot Explorer in a Matrix*
The program must generate a matrix of at least 5x5. The robot starts its journey at the position (0,0) in the matrix and must exit at the position (4,4) or at the maximum position if the matrix size changes. The number and position of obstacles are random.

The robot can only:

- Move forward.
- Turn left.
- Turn right to look for a free path.

If the robot cannot exit, it must print: "Impossible to reach the destination" on the screen.

If the robot reaches its destination, it should print the map showing free spaces and obstacles (using X for obstacles and o for free spaces). It should also print the route it followed and display a second map with the robot's "path" marked by arrows.

### *Code*

```python
import random

class RobotExplorador:
    def __init__(self, n=5):
        # Initializes the map size
        self.n = n
        # Creates an n x n matrix filled with 'o' (free spaces)
        self.matriz = [['o' for _ in range(n)] for _ in range(n)]
        # Initial position of the robot
        self.robot_pos = (0, 0)
        # Destination at the bottom-right corner
        self.destino = (n - 1, n - 1)
        # List to store the path taken
        self.camino = []
        # The robot starts moving to the right ('D' = Right)
        self.direccion = 'D'

        # Generate random obstacles on the map
        self.generar_obstaculos()
        # Begin the search for a path to the destination
        self.encontrar_camino()

    def generar_obstaculos(self):
        """Generates random obstacles without blocking the start or the destination."""
        # Random number of obstacles
        num_obstaculos = random.randint(self.n, self.n * 2)
        for _ in range(num_obstaculos):
            while True:
                # Generate random coordinates for the obstacles
                x, y = random.randint(0, self.n - 1), random.randint(0, self.n - 1)
                # Ensure that obstacles are not placed at the start or destination
                if (x, y) not in [(0, 0), self.destino]:
                    self.matriz[x][y] = 'X'
                    break

    def encontrar_camino(self):
        """Executes the robot's navigation logic to find the exit."""
        # Initial position of the robot
        x, y = self.robot_pos
        # Save the initial position in the path
        self.camino.append((x, y))

        # Possible movements with their directions
        movimientos = {
            'D': (0, 1),  # Right →
            'I': (0, -1), # Left ←
            'A': (-1, 0), # Up ↑
            'B': (1, 0)   # Down ↓
        }

        while (x, y) != self.destino:
            movido = False  # Flag to indicate if a move was made this turn

            # Attempt to move in the current direction
            dx, dy = movimientos[self.direccion]
            nuevo_x, nuevo_y = x + dx, y + dy

            if self.movimiento_valido(nuevo_x, nuevo_y):
                # If the move is valid, update the robot's position
                x, y = nuevo_x, nuevo_y
                # Save the new position in the path
                self.camino.append((x, y))
                movido = True
            else:
                # If it cannot move, try turning clockwise
                self.direccion = self.girar_derecha()

            # If the robot cannot move in any direction, it is trapped
            if not movido and not any(self.movimiento_valido(x + dx, y + dy) for dx, dy in movimientos.values()):
                print("\nImpossible to reach the destination.")
                self.imprimir_mapa()
                return

        print("\nThe robot reached the destination.")
        self.imprimir_mapa()
        self.mostrar_ruta()

    def movimiento_valido(self, x, y):
        """Checks if the position (x, y) is valid and not an obstacle."""
        return 0 <= x < self.n and 0 <= y < self.n and self.matriz[x][y] != 'X'

    def girar_derecha(self):
        """Changes the movement direction clockwise."""
        direcciones = ['D', 'B', 'I', 'A']  # Order: Right, Down, Left, Up
        return direcciones[(direcciones.index(self.direccion) + 1) % 4]

    def imprimir_mapa(self):
        """Prints the map with obstacles and free spaces."""
        print("\nTerrain Map:")
        for i in range(self.n):
            for j in range(self.n):
                if (i, j) in self.camino:
                    print("*", end=" ")  # Marks the path with *
                else:
                    print(self.matriz[i][j], end=" ")
            print()

    def mostrar_ruta(self):
        """Displays the map with arrows indicating the route taken."""
        mapa_ruta = [['o' for _ in range(self.n)] for _ in range(self.n)]

        # Dictionary to represent directions with arrows
        flechas = {
            (0, 1): '→',   # Right
            (0, -1): '←',  # Left
            (-1, 0): '↑',  # Up
            (1, 0): '↓'    # Down
        }

        for i in range(len(self.camino) - 1):
            x1, y1 = self.camino[i]
            x2, y2 = self.camino[i + 1]
            dx, dy = x2 - x1, y2 - y1
            mapa_ruta[x1][y1] = flechas[(dx, dy)]

        mapa_ruta[self.destino[0]][self.destino[1]] = 'F'  # 'F' indicates the destination

        print("\nMap with the Route Taken:")
        for fila in mapa_ruta:
            print(" ".join(fila))


# Execute the program
RobotExplorador()
```
### *Code Description*
This program implements a robot explorer in a matrix, which must find a path from the starting position (0,0) to the destination at (n-1, n-1), while avoiding randomly generated obstacles. It uses Object-Oriented Programming (OOP) in Python to structure the solution by encapsulating all the logic within the RobotExplorador class.

- Initialization of the Robot and the Map:

    - An n x n matrix is created with free spaces ('o'), where n is the map size (default is 5x5).
    - The robot's starting position is set at (0,0).
    - The destination is defined at (n-1, n-1).
    - A list named camino is initialized to record the positions visited by the robot.

- Generation of Random Obstacles:

    - A random number of obstacles (between n and 2n) are placed at random positions in the matrix.
    - It is ensured that neither the starting position nor the destination are blocked.

Pathfinding:

    - The robot begins at (0,0) and attempts to move in the predetermined direction (right).
    - It can move in four directions (→ ↓ ← ↑) depending on which paths are available.
    - If it encounters an obstacle, the robot turns clockwise until it finds a free path.
    - If the robot becomes trapped with no possible moves, the program prints "Impossible to reach the destination" and terminates.
    - If the robot reaches the destination, the program prints:
        - The terrain map with obstacles ('X') and free spaces ('o').
        - A second map displaying the route taken, represented with arrows (→ ↓ ← ↑).

Below is a brief explanation of each method implemented in the code:

- Method generar_obstaculos():

    - Places random obstacles on the map while ensuring that the entrance and destination remain unblocked.

- Method encontrar_camino():

    - Implements the robot's navigation logic, moving it toward the destination or detecting if it is trapped.

- Method movimiento_valido(x, y):

    - Checks whether the position (x, y) is valid (within the matrix bounds) and free of obstacles.

- Method girar_derecha():

    - Changes the robot's current movement direction clockwise.

- Method imprimir_mapa():

    - Prints the terrain map showing obstacles ('X') and free spaces ('o'), marking the robot's path with an asterisk (*).

- Method mostrar_ruta():

    - Displays a map with arrows (→ ↓ ← ↑) indicating the direction of movement along the path taken by the robot, with 'F' marking the destination.
---

## *Problem 7: Store Inventory Management*
A store wants to manage its product inventory. For this, you must implement a system in Python that allows you to:
- Create products, each with a name, price, and stock quantity.
- Update the stock quantity when products are sold.
- Display a product's information along with its availability.
- Calculate the total inventory value (price × quantity of each product).

### *Code and Description*

This program implements an inventory management system for a supermarket, allowing the user to manage products using Object-Oriented Programming (OOP) in Python. A class called Supermercado is used, which encapsulates all functionalities within various methods.

#### *Initialization of the Supermercado Class*

```python
class Supermercado:
    def __init__(self):
        """Initializes the supermarket with a dictionary to store products and runs the menu."""
        self.productos = {}  # Dictionary to store the products
        self.ejecutar_menu()  # Runs the menu when the class is instantiated
```
- Explanation:
    - The Supermercado class is defined to represent the inventory system.
    - self.productos = {} initializes an empty dictionary to store the products. Each product will have its name as the key and an inner dictionary containing its price and stock quantity as the value.
    - self.ejecutar_menu() automatically calls the menu method, displaying the menu options when the program starts.

#### *Method mostrar_menu(): Show Available Options*
```python
    def mostrar_menu(self):
        """Displays the available options in the menu."""
        print("\nMenu"
              "\n 1. Add products"
              "\n 2. Sell products"
              "\n 3. Product information"
              "\n 4. Total inventory value"
              "\n 5. Exit")
```
- Explanation:
    - This method displays a menu with the available options for the user.
    - It has no inputs or outputs beyond simply printing the menu on the screen.

#### *Method ejecutar_menu(): Main Flow Control*
```python
    def ejecutar_menu(self):
        """Runs the menu in a loop until the user chooses to exit."""
        while True:
            self.mostrar_menu()  # Displays the menu options
            try:
                opcion = int(input("Which option do you choose?: "))  # Prompts the user to choose an option
                if opcion == 1:
                    self.agregar_producto()  # Calls the method to add a product
                elif opcion == 2:
                    self.vender_producto()  # Calls the method to sell a product
                elif opcion == 3:
                    self.mostrar_producto()  # Calls the method to display product information
                elif opcion == 4:
                    self.valor_inventario()  # Calls the method to calculate the total inventory value
                elif opcion == 5:
                    print("Exiting the program...")  # Exit message
                    break  # Exits the loop and ends the program
                else:
                    print("Invalid option, try again.")  # Message for an invalid option
            except ValueError:
                print("Error: Please enter a valid number.")  # Error message if the input is not numeric
```
- Explanation:
    - A while True loop keeps the program running until the user chooses to exit.
    - The input() function captures the option chosen by the user.
    - A try-except block handles errors in case the user enters a non-numeric value.
    - Depending on the option entered, the corresponding method is called (such as agregar_producto(), vender_producto(), etc.).
    - If the user chooses option 5, the program prints "Exiting the program..." and terminates the loop.

#### *Method agregar_producto(): Add Products to Inventory*
```python
    def agregar_producto(self):
        """Adds a new product with a name, price, and quantity to the inventory."""
        nombre = input("Product name: ").lower()  # Prompts for the product name and converts it to lowercase
        if nombre in self.productos:
            print(f"The product '{nombre}' already exists. Use the sell option to reduce stock.")
            return  # If the product already exists, show a message and exit the method
        try:
            precio = float(input("Product price: "))  # Prompts for the product price
            cantidad = int(input("Stock quantity: "))  # Prompts for the stock quantity
            if precio < 0 or cantidad < 0:
                print("Error: Price and quantity must be positive values.")
                return  # If price or quantity are negative, show an error message and exit the method
        except ValueError:
            print("Error: Please enter valid numeric values.")  # Error message if the input is not numeric
            return

        self.productos[nombre] = {"precio": precio, "stock": cantidad}  # Adds the product to the dictionary
        print(f"The product '{nombre}' was added successfully.")  # Confirmation message
```
- Explanation:
    - The method asks for the product name and converts it to lowercase to avoid treating "Apple" and "apple" as different products.
    - It checks if the product already exists in self.productos. If so, it displays a warning message and does not add it again.
    - It prompts for the price and quantity, converting them to float and int, respectively.
    - It validates that the values are positive; if not, it displays an error message.
    - If all inputs are valid, the product is stored in the dictionary with its name as the key and its price and stock as the associated values.
    - A confirmation message is displayed to inform that the product was added.

#### *Method vender_producto(): Reduce Stock When Selling Products*
```python
    def vender_producto(self):
        """Reduces the stock quantity when products are sold."""
        nombre = input("Product name to sell: ").lower()  # Prompts for the product name and converts it to lowercase
        if nombre not in self.productos:
            print(f"The product '{nombre}' does not exist in the inventory.")
            return  # If the product does not exist, show a message and exit the method
        try:
            cantidad = int(input("Quantity to sell: "))  # Prompts for the quantity to sell
            if cantidad <= 0:
                print("Error: Quantity must be greater than zero.")
                return  # If the quantity is less than or equal to zero, show an error message and exit the method
        except ValueError:
            print("Error: Please enter a valid integer.")
            return

        if cantidad > self.productos[nombre]['stock']:
            print(f"Not enough stock available. Only {self.productos[nombre]['stock']} units are available.")
        else:
            self.productos[nombre]['stock'] -= cantidad  # Reduces the stock quantity
            print(f"Sale completed. Remaining stock of '{nombre}': {self.productos[nombre]['stock']}")

            if self.productos[nombre]['stock'] == 0:
                print(f"The product '{nombre}' is out of stock and will be removed from the inventory.")
                del self.productos[nombre]  # Removes the product if its stock reaches 0
```
- Explanation:
    - The method prompts for the product name and converts it to lowercase to ensure consistency.
    - It checks if the product exists in the inventory; if not, a warning message is displayed and the process stops.
    - It then asks for the quantity to sell and converts it to an integer.
    - It validates the input by ensuring the quantity is greater than zero and handles any non-numeric input using a try-except block.
    - If there is sufficient stock, the quantity is subtracted from the inventory.
    - If the stock reaches 0, the product is removed from the dictionary, and a message is shown indicating that the product is out of stock.

#### *Method mostrar_producto(): Consult Product Information*
```python
    def mostrar_producto(self):
        """Displays the information of a specific product."""
        nombre = input("Product name to check: ").lower()  # Prompts for the product name and converts it to lowercase
        if nombre in self.productos:
            info = self.productos[nombre]
            disponibilidad = "Available" if info['stock'] > 0 else "Out of stock"
            print(f"Product: {nombre}, Price: ${info['precio']:.2f}, Stock: {info['stock']} ({disponibilidad})")
        else:
            print(f"The product '{nombre}' does not exist in the inventory.")
```
- Explanation:
    - The method asks for the product name and converts it to lowercase.
    - It checks if the product exists in the inventory.
        - If it does, it retrieves the product’s price and stock, and indicates whether the product is available or out of stock.
    - If the product is not found, a warning message is displayed.

#### *Method valor_inventario(): Calculate the Total Inventory Value*
```python
    def valor_inventario(self):
        """Calculates the total inventory value (price * quantity of each product)."""
        if not self.productos:
            print("The inventory is empty.")
            return

        total = sum(info["precio"] * info["stock"] for info in self.productos.values())  # Calculates the total inventory value
        print(f"Total inventory value: ${total:.2f}")  # Displays the total inventory value

# Execute program
supermercado = Supermercado()
```
- Explanation:
    - The method first checks if the inventory is empty; if so, it displays a message and stops the calculation.
    - It calculates the total value by summing the product of the price and stock for each product.
    - The result is printed on the screen, showing the total inventory value.
---
# Libraries Used

To solve the problems requested for Laboratory 1, only two libraries were used:

    - math library: to perform the product calculation without a for loop
    - random library: to generate random numbers

# References
- Lutz, M. (2013). Learning Python. O'Reilly Media.
- Van Rossum, G. (1991). Python programming language. Python Software Foundation.
- Python Software Foundation. (2023). Python documentation. https://docs.python.org/3/
- Booch, G. (1994). Object-Oriented Analysis and Design with Applications. Addison-Wesley.