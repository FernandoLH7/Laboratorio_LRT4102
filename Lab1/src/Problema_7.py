class Supermercado:
    def __init__(self):
        """Inicializa el supermercado con un diccionario para almacenar los productos y ejecuta el menú."""
        self.productos = {}
        self.ejecutar_menu()

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
            self.mostrar_menu()
            try:
                opcion = int(input("¿Cuál es la opción que eliges?: "))
                if opcion == 1:
                    self.agregar_producto()
                elif opcion == 2:
                    self.vender_producto()
                elif opcion == 3:
                    self.mostrar_producto()
                elif opcion == 4:
                    self.valor_inventario()
                elif opcion == 5:
                    print("Saliendo del programa...")
                    break
                else:
                    print("Opción no válida, intenta de nuevo.")
            except ValueError:
                print("Error: Ingresa un número válido.")

    def agregar_producto(self):
        """Añade un nuevo producto con nombre, precio y cantidad al inventario."""
        nombre = input("Nombre del producto: ").lower()
        if nombre in self.productos:
            print(f"El producto '{nombre}' ya existe. Usa la opción de venta para reducir stock.")
            return
        try:
            precio = float(input("Precio del producto: "))
            cantidad = int(input("Cantidad en stock: "))
            if precio < 0 or cantidad < 0:
                print("Error: Precio y cantidad deben ser valores positivos.")
                return
        except ValueError:
            print("Error: Ingresa valores numéricos válidos.")
            return

        self.productos[nombre] = {"precio": precio, "stock": cantidad}
        print(f"El producto '{nombre}' fue añadido correctamente.")

    def vender_producto(self):
        """Reduce la cantidad en stock cuando se venden productos."""
        nombre = input("Nombre del producto a vender: ").lower()
        if nombre not in self.productos:
            print(f"El producto '{nombre}' no existe en el inventario.")
            return
        try:
            cantidad = int(input("Cantidad a vender: "))
            if cantidad <= 0:
                print("Error: La cantidad debe ser mayor a cero.")
                return
        except ValueError:
            print("Error: Ingresa un número entero válido.")
            return

        if cantidad > self.productos[nombre]['stock']:
            print(f"No hay suficiente stock disponible. Solo hay {self.productos[nombre]['stock']} unidades.")
        else:
            self.productos[nombre]['stock'] -= cantidad
            print(f"Venta realizada. Stock restante de '{nombre}': {self.productos[nombre]['stock']}")

            if self.productos[nombre]['stock'] == 0:
                print(f"El producto '{nombre}' se ha agotado y será eliminado del inventario.")
                del self.productos[nombre]  # Elimina el producto si su stock llega a 0

    def mostrar_producto(self):
        """Muestra la información de un producto específico."""
        nombre = input("Nombre del producto a consultar: ").lower()
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

        total = sum(info["precio"] * info["stock"] for info in self.productos.values())
        print(f"Valor total del inventario: ${total:.2f}")

# Ejecutar el programa
supermercado = Supermercado()