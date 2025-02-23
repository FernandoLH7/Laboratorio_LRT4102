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