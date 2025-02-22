class Numer:
    def __init__(self):
        self.horas = int(input("Ingresa el número de horas que has trabajado: "))
        self.costo = int(input("Ingrese el costo de cada hora trabajada: "))

    def calcular(self):
        salario = self.horas * self.costo
        print ("El salario que te corresponde es de: ", salario,"$")

paga = Numer()
paga.calcular()