from pybricks.pupdevices import Motor, ForceSensor
from pybricks.parameters import Port
import math

# Obtiene los dos motores.
motorRight = Motor(Port.D)
motorLeft = Motor(Port.C)

# Variable para tener siempre la misma velocidad.
motor_velocity = 200

# Cantidad de vueltas necesarias para avanzar y girar.
straight_turns = float(2*33.5/(math.pi*5.6))
turn_turns = float(0.25*11.3)/5.6

# Avanza y gira 4 veces, hace un cuadrado.
for i in range(4):
    # Acciona los motores para avanzar.
    # Utiliza el valor que la función `motor_velocity()` devuelta para la velocidad.
    motorLeft.run_angle(motor_velocity, float(-360*straight_turns), wait=False)
    motorRight.run_angle(motor_velocity, float(360*straight_turns), wait=True)
    # El wait se pone en el 1º False y en el 2º True para que avanza con ambos motores
    # y luego no continúe hasta terminar de avanzar.

    # Acciona los motores para girar.
    motorLeft.run_angle(motor_velocity/2, float(-360*turn_turns), wait=False)
    motorRight.run_angle(motor_velocity/2, float(-360*turn_turns), wait=True)
