from pybricks.pupdevices import Motor, ForceSensor
from pybricks.parameters import Port
# from decimal import Decimal, getcontext
import math

# getcontext().prec = 64
motorRight = Motor(Port.D)
motorLeft = Motor(Port.C)

motor_velocity = 200

straight_turns = float(2*33.5/(math.pi*5.6))
turn_turns = float(0.25*11.3)/5.6

for i in range(40):
    # Acciona el motor como antes.
    # Utiliza el valor que la función `motor_velocity()` devuelva para la velocidad.
    motorLeft.run_angle(motor_velocity, float(-360*straight_turns), wait=False)
    motorRight.run_angle(motor_velocity, float(360*straight_turns), wait=True)

    motorLeft.run_angle(motor_velocity/2, float(-360*turn_turns), wait=False)
    motorRight.run_angle(motor_velocity/2, float(-360*turn_turns), wait=True)
