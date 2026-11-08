from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
import math

# Obtiene el sensor de orientación integrado en el hub
hub = PrimeHub()

# Montaje en espejo: invertimos el izquierdo para que positivo = avanzar en ambos
motorRight = Motor(Port.D)
motorLeft = Motor(Port.C, Direction.COUNTERCLOCKWISE)

# --- Parámetros (cm) ---
WHEEL_DIAMETER = 5.6    # rueda SPIKE Prime de 56 mm
LADO = 2 * 33.5                      # 67 cm
GRADOS_LADO = LADO * 360 / (math.pi * WHEEL_DIAMETER)   # grados de motor por lado

# --- Velocidades (°/s de motor) ---
V_RECTA = 200
V_MIN_RECTA = 60
V_GIRO_MAX = 150
V_GIRO_MIN = 40

# --- Ganancias del control proporcional (ajustables) ---
KP_RECTA = 6        # corrección de rumbo en recta
KP_GIRO = 8         # velocidad de giro según el error angular
TOLERANCIA = 0.7    # grados de error aceptados al terminar un giro

# Función para detener ambos motores
def parar():
    motorLeft.hold()
    motorRight.hold()

# Función para avanzar un lado en línea recta corrigiendo el rumbo con el IMU
def recta(rumbo_objetivo):
    motorLeft.reset_angle(0)
    motorRight.reset_angle(0)

    while True:
        recorrido = (motorLeft.angle() + motorRight.angle()) / 2
        restante = GRADOS_LADO - recorrido
        if restante <= 0:
            break

        # Frena de forma suave al acercarse al final
        velocidad = max(V_MIN_RECTA, min(V_RECTA, restante * 3))

        # heading() crece en sentido horario. Si el robot se desvía a la derecha,
        # el error es negativo: frenamos la rueda izquierda y aceleramos la derecha
        error = rumbo_objetivo - hub.imu.heading()
        correccion = KP_RECTA * error

        motorLeft.run(velocidad + correccion)
        motorRight.run(velocidad - correccion)
        wait(10)

    parar()

# Gira sobre su propio eje hasta alcanzar el rumbo absoluto indicado por el IMU
def girar_a(rumbo_objetivo):
    estable = 0
    while estable < 5:                       # ~50 ms seguidos dentro de tolerancia
        error = rumbo_objetivo - hub.imu.heading()

        if abs(error) <= TOLERANCIA:
            estable += 1
            parar()
        else:
            estable = 0
            v = max(V_GIRO_MIN, min(V_GIRO_MAX, KP_GIRO * abs(error)))
            v = v if error > 0 else -v
            motorLeft.run(v)                 # sentido horario: izquierda avanza
            motorRight.run(-v)               # derecha retrocede
        wait(10)

    parar()


# Calibración del giroscopio: espera a que el hub esté listo y completamente quieto
while not hub.imu.ready() or not hub.imu.stationary():
    wait(50)
hub.imu.reset_heading(0)

for i in range(4):
    rumbo = 90 * i               # rumbo absoluto del lado actual
    recta(rumbo)
    girar_a(rumbo + 90)          # giro hacia el rumbo del siguiente lado
    wait(100)                    # pequeña pausa para que el robot se estabilice

parar()
print("Error final de rumbo:", hub.imu.heading() - 90 * 4, "grados")
