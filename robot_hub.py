"""
robot_hub.py - Programa que corre DENTRO del hub SPIKE Prime (firmware Pybricks)
=================================================================================
Puertos:
  A -> rueda izquierda        E -> rueda derecha
  C -> sensor de color        D -> sensor de distancia
  F -> motor del gatillo

Protocolo (la PC manda paquetes de 4 bytes: 3 letras + 1 byte con un valor 0-100):
  ADL = adelante   ATR = atrás   IZQ = girar izquierda   DER = girar derecha
                   (valor = velocidad en %)
  PAR = detener (también cancela el modo línea)
  FUE = disparar   (valor = potencia en %)
  LIN = seguir línea negra (valor = velocidad en %)
  CAN = calibrar NEGRO con lo que el sensor ve ahora
  CAB = calibrar BLANCO con lo que el sensor ve ahora

El hub responde por la consola (una línea cada vez):
  LISTO     -> el programa ya arrancó
  D<numero> -> distancia en milímetros (2000 = no detecta nada)
  R<numero>  -> reflexión del sensor de color (0-100)
  CN<numero> -> valor de NEGRO calibrado
  CB<numero> -> valor de BLANCO calibrado
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch

from usys import stdin
from uselect import poll

hub = PrimeHub()
rueda_izq = Motor(Port.A, Direction.COUNTERCLOCKWISE)
rueda_der = Motor(Port.E, Direction.CLOCKWISE)

# Si el gatillo gira hacia el lado equivocado, cambia la dirección aquí.
gatillo = Motor(Port.F, Direction.COUNTERCLOCKWISE)

sensor_color = ColorSensor(Port.C)
sensor = UltrasonicSensor(Port.D)

VEL_RUEDAS_MAX = 900     
VEL_DISPARO_MAX = 900    
GRADOS_DISPARO = 360     
PERIODO_REPORTE_MS = 200

# --- Seguidor de línea ------------------------------------------------
# El robot sigue el BORDE de la línea negra: intenta mantener el sensor
# justo entre el negro y el blanco (la mitad entre ambos valores).
GANANCIA = 0.8    
SIGNO_GIRO = 1     
negro = 10         
blanco = 70
modo_linea = False
vel_linea = 30

entrada = poll()
entrada.register(stdin)
reloj = StopWatch()


def mover(signo_izq, signo_der, porcentaje):
    velocidad = VEL_RUEDAS_MAX * porcentaje / 100
    rueda_izq.run(signo_izq * velocidad)
    rueda_der.run(signo_der * velocidad)


def detener():
    rueda_izq.brake()
    rueda_der.brake()


def seguir_linea():
    reflejo = sensor_color.reflection()
    umbral = (negro + blanco) / 2
    mitad = max((blanco - negro) / 2, 1)
    error = max(-1, min(1, (reflejo - umbral) / mitad))   # entre -1 y 1
    base = VEL_RUEDAS_MAX * vel_linea / 100
    correccion = SIGNO_GIRO * GANANCIA * error * base
    rueda_izq.run(min(base + correccion, VEL_RUEDAS_MAX))
    rueda_der.run(min(base - correccion, VEL_RUEDAS_MAX))


def procesar(comando, valor):
    global modo_linea, vel_linea, negro, blanco

    if comando in (b"ADL", b"ATR", b"IZQ", b"DER", b"PAR"):
        modo_linea = False 

    if comando == b"ADL":
        mover(1, 1, valor)
    elif comando == b"ATR":
        mover(-1, -1, valor)
    elif comando == b"IZQ":
        mover(-1, 1, valor)
    elif comando == b"DER":
        mover(1, -1, valor)
    elif comando == b"PAR":
        detener()
    elif comando == b"LIN":
        vel_linea = max(valor, 10)
        modo_linea = True
    elif comando == b"CAN":
        negro = sensor_color.reflection()
        print("CN" + str(negro))
    elif comando == b"CAB":
        blanco = sensor_color.reflection()
        print("CB" + str(blanco))
    elif comando == b"FUE":
        potencia = max(valor, 10)
        velocidad = VEL_DISPARO_MAX * potencia / 100
        gatillo.run_angle(velocidad, GRADOS_DISPARO, wait=False)


print("LISTO")
print("CN" + str(negro))
print("CB" + str(blanco))
ultimo_envio = reloj.time()

while True:
    # 1) Atender todos los comandos que hayan llegado
    while entrada.poll(0):
        datos = stdin.buffer.read(4)
        if datos is None or len(datos) < 4:
            break
        procesar(bytes(datos[:3]), datos[3])

    # 2) Si está activo el modo línea, corregir el rumbo
    if modo_linea:
        seguir_linea()

    # 3) Reportar distancia y reflejo cada cierto tiempo
    if reloj.time() - ultimo_envio >= PERIODO_REPORTE_MS:
        print("D" + str(sensor.distance()))
        print("R" + str(sensor_color.reflection()))
        ultimo_envio = reloj.time()

    wait(10)
