"""

Puertos:
  A -> rueda izquierda        E -> rueda derecha
  B -> motor del ÁNGULO del cañón (el nuevo, sobre el hub)
  C -> sensor de color        D -> sensor de distancia
  F -> motor del gatillo

Protocolo: la PC manda paquetes de 5 bytes = 3 letras + un número entero de 2 bytes
(con signo, big-endian). Comandos:
  ADL / ATR / IZQ / DER  mover (valor = velocidad en %)
  PAR                    detener las ruedas (también cancela el modo línea)
  FUE                    disparar (valor = potencia en %)
  LIN                    seguir línea negra (valor = velocidad en %)
  CAN / CAB              calibrar NEGRO / BLANCO con lo que ve el sensor de color
  ASU / ABA              subir / bajar el cañón mientras se mantenga (valor = velocidad en %)
  APA                    parar el motor del ángulo y mantener la posición
  ANG                    ir a una posición del motor (valor = grados del motor)
  CER                    fijar la posición actual del motor del ángulo como 0

El hub responde por la consola (una línea cada vez):
  LISTO       el programa ya arrancó
  D<numero>   distancia en milímetros (2000 = no detecta nada)
  R<numero>   reflexión del sensor de color (0-100)
  CN<numero>  NEGRO calibrado         CB<numero>  BLANCO calibrado
  A<numero>   posición del motor del ángulo, en grados del motor (puede ser negativa)
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait, StopWatch

from usys import stdin
from uselect import poll

hub = PrimeHub()

# Si al presionar "Adelante" el robot RETROCEDE, intercambia CLOCKWISE y
# COUNTERCLOCKWISE en las dos líneas de las ruedas.
rueda_izq = Motor(Port.A, Direction.COUNTERCLOCKWISE)
rueda_der = Motor(Port.E, Direction.CLOCKWISE)

# Si el botón "Subir" BAJA el cañón, cambia la dirección aquí (la calibración
# del ángulo funciona igual, pero así los botones quedan al derecho).
motor_angulo = Motor(Port.B, Direction.CLOCKWISE)

# Si el gatillo gira hacia el lado equivocado, cambia la dirección aquí.
gatillo = Motor(Port.F, Direction.COUNTERCLOCKWISE)

sensor_color = ColorSensor(Port.C)
sensor = UltrasonicSensor(Port.D)

VEL_RUEDAS_MAX = 900     # grados/segundo cuando el slider está al 100 %
VEL_DISPARO_MAX = 900    # grados/segundo cuando la potencia está al 100 %
GRADOS_DISPARO = 360     # una rotación completa del gatillo
VEL_ANGULO_MAX = 200     # grados/segundo al subir/bajar el cañón con los botones (100 %)
VEL_ANGULO_AUTO = 150    # grados/segundo al ir solo a una posición (lento = más seguro)
PERIODO_REPORTE_MS = 200

# --- Seguidor de línea ------------------------------------------------

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
        modo_linea = False   # cualquier orden manual cancela el modo línea

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
        # wait=False: el hub sigue atendiendo comandos mientras gira el gatillo
        gatillo.run_angle(velocidad, GRADOS_DISPARO, wait=False)
    elif comando == b"ASU":
        motor_angulo.run(VEL_ANGULO_MAX * valor / 100)
    elif comando == b"ABA":
        motor_angulo.run(-VEL_ANGULO_MAX * valor / 100)
    elif comando == b"APA":
        motor_angulo.hold()      # se queda firme en la posición, para que el cañón no caiga
    elif comando == b"ANG":
        motor_angulo.run_target(VEL_ANGULO_AUTO, valor, then=Stop.HOLD, wait=False)
    elif comando == b"CER":
        motor_angulo.reset_angle(0)


print("LISTO")
print("CN" + str(negro))
print("CB" + str(blanco))
ultimo_envio = reloj.time()

while True:
    # 1) Atender todos los comandos que hayan llegado
    while entrada.poll(0):
        datos = stdin.buffer.read(5)
        if datos is None or len(datos) < 5:
            break
        valor = (datos[3] << 8) | datos[4]
        if valor > 32767:            # número con signo de 2 bytes
            valor -= 65536
        procesar(bytes(datos[:3]), valor)

    # 2) Si está activo el modo línea, corregir el rumbo
    if modo_linea:
        seguir_linea()

    # 3) Reportar los sensores cada cierto tiempo
    if reloj.time() - ultimo_envio >= PERIODO_REPORTE_MS:
        print("D" + str(sensor.distance()))
        print("R" + str(sensor_color.reflection()))
        print("A" + str(motor_angulo.angle()))
        ultimo_envio = reloj.time()

    wait(10)
