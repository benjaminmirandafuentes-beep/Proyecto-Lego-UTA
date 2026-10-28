
MODELO
  La pelota sale de la boca del cañón con velocidad v y ángulo θ, desde una altura y0
  y una posición horizontal x0 (medida desde el SENSOR de distancia):

      x0 = PIVOTE_ADELANTE + LARGO·cosθ            y0 = ALTURA_PIVOTE + LARGO·sinθ

  Sin aire, el tiempo de vuelo sale de  y0 + v·sinθ·t - g·t²/2 = 0  y el alcance es:

      t = ( v·sinθ + sqrt( v²·sin²θ + 2·g·y0 ) ) / g
      alcance = x0 + v·cosθ·t

  Casos particulares: con y0 = 0 queda R = v²·sen(2θ)/g, y con θ = 0 (disparo
  horizontal) queda la caída libre: R = v·sqrt(2·y0/g).
USO
  alcance(θ)        -> a qué distancia caería la pelota con ese ángulo
  angulo_para(D)    -> qué ángulo hace falta para caer a D cm (la ecuación se
                       resuelve numéricamente; hay una trayectoria baja y una alta)
  evaluar(D, θ)     -> ¿con el ángulo actual hay que acercarse, alejarse o disparar?

CONVENCIÓN DE MEDIDA (importante): el alcance se mide desde el SENSOR hasta el
punto donde la pelota toca el piso por PRIMERA vez, en la dirección del disparo.


import json
import math
import os

ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion_disparos.json")

# ----------------------------------------------------------------------
# GEOMETRÍA DEL ROBOT: mide estos valores con una regla y edítalos aquí
# (los de abajo son solo ejemplos para que el programa corra).
# ----------------------------------------------------------------------
ALTURA_PIVOTE_CM = 15.0     # altura del eje de giro del cañón sobre el piso
LARGO_CANON_CM = 20.0       # del eje de giro a la boca del cañón (por donde sale la pelota)
PIVOTE_ADELANTE_CM = 0.0    # cuánto está el eje DELANTE (+) o DETRÁS (-) del sensor de distancia
ANGULO_MIN_DEG = 15.0       # límites mecánicos del cañón (no pedirle más que esto al motor)
ANGULO_MAX_DEG = 70.0
# ----------------------------------------------------------------------

POTENCIA_DISPARO = 30       # % fijo del motor del gatillo
TOLERANCIA_MIN_CM = 5.0     # el sensor tiene ~1 cm de resolución; menos que esto no tiene sentido
G = 981.0                   # gravedad en cm/s²



def punto_de_salida(angulo_deg):
    """(x0, y0) de la boca del cañón: x desde el sensor, y desde el piso (cm)."""
    t = math.radians(angulo_deg)
    return (PIVOTE_ADELANTE_CM + LARGO_CANON_CM * math.cos(t),
            max(ALTURA_PIVOTE_CM + LARGO_CANON_CM * math.sin(t), 0.0))


def alcance_con(v, angulo_deg):
    """Alcance (cm desde el sensor) de un disparo con velocidad v (cm/s) y ese ángulo."""
    x0, y0 = punto_de_salida(angulo_deg)
    t = math.radians(angulo_deg)
    vy = v * math.sin(t)
    tiempo = (vy + math.sqrt(vy ** 2 + 2 * G * y0)) / G
    return x0 + v * math.cos(t) * tiempo


def velocidad_para(alcance_cm, angulo_deg):
    """Velocidad de salida que produce ese alcance con ese ángulo (None si es imposible).
    El alcance crece con v, así que se resuelve por bisección."""
    bajo, alto = 1.0, 5000.0
    if alcance_con(bajo, angulo_deg) > alcance_cm or alcance_con(alto, angulo_deg) < alcance_cm:
        return None
    for _ in range(60):
        medio = (bajo + alto) / 2
        if alcance_con(medio, angulo_deg) < alcance_cm:
            bajo = medio
        else:
            alto = medio
    return (bajo + alto) / 2


def _interpolar(xs, ys, x):
    """Interpolación lineal por tramos (con extrapolación en los extremos). xs ascendente."""
    i = 1
    while i < len(xs) - 1 and x > xs[i]:
        i += 1
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1])


# ----------------------------------------------------------------------
# Modelo con los datos de calibración
# ----------------------------------------------------------------------
class ModeloDisparo:
    def __init__(self, archivo=ARCHIVO):
        self.archivo = archivo          # None = no guardar en disco (útil para pruebas)
        self.disparos = []              # [(ángulo °, alcance cm), ...]
        self.puntos_angulo = []         # [(grados del motor, ángulo real del cañón °), ...]
        self.v0 = None                  # velocidad de salida efectiva (cm/s)
        self.error_medio_cm = None
        self._cargar()
        self._ajustar()

    # -- datos ----------------------------------------------------------
    def _cargar(self):
        if not (self.archivo and os.path.exists(self.archivo)):
            return
        try:
            with open(self.archivo, encoding="utf-8") as f:
                datos = json.load(f)
            if isinstance(datos, list):
                datos = {"disparos": datos}
            angulo_previo = float(datos.get("angulo_deg", 45.0))
            if "version" in datos:                         
                self.disparos = [(float(a), float(r)) for a, r in datos.get("disparos", [])
                                 if float(r) > 0]
                self.puntos_angulo = [(float(m), float(a)) for m, a in datos.get("puntos_angulo", [])]
            elif "alcances" in datos:                      
                self.disparos = [(angulo_previo, float(r)) for r in datos["alcances"] if float(r) > 0]
            else:                                         
                self.disparos = [(angulo_previo, float(par[1])) for par in datos.get("disparos", [])
                                 if float(par[1]) > 0]
        except (OSError, ValueError, TypeError, AttributeError, IndexError, KeyError):
            self.disparos, self.puntos_angulo = [], []

    def _guardar(self):
        if self.archivo:
            with open(self.archivo, "w", encoding="utf-8") as f:
                json.dump({"version": 3, "disparos": self.disparos,
                           "puntos_angulo": self.puntos_angulo}, f)

    def agregar(self, angulo_deg, alcance_cm):
        """Registra un disparo"""
        if not 0 < angulo_deg < 90:
            raise ValueError("El ángulo debe estar entre 0 y 90 grados.")
        if alcance_cm <= 0:
            raise ValueError("El alcance debe ser mayor que 0.")
        self.disparos.append((float(angulo_deg), float(alcance_cm)))
        self._guardar()
        self._ajustar()

    def borrar(self):
        self.disparos = []
        self._guardar()
        self._ajustar()

    # -- ajuste de la velocidad de salida ----------------------------------
    def _ajustar(self):
        self.v0 = self.error_medio_cm = None
        velocidades = [v for v in (velocidad_para(r, a) for a, r in self.disparos) if v is not None]
        if not velocidades:
            return
        self.v0 = sum(velocidades) / len(velocidades)
        errores = [abs(r - alcance_con(self.v0, a)) for a, r in self.disparos]
        self.error_medio_cm = sum(errores) / len(errores)

    @property
    def n(self):
        return len(self.disparos)

    @property
    def tolerancia(self):
        """Zona de acierto: lo que se puede garantizar según qué tan bien explica el modelo los disparos."""
        return max(TOLERANCIA_MIN_CM, self.error_medio_cm or 0.0)

    # -- predicciones -----------------------------------------------------
    def alcance(self, angulo_deg):
        """Alcance (cm) que se espera con ese ángulo, o None si aún no hay datos."""
        return None if self.v0 is None else alcance_con(self.v0, angulo_deg)

    def rango_alcance(self):
        """(mínimo, máximo) de alcance con el cañón entre ANGULO_MIN y ANGULO_MAX."""
        if self.v0 is None:
            return None
        valores = [alcance_con(self.v0, ANGULO_MIN_DEG + i * 0.5)
                   for i in range(int((ANGULO_MAX_DEG - ANGULO_MIN_DEG) / 0.5) + 1)]
        return min(valores), max(valores)

    def angulo_para(self, distancia_cm, trayectoria="baja"):
        """
        Ángulo (°) con el que la pelota cae a 'distancia_cm' del sensor.
        trayectoria: "baja" (más tensa y rápida) o "alta" (parabólica). Si hay una sola solución, esa.
        Devuelve (ángulo, None) o (None, motivo).
        """
        if self.v0 is None:
            return None, "Faltan datos: dispara y guarda al menos 1 disparo (con su ángulo)."
        paso = 0.1
        n = int(round((ANGULO_MAX_DEG - ANGULO_MIN_DEG) / paso))
        angulos = [ANGULO_MIN_DEG + i * paso for i in range(n + 1)]
        alcances = [alcance_con(self.v0, a) for a in angulos]
        raices = []
        for i in range(n):
            f0, f1 = alcances[i] - distancia_cm, alcances[i + 1] - distancia_cm
            if f0 == 0:
                raices.append(angulos[i])
            elif f0 * f1 < 0:
                raices.append(angulos[i] + paso * abs(f0) / (abs(f0) + abs(f1)))
        if raices:
            return round(raices[0] if trayectoria == "baja" else raices[-1], 1), None
        minimo, maximo = min(alcances), max(alcances)
        if distancia_cm > maximo:
            return None, (f"Fuera de alcance: lo máximo es ~{maximo:.0f} cm. "
                          f"Acércate {distancia_cm - maximo:.0f} cm.")
        return None, (f"Muy cerca: lo mínimo es ~{minimo:.0f} cm. "
                      f"Aléjate {minimo - distancia_cm:.0f} cm.")

    def evaluar(self, distancia_cm, angulo_deg):
        """
        Con el ángulo actual, ¿hay que acercarse, alejarse o disparar?
        Devuelve (estado, diferencia_cm, mensaje) con estado en 'sin_datos'|'lejos'|'listo'|'cerca'.
        diferencia = distancia medida - alcance previsto (positiva = hay que acercarse).
        """
        prevision = self.alcance(angulo_deg)
        if prevision is None:
            return "sin_datos", None, "Calibra: dispara y anota el ángulo y dónde cae la pelota."
        dif = distancia_cm - prevision
        if abs(dif) <= self.tolerancia:
            return "listo", dif, "¡DISPARA AHORA!"
        if dif > 0:
            return "lejos", dif, f"Muy lejos para este ángulo: acércate {dif:.0f} cm"
        return "cerca", dif, f"Muy cerca para este ángulo: aléjate {-dif:.0f} cm"

    # -- motor del ángulo: grados del motor <-> ángulo real del cañón --------
    def agregar_punto_angulo(self, motor_deg, angulo_deg):
        """Asocia una posición del motor con el ángulo real del cañón (medido con un celular)."""
        if not 0 < angulo_deg < 90:
            raise ValueError("El ángulo debe estar entre 0 y 90 grados.")
        self.puntos_angulo.append((float(motor_deg), float(angulo_deg)))
        self._guardar()

    def borrar_puntos_angulo(self):
        self.puntos_angulo = []
        self._guardar()

    def _pares_mapa(self):
        """Puntos ordenados por posición del motor; los repetidos se promedian."""
        grupos = {}
        for m, a in self.puntos_angulo:
            grupos.setdefault(round(m), []).append(a)
        return sorted((m, sum(v) / len(v)) for m, v in grupos.items())

    @property
    def mapa_listo(self):
        """Hay al menos 2 puntos distintos y el ángulo crece (o decrece) siempre con el motor."""
        pares = self._pares_mapa()
        if len(pares) < 2:
            return False
        angulos = [a for _, a in pares]
        subiendo = all(b > a for a, b in zip(angulos, angulos[1:]))
        bajando = all(b < a for a, b in zip(angulos, angulos[1:]))
        return subiendo or bajando

    def angulo_calibrado(self, angulo_deg):
        """True si ese ángulo está dentro de lo que se midió al calibrar el motor (sin extrapolar)."""
        angulos = [a for _, a in self._pares_mapa()]
        return bool(angulos) and min(angulos) - 0.5 <= angulo_deg <= max(angulos) + 0.5

    def angulo_de_motor(self, motor_deg):
        pares = self._pares_mapa()
        return _interpolar([m for m, _ in pares], [a for _, a in pares], motor_deg)

    def motor_de_angulo(self, angulo_deg):
        pares = sorted(((a, m) for m, a in self._pares_mapa()))
        return _interpolar([a for a, _ in pares], [m for _, m in pares], angulo_deg)

    # -- texto ----------------------------------------------------------
    def resumen(self):
        lineas = []
        if self.v0 is None:
            lineas.append(f"Sin disparos guardados. Dispara (potencia fija {POTENCIA_DISPARO} %), "
                          "y anota el ángulo y dónde cae.")
        else:
            lineas.append(f"Velocidad de salida estimada: {self.v0 / 100:.2f} m/s "
                          f"({self.n} disparo(s), error medio ±{self.error_medio_cm:.1f} cm)")
            minimo, maximo = self.rango_alcance()
            lineas.append(f"Con el cañón entre {ANGULO_MIN_DEG:g}° y {ANGULO_MAX_DEG:g}° "
                          f"llega de {minimo:.0f} a {maximo:.0f} cm")
            if self.n < 3:
                lineas.append("Guarda al menos 3 disparos con ángulos distintos.")
        if self.mapa_listo:
            lineas.append(f"Motor del ángulo calibrado ({len(self._pares_mapa())} puntos).")
        elif self.puntos_angulo:
            lineas.append("Motor del ángulo: los puntos no son coherentes, revísalos.")
        else:
            lineas.append("Motor del ángulo sin calibrar: guarda 2 puntos (ver instrucciones).")
        return "\n".join(lineas)


# ----------------------------------------------------------------------
# Prueba rápida:  python balistica.py   (robot simulado, no toca tus datos)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import random

    random.seed(5)
    modelo = ModeloDisparo(archivo=None)

    V_REAL = 430.0       # cm/s: velocidad "real" de salida del robot simulado
    ARRASTRE = 0.93      # el aire acorta el alcance ~7 %
    for angulo in (25, 35, 45, 55, 65):
        for _ in range(2):
            real = alcance_con(V_REAL, angulo) * ARRASTRE * random.uniform(0.96, 1.04)
            modelo.agregar(angulo, real)
    # motor simulado: 0° del motor = 15° del cañón, y 300° del motor = 65° del cañón
    modelo.agregar_punto_angulo(0, 15)
    modelo.agregar_punto_angulo(300, 65)

    print(modelo.resumen(), "\n")
    for distancia in (60, 100, 140, 180, 230, 20):
        angulo, aviso = modelo.angulo_para(distancia)
        if angulo is None:
            print(f"Objetivo a {distancia:>3} cm -> {aviso}")
        else:
            alta, _ = modelo.angulo_para(distancia, "alta")
            motor = modelo.motor_de_angulo(angulo)
            print(f"Objetivo a {distancia:>3} cm -> ángulo {angulo}° (alta: {alta}°), "
                  f"motor a {motor:.0f}°, predice {modelo.alcance(angulo):.0f} cm")
    print("\nCon el cañón a 38°:", modelo.evaluar(150, 38)[2], "| y a 150 cm exactos del alcance:",
          modelo.evaluar(modelo.alcance(38), 38)[2])
