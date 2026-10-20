"""
balistica.py - Potencia de disparo para acertar a una distancia dada
=====================================================================
Dos métodos para el mismo problema (el usuario elige cuál usar en el mando):

1) EMPÍRICO (ajuste de datos)
        alcance = a * potencia ^ b
   'a' y 'b' se ajustan con los disparos de calibración (mínimos cuadrados en
   escala logarítmica). No necesita conocer el ángulo ni la altura del cañón.

2) FÍSICO (tiro parabólico, que incluye la caída libre)
   La pelota sale con velocidad v, ángulo θ y altura h, y cae por la gravedad g:

        y(x) = h + x·tanθ - g·x² / (2·v²·cos²θ)

   Si debe tocar el piso (y = 0) a la distancia R, la velocidad necesaria es:

        v = sqrt( g·R² / ( 2·cos²θ·(h + R·tanθ) ) )

   Se supone que la velocidad de salida es proporcional a la potencia:

        v = k · potencia      ->      potencia = v / k

   Con θ y h medidos, la única incógnita es k, que sale de los disparos de
   calibración. Casos particulares: con h = 0 queda R = v²·sen(2θ)/g, y con
   θ = 0 (disparo horizontal) queda la caída libre: R = v·sqrt(2h/g).
   Este modelo NO incluye la resistencia del aire: se nota en su error.

CONVENCIÓN DE MEDIDA (importante): todas las distancias se miden desde el
SENSOR hasta el punto donde la pelota toca el piso por PRIMERA vez, en la
dirección del disparo. Así, la distancia que lee el sensor al apuntar al
objetivo es directamente el alcance que se necesita.

Los disparos de calibración (y el ángulo y la altura) se guardan en
calibracion_disparos.json, junto a este archivo.
"""

import json
import math
import os

ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion_disparos.json")
POT_MIN = 10    # mismos límites que el slider del mando
POT_MAX = 100
G = 981.0       # gravedad en cm/s²

METODOS = ("empirico", "fisico")
NOMBRES = {"empirico": "Empírico", "fisico": "Físico"}


class ModeloDisparo:
    def __init__(self, archivo=ARCHIVO):
        self.archivo = archivo      # None = no guardar en disco (útil para pruebas)
        self.disparos = []          # [(potencia %, alcance cm), ...]
        self.angulo_deg = 45.0      # ángulo del cañón sobre la horizontal
        self.altura_cm = 20.0       # altura de la boca del cañón sobre el piso
        self.a = self.b = self.error_empirico_cm = None
        self.k = self.error_fisico_cm = None
        self._cargar()
        self._ajustar()

    # -- datos ----------------------------------------------------------
    def _cargar(self):
        if not (self.archivo and os.path.exists(self.archivo)):
            return
        try:
            with open(self.archivo, encoding="utf-8") as f:
                datos = json.load(f)
            if isinstance(datos, list):      # formato anterior: solo la lista de disparos
                datos = {"disparos": datos}
            self.disparos = [(float(p), float(r)) for p, r in datos.get("disparos", [])]
            self.angulo_deg = float(datos.get("angulo_deg", self.angulo_deg))
            self.altura_cm = float(datos.get("altura_cm", self.altura_cm))
        except (OSError, ValueError, TypeError, AttributeError):
            self.disparos = []

    def _guardar(self):
        if self.archivo:
            with open(self.archivo, "w", encoding="utf-8") as f:
                json.dump({"disparos": self.disparos, "angulo_deg": self.angulo_deg,
                           "altura_cm": self.altura_cm}, f)

    def agregar(self, potencia, alcance_cm):
        """Registra un disparo de calibración: con esta potencia cayó a este alcance."""
        if potencia <= 0 or alcance_cm <= 0:
            raise ValueError("La potencia y el alcance deben ser mayores que 0.")
        self.disparos.append((float(potencia), float(alcance_cm)))
        self._guardar()
        self._ajustar()

    def configurar_fisica(self, angulo_deg, altura_cm):
        """Ángulo (0 a 90°) y altura de salida (cm) que usa el método físico."""
        if not 0 <= angulo_deg < 90:
            raise ValueError("El ángulo debe estar entre 0 y 90 grados.")
        if altura_cm < 0:
            raise ValueError("La altura no puede ser negativa.")
        self.angulo_deg, self.altura_cm = float(angulo_deg), float(altura_cm)
        self._guardar()
        self._ajustar()

    def borrar(self):
        self.disparos = []
        self._guardar()
        self._ajustar()

    # -- ajuste ---------------------------------------------------------
    def _ajustar(self):
        self._ajustar_empirico()
        self._ajustar_fisico()

    def _ajustar_empirico(self):
        self.a = self.b = self.error_empirico_cm = None
        if len({p for p, _ in self.disparos}) < 2:
            return                      # hacen falta al menos 2 potencias distintas
        xs = [math.log(p) for p, _ in self.disparos]
        ys = [math.log(r) for _, r in self.disparos]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        if b <= 0:
            return                      # sin sentido: más potencia no llega más lejos
        a = math.exp(my - b * mx)
        self.a, self.b = a, b
        self.error_empirico_cm = sum(abs(r - a * p ** b) for p, r in self.disparos) / n

    def _velocidad_necesaria(self, alcance_cm):
        """Velocidad de salida (cm/s) para caer a 'alcance_cm' (despeje de la parábola)."""
        t = math.radians(self.angulo_deg)
        den = 2 * math.cos(t) ** 2 * (self.altura_cm + alcance_cm * math.tan(t))
        if den <= 0:
            return None
        return math.sqrt(G * alcance_cm ** 2 / den)

    def _alcance_con_velocidad(self, v):
        """Alcance (cm) de un disparo con velocidad v (cm/s): resuelve h + vy·t - g·t²/2 = 0."""
        t = math.radians(self.angulo_deg)
        vy = v * math.sin(t)
        tiempo = (vy + math.sqrt(vy ** 2 + 2 * G * self.altura_cm)) / G
        return v * math.cos(t) * tiempo

    def _ajustar_fisico(self):
        self.k = self.error_fisico_cm = None
        pares = []
        for p, r in self.disparos:
            v = self._velocidad_necesaria(r)
            if v is not None:
                pares.append((p, v))
        if not pares:
            return
        # k por mínimos cuadrados para v = k·P (recta que pasa por el origen)
        k = sum(p * v for p, v in pares) / sum(p * p for p, _ in pares)
        if k <= 0:
            return
        self.k = k
        self.error_fisico_cm = sum(abs(r - self._alcance_con_velocidad(k * p))
                                   for p, r in self.disparos) / len(self.disparos)

    # -- estado ---------------------------------------------------------
    def listo(self, metodo="empirico"):
        return (self.k if metodo == "fisico" else self.a) is not None

    def _empirico_incoherente(self):
        """Hay 2 o más potencias distintas pero el ajuste no tiene sentido físico."""
        return len({p for p, _ in self.disparos}) >= 2 and not self.listo("empirico")

    # -- uso ------------------------------------------------------------
    def alcance(self, potencia, metodo="empirico"):
        """Alcance (cm) que el modelo predice para una potencia."""
        if metodo == "fisico":
            return self._alcance_con_velocidad(self.k * potencia)
        return self.a * potencia ** self.b

    def potencia_para(self, distancia_cm, metodo="empirico"):
        """
        Devuelve (potencia entera entre 10 y 100, aviso).
        Si no se puede calcular, devuelve (None, motivo).
        """
        if metodo not in METODOS:
            raise ValueError(f"Método desconocido: {metodo}")
        if distancia_cm <= 0:
            return None, "Distancia no válida."

        if metodo == "empirico":
            if self._empirico_incoherente():
                return None, ("Los datos no tienen sentido (más potencia no llega más lejos). "
                              "Revisa las mediciones o borra la calibración.")
            if not self.listo(metodo):
                return None, "Faltan datos: calibra con al menos 2 potencias distintas."
            p = (distancia_cm / self.a) ** (1 / self.b)
        else:
            if not self.disparos:
                return None, "Faltan datos: guarda al menos 1 disparo de calibración."
            v = self._velocidad_necesaria(distancia_cm)
            if not self.listo(metodo) or v is None:
                return None, "Con ese ángulo y esa altura el modelo no puede explicar los disparos."
            p = v / self.k
        return self._acotar(p, distancia_cm, metodo)

    def _acotar(self, p, distancia_cm, metodo):
        aviso = None
        if p > POT_MAX:
            aviso = (f"Fuera de alcance: con {POT_MAX} % llega a "
                     f"~{self.alcance(POT_MAX, metodo):.0f} cm.")
            p = POT_MAX
        elif p < POT_MIN:
            aviso = (f"Muy cerca: con {POT_MIN} % ya llega a "
                     f"~{self.alcance(POT_MIN, metodo):.0f} cm.")
            p = POT_MIN
        else:
            minimo = min(r for _, r in self.disparos)
            maximo = max(r for _, r in self.disparos)
            if distancia_cm < minimo or distancia_cm > maximo:
                aviso = (f"Fuera del rango calibrado ({minimo:.0f}-{maximo:.0f} cm): "
                         "es una estimación.")
        return int(round(p)), aviso

    def resumen(self, metodo=None):
        """Texto de estado. Sin argumento, muestra los dos métodos (útil para compararlos)."""
        if metodo is None:
            return self.resumen("empirico") + "\n" + self.resumen("fisico")
        n = len(self.disparos)
        nombre = NOMBRES[metodo]
        if n == 0:
            return f"{nombre}: sin disparos de calibración."
        if metodo == "empirico":
            if self._empirico_incoherente():
                return f"{nombre}: datos incoherentes, revisa las mediciones."
            if not self.listo(metodo):
                return f"{nombre}: {n} disparo(s); faltan potencias distintas."
            return (f"{nombre} ({n} disparos): alcance = {self.a:.3g}·P^{self.b:.2f}, "
                    f"error medio ±{self.error_empirico_cm:.1f} cm")
        if not self.listo(metodo):
            return f"{nombre}: revisa el ángulo y la altura."
        return (f"{nombre} ({n} disparos): v = {self.k:.2f}·P cm/s "
                f"(θ={self.angulo_deg:g}°, h={self.altura_cm:g} cm), "
                f"error medio ±{self.error_fisico_cm:.1f} cm")


# ----------------------------------------------------------------------
# Prueba rápida:  python balistica.py   (robot simulado, no toca tus datos)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import random

    random.seed(1)
    modelo = ModeloDisparo(archivo=None)
    modelo.configurar_fisica(angulo_deg=45, altura_cm=20)

    K_REAL = 4.6          # cm/s por cada 1 % de potencia
    ARRASTRE = 0.92       # el aire frena la pelota: alcanza ~8 % menos que en el vacío
    for potencia in (30, 50, 70, 90):
        for _ in range(3):
            ideal = modelo._alcance_con_velocidad(K_REAL * potencia)
            modelo.agregar(potencia, ideal * ARRASTRE * random.uniform(0.95, 1.05))

    print(modelo.resumen())
    print()
    for objetivo in (40, 80, 120, 160, 400):
        emp, aviso_e = modelo.potencia_para(objetivo, "empirico")
        fis, aviso_f = modelo.potencia_para(objetivo, "fisico")
        extra = f" | {aviso_f}" if aviso_f else ""
        print(f"Objetivo a {objetivo:>3} cm -> empírico {emp} %, físico {fis} %{extra}")
