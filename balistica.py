"""
El problema pasa a ser de POSICIÓN: el robot debe quedar a una distancia R0 del
objetivo. Este módulo hace tres cosas:

 1) Estima R0 con los disparos de calibración (promedio) y cuánto varía de un
    disparo a otro (desviación estándar).
 2) Compara la distancia que mide el sensor con R0 y dice si hay que acercarse,
    alejarse o disparar. La zona de acierto es  R0 ± tolerancia, donde
    tolerancia = max(5 cm, desviación): si los disparos varían mucho, la zona es
    más ancha porque es lo mejor que se puede garantizar.

        y(x) = h + x·tanθ - g·x² / (2·v²·cos²θ)
        v = sqrt( g·R² / ( 2·cos²θ·(h + R·tanθ) ) )       (velocidad para llegar a R)

    Casos particulares: con h = 0 queda R = v²·sen(2θ)/g, y con θ = 0 (disparo
    horizontal) queda la caída libre: R = v·sqrt(2h/g). No incluye la resistencia
    del aire, que en una pelota de ping pong sí importa: la velocidad estimada es
    la "efectiva" (ya incluye ese efecto en promedio).

CONVENCIÓN DE MEDIDA (importante): el alcance se mide desde el SENSOR hasta el
punto donde la pelota toca el piso por PRIMERA vez, en la dirección del disparo.
Así, la distancia que lee el sensor al apuntar al objetivo se compara directamente
con el alcance.

Si cambias el ángulo del cañón, hay que borrar la calibración y volver a medir.
Los datos se guardan en calibracion_disparos.json, junto a este archivo.
"""

import json
import math
import os

ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion_disparos.json")

POTENCIA_DISPARO = 30       # % fijo del motor del gatillo
TOLERANCIA_MIN_CM = 5.0     # el sensor tiene ~1 cm de resolución; menos que esto no tiene sentido
G = 981.0                   # gravedad en cm/s²


class ModeloDisparo:
    def __init__(self, archivo=ARCHIVO):
        self.archivo = archivo      # None = no guardar en disco (útil para pruebas)
        self.alcances = []          # alcances medidos (cm), uno por disparo
        self.angulo_deg = 45.0      # ángulo del cañón sobre la horizontal
        self.altura_cm = 20.0       # altura de la boca del cañón sobre el piso
        self._cargar()

    # -- datos ----------------------------------------------------------
    def _cargar(self):
        if not (self.archivo and os.path.exists(self.archivo)):
            return
        try:
            with open(self.archivo, encoding="utf-8") as f:
                datos = json.load(f)
            if isinstance(datos, list):          # formato más antiguo: solo una lista
                datos = {"disparos": datos}
            brutos = datos.get("alcances")
            if brutos is None:                   # formato anterior: pares [potencia, alcance]
                brutos = [par[1] for par in datos.get("disparos", [])]
            self.alcances = [float(r) for r in brutos if float(r) > 0]
            self.angulo_deg = float(datos.get("angulo_deg", self.angulo_deg))
            self.altura_cm = float(datos.get("altura_cm", self.altura_cm))
        except (OSError, ValueError, TypeError, AttributeError, IndexError, KeyError):
            self.alcances = []

    def _guardar(self):
        if self.archivo:
            with open(self.archivo, "w", encoding="utf-8") as f:
                json.dump({"alcances": self.alcances, "angulo_deg": self.angulo_deg,
                           "altura_cm": self.altura_cm}, f)

    def agregar(self, alcance_cm):
        """Registra dónde cayó un disparo (cm desde el sensor)."""
        if alcance_cm <= 0:
            raise ValueError("El alcance debe ser mayor que 0.")
        self.alcances.append(float(alcance_cm))
        self._guardar()

    def borrar(self):
        self.alcances = []
        self._guardar()

    def configurar_fisica(self, angulo_deg, altura_cm):
        """Ángulo (0 a 90°) y altura de salida (cm), usados solo para la estimación física."""
        if not 0 <= angulo_deg < 90:
            raise ValueError("El ángulo debe estar entre 0 y 90 grados.")
        if altura_cm < 0:
            raise ValueError("La altura no puede ser negativa.")
        self.angulo_deg, self.altura_cm = float(angulo_deg), float(altura_cm)
        self._guardar()

    # -- estadística ----------------------------------------------------
    @property
    def n(self):
        return len(self.alcances)

    @property
    def alcance_estimado(self):
        """R0: promedio de los alcances medidos, o None si no hay datos."""
        return sum(self.alcances) / self.n if self.n else None

    @property
    def desviacion(self):
        """Cuánto varía el alcance entre disparos (desviación estándar muestral)."""
        if self.n < 2:
            return 0.0
        m = self.alcance_estimado
        return math.sqrt(sum((r - m) ** 2 for r in self.alcances) / (self.n - 1))

    @property
    def tolerancia(self):
        return max(TOLERANCIA_MIN_CM, self.desviacion)

    # -- cuándo disparar -----------------------------------------------
    def evaluar(self, distancia_cm):
        """
        Compara la distancia al objetivo con el alcance del cañón.
        Devuelve (estado, diferencia_cm, mensaje) con estado en:
          'sin_datos' | 'lejos' | 'listo' | 'cerca'
        diferencia = distancia medida - alcance (positiva = hay que acercarse).
        """
        r0 = self.alcance_estimado
        if r0 is None:
            return "sin_datos", None, "Calibra: dispara y anota dónde cae la pelota."
        dif = distancia_cm - r0
        if abs(dif) <= self.tolerancia:
            return "listo", dif, "¡DISPARA AHORA!"
        if dif > 0:
            return "lejos", dif, f"Muy lejos: acércate {dif:.0f} cm"
        return "cerca", dif, f"Muy cerca: aléjate {-dif:.0f} cm"

    # -- física (tiro parabólico) ----------------------------------------
    def _velocidad_necesaria(self, alcance_cm, angulo_deg=None):
        t = math.radians(self.angulo_deg if angulo_deg is None else angulo_deg)
        den = 2 * math.cos(t) ** 2 * (self.altura_cm + alcance_cm * math.tan(t))
        if den <= 0:
            return None
        return math.sqrt(G * alcance_cm ** 2 / den)

    def _alcance_con_velocidad(self, v, angulo_deg=None):
        t = math.radians(self.angulo_deg if angulo_deg is None else angulo_deg)
        vy = v * math.sin(t)
        tiempo = (vy + math.sqrt(vy ** 2 + 2 * G * self.altura_cm)) / G
        return v * math.cos(t) * tiempo

    def velocidad_salida(self):
        """Velocidad de salida de la pelota (cm/s) que explica el alcance medido, o None."""
        r0 = self.alcance_estimado
        return None if r0 is None else self._velocidad_necesaria(r0)

    def alcance_con_angulo(self, angulo_deg):
        """Alcance (cm) que tendría la pelota con otro ángulo y la misma velocidad de salida."""
        v = self.velocidad_salida()
        return None if v is None else self._alcance_con_velocidad(v, angulo_deg)

    # -- texto ----------------------------------------------------------
    def resumen(self):
        r0 = self.alcance_estimado
        if r0 is None:
            return (f"Sin disparos guardados. Dispara (potencia fija {POTENCIA_DISPARO} %) "
                    "y anota dónde cae.")
        texto = f"Alcance estimado: {r0:.0f} cm (zona de acierto ±{self.tolerancia:.0f} cm), {self.n} disparo(s)"
        if self.n < 3:
            texto += "\nGuarda al menos 3 a 5 disparos para que sea confiable."
        v = self.velocidad_salida()
        if v is not None:
            texto += (f"\nVelocidad de salida estimada: {v / 100:.1f} m/s "
                      f"(θ={self.angulo_deg:g}°, h={self.altura_cm:g} cm)")
        return texto
            
if __name__ == "__main__":
    import random

    random.seed(3)
    modelo = ModeloDisparo(archivo=None)
    modelo.configurar_fisica(angulo_deg=45, altura_cm=20)

    V_REAL = 420.0     # cm/s: velocidad "real" de salida del robot simulado
    for _ in range(6):
        ideal = modelo._alcance_con_velocidad(V_REAL)
        modelo.agregar(ideal * 0.92 * random.uniform(0.96, 1.04))   # el aire acorta ~8 %

    print(modelo.resumen())
    print()
    r0 = modelo.alcance_estimado
    for distancia in (r0 + 60, r0 + 12, r0 + 2, r0 - 3, r0 - 25):
        estado, _, mensaje = modelo.evaluar(distancia)
        print(f"Sensor mide {distancia:>5.0f} cm -> {estado:<5} | {mensaje}")
    print("\nSi más adelante el cañón cambia de ángulo (misma velocidad de salida):")
    for angulo in (30, 40, 45, 55, 65):
        print(f"  θ = {angulo}° -> alcance ~{modelo.alcance_con_angulo(angulo):.0f} cm")
