"""

Antes de conectar:
  * Hub encendido, con la luz del botón central parpadeando en azul.
  
Arquitectura (cliente - servidor):
  PC (cliente)  --Bluetooth-->  hub (servidor): recibe órdenes y mueve motores
  PC (cliente)  <--Bluetooth--  hub (servidor): informa distancia y reflexión de color
  El seguidor de línea se calcula DENTRO del hub (rápido, sin depender del Bluetooth).

Controles de teclado: W A S D o flechas = mover (mantener presionado), Espacio = disparar.
"""

import asyncio
import os
import queue
import random
import statistics
import sys
import threading
import tkinter as tk
from tkinter import messagebox

from balistica import ModeloDisparo, POTENCIA_DISPARO   # física del disparo (mismo folder)


NOMBRE_HUB = "fldsmdfr"

RUTA_PROGRAMA_HUB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_hub.py")

VEL_DISPARO_MAX = 900   
DIST_MAX_CM = 200       
BARRA_ANCHO = 320       
DIST_ALERTA_CM = 20     
SIN_OBJETO_MM = 2000    

# Colores
FUENTE = "Segoe UI"
BG = "#EDEFF0"
TEXTO = "#1F2D33"
GRIS = "#66777F"
VERDE = "#2E8B57"
ROJO = "#C62828"
AZUL = "#1B7F8C"
AZUL_OSC = "#12606A"
NARANJA = "#F28C1E"
NARANJA_OSC = "#C96F0B"
MORADO = "#6A4C93"
MORADO_OSC = "#523775"


class ConexionHub:
    """Habla con el hub por Bluetooth usando pybricksdev, en un hilo aparte."""

    def __init__(self, eventos):
        self.eventos = eventos          # cola hacia la ventana
        self.comandos = queue.Queue()   # cola desde la ventana
        self._buffer = b""
        self._hilo = threading.Thread(target=self._correr, daemon=True)

    def iniciar(self):
        self._hilo.start()

    def enviar(self, paquete):
        self.comandos.put(paquete)

    def cerrar(self):
        self.comandos.put(None)

    def _evento(self, tipo, dato=None):
        self.eventos.put((tipo, dato))

    def _correr(self):
        asyncio.run(self._principal())

    def _al_recibir(self, datos):
        """Llega texto del hub; se junta y se corta por líneas."""
        self._buffer += bytes(datos)
        while b"\n" in self._buffer:
            linea, self._buffer = self._buffer.split(b"\n", 1)
            linea = linea.strip().decode(errors="ignore")
            try:
                if linea == "LISTO":
                    self._evento("listo", self._nombre)
                elif linea.startswith("CN"):
                    self._evento("calibracion", ("negro", int(linea[2:])))
                elif linea.startswith("CB"):
                    self._evento("calibracion", ("blanco", int(linea[2:])))
                elif linea.startswith("D"):
                    self._evento("distancia", int(linea[1:]))
                elif linea.startswith("R"):
                    self._evento("reflexion", int(linea[1:]))
                elif linea.startswith("A"):
                    self._evento("angulo_motor", int(linea[1:]))
            except ValueError:
                pass

    async def _principal(self):
        from pybricksdev.ble import find_device
        from pybricksdev.connections.pybricks import PybricksHubBLE

        hub = None
        self._nombre = "hub"
        try:
            self._evento("estado", "Buscando el hub (luz azul parpadeando)...")
            dispositivo = await find_device(NOMBRE_HUB, timeout=15)
            self._nombre = dispositivo.name or "hub"

            self._evento("estado", f"Conectando con {self._nombre}...")
            hub = PybricksHubBLE(dispositivo)
            await hub.connect()
            hub.stdout_observable.subscribe(self._al_recibir)

            self._evento("estado", "Cargando el programa en el hub...")
            await hub.run(RUTA_PROGRAMA_HUB, wait=False, print_output=False, line_handler=False)

            while True:
                try:
                    paquete = self.comandos.get_nowait()
                except queue.Empty:
                    await asyncio.sleep(0.02)
                    continue
                if paquete is None:
                    break
                await hub.write(paquete)
                await asyncio.sleep(0.03)   # evita saturar el Bluetooth

        except asyncio.TimeoutError:
            self._evento("error", "No se encontró el hub. ¿Está encendido con luz azul parpadeando?")
        except Exception as e:
            self._evento("error", f"Error de conexión: {e}")
        finally:
            if hub is not None:
                try:
                    await hub.write(b"PAR\x00\x00")
                    await hub.stop_user_program()
                except Exception:
                    pass
                try:
                    await hub.disconnect()
                except Exception:
                    pass
            self._evento("cerrado")


class ConexionDemo:
    """Imita al hub para probar la ventana sin robot."""

    def __init__(self, eventos):
        self.eventos = eventos
        self.movimiento = "PAR"
        self.distancia_mm = 900
        self.reflexion = 40
        self.motor_angulo = 0.0       # posición simulada del motor del ángulo (grados del motor)
        self.motor_objetivo = None    # destino de un comando ANG
        self.motor_giro = 0           # +1 / -1 mientras se mantiene Subir / Bajar
        self.activo = True

    def iniciar(self):
        threading.Thread(target=self._bucle, daemon=True).start()

    def enviar(self, paquete):
        comando = paquete[:3].decode()
        valor = int.from_bytes(paquete[3:5], "big", signed=True)
        print(f"[DEMO] {comando} valor={valor}")
        if comando in ("ADL", "ATR", "PAR"):
            self.movimiento = comando
        elif comando == "CAN":
            self.eventos.put(("calibracion", ("negro", self.reflexion)))
        elif comando == "CAB":
            self.eventos.put(("calibracion", ("blanco", self.reflexion)))
        elif comando in ("ASU", "ABA"):
            self.motor_giro, self.motor_objetivo = (1 if comando == "ASU" else -1), None
        elif comando == "APA":
            self.motor_giro, self.motor_objetivo = 0, None
        elif comando == "ANG":
            self.motor_objetivo, self.motor_giro = valor, 0
        elif comando == "CER":
            self.motor_angulo, self.motor_objetivo, self.motor_giro = 0.0, None, 0

    def cerrar(self):
        self.activo = False

    def _bucle(self):
        import time
        self.eventos.put(("estado", "Modo demo: conectando..."))
        time.sleep(1.0)
        self.eventos.put(("listo", "demo"))
        while self.activo:
            if self.movimiento == "ADL":
                self.distancia_mm -= 60
            elif self.movimiento == "ATR":
                self.distancia_mm += 60
            self.distancia_mm = max(40, min(self.distancia_mm + random.randint(-3, 3), 2100))
            valor = SIN_OBJETO_MM if self.distancia_mm >= SIN_OBJETO_MM else self.distancia_mm
            self.eventos.put(("distancia", valor))
            self.reflexion = max(3, min(90, self.reflexion + random.randint(-12, 12)))
            self.eventos.put(("reflexion", self.reflexion))
            # motor del ángulo simulado: ~150 grados/segundo
            if self.motor_giro:
                self.motor_angulo += 30 * self.motor_giro
            elif self.motor_objetivo is not None:
                resto = self.motor_objetivo - self.motor_angulo
                self.motor_angulo += max(-30, min(30, resto))
            self.eventos.put(("angulo_motor", round(self.motor_angulo)))
            time.sleep(0.2)
        self.eventos.put(("cerrado", None))


# ======================================================================
# Ventana
# ======================================================================
TECLAS = {
    "w": "ADL", "Up": "ADL",
    "s": "ATR", "Down": "ATR",
    "a": "IZQ", "Left": "IZQ",
    "d": "DER", "Right": "DER",
}


class Mando:
    def __init__(self, root, conexion, eventos):
        self.root = root
        self.conexion = conexion
        self.eventos = eventos

        self.velocidad = tk.IntVar(value=50)

        self.listo = False
        self.conexion_viva = True
        self.cerrando = False
        self.recargando = False
        self.mov_actual = "PAR"
        self.teclas_abajo = set()
        self._tarea_velocidad = None
        self.modo_linea = False
        self.reflexion = None
        self.negro = None
        self.blanco = None
        self.modelo = ModeloDisparo()
        self.dist_actual = None       # última distancia medida (cm) o None
        self.motor_deg = None         # posición del motor del ángulo (grados del motor)
        self._en_zona = False         # True cuando la distancia es la correcta para disparar
        self.distancias = []          # últimas lecturas del sensor, en cm

        self._construir()
        self._actualizar_guia()
        root.bind("<KeyPress>", self._tecla_abajo)
        root.bind("<KeyRelease>", self._tecla_arriba)
        root.bind("<Button-1>", self._clic_en_ventana)
        root.protocol("WM_DELETE_WINDOW", self.cerrar)

        self.conexion.iniciar()
        self._revisar_eventos()

    # -- construcción ---------------------------------------------------
    def _construir(self):
        r = self.root
        r.title("Control remoto - Cañón lanzador")
        r.configure(bg=BG)
        r.resizable(False, False)

        self.lbl_estado = tk.Label(r, text="Iniciando...", bg=BG, fg=GRIS, font=(FUENTE, 10))
        self.lbl_estado.grid(row=0, column=0, columnspan=3, pady=(12, 4))

        # Indicador de distancia
        caja = tk.Frame(r, bg=BG)
        caja.grid(row=1, column=0, columnspan=2, pady=(0, 10))
        tk.Label(caja, text="Distancia al objeto", bg=BG, fg=GRIS, font=(FUENTE, 10)).pack()
        self.lbl_dist = tk.Label(caja, text="-- cm", bg=BG, fg=TEXTO, font=(FUENTE, 32, "bold"))
        self.lbl_dist.pack()
        self.barra = tk.Canvas(caja, width=BARRA_ANCHO, height=16, bg="#CBD2D4",
                               highlightthickness=0)
        self.barra.pack(pady=(2, 0))
        # Dice si hay que acercarse, alejarse o disparar
        self.lbl_guia = tk.Label(caja, text="", bg=BG, fg=GRIS, font=(FUENTE, 15, "bold"))
        self.lbl_guia.pack(pady=(8, 0))

        # Cruceta de movimiento
        cruz = tk.Frame(r, bg=BG)
        cruz.grid(row=2, column=0, padx=24, pady=8)
        self._boton_mov(cruz, "▲", "ADL", 0, 1)
        self._boton_mov(cruz, "◀", "IZQ", 1, 0)
        self._boton_mov(cruz, "▶", "DER", 1, 2)
        self._boton_mov(cruz, "▼", "ATR", 2, 1)
        tk.Button(cruz, text="■", width=3, height=1, font=(FUENTE, 16), bg="#9AA5A8", fg="white",
                  activebackground="#7D888B", relief="flat", takefocus=0,
                  command=lambda: self.mover("PAR")).grid(row=1, column=1, padx=3, pady=3)

        # Botón de disparo
        self.btn_fuego = tk.Button(r, text="DISPARAR", width=11, height=4, font=(FUENTE, 14, "bold"),
                                   bg=NARANJA, fg="white", activebackground=NARANJA_OSC,
                                   activeforeground="white", relief="flat", takefocus=0,
                                   command=self.disparar)
        self.btn_fuego.grid(row=2, column=1, padx=24)

        # Seguidor de línea (automático)
        linea = tk.Frame(r, bg=BG)
        linea.grid(row=3, column=0, columnspan=2, padx=24, pady=(8, 0))
        self.btn_linea = tk.Button(linea, text="SEGUIR LÍNEA (AUTOMÁTICO)", width=30,
                                   font=(FUENTE, 11, "bold"), bg=MORADO, fg="white",
                                   activebackground=MORADO_OSC, activeforeground="white",
                                   relief="flat", pady=6, takefocus=0, command=self.alternar_linea)
        self.btn_linea.pack(fill="x")
        fila = tk.Frame(linea, bg=BG)
        fila.pack(pady=(6, 0))
        for texto, cmd in (("Calibrar NEGRO", "CAN"), ("Calibrar BLANCO", "CAB")):
            tk.Button(fila, text=texto, font=(FUENTE, 9), bg="#CBD2D4", fg=TEXTO,
                      activebackground="#B5BEC1", relief="flat", padx=10, takefocus=0,
                      command=lambda c=cmd: self.calibrar(c)).pack(side="left", padx=4)
        self.lbl_color = tk.Label(linea, text="Reflexión: --   Negro: --   Blanco: --",
                                  bg=BG, fg=GRIS, font=(FUENTE, 9))
        self.lbl_color.pack(pady=(4, 0))

        # Velocidad de avance
        ajustes = tk.Frame(r, bg=BG)
        ajustes.grid(row=4, column=0, columnspan=2, padx=24, pady=(6, 10))
        self._slider(ajustes, "Velocidad de avance", self.velocidad, self._velocidad_cambio)

        # Ángulo del cañón (motor de elevación)
        lado = tk.Frame(r, bg=BG)       # columna derecha
        lado.grid(row=1, column=2, rowspan=4, sticky="n", padx=(8, 24), pady=(0, 10))
        ang = tk.Frame(lado, bg=BG)
        ang.pack(pady=(0, 8))
        tk.Label(ang, text="Ángulo del cañón", bg=BG, fg=TEXTO, font=(FUENTE, 10, "bold")).pack()
        self.lbl_motor = tk.Label(ang, text="", bg=BG, fg=TEXTO, font=(FUENTE, 10))
        self.lbl_motor.pack(pady=(2, 0))
        fila_a = tk.Frame(ang, bg=BG)
        fila_a.pack(pady=(6, 0))
        tk.Label(fila_a, text="Escribir ángulo:", bg=BG, fg=GRIS, font=(FUENTE, 9)).pack(side="left")
        self.entrada_angulo = tk.Entry(fila_a, width=6, justify="center")
        self.entrada_angulo.insert(0, "45")
        self.entrada_angulo.pack(side="left", padx=(4, 0))
        self.entrada_angulo.bind("<KeyRelease>", lambda e: self._actualizar_guia())
        tk.Label(fila_a, text="°", bg=BG, fg=TEXTO).pack(side="left", padx=(1, 12))
        for texto, cmd in (("▲ Subir", "ASU"), ("▼ Bajar", "ABA")):
            b = tk.Button(fila_a, text=texto, font=(FUENTE, 9, "bold"), bg=AZUL, fg="white",
                          activebackground=AZUL_OSC, activeforeground="white", relief="flat",
                          padx=8, takefocus=0)
            b.pack(side="left", padx=3)
            b.bind("<ButtonPress-1>", lambda e, c=cmd: self.jog_angulo(c))
            b.bind("<ButtonRelease-1>", lambda e: self.jog_angulo("APA"))
        self.btn_apuntar = tk.Button(ang, text="APUNTAR AUTOMÁTICO (según la distancia)", width=38,
                                     font=(FUENTE, 10, "bold"), bg=MORADO, fg="white",
                                     activebackground=MORADO_OSC, activeforeground="white",
                                     relief="flat", pady=5, takefocus=0, command=self.apuntar)
        self.btn_apuntar.pack(fill="x", pady=(8, 0))
        self.lbl_apunte = tk.Label(ang, text="", bg=BG, fg=GRIS, font=(FUENTE, 9),
                                   wraplength=340, justify="center")
        self.lbl_apunte.pack(pady=(4, 0))

        # Calibración
        calib = tk.Frame(lado, bg=BG)
        calib.pack(pady=(10, 0))
        tk.Label(calib, text=f"Calibración (disparo fijo al {POTENCIA_DISPARO} %)", bg=BG, fg=TEXTO,
                 font=(FUENTE, 10, "bold")).pack()
        tk.Label(calib, text="Motor del ángulo:", bg=BG, fg=GRIS,
                 font=(FUENTE, 9)).pack(pady=(6, 0))
        fm = tk.Frame(calib, bg=BG)
        fm.pack(pady=(2, 0))
        for texto, comando in (("Fijar cero aquí", self.fijar_cero),
                               ("Guardar punto", self.guardar_punto_angulo),
                               ("Borrar puntos", self.borrar_puntos_angulo)):
            tk.Button(fm, text=texto, font=(FUENTE, 9), bg="#CBD2D4", fg=TEXTO,
                      activebackground="#B5BEC1", relief="flat", padx=6, takefocus=0,
                      command=comando).pack(side="left", padx=(5, 0))
        fila2 = tk.Frame(calib, bg=BG)
        fila2.pack(pady=(8, 0))
        tk.Label(fila2, text="El disparo cayó a", bg=BG, fg=TEXTO, font=(FUENTE, 9)).pack(side="left")
        self.entrada_alcance = tk.Entry(fila2, width=6, justify="center")
        self.entrada_alcance.pack(side="left", padx=4)
        self.entrada_alcance.bind("<Return>", lambda e: self.guardar_disparo())
        tk.Label(fila2, text="cm", bg=BG, fg=TEXTO, font=(FUENTE, 9)).pack(side="left")
        for texto, comando in (("Guardar", self.guardar_disparo), ("Borrar datos", self.borrar_datos)):
            tk.Button(fila2, text=texto, font=(FUENTE, 9), bg="#CBD2D4", fg=TEXTO,
                      activebackground="#B5BEC1", relief="flat", padx=8, takefocus=0,
                      command=comando).pack(side="left", padx=(6, 0))
        self.lbl_modelo = tk.Label(calib, text=self._texto_modelo(), bg=BG, fg=GRIS,
                                   font=(FUENTE, 9), wraplength=340, justify="center")
        self.lbl_modelo.pack(pady=(6, 0))

    def _boton_mov(self, padre, texto, cmd, fila, col):
        b = tk.Button(padre, text=texto, width=3, height=1, font=(FUENTE, 16), bg=AZUL, fg="white",
                      activebackground=AZUL_OSC, activeforeground="white", relief="flat", takefocus=0)
        b.grid(row=fila, column=col, padx=3, pady=3)
        b.bind("<ButtonPress-1>", lambda e: self.mover(cmd))
        b.bind("<ButtonRelease-1>", lambda e: self.mover("PAR"))

    def _slider(self, padre, titulo, variable, al_cambiar):
        etiqueta = tk.Label(padre, text=f"{titulo}: {variable.get()} %", bg=BG, fg=TEXTO,
                            font=(FUENTE, 10))
        etiqueta.pack(anchor="w", pady=(6, 0))

        def cambio(valor):
            etiqueta.config(text=f"{titulo}: {int(float(valor))} %")
            if al_cambiar:
                al_cambiar()

        escala = tk.Scale(padre, from_=10, to=100, orient="horizontal", variable=variable,
                          showvalue=False, length=320, bg=BG, troughcolor="#CBD2D4",
                          highlightthickness=0, takefocus=0, command=cambio)
        escala.pack()
        # Al soltar el slider, el foco vuelve a la ventana para que las flechas muevan el robot
        escala.bind("<ButtonRelease-1>", lambda e: self.root.focus_set())
        return escala

    # -- acciones -------------------------------------------------------
    def mover(self, cmd):
        if not self.listo:
            return
        self.mov_actual = cmd
        self._poner_modo_linea(cmd == "LIN")
        valor = 0 if cmd == "PAR" else self.velocidad.get()
        self.conexion.enviar(self._paquete(cmd, valor))

    def _poner_modo_linea(self, activo):
        self.modo_linea = activo
        if activo:
            self.btn_linea.config(text="DETENER (SIGUIENDO LÍNEA...)", bg=ROJO,
                                  activebackground="#9B1F1F")
        else:
            self.btn_linea.config(text="SEGUIR LÍNEA (AUTOMÁTICO)", bg=MORADO,
                                  activebackground=MORADO_OSC)

    def alternar_linea(self):
        if self.modo_linea:
            self.mover("PAR")
        else:
            self.mover("LIN")

    @staticmethod
    def _paquete(cmd, valor=0):
        """3 letras + número entero de 2 bytes (con signo): lo que entiende robot_hub.py."""
        return cmd.encode() + int(valor).to_bytes(2, "big", signed=True)

    def _angulo_actual(self):
        """Ángulo del cañón en uso: el que mide el motor (si está calibrado) o el escrito a mano."""
        if self.modelo.mapa_listo and self.motor_deg is not None:
            angulo = self.modelo.angulo_de_motor(self.motor_deg)
        else:
            try:
                angulo = float(self.entrada_angulo.get().replace(",", "."))
            except ValueError:
                return None
        return angulo if 0 < angulo < 90 else None

    def guardar_disparo(self):
        """Registra dónde cayó un disparo, junto con el ángulo que tenía el cañón."""
        angulo = self._angulo_actual()
        if angulo is None:
            self._aviso_modelo("Escribe el ángulo del cañón con el que disparaste (entre 0 y 90).", ROJO)
            return
        try:
            alcance = float(self.entrada_alcance.get().replace(",", "."))
            self.modelo.agregar(angulo, alcance)
        except ValueError:
            self._aviso_modelo("Escribe la distancia (en cm, mayor que 0) donde cayó la pelota.", ROJO)
            return
        self.entrada_alcance.delete(0, "end")
        self.root.focus_set()
        self._aviso_modelo(f"Guardado: {angulo:.1f}° -> {alcance:.0f} cm.\n" + self._texto_modelo(), VERDE)
        self._actualizar_guia()

    def borrar_datos(self):
        if self.modelo.n and messagebox.askyesno(
                "Borrar disparos", "¿Borrar todos los disparos guardados?"):
            self.modelo.borrar()
            self._aviso_modelo(self._texto_modelo())
            self._actualizar_guia()

    # -- motor del ángulo -------------------------------------------------
    def jog_angulo(self, cmd):
        """Subir / bajar el cañón mientras se mantiene el botón; al soltar, APA lo deja firme."""
        if self.listo:
            self.conexion.enviar(self._paquete(cmd, 0 if cmd == "APA" else 40))

    def fijar_cero(self):
        if self.listo:
            self.conexion.enviar(self._paquete("CER"))
            self._aviso_modelo("Cero fijado aquí. Hazlo siempre con el cañón en la MISMA posición "
                               "de reposo, para que los puntos guardados sigan valiendo.", VERDE)

    def guardar_punto_angulo(self):
        """Asocia la posición actual del motor con el ángulo real escrito (medido con el celular)."""
        try:
            angulo = float(self.entrada_angulo.get().replace(",", "."))
        except ValueError:
            angulo = None
        if self.motor_deg is None:
            self._aviso_modelo("Aún no llega la posición del motor: espera el mensaje 'Listo'.", ROJO)
            return
        if angulo is None or not 0 < angulo < 90:
            self._aviso_modelo("Escribe en la caja el ángulo REAL del cañón (medido con el celular).", ROJO)
            return
        self.modelo.agregar_punto_angulo(self.motor_deg, angulo)
        self.root.focus_set()
        self._aviso_modelo(f"Punto guardado: motor {self.motor_deg}° = cañón {angulo:g}°.\n"
                           + self._texto_modelo(), VERDE)
        self._actualizar_guia()

    def borrar_puntos_angulo(self):
        if self.modelo.puntos_angulo and messagebox.askyesno(
                "Borrar puntos", "¿Borrar la calibración del motor del ángulo?"):
            self.modelo.borrar_puntos_angulo()
            self._aviso_modelo(self._texto_modelo())
            self._actualizar_guia()

    def apuntar(self):
        """Calcula con la física el ángulo para la distancia al objetivo y mueve el motor."""
        recientes = self.distancias[-3:]
        if not recientes:
            self._apunte("Sin objeto a la vista: apunta el sensor al objetivo.", ROJO)
            return
        objetivo = statistics.median(recientes)
        angulo, aviso = self.modelo.angulo_para(objetivo)
        if angulo is None:
            self._apunte(aviso, ROJO)
            return
        texto = f"Para {objetivo:.0f} cm: ángulo {angulo:.1f}°"
        if not self.modelo.mapa_listo:
            self._apunte(texto + ". El motor no está calibrado: ajústalo a mano y escribe el ángulo.",
                         NARANJA_OSC)
        elif not self.modelo.angulo_calibrado(angulo):
            self._apunte(texto + ", pero está fuera de lo que calibraste del motor: "
                         "no lo muevo. Guarda un punto cerca de ese ángulo.", ROJO)
        elif self.listo:
            destino = round(self.modelo.motor_de_angulo(angulo))
            self.conexion.enviar(self._paquete("ANG", destino))
            self._apunte(texto + f" (motor a {destino}°)", VERDE)

    def _apunte(self, texto, color=GRIS):
        self.lbl_apunte.config(text=texto, fg=color)

    def _texto_modelo(self):
        texto = self.modelo.resumen()
        rango = self.modelo.rango_alcance()
        if rango and rango[1] > DIST_MAX_CM:
            texto += f"\nOjo: el sensor no mide más allá de {DIST_MAX_CM} cm."
        return texto

    def _refrescar_angulo(self):
        if self.modelo.mapa_listo and self.motor_deg is not None:
            texto = f"En uso: {self.modelo.angulo_de_motor(self.motor_deg):.1f}° (lo mide el motor)"
        else:
            angulo = self._angulo_actual()
            texto = f"En uso: {'--' if angulo is None else format(angulo, 'g')}° (escrito a mano)"
        if self.motor_deg is not None:
            texto += f"   |   motor en {self.motor_deg}°"
        self.lbl_motor.config(text=texto)

    def _actualizar_guia(self):
        """Dice si, con el ángulo actual, hay que acercarse, alejarse o disparar."""
        self._refrescar_angulo()
        recientes = self.distancias[-3:]      # mediana de las últimas lecturas: evita falsos avisos
        angulo = self._angulo_actual()
        if not recientes:
            estado, texto = None, f"Sin objeto a la vista (el sensor llega a {DIST_MAX_CM} cm)"
        elif angulo is None:
            estado, texto = None, "Escribe el ángulo del cañón (entre 0 y 90)."
        else:
            estado, _, texto = self.modelo.evaluar(statistics.median(recientes), angulo)
        color = {"listo": VERDE, "lejos": NARANJA_OSC, "cerca": NARANJA_OSC}.get(estado, GRIS)
        self.lbl_guia.config(text=texto, fg=color)
        self._en_zona = estado == "listo"
        self._estilo_boton_fuego()
        self._dibujar_barra()

    def _estilo_boton_fuego(self):
        if self.recargando:
            self.btn_fuego.config(text="RECARGANDO", state="disabled", bg=NARANJA)
        elif self._en_zona:      # verde: es el momento de disparar
            self.btn_fuego.config(text="DISPARAR", state="normal", bg=VERDE,
                                  activebackground="#23704A")
        else:
            self.btn_fuego.config(text="DISPARAR", state="normal", bg=NARANJA,
                                  activebackground=NARANJA_OSC)

    def _dibujar_barra(self):
        """Barra de distancia: el tramo verde es la zona donde conviene disparar con el ángulo actual."""
        b = self.barra
        b.delete("all")
        k = BARRA_ANCHO / DIST_MAX_CM
        angulo = self._angulo_actual()
        prevision = None if angulo is None else self.modelo.alcance(angulo)
        if prevision is not None:
            ini = max(prevision - self.modelo.tolerancia, 0) * k
            fin = min(prevision + self.modelo.tolerancia, DIST_MAX_CM) * k
            if fin > ini:
                b.create_rectangle(ini, 0, fin, 16, fill="#9FD8B4", width=0)
        if self.dist_actual is not None:
            x = min(self.dist_actual, DIST_MAX_CM) * k
            b.create_rectangle(0, 5, x, 11, fill=AZUL, width=0)
            b.create_line(x, 0, x, 16, fill=TEXTO, width=3)

    def _aviso_modelo(self, texto, color=GRIS):
        self.lbl_modelo.config(text=texto, fg=color)

    def _clic_en_ventana(self, e):
        if not isinstance(e.widget, tk.Entry):
            self.root.focus_set()

    def calibrar(self, cmd):
        """CAN = guardar el negro, CAB = guardar el blanco (con el sensor sobre ese color)."""
        if self.listo:
            self.conexion.enviar(self._paquete(cmd))

    def _refrescar_color(self):
        def txt(v):
            return "--" if v is None else str(v)
        self.lbl_color.config(text=f"Reflexión: {txt(self.reflexion)}   "
                                   f"Negro: {txt(self.negro)}   Blanco: {txt(self.blanco)}")

    def _velocidad_cambio(self):
        """Si el robot ya se está moviendo, aplica la nueva velocidad (con pequeña espera)."""
        if self.mov_actual == "PAR":
            return
        if self._tarea_velocidad is not None:
            self.root.after_cancel(self._tarea_velocidad)
        self._tarea_velocidad = self.root.after(100, lambda: self.mover(self.mov_actual))

    def disparar(self):
        if not self.listo or self.recargando:
            return
        potencia = POTENCIA_DISPARO
        self.conexion.enviar(self._paquete("FUE", potencia))
        self.recargando = True
        self._estilo_boton_fuego()
        espera_ms = int((360 / (VEL_DISPARO_MAX * potencia / 100) + 0.4) * 1000)
        self.root.after(espera_ms, self._fin_recarga)

    def _fin_recarga(self):
        if self.cerrando:
            return      # la ventana se está cerrando
        self.recargando = False
        self._estilo_boton_fuego()

    def _tecla_abajo(self, e):
        if isinstance(self.root.focus_get(), tk.Entry):
            return      # se está escribiendo un número: no mover el robot
        tecla = e.keysym.lower() if len(e.keysym) == 1 else e.keysym
        if tecla in TECLAS:
            if tecla not in self.teclas_abajo:      # ignora la repetición automática
                self.teclas_abajo.add(tecla)
                self.mover(TECLAS[tecla])
        elif tecla == "space":
            self.disparar()

    def _tecla_arriba(self, e):
        if isinstance(self.root.focus_get(), tk.Entry):
            return
        tecla = e.keysym.lower() if len(e.keysym) == 1 else e.keysym
        if tecla in self.teclas_abajo:
            self.teclas_abajo.discard(tecla)
            if not self.teclas_abajo:
                self.mover("PAR")

    def cerrar(self):
        if self.cerrando:
            return
        self.cerrando = True
        if not self.conexion_viva:
            self.root.destroy()
            return
        self.lbl_estado.config(text="Cerrando...", fg=GRIS)
        self.conexion.cerrar()
        self.root.after(4000, self.root.destroy)    # por si algo se queda colgado

    # -- eventos que llegan del hilo de Bluetooth -----------------------
    def _revisar_eventos(self):
        try:
            while True:
                tipo, dato = self.eventos.get_nowait()
                if tipo == "estado":
                    self.lbl_estado.config(text=dato, fg=GRIS)
                elif tipo == "listo":
                    self.listo = True
                    self.lbl_estado.config(text=f"Conectado a {dato}. Listo.", fg=VERDE)
                elif tipo == "error":
                    self.listo = False
                    self.lbl_estado.config(text=dato, fg=ROJO)
                elif tipo == "distancia":
                    self._mostrar_distancia(dato)
                elif tipo == "reflexion":
                    self.reflexion = dato
                    self._refrescar_color()
                elif tipo == "angulo_motor":
                    self.motor_deg = dato
                    self._actualizar_guia()
                elif tipo == "calibracion":
                    nombre, valor = dato
                    if nombre == "negro":
                        self.negro = valor
                    else:
                        self.blanco = valor
                    self._refrescar_color()
                elif tipo == "cerrado":
                    self.conexion_viva = False
                    self.listo = False
                    if self.cerrando:
                        self.root.destroy()
                        return
        except queue.Empty:
            pass
        self.root.after(50, self._revisar_eventos)

    def _mostrar_distancia(self, mm):
        if mm >= SIN_OBJETO_MM:
            self.distancias = []
            self.dist_actual = None
            self.lbl_dist.config(text="Sin objeto", fg=GRIS)
        else:
            cm = mm / 10
            self.distancias = (self.distancias + [cm])[-5:]
            self.dist_actual = cm
            self.lbl_dist.config(text=f"{cm:.0f} cm", fg=ROJO if cm < DIST_ALERTA_CM else TEXTO)
        self._actualizar_guia()


# ======================================================================
# Utilidad: listar los hubs cercanos para saber cuál es el tuyo
# ======================================================================
async def buscar_hubs():
    from bleak import BleakScanner
    from pybricksdev.ble import PYBRICKS_SERVICE_UUID

    print("Buscando hubs Pybricks cercanos (6 segundos)...")
    vistos = await BleakScanner.discover(timeout=6, return_adv=True,
                                         service_uuids=[PYBRICKS_SERVICE_UUID])
    if not vistos:
        print("No se encontró ninguno. Revisa que el hub esté encendido con luz azul parpadeando.")
        return
    lista = sorted(vistos.values(), key=lambda par: par[1].rssi, reverse=True)
    print("Nombre del hub        Señal (más cerca = número menos negativo)")
    for dispositivo, adv in lista:
        print(f"  {adv.local_name or dispositivo.name or '(sin nombre)':<20}  {adv.rssi} dBm")
    print("\nPon el nombre de TU hub en NOMBRE_HUB, al inicio de este archivo.")


def main():
    if "--buscar" in sys.argv:
        asyncio.run(buscar_hubs())
        return
    root = tk.Tk()
    eventos = queue.Queue()
    conexion = ConexionDemo(eventos) if "--demo" in sys.argv else ConexionHub(eventos)
    Mando(root, conexion, eventos)
    root.mainloop()


if __name__ == "__main__":
    main()
