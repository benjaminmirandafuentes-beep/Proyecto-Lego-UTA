"""
Antes de conectar:
  * Hub encendido, con la luz del botón central parpadeando en azul.
  * code.pybricks.com cerrado (solo un programa puede usar el Bluetooth del hub).

Arquitectura (cliente - servidor):
  PC (cliente)  --Bluetooth-->  hub (servidor): recibe órdenes y mueve motores
  PC (cliente)  <--Bluetooth--  hub (servidor): informa distancia y reflexión de color
 

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
from tkinter import ttk

from balistica import ModeloDisparo, NOMBRES


NOMBRE_HUB = "fldsmdfr"

RUTA_PROGRAMA_HUB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_hub.py")

VEL_DISPARO_MAX = 900   
DIST_MAX_CM = 200       
DIST_ALERTA_CM = 20     
SIN_OBJETO_MM = 2000 


# DISEÑO

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
                    await hub.write(b"PAR\x00")
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
        self.activo = True

    def iniciar(self):
        threading.Thread(target=self._bucle, daemon=True).start()

    def enviar(self, paquete):
        comando, valor = paquete[:3].decode(), paquete[3]
        print(f"[DEMO] {comando} valor={valor}")
        if comando in ("ADL", "ATR", "PAR"):
            self.movimiento = comando
        elif comando == "CAN":
            self.eventos.put(("calibracion", ("negro", self.reflexion)))
        elif comando == "CAB":
            self.eventos.put(("calibracion", ("blanco", self.reflexion)))

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
        self.potencia = tk.IntVar(value=60)
        self.metodo = tk.StringVar(value="empirico")   # método de puntería elegido

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
        self.ultima_potencia = None   # potencia del último disparo (para calibrar)
        self.distancias = []          # últimas lecturas del sensor, en cm

        self._construir()
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
        self.lbl_estado.grid(row=0, column=0, columnspan=2, pady=(12, 4))

        # Indicador de distancia
        caja = tk.Frame(r, bg=BG)
        caja.grid(row=1, column=0, columnspan=2, pady=(0, 10))
        tk.Label(caja, text="Distancia al objeto", bg=BG, fg=GRIS, font=(FUENTE, 10)).pack()
        self.lbl_dist = tk.Label(caja, text="-- cm", bg=BG, fg=TEXTO, font=(FUENTE, 32, "bold"))
        self.lbl_dist.pack()
        self.barra = ttk.Progressbar(caja, length=320, maximum=DIST_MAX_CM)
        self.barra.pack(pady=(2, 0))

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

        # Barras de ajuste
        ajustes = tk.Frame(r, bg=BG)
        ajustes.grid(row=4, column=0, columnspan=2, padx=24, pady=(6, 16))
        self._slider(ajustes, "Velocidad de avance", self.velocidad, self._velocidad_cambio)
        self.esc_potencia = self._slider(ajustes, "Potencia de disparo", self.potencia, None)

        # Puntería: la potencia que acierta a la distancia que mide el sensor
        punt = tk.Frame(r, bg=BG)
        punt.grid(row=5, column=0, columnspan=2, padx=24, pady=(0, 14))
        tk.Button(punt, text="AUTO-POTENCIA (según la distancia)", width=34,
                  font=(FUENTE, 10, "bold"), bg=AZUL, fg="white", activebackground=AZUL_OSC,
                  activeforeground="white", relief="flat", pady=5, takefocus=0,
                  command=self.auto_potencia).pack(fill="x")
        fm = tk.Frame(punt, bg=BG)
        fm.pack(pady=(8, 0))
        tk.Label(fm, text="Método:", bg=BG, fg=TEXTO, font=(FUENTE, 9)).pack(side="left")
        for valor, texto in (("empirico", "Empírico (datos)"), ("fisico", "Físico (parabólico)")):
            tk.Radiobutton(fm, text=texto, value=valor, variable=self.metodo, bg=BG, fg=TEXTO,
                           activebackground=BG, font=(FUENTE, 9), takefocus=0,
                           command=self._cambio_metodo).pack(side="left", padx=4)
        ff = tk.Frame(punt, bg=BG)
        ff.pack(pady=(4, 0))
        tk.Label(ff, text="Ángulo del cañón", bg=BG, fg=TEXTO, font=(FUENTE, 9)).pack(side="left")
        self.entrada_angulo = tk.Entry(ff, width=5, justify="center")
        self.entrada_angulo.insert(0, f"{self.modelo.angulo_deg:g}")
        self.entrada_angulo.pack(side="left", padx=4)
        tk.Label(ff, text="°    Altura de salida", bg=BG, fg=TEXTO,
                 font=(FUENTE, 9)).pack(side="left")
        self.entrada_altura = tk.Entry(ff, width=5, justify="center")
        self.entrada_altura.insert(0, f"{self.modelo.altura_cm:g}")
        self.entrada_altura.pack(side="left", padx=4)
        tk.Label(ff, text="cm", bg=BG, fg=TEXTO, font=(FUENTE, 9)).pack(side="left")
        for caja in (self.entrada_angulo, self.entrada_altura):
            caja.bind("<Return>", lambda e: self._cambio_metodo())
        fila2 = tk.Frame(punt, bg=BG)
        fila2.pack(pady=(8, 0))
        tk.Label(fila2, text="El último disparo cayó a", bg=BG, fg=TEXTO,
                 font=(FUENTE, 9)).pack(side="left")
        self.entrada_alcance = tk.Entry(fila2, width=6, justify="center")
        self.entrada_alcance.pack(side="left", padx=4)
        self.entrada_alcance.bind("<Return>", lambda e: self.guardar_disparo())
        tk.Label(fila2, text="cm", bg=BG, fg=TEXTO, font=(FUENTE, 9)).pack(side="left")
        tk.Button(fila2, text="Guardar", font=(FUENTE, 9), bg="#CBD2D4", fg=TEXTO,
                  activebackground="#B5BEC1", relief="flat", padx=10, takefocus=0,
                  command=self.guardar_disparo).pack(side="left", padx=(8, 0))
        self.lbl_modelo = tk.Label(punt, text=self.modelo.resumen(), bg=BG, fg=GRIS,
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
        self.conexion.enviar(cmd.encode() + bytes([valor]))

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

    def auto_potencia(self):
        """Lee la distancia al objetivo y ajusta el slider con la potencia calculada."""
        if not self.distancias:
            self._aviso_modelo("No hay objeto a la vista: apunta el sensor al objetivo.", ROJO)
            return
        if not self._aplicar_fisica():
            return
        objetivo = statistics.median(self.distancias)   # la mediana evita lecturas sueltas raras
        metodo = self.metodo.get()
        potencia, aviso = self.modelo.potencia_para(objetivo, metodo)
        if potencia is None:
            self._aviso_modelo(aviso, ROJO)
            return
        self.esc_potencia.set(potencia)
        texto = f"[{NOMBRES[metodo]}] Objetivo a {objetivo:.0f} cm -> potencia {potencia} %"
        if aviso:
            texto += "\n" + aviso
        self._aviso_modelo(texto, NARANJA_OSC if aviso else VERDE)

    def guardar_disparo(self):
        """Registra dónde cayó el último disparo para que el modelo aprenda."""
        if self.ultima_potencia is None:
            self._aviso_modelo("Primero haz un disparo y mide dónde cayó la pelota.", ROJO)
            return
        if not self._aplicar_fisica():
            return
        try:
            alcance = float(self.entrada_alcance.get().replace(",", "."))
            self.modelo.agregar(self.ultima_potencia, alcance)
        except ValueError:
            self._aviso_modelo("Escribe la distancia (en cm, mayor que 0) donde cayó la pelota.", ROJO)
            return
        guardado = f"Guardado: {self.ultima_potencia} % -> {alcance:.0f} cm."
        self.ultima_potencia = None          # evita guardar dos veces el mismo disparo
        self.entrada_alcance.delete(0, "end")
        self.root.focus_set()
        self._aviso_modelo(guardado + "\n" + self.modelo.resumen(), VERDE)

    def _aplicar_fisica(self):
        """Lee el ángulo y la altura de las cajas y se los pasa al modelo. False si no son válidos."""
        try:
            angulo = float(self.entrada_angulo.get().replace(",", "."))
            altura = float(self.entrada_altura.get().replace(",", "."))
            self.modelo.configurar_fisica(angulo, altura)
        except ValueError as e:
            texto = str(e) if "ángulo" in str(e) or "altura" in str(e) else \
                "Escribe números en el ángulo (0 a 90) y en la altura (cm)."
            self._aviso_modelo(texto, ROJO)
            return False
        return True

    def _cambio_metodo(self):
        self.root.focus_set()
        if self._aplicar_fisica():
            self._aviso_modelo(self.modelo.resumen())

    def _aviso_modelo(self, texto, color=GRIS):
        self.lbl_modelo.config(text=texto, fg=color)

    def _clic_en_ventana(self, e):
        if not isinstance(e.widget, tk.Entry):
            self.root.focus_set()

    def calibrar(self, cmd):
        """CAN = guardar el negro, CAB = guardar el blanco (con el sensor sobre ese color)."""
        if self.listo:
            self.conexion.enviar(cmd.encode() + b"\x00")

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
        potencia = self.potencia.get()
        self.ultima_potencia = potencia
        self.conexion.enviar(b"FUE" + bytes([potencia]))
        self.recargando = True
        self.btn_fuego.config(text="RECARGANDO", state="disabled")
        espera_ms = int((360 / (VEL_DISPARO_MAX * potencia / 100) + 0.4) * 1000)
        self.root.after(espera_ms, self._fin_recarga)

    def _fin_recarga(self):
        if self.cerrando:
            return      # la ventana se está cerrando
        self.recargando = False
        self.btn_fuego.config(text="DISPARAR", state="normal")

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
            self.lbl_dist.config(text="Sin objeto", fg=GRIS)
            self.barra["value"] = DIST_MAX_CM
            return
        cm = mm / 10
        self.distancias = (self.distancias + [cm])[-5:]
        self.lbl_dist.config(text=f"{cm:.0f} cm", fg=ROJO if cm < DIST_ALERTA_CM else TEXTO)
        self.barra["value"] = min(cm, DIST_MAX_CM)


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
