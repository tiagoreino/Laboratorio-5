"""Laboratorio 5 D. Ejecutar con Python 3.10+, Tk, pyserial y matplotlib."""

import csv
import math
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import NamedTuple


# 1. ECUACIONES Y CALIBRACIÓN
# Editar estos cuatro valores antes de ejecutar el monitor.
VREF_V = 5.0          # AVcc medida en el Arduino #2
GANANCIA = 21.1762       # Ganancia del amplificador, sin unidad
OFFSET_SALIDA_V = 0.0 # Tensión a la salida del amplificador con I = 0
R_SHUNT_OHM = 4.99     # Resistencia shunt medida


def convertir_adc(adc):
    """Devuelve Vout (V), Vshunt (V) e I (mA) para un código ADC de 10 bits."""
    # Un paso del ADC ideal equivale a Vref / 1024 voltios.
    vout_v = adc * VREF_V / 1024

    # Restar el cero del amplificador y deshacer su ganancia.
    vshunt_v = (vout_v - OFFSET_SALIDA_V) / GANANCIA

    # Ley de Ohm: V / ohmios da amperios. Multiplicar por 1000 da mA.
    corriente_ma = (vshunt_v / R_SHUNT_OHM) * 1000
    return vout_v, vshunt_v, corriente_ma


# 2. ADQUISICIÓN, PROTOCOLO Y CSV
BAUDIOS = 115200
PERIODO_MS = 10
# Paso identifica cuál de las tres visitas a ACTIVO está en curso.
ESTADOS = ("IDLE", "ACTIVO 1", "ADC NR", "ACTIVO 2", "POWER-DOWN", "ACTIVO 3")
ENCABEZADO = ("TIEMPO_S", "TENSION_SHUNT_V", "CORRIENTE_MA", "ESTADO", "PASO", "ADC", "SECUENCIA")
MASCARA_32 = 0xFFFFFFFF
MITAD_32 = 0x80000000


class Trama(NamedTuple):
    """Campos del protocolo original del medidor."""
    tipo: str
    ms: int = 0
    secuencia: int = 0
    adc: int = 0


def interpretar_trama(linea):
    """Valida la cabecera y los campos de adquisición."""
    if linea == b"H,LAB5,2":
        return Trama("H")
    if not 1 <= len(linea) <= 80:
        raise ValueError("Longitud inválida")
    campos = linea.decode("ascii").split(",")
    tipo = campos[0]
    cantidades = {"S": 4, "P": 3, "Y": 3, "N": 3}
    if cantidades.get(tipo) != len(campos):
        raise ValueError("Tipo o cantidad de campos inválidos")

    def entero(indice, limite):
        valor = campos[indice]
        if not (valor.isascii() and valor.isdigit() and len(valor) <= 10):
            raise ValueError("Número inválido")
        numero = int(valor)
        if numero > limite:
            raise ValueError("Valor fuera de rango")
        return numero

    ms, secuencia = entero(1, MASCARA_32), entero(2, MASCARA_32)
    return Trama(tipo, ms, secuencia, adc=entero(3, 1023) if tipo == "S" else 0)


def nombre_estado(paso):
    return ESTADOS[int(paso)] if paso != "?" else "?"


class Adquisicion:
    """Aplica las tramas en orden y prepara filas, con diagnóstico en memoria."""

    def __init__(self):
        self.estado = "?"
        self.ultimo_ms = self.secuencia = self.ultima_muestra_ms = None
        self.tiempo_ms = None
        self.pendiente_desde = None
        self.reinicios = self.perdidas = self.muestras_perdidas = self.invalidas = 0
        self.muestras = 0
        self.aviso = "Esperando tramas del medidor."
        # Historial visible acotado. El CSV guarda todas las muestras válidas.
        self.corrientes = deque(maxlen=6500)
        self.estados = deque(maxlen=6500)

    def cambiar_estado(self, estado):
        anterior = self.estado
        self.estado = estado
        if self.tiempo_ms is not None and (estado != anterior or not self.estados):
            segundos = self.tiempo_ms / 1000
            # Un tramo sin referencia se dibuja como hueco, sin un séptimo nivel.
            if estado == "?" and anterior != "?":
                self.estados.append((segundos, int(anterior)))
            self.estados.append((segundos, int(estado) if estado != "?" else math.nan))

    def reiniciar_reloj(self):
        if self.ultimo_ms is not None:
            self.reinicios += 1
            self.corrientes.append((self.tiempo_ms / 1000, math.nan))
        self.ultimo_ms = self.secuencia = self.ultima_muestra_ms = None
        self.pendiente_desde = None
        self.cambiar_estado("?")
        self.aviso = "Arranque del medidor. Verifica IDLE en el UNO #1 y sincroniza."

    def procesar(self, linea):
        """Devuelve una fila del CSV de corriente o None."""
        try:
            trama = interpretar_trama(linea)
        except ValueError:
            self.invalidas += 1
            self.cambiar_estado("?")
            self.aviso = "Trama inválida. Verifica que el medidor use el protocolo LAB5,2."
            return None

        tipo, ms, secuencia = trama.tipo, trama.ms, trama.secuencia
        if tipo == "H":
            self.reiniciar_reloj()
            return None

        paso = faltan = 0
        if self.ultimo_ms is not None:
            paso = ((ms - self.ultimo_ms + MITAD_32) & MASCARA_32) - MITAD_32
            salto = (secuencia - self.secuencia) & MASCARA_32
            # Un evento puede encolarse justo antes de que termine
            # ADC, cuya marca corresponde al inicio de la conversión.
            if paso < -1 or salto >= MITAD_32:
                self.reiniciar_reloj()
                paso = 0
            elif salto == 0:
                return None
            elif salto > 1:
                faltan = salto - 1

        if self.tiempo_ms is None:
            self.tiempo_ms = 0
            self.aviso = "Adquisición activa. Verifica IDLE en el UNO #1 y sincroniza."
        elif self.ultimo_ms is None:
            # Separación convencional entre segmentos de un mismo CSV.
            self.tiempo_ms += PERIODO_MS
        else:
            self.tiempo_ms += max(paso, 0)
        if paso >= 0:
            self.ultimo_ms = ms
        self.secuencia = secuencia
        segundos = (self.tiempo_ms + min(paso, 0)) / 1000

        if faltan:
            self.perdidas += faltan
            self.cambiar_estado("?")
            self.aviso = "Faltan tramas. Verifica IDLE en el UNO #1 y sincroniza."

        # Sólo Y fija IDLE y sólo P avanza un paso. N conserva la referencia
        # anterior, como en el Problema C. Las muestras no cambian de estado.
        if tipo in ("Y", "N"):
            self.pendiente_desde = None
            if tipo == "Y":
                self.cambiar_estado("0")
                self.aviso = "Referencia manual IDLE registrada."
            else:
                self.aviso = "Suelta S1, verifica IDLE y reintenta."
        elif tipo == "P" and self.estado != "?":
            self.cambiar_estado(str((int(self.estado) + 1) % len(ESTADOS)))

        if tipo != "S":
            return None
        if self.ultima_muestra_ms is not None:
            intervalo = (ms - self.ultima_muestra_ms) & MASCARA_32
            if intervalo != PERIODO_MS:
                self.corrientes.append((segundos, math.nan))
                if intervalo >= PERIODO_MS and intervalo % PERIODO_MS == 0:
                    self.muestras_perdidas += intervalo // PERIODO_MS - 1
                else:
                    self.invalidas += 1
                    self.cambiar_estado("?")
                    self.aviso = "Intervalo de muestreo inesperado. Revisa la conexión."
                    return None
        self.ultima_muestra_ms = ms

        _, vshunt, corriente = convertir_adc(trama.adc)
        self.muestras += 1
        self.corrientes.append((segundos, corriente))
        return (f"{segundos:.3f}", f"{vshunt:.6f}", f"{corriente:.4f}",
                nombre_estado(self.estado), self.estado, trama.adc, secuencia)


def leer_puerto(puerto, archivo, modelo, candado, detener, pedir_a):
    """Lector y escritor en segundo plano, independientes del refresco gráfico."""
    pendiente = b""
    descartar_linea = False
    proximo_flush = time.monotonic() + 1
    try:
        with puerto, archivo:
            escritor = csv.writer(archivo, lineterminator="\n")
            escritor.writerow(ENCABEZADO)
            archivo.flush()
            while not detener.is_set():
                if pedir_a.is_set():
                    with candado:
                        pedir_a.clear()
                        modelo.pendiente_desde = time.monotonic()
                        modelo.aviso = "Esperando confirmación de IDLE."
                    puerto.write(b"A")

                datos = puerto.read(4096)
                if datos:
                    *lineas, pendiente = (pendiente + datos).split(b"\n")
                    # Una línea excesiva se descarta completa hasta LF,
                    # aunque llegue en varios bloques de la UART.
                    if descartar_linea and lineas:
                        lineas[0] = b"!"
                        descartar_linea = False
                    if len(pendiente) > 80 or descartar_linea:
                        pendiente = b""
                        descartar_linea = True
                    for linea in lineas:
                        linea = linea.removesuffix(b"\r")
                        if not linea:
                            continue
                        with candado:
                            fila = modelo.procesar(linea)
                        if fila is not None:
                            escritor.writerow(fila)

                ahora = time.monotonic()
                if ahora >= proximo_flush:
                    archivo.flush()
                    proximo_flush = ahora + 1
                    with candado:
                        if modelo.pendiente_desde is not None and ahora - modelo.pendiente_desde >= 2:
                            modelo.aviso = "Sin respuesta a la sincronización. Reintenta o reconecta el puerto."
                            modelo.pendiente_desde = None
    except OSError as error:
        with candado:
            modelo.aviso = f"Captura interrumpida: {error}"


# 3. INTERFAZ Y GRÁFICAS
def main():
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    import serial
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.figure import Figure
    from serial.tools import list_ports

    raiz = tk.Tk()
    raiz.title("Laboratorio 5 D · Medidor de corriente")
    raiz.geometry("1020x700")
    if (not all(math.isfinite(v) for v in (VREF_V, GANANCIA, OFFSET_SALIDA_V, R_SHUNT_OHM))
            or min(VREF_V, GANANCIA, R_SHUNT_OHM) <= 0):
        messagebox.showerror("Calibración", "Revisa los cuatro parámetros al inicio del archivo.")
        raiz.destroy()
        return

    modelo = Adquisicion()
    candado = threading.Lock()
    detener, pedir_a = threading.Event(), threading.Event()
    lector = None
    cerrando = False

    controles = ttk.Frame(raiz, padding=10)
    controles.pack(fill="x")
    ttk.Label(controles, text="Puerto COM").pack(side="left", padx=(0, 6))
    selector = ttk.Combobox(controles, state="readonly", width=12)
    selector.pack(side="left")
    seguir = tk.BooleanVar(value=True)
    aviso = tk.StringVar(value="Selecciona el COM del Arduino UNO #2.")
    diagnostico = tk.StringVar()
    destino = tk.StringVar()

    figura = Figure(figsize=(9, 5), layout="constrained")
    eje_i, eje_e = figura.subplots(2, 1, sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    linea_i, = eje_i.plot([], [], color="#126e97", linewidth=1)
    linea_e, = eje_e.step([], [], where="post", color="#8a4733")
    eje_i.set_ylabel("Corriente (mA)")
    eje_e.set_ylabel("Estado inferido")
    eje_e.set_yticks(range(len(ESTADOS)), ESTADOS, fontsize=7)
    eje_e.set_ylim(-0.4, 5.4)
    eje_i.grid(alpha=0.25)
    eje_e.grid(alpha=0.25)
    eje_e.set_xlabel("Tiempo de adquisición (s)")

    def actualizar_puertos():
        anterior = selector.get()
        puertos = [p.device for p in sorted(list_ports.comports())]
        selector.configure(values=puertos)
        selector.set(anterior if anterior in puertos else next(iter(puertos), ""))

    def conectar():
        nonlocal lector, modelo
        if lector is not None and lector.is_alive():
            detener.set()
            return
        if not selector.get():
            messagebox.showinfo("Puerto COM", "Selecciona el puerto del UNO #2.")
            return
        carpeta = filedialog.askdirectory(title="Carpeta para guardar el CSV")
        if not carpeta:
            return
        ruta = Path(carpeta) / datetime.now().strftime("Lab5_%Y%m%d_%H%M%S_%f.csv")
        try:
            puerto = serial.Serial(selector.get(), BAUDIOS, timeout=0.05, write_timeout=0.5)
            try:
                archivo = ruta.open("x", encoding="utf-8", newline="")
            except OSError:
                puerto.close()
                raise
        except OSError as error:
            messagebox.showerror("Error de conexión", str(error))
            return
        modelo = Adquisicion()
        detener.clear()
        pedir_a.clear()
        destino.set(str(ruta))
        seguir.set(True)
        lector = threading.Thread(target=leer_puerto, args=(puerto, archivo, modelo, candado,
                                                          detener, pedir_a))
        lector.start()

    def sincronizar_a():
        if lector is not None and lector.is_alive() and not detener.is_set():
            with candado:
                if modelo.ultimo_ms is not None and modelo.pendiente_desde is None:
                    pedir_a.set()

    actualizar = ttk.Button(controles, text="Actualizar", command=actualizar_puertos)
    actualizar.pack(side="left", padx=6)
    conexion = ttk.Button(controles, text="Conectar", command=conectar)
    conexion.pack(side="left", padx=6)
    sincronizar = ttk.Button(controles, text="Sincronizar IDLE", command=sincronizar_a, state="disabled")
    sincronizar.pack(side="left", padx=6)
    ttk.Checkbutton(controles, text="Seguir últimos 60 s", variable=seguir).pack(side="right")

    ttk.Label(raiz, text="Para sincronizar, deja S1 suelto y verifica IDLE en el UNO #1.",
              padding=(10, 0, 10, 4)).pack(anchor="w")
    for variable in (aviso, diagnostico, destino):
        ttk.Label(raiz, textvariable=variable, padding=(10, 0, 10, 4)).pack(anchor="w")
    panel = ttk.Frame(raiz)
    panel.pack(fill="both", expand=True)
    lienzo = FigureCanvasTkAgg(figura, master=panel)
    barra = NavigationToolbar2Tk(lienzo, panel, pack_toolbar=False)
    barra.pack(side="bottom", fill="x")
    lienzo.get_tk_widget().pack(fill="both", expand=True)

    def refrescar():
        activo = lector is not None and lector.is_alive()
        if cerrando and not activo:
            raiz.destroy()
            return
        if lector is not None:
            with candado:
                texto = modelo.aviso
                resumen = (
                    f"Estado {nombre_estado(modelo.estado)} | "
                    f"Muestras {modelo.muestras} | "
                    f"Reinicios {modelo.reinicios} | Tramas perdidas {modelo.perdidas} | "
                    f"Muestras faltantes {modelo.muestras_perdidas} | Inválidas {modelo.invalidas}"
                )
                listo = modelo.ultimo_ms is not None and modelo.pendiente_desde is None
                corrientes = list(modelo.corrientes)
                estados = list(modelo.estados)
                ultimo_s = modelo.tiempo_ms / 1000 if modelo.tiempo_ms is not None else 0
            if activo or texto.startswith("Captura interrumpida"):
                aviso.set(texto)
            else:
                aviso.set("Captura cerrada. CSV guardado.")
            diagnostico.set(resumen)
            linea_i.set_data(*zip(*corrientes) if corrientes else ([], []))
            # Prolongar el escalón actual sin guardar copias por cada muestra.
            if estados:
                estados.append((ultimo_s, estados[-1][1]))
            linea_e.set_data(*zip(*estados) if estados else ([], []))
            if seguir.get():
                fin = max(1, ultimo_s)
                eje_i.set_xlim(max(0, fin - 60), fin)
                eje_i.relim()
                eje_i.autoscale_view(scalex=False)
            lienzo.draw_idle()
        else:
            listo = False
        conexion.configure(text="Desconectar" if activo else "Conectar",
                           state="disabled" if activo and detener.is_set() else "normal")
        selector.configure(state="disabled" if activo else "readonly")
        actualizar.configure(state="disabled" if activo else "normal")
        sincronizar.configure(state="normal" if activo and listo and not detener.is_set()
                              and not pedir_a.is_set() else "disabled")
        raiz.after(250, refrescar)

    def cerrar():
        nonlocal cerrando
        cerrando = True
        detener.set()

    actualizar_puertos()
    raiz.protocol("WM_DELETE_WINDOW", cerrar)
    refrescar()
    raiz.mainloop()


if __name__ == "__main__":
    main()
