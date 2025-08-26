import serial
import matplotlib.pyplot as plt
import numpy as np
import time

# Configurar puerto
ser = serial.Serial('COM10', 460800, timeout=1)

def leer_trama():
    """Lee una trama completa del ESP32 entre START y END"""
    linea = ""
    trama = []
    while True:
        linea = ser.readline().decode("utf-8").strip()
        if not linea:
            continue
        if linea == "START":
            trama = []
        elif linea == "END":
            break
        else:
            trama.append(linea)
    return trama

def parsear_trama(trama):
    """Convierte la trama en parámetros y datos"""
    sample_period = None
    total = None
    datos = []

    for linea in trama:
        if linea.startswith("SAMPLE_PERIOD_US"):
            sample_period = float(linea.split("=")[1])
        elif linea.startswith("TOTAL"):
            total = int(linea.split("=")[1])
        elif linea.startswith("DATA"):
            valores = linea.split("=")[1]
            datos = [float(x) for x in valores.split(",") if x.strip()]
    return sample_period, total, np.array(datos)


# ---- Configuración de la gráfica ----
plt.ion()  # modo interactivo
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

linea_signal, = ax1.plot([], [], lw=1.5)
ax1.set_title("Señal muestreada")
ax1.set_xlabel("Muestra")
ax1.set_ylabel("Amplitud")

linea_fft, = ax2.plot([], [], lw=1.5)
ax2.set_title("Transformada de Fourier (FFT)")
ax2.set_xlabel("Frecuencia [Hz]")
ax2.set_ylabel("Magnitud")
ax2.set_xlim(0, 250)  # limitar eje x para mejor visualización

plt.tight_layout()
plt.show()


try:
    while True:
        # pedir datos al ESP32
        ser.write(b'GET\n')  

        # leer trama completa
        trama = leer_trama()
        sample_period, total, datos = parsear_trama(trama)

        if datos.size == 0:
            print("No llegaron datos")
            continue

        # frecuencia de muestreo
        fs = 1e6 / sample_period  

        # ---- Actualizar gráfica ----
        # Señal
        linea_signal.set_data(np.arange(len(datos)), datos)
        ax1.relim()
        ax1.autoscale_view()

        # FFT
        N = len(datos)
        freqs = np.fft.rfftfreq(N, d=1/fs)
        fft_vals = np.abs(np.fft.rfft(datos))

        linea_fft.set_data(freqs, fft_vals)
        ax2.relim()
        ax2.autoscale_view()

        fig.canvas.draw()
        fig.canvas.flush_events()

        # espera 1 segundo entre actualizaciones
        time.sleep(1)

except KeyboardInterrupt:
    print("Interrupción manual (Ctrl+C).")

finally:
    if ser.is_open:
        ser.close()
        print("Puerto cerrado.")
