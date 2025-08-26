import serial
import matplotlib.pyplot as plt
import numpy as np

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

        # ---- Graficar ----
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

        # Señal
        ax1.plot(datos)
        ax1.set_title("Señal muestreada")
        ax1.set_xlabel("Muestra")
        ax1.set_ylabel("Amplitud")

        # FFT
        N = len(datos)
        freqs = np.fft.rfftfreq(N, d=1/fs)
        fft_vals = np.abs(np.fft.rfft(datos))

        ax2.plot(freqs, fft_vals)
        ax2.set_title("Transformada de Fourier (FFT)")
        ax2.set_xlabel("Frecuencia [Hz]")
        ax2.set_ylabel("Magnitud")

        plt.tight_layout()
        plt.show()

except KeyboardInterrupt:
    print("Interrupción manual (Ctrl+C).")

finally:
    if ser.is_open:
        ser.close()
        print("Puerto cerrado.")
