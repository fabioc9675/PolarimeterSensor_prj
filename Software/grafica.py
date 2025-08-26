import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

ser = serial.Serial('COM10', 460800, timeout=1)

try:
    bsize = 49
    buffer = deque([0]*bsize, maxlen=bsize)

    fig, ax = plt.subplots()
    line, = ax.plot(range(bsize), list(buffer))
    ax.set_ylim(0, 3000)
    ax.set_xlim(0, bsize-1)

    def update(frame):
        try:
            data = ser.readline().decode('utf-8').strip()
            if data:
                valores = [float(x) for x in data.split(",") if x]
                if len(valores) == bsize:
                    buffer.clear()
                    buffer.extend(valores)
                    line.set_ydata(list(buffer))
        except Exception as e:
            print("Error leyendo:", e)
        return line,

    def on_close(event):
        if ser.is_open:
            ser.close()
            print("Puerto cerrado desde ventana.")

    fig.canvas.mpl_connect("close_event", on_close)
    ani = animation.FuncAnimation(fig, update, interval=10, blit=True)
    plt.show()

finally:
    if ser.is_open:
        ser.close()
        print("Puerto cerrado desde finally.")
