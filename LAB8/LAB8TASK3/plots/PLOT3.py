import serial
import matplotlib.pyplot as plt

# === Setup Serial ===
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
plt.ion()

temp = []
x_vals = []
y_vals = []
z_vals = []
time_ms = []
cnt = 0

plt.figure()

while True:
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode().strip()
            values = line.split(',')

            if len(values) == 4:
                t = int(values[0])
                x = float(values[1])
                y = float(values[2])
                z = float(values[3])

                temp.append(t)
                x_vals.append(x)
                y_vals.append(y)
                z_vals.append(z)
                time_ms.append(cnt)
                cnt += 1

                # Keep last 200 samples
                if len(temp) > 200:
                    temp.pop(0)
                    x_vals.pop(0)
                    y_vals.pop(0)
                    z_vals.pop(0)
                    time_ms.pop(0)

                plt.clf()

                plt.subplot(2,1,1)
                plt.title("Temperature")
                plt.plot(time_ms, temp, 'r')
                plt.grid(True)

                plt.subplot(2,1,2)
                plt.title("Angular Velocity (dps)")
                plt.plot(time_ms, x_vals, label='X')
                plt.plot(time_ms, y_vals, label='Y')
                plt.plot(time_ms, z_vals, label='Z')
                plt.legend()
                plt.grid(True)

                plt.pause(0.01)

        except:
            pass