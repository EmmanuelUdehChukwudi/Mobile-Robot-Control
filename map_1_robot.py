import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ser = serial.Serial('/dev/ttyUSB0', 115200)

x_pos = []
y_pos = []

def update(frame):
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            x, y, phi = map(float, line.split(','))
            x_pos.append(x)
            y_pos.append(-y)
            
            ax.clear()
            ax.plot(y_pos, x_pos, 'bo-', label='Path')
            ax.plot(y_pos[-1], x_pos[-1], 'ro', label='Current Position') 
            ax.set_title('Robot Path')
            ax.set_xlabel('Y Position (m)')
            ax.set_ylabel('X Position (m)')
            ax.legend()
            ax.axis('equal')
            
            ax.set_xlim(-10, 10)
            ax.set_ylim(-10, 10)
            
            ax.set_xticks(range(-10, 11))
            ax.set_yticks(range(-10, 11))
            ax.grid(True)

        except ValueError:
            print(f"Could not parse line: {line}")

fig, ax = plt.subplots()
fig.suptitle('Odometry Data')

ani = FuncAnimation(fig, update, interval=10)

plt.show()
ser.close()
