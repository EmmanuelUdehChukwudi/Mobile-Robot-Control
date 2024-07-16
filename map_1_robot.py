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
            print(x,y)
            x_pos.append(x)  
            y_pos.append(-y)  
            ax.clear() 
            ax.plot(y_pos, x_pos, 'bo-', markersize=1, label='Path')  
            ax.plot(y_pos[-1], x_pos[-1], 'ro', markersize=6, label='Current Position') 
            ax.set_title('Robot Path', fontsize=16, fontweight='bold')
            ax.set_xlabel('Y Position (cm)', fontsize=14)
            ax.set_ylabel('X Position (cm)', fontsize=14)
            ax.legend(fontsize=6)
            ax.axis('equal')
            ax.set_xlim(-1000, 1000)  
            ax.set_ylim(-1000, 1000) 
            ax.grid(True, linestyle='--', linewidth=0.5, color='gray')
            
            
            ax.set_xticks(range(-1000, 1001, 100))
            ax.set_yticks(range(-1000, 1001, 100))
            ax.set_facecolor('lightyellow')  

        except ValueError:
            print(f"Could not parse line: {line}")

fig, ax = plt.subplots()
fig.suptitle('Odometry Data')

ani = FuncAnimation(fig, update, interval=5) 
plt.show()
ser.close()
