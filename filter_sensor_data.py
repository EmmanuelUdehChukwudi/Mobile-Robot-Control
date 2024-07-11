import serial
import time
import matplotlib.pyplot as plt

def sma_filter(data_points, window_size=20):
    if len(data_points) < window_size:
        return data_points
    sma_list = []
    for i in range(len(data_points) - window_size + 1):
        window = data_points[i:i + window_size]
        sma = sum(window) / window_size
        sma_list.append(sma)
    return sma_list

def ema_filter(data_points, alpha=0.2):
    ema_list = []
    ema_prev = data_points[0]
    for i in range(len(data_points)):
        ema_next = alpha * data_points[i] + (1 - alpha) * ema_prev
        ema_list.append(ema_next)
        ema_prev = ema_next
    return ema_list

def read_from_serial(port, window_size=10):
    left_raw_data_points = []
    right_raw_data_points = []
    left_filtered_data_points_sma = []
    right_filtered_data_points_sma = []
    left_filtered_data_points_ema = []
    right_filtered_data_points_ema = []
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        
        plt.ion() 
        fig, ax = plt.subplots(2, 1, sharex=True)
        left_raw_line, = ax[0].plot([], [], label='Left Raw Data')
        left_filtered_line_sma, = ax[0].plot([], [], label='Left Filtered Data (SMA)')
        left_filtered_line_ema, = ax[0].plot([], [], label='Left Filtered Data (EMA)')
        right_raw_line, = ax[1].plot([], [], label='Right Raw Data')
        right_filtered_line_sma, = ax[1].plot([], [], label='Right Filtered Data (SMA)')
        right_filtered_line_ema, = ax[1].plot([], [], label='Right Filtered Data (EMA)')
        ax[0].legend()
        ax[1].legend()
        
        plt.show() 
        
        while True:
            try:
                line = ser.readline()
                try:
                    line = line.decode('utf-8').strip()
                except UnicodeDecodeError as e:
                    print(f"Decode error: {e}")
                    continue

                if line:
                    velocities = line.split(',')
                    if len(velocities) == 2:
                        try:
                            Vr = float(velocities[0])
                            Vl = float(velocities[1])
                            print(f"Vr: {Vr}, Vl: {Vl}")
                        except ValueError as e:
                            print(f"Error converting to float: {e}")
                            continue

                        left_raw_data_points.append(Vl)
                        right_raw_data_points.append(Vr)
                        
                        # Calculate SMA filtered values
                        if len(left_raw_data_points) >= window_size and len(right_raw_data_points) >= window_size:
                            res_l_sma = sma_filter(left_raw_data_points, window_size)
                            res_r_sma = sma_filter(right_raw_data_points, window_size)
                            left_filtered_data_points_sma.append(res_l_sma[-1])
                            right_filtered_data_points_sma.append(res_r_sma[-1])
                        else:
                            left_filtered_data_points_sma.append(Vl)
                            right_filtered_data_points_sma.append(Vr)
                        
                        # Calculate EMA filtered values
                        if len(left_raw_data_points) > 1 and len(right_raw_data_points) > 1:  # EMA requires at least two points
                            res_l_ema = ema_filter(left_raw_data_points)
                            res_r_ema = ema_filter(right_raw_data_points)
                            left_filtered_data_points_ema.append(res_l_ema[-1])
                            right_filtered_data_points_ema.append(res_r_ema[-1])
                        else:
                            left_filtered_data_points_ema.append(Vl)
                            right_filtered_data_points_ema.append(Vr)
                        
                        # Update plot data
                        left_raw_line.set_data(range(len(left_raw_data_points)), left_raw_data_points)
                        left_filtered_line_sma.set_data(range(len(left_filtered_data_points_sma)), left_filtered_data_points_sma)
                        left_filtered_line_ema.set_data(range(len(left_filtered_data_points_ema)), left_filtered_data_points_ema)
                        right_raw_line.set_data(range(len(right_raw_data_points)), right_raw_data_points)
                        right_filtered_line_sma.set_data(range(len(right_filtered_data_points_sma)), right_filtered_data_points_sma)
                        right_filtered_line_ema.set_data(range(len(right_filtered_data_points_ema)), right_filtered_data_points_ema)

                        for a in ax:
                            a.relim()
                            a.autoscale_view()
                        
                        plt.draw()
                        plt.pause(0.001)
            except Exception as e:
                print(f"Error reading line: {e}")
                break
    except Exception as e:
        print(f"Failed to open serial port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    read_from_serial(port)
