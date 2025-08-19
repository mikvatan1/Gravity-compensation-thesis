import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# Close all open figures
plt.close('all')

# === USER SETTINGS ===
port = 'COM3'  # Updated to match your Arduino port
baudrate = 9600  # Updated to match your Arduino baud rate
max_lines = 2000
skip_first = 5
skip_last = 2
buffer_size = 2000

# === Setup ===
ser = serial.Serial(port, baudrate)
ser.reset_input_buffer()  # Flush old junk from buffer

# Data buffers
error_vals = deque(maxlen=buffer_size)
control_vals = deque(maxlen=buffer_size)
position_vals = deque(maxlen=buffer_size)
target_vals = deque(maxlen=buffer_size)
time_vals = deque(maxlen=buffer_size)
p_vals = deque(maxlen=buffer_size)
i_vals = deque(maxlen=buffer_size)
d_vals = deque(maxlen=buffer_size)

plt.ion()
fig, ax = plt.subplots()
fig_pid, ax_pid = plt.subplots()  # Separate plot for PID components

line_count = 0

while True:
    line = ser.readline().decode(errors='ignore').strip()

    if line_count < skip_first:
        line_count += 1
        continue

    if "END" in line:
        print("Arduino finished test.")
        break

    if line_count >= (max_lines - skip_last):
        print("Reached max usable lines, exiting...")
        break

    try:
        parts = line.strip().split(",")

        if len(parts) != 8:
            raise ValueError("Line does not have exactly 8 parts")

        timestamp = float(parts[0])  # already in seconds
        error = float(parts[1])  # error in mm (no scaling needed)
        control = float(parts[2])  # control output (no scaling needed)
        position = float(parts[3])  # position in mm (no scaling needed)
        target = float(parts[4])  # target in mm (no scaling needed)
        p = float(parts[5])  # P term
        i = float(parts[6])  # I term
        d = float(parts[7])  # D term

        # Store values
        time_vals.append(timestamp)
        error_vals.append(error)
        control_vals.append(control)
        position_vals.append(position)
        target_vals.append(target)
        p_vals.append(p)
        i_vals.append(i)
        d_vals.append(d)

        # Main plot
        ax.clear()
        ax.plot(time_vals, error_vals, label='Error')
        ax.plot(time_vals, control_vals, label='Control')
        ax.plot(time_vals, position_vals, label='Position')
        ax.plot(time_vals, target_vals, label='Target')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Values (mm, control output)")
        ax.legend()

        # PID component plot
        ax_pid.clear()
        ax_pid.plot(time_vals, p_vals, label='P')
        ax_pid.plot(time_vals, i_vals, label='I')
        ax_pid.plot(time_vals, d_vals, label='D')
        ax.axhline(25.5, color='black', linestyle=':', linewidth=1, label='Max PWM')
        ax.axhline(-25.5, color='black', linestyle=':', linewidth=1)
        ax_pid.set_xlabel("Time (s)")
        ax_pid.set_ylabel("PID Contributions")
        ax_pid.legend()

        plt.pause(0.001)
        line_count += 1

    except ValueError as e:
        print(f"[Skipped] {line} → {e}")
        continue

# === Settling Time Calculation ===
settling_threshold = 5  # mm
required_stable_points = 10

settling_time = None
final_error = error_vals[-1]

for i in range(len(error_vals) - required_stable_points):
    window = list(error_vals)[i:i + required_stable_points]
    if all(abs(e - final_error) <= settling_threshold for e in window):
        settling_time = time_vals[i]
        break

# === Final Outputs ===
print("\n=== Final Values ===")
print(f"Time: {time_vals[-1]:.3f} s")
print(f"Error: {error_vals[-1]:.3f} mm")
print(f"Control: {control_vals[-1]:.3f}")
print(f"Position: {position_vals[-1]:.3f} mm")
print(f"Target: {target_vals[-1]:.3f} mm")
if settling_time is not None:
    print(f"Settling Time: {settling_time:.3f} seconds")
else:
    print("No settling time detected.")

plt.ioff()
plt.show(block=True)
ser.close()
