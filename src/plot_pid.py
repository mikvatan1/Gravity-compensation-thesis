import serial
import pandas as pd
import matplotlib.pyplot as plt
import time

# Adjust this to match your Arduino COM port
port = 'COM3'  # Update if needed
baud = 9600
timeout = 1

# How long to collect data (in seconds)
duration = 20  

ser = serial.Serial(port, baud, timeout=timeout)
print("Reading data for", duration, "seconds...")

data = []
start_time = time.time()

while (time.time() - start_time) < duration:
    try:
        line = ser.readline().decode('utf-8').strip()
        if line.count(",") == 7:  # Expecting 8 values
            values = list(map(float, line.split(",")))
            data.append(values)
    except:
        continue

ser.close()

if not data:
    print("No data received.")
    exit()

# Create DataFrame
columns = ["time", "error", "control", "position", "target", "P", "I", "D"]
df = pd.DataFrame(data, columns=columns)

# Plot
plt.figure(figsize=(12, 6))
plt.plot(df["time"], df["position"], label="Actual Position")
plt.plot(df["time"], df["target"], label="Target Position", linestyle='--')
plt.plot(df["time"], df["error"], label="Error", linestyle=':')
plt.plot(df["time"], df["control"], label="PID Output", alpha=0.6)

plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("PID Controller Performance")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Plot P, I, D terms ---
plt.figure(figsize=(12, 6))
plt.plot(df["time"], df["P"], label="P Term")
plt.plot(df["time"], df["I"], label="I Term")
plt.plot(df["time"], df["D"], label="D Term")
plt.xlabel("Time (s)")
plt.ylabel("PID Term Value")
plt.title("PID Terms Over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()