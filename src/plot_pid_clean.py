import serial
import pandas as pd
import matplotlib.pyplot as plt
import time
import numpy as np

# Adjust this to match your Arduino COM port
port = 'COM3'  # Update if needed
baud = 9600
timeout = 0.1

# How long to collect data (in seconds) - updated to match Arduino's 10s limit
duration = 5  # Give some buffer beyond the 10s Arduino runtime  

ser = serial.Serial(port, baud, timeout=timeout)
print("Reading data for", duration, "seconds...")

data = []
start_time = time.time()
sample_count = 0

while (time.time() - start_time) < duration:
    try:
        line = ser.readline().decode('utf-8').strip()
        
        # Check for END signal from Arduino
        if line == "END":
            print("Arduino finished test - stopping data collection")
            break
            
        if line.count(",") == 7:  # Expecting 8 values
            values = list(map(float, line.split(",")))
            data.append(values)
            sample_count += 1
                
    except ValueError:
        # Skip lines that can't be parsed as floats
        continue
    except:
        # Skip any other parsing errors but continue collecting
        continue

ser.close()

if not data:
    print("No data received.")
    exit()

# Create DataFrame
columns = ["time", "error", "control", "position", "target", "P", "I", "D"]
df = pd.DataFrame(data, columns=columns)

# Filter to only show first 10 seconds
df = df[df["time"] <= 10.0]
print(f"Data collected: {len(df)} samples")

# Constants from Arduino code for weight calculation
FORCE_TO_TARGET = 1.0 / (1.97 * 2)  # 1/(k_spring * num_springs) = 0.253807
ADC_TO_LOAD = (5.0 / 1023.0) * 29.361  # From main.cpp

# Calculate current load from target position
# The Arduino uses: force = load * 6, then target = force * FORCE_TO_TARGET
# So: target = (load * 6) * FORCE_TO_TARGET
# Therefore: load = target / (6 * FORCE_TO_TARGET)
current_load = df["target"].mean() / (6.0 * FORCE_TO_TARGET) if len(df) > 0 else 0.0
print(f"Detected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")

# Apply simple smoothing to reduce noise
def smooth_data(y, window=3):
    """Apply simple moving average to smooth the data"""
    if len(y) < window:
        return y
    
    df_temp = pd.DataFrame({'data': y})
    smoothed = df_temp['data'].rolling(window=window, center=True, min_periods=1).mean()
    return smoothed.values

# Apply smoothing to data for visualization
position_smooth = smooth_data(df["position"].values, window=3)
error_smooth = smooth_data(df["error"].values, window=3)
control_smooth = smooth_data(df["control"].values, window=3)
p_smooth = smooth_data(df["P"].values, window=3)
i_smooth = smooth_data(df["I"].values, window=3)
d_smooth = smooth_data(df["D"].values, window=3)

# Create plots with separate subplots
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

# Position tracking
ax1.plot(df["time"], position_smooth, label="Actual Position", linewidth=2.0, color='blue')
ax1.plot(df["time"], df["target"], label="Target Position", linestyle='--', linewidth=1.8, color='red')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Position (mm)")
ax1.set_title(f"Position Tracking\nDetected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")
ax1.legend()
ax1.grid(True, alpha=0.2)

# Error
ax2.plot(df["time"], error_smooth, label="Position Error", color='red', linewidth=2.0)
ax2.axhline(y=0, color='black', linestyle=':', alpha=0.5)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Error (mm)")
ax2.set_title(f"Position Error\nDetected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")
ax2.legend()
ax2.grid(True, alpha=0.2)

# PID Output
ax3.plot(df["time"], control_smooth, label="PID Output", color='orange', linewidth=2.0)
ax3.axhline(y=0, color='black', linestyle=':', alpha=0.5)
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("PID Output")
ax3.set_title(f"PID Control Output\nDetected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")
ax3.legend()
ax3.grid(True, alpha=0.2)

# PID Terms
ax4.plot(df["time"], p_smooth, label="P Term", linewidth=1.8, color='green')
ax4.plot(df["time"], i_smooth, label="I Term", linewidth=1.8, color='blue')
ax4.plot(df["time"], d_smooth, label="D Term", linewidth=1.8, color='purple')
ax4.axhline(y=0, color='black', linestyle=':', alpha=0.5)
ax4.set_xlabel("Time (s)")
ax4.set_ylabel("PID Term Value")
ax4.set_title(f"PID Terms\nDetected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")
ax4.legend()
ax4.grid(True, alpha=0.2)

plt.tight_layout()
plt.show()

# Individual PID Terms Plot
plt.figure(figsize=(12, 6))
plt.plot(df["time"], p_smooth, label="P Term", linewidth=2.0, color='green')
plt.plot(df["time"], i_smooth, label="I Term", linewidth=2.0, color='blue')
plt.plot(df["time"], d_smooth, label="D Term", linewidth=2.0, color='purple')
plt.axhline(y=0, color='black', linestyle=':', alpha=0.5)
plt.xlabel("Time (s)")
plt.ylabel("PID Term Value")
plt.title(f"PID Terms Over Time\nDetected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")
plt.legend()
plt.grid(True, alpha=0.2)
plt.tight_layout()
plt.show()

print("Plots displayed successfully!")
