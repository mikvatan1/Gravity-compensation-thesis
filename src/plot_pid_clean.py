import serial
import pandas as pd
import matplotlib.pyplot as plt
import time
import numpy as np
import os
from datetime import datetime

# Adjust this to match your Arduino COM port
port = 'COM3'  # Update if needed
baud = 115200
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
            
        if line.count(",") == 8:  # Expecting 9 values now (added detectedLoad)
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
columns = ["time", "error", "control", "position", "target", "detectedLoad", "P", "I", "D"]
df = pd.DataFrame(data, columns=columns)

# Filter to only show first 10 seconds
df = df[df["time"] <= 10.0]
print(f"Data collected: {len(df)} samples")

# Constants from Arduino code for weight calculation
FORCE_TO_TARGET = 1.0 / (1.97 * 2)  # 1/(k_spring * num_springs) = 0.253807
ADC_TO_LOAD = (5.0 / 1023.0) * 29.361  # From main.cpp


# Calculate current load from detectedLoad field directly
current_load = df["detectedLoad"].mean() if len(df) > 0 else 0.0
print(f"Detected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")

# Apply enhanced smoothing for ultra-smooth lines
def smooth_data(y, window=7):
    """Apply moving average with larger window for smoother lines"""
    if len(y) < window:
        return y
    
    df_temp = pd.DataFrame({'data': y})
    smoothed = df_temp['data'].rolling(window=window, center=True, min_periods=1).mean()
    return smoothed.values

# Apply smoothing to all data
position_smooth = smooth_data(df["position"].values, window=7)
error_smooth = smooth_data(df["error"].values, window=7)
control_smooth = smooth_data(df["control"].values, window=7)
target_smooth = smooth_data(df["target"].values, window=7)
p_smooth = smooth_data(df["P"].values, window=7)
i_smooth = smooth_data(df["I"].values, window=7)
d_smooth = smooth_data(df["D"].values, window=7)

# Create plots with smooth lines - 2 graphs: Combined signals + PID terms
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

# Combined plot: Position, Target, Error, and Control Output
ax1.plot(df["time"], position_smooth, label="Actual Position", linewidth=2.5, color='blue', alpha=0.8)
ax1.plot(df["time"], target_smooth, label="Target Position", linestyle='-', linewidth=2.0, color='red', alpha=0.8)
ax1.plot(df["time"], error_smooth, label="Position Error", linewidth=2.0, color='orange', alpha=0.8)
ax1.plot(df["time"], control_smooth, label="PID Output", linewidth=2.0, color='green', alpha=0.8)
ax1.axhline(y=0, color='black', linestyle=':', alpha=0.5)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Values (mm / control units)")
ax1.set_title(f"System Response\nDetected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")
ax1.legend()
ax1.grid(True, alpha=0.3)

# PID Terms with smooth lines
ax2.plot(df["time"], p_smooth, label="P Term", linewidth=2.5, color='green', alpha=0.8)
ax2.plot(df["time"], i_smooth, label="I Term", linewidth=2.5, color='blue', alpha=0.8)
ax2.plot(df["time"], d_smooth, label="D Term", linewidth=2.5, color='purple', alpha=0.8)
ax2.axhline(y=0, color='black', linestyle=':', alpha=0.5)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("PID Term Value")
ax2.set_title(f"PID Terms\nDetected Load: {current_load:.2f} N ({current_load/9.81:.1f} kg)")
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()

# Create PID_graphs folder if it doesn't exist
graphs_folder = "PID_graphs"
if not os.path.exists(graphs_folder):
    os.makedirs(graphs_folder)
    print(f"Created folder: {graphs_folder}")

# Generate filename with timestamp and load info
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
load_kg = current_load / 9.81
filename = f"PID_analysis_{timestamp}_Load_{load_kg:.1f}kg.png"
filepath = os.path.join(graphs_folder, filename)

# Save the figure
plt.savefig(filepath, dpi=300, bbox_inches='tight', facecolor='white')
print(f"✓ Graph saved to: {filepath}")

plt.show()

print("\n=== Final Values ===")
print(f"Final Error: {df['error'].iloc[-1]:.3f} mm")
print(f"Final Position: {df['position'].iloc[-1]:.3f} mm")
print(f"Final Target: {df['target'].iloc[-1]:.3f} mm")
print(f"Mean Absolute Error: {abs(df['error']).mean():.3f} mm")
print(f"Position Standard Deviation: {df['position'].std():.3f} mm")
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
