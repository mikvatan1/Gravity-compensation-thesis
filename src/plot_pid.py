import serial
import pandas as pd
import matplotlib.pyplot as plt
import time
import os
from datetime import datetime
import numpy as np

# Create a timestamp for unique filenames
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
save_dir = "PID_Analysis_Results"

# Create directory if it doesn't exist
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Adjust this to match your Arduino COM port
port = 'COM3'  # Update if needed
baud = 9600
timeout = 0.1  # Reduced from 1 to 0.1 for faster sampling

# How long to collect data (in seconds)
duration = 20  

ser = serial.Serial(port, baud, timeout=timeout)
print("Reading data for", duration, "seconds... (optimized for high sample rate)")

data = []
start_time = time.time()
sample_count = 0

while (time.time() - start_time) < duration:
    try:
        line = ser.readline().decode('utf-8').strip()
        if line.count(",") == 7:  # Expecting 8 values
            values = list(map(float, line.split(",")))
            data.append(values)
            sample_count += 1
            
            # Show progress every 100 samples
            if sample_count % 100 == 0:
                elapsed = time.time() - start_time
                rate = sample_count / elapsed
                print(f"Samples collected: {sample_count}, Rate: {rate:.1f} samples/sec")
                
    except ValueError:
        # Skip lines that can't be parsed as floats
        continue
    except:
        # Skip any other parsing errors but continue collecting
        continue

ser.close()

print(f"\nData collection complete! Collected {len(data)} samples in {duration} seconds")
print(f"Average sampling rate: {len(data)/duration:.1f} samples/sec")

if not data:
    print("No data received.")
    exit()

# Create DataFrame
columns = ["time", "error", "control", "position", "target", "P", "I", "D"]
df = pd.DataFrame(data, columns=columns)

# Filter to only show first 5 seconds
df = df[df["time"] <= 5.0]

# Apply simple smoothing to reduce noise
def smooth_data(y, window=3):
    """Apply simple moving average to smooth the data"""
    if len(y) < window:
        return y
    
    # Convert to pandas Series for rolling mean
    import pandas as pd
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

# Plot with separate subplots to handle different scales
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

# Position tracking
ax1.plot(df["time"], position_smooth, label="Actual Position", linewidth=2.0, linestyle='-', marker=None, antialiased=True, color='blue')
ax1.plot(df["time"], df["target"], label="Target Position", linestyle='--', linewidth=1.8, marker=None, antialiased=True, color='red')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Position (mm)")
ax1.set_title("Position Tracking")
ax1.legend()
ax1.grid(True, alpha=0.2, linestyle='-', linewidth=0.5)

# Error
ax2.plot(df["time"], error_smooth, label="Position Error", color='red', linewidth=2.0, linestyle='-', marker=None, antialiased=True)
ax2.axhline(y=0, color='black', linestyle=':', alpha=0.5, linewidth=1)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Error (mm)")
ax2.set_title("Position Error")
ax2.legend()
ax2.grid(True, alpha=0.2, linestyle='-', linewidth=0.5)

# PID Output (separate scale)
ax3.plot(df["time"], control_smooth, label="PID Output", color='orange', linewidth=2.0, linestyle='-', marker=None, antialiased=True)
ax3.axhline(y=0, color='black', linestyle=':', alpha=0.5, linewidth=1)
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("PID Output")
ax3.set_title("PID Control Output")
ax3.legend()
ax3.grid(True, alpha=0.2, linestyle='-', linewidth=0.5)

# PID Terms
ax4.plot(df["time"], p_smooth, label="P Term", alpha=1.0, linewidth=1.8, linestyle='-', marker=None, antialiased=True, color='green')
ax4.plot(df["time"], i_smooth, label="I Term", alpha=1.0, linewidth=1.8, linestyle='-', marker=None, antialiased=True, color='blue')
ax4.plot(df["time"], d_smooth, label="D Term", alpha=1.0, linewidth=1.8, linestyle='-', marker=None, antialiased=True, color='purple')
ax4.axhline(y=0, color='black', linestyle=':', alpha=0.5, linewidth=1)
ax4.set_xlabel("Time (s)")
ax4.set_ylabel("PID Term Value")
ax4.set_title("PID Terms")
ax4.legend(fontsize=8)
ax4.grid(True, alpha=0.2, linestyle='-', linewidth=0.5)
plt.tight_layout()

# Save the main analysis plot
main_plot_filename = f"{save_dir}/PID_Analysis_{timestamp}.png"
plt.savefig(main_plot_filename, dpi=300, bbox_inches='tight')
print(f"Saved main analysis plot: {main_plot_filename}")
plt.show()

# Print some statistics to help debug
print("=== PID Analysis ===")
print(f"PID Output - Min: {df['control'].min():.2f}, Max: {df['control'].max():.2f}, Mean: {df['control'].mean():.2f}")
print(f"P Term - Min: {df['P'].min():.2f}, Max: {df['P'].max():.2f}, Mean: {df['P'].mean():.2f}")
print(f"I Term - Min: {df['I'].min():.2f}, Max: {df['I'].max():.2f}, Mean: {df['I'].mean():.2f}")
print(f"D Term - Min: {df['D'].min():.2f}, Max: {df['D'].max():.2f}, Mean: {df['D'].mean():.2f}")
print(f"Error - Min: {df['error'].min():.2f}, Max: {df['error'].max():.2f}, Mean: {df['error'].mean():.2f}")

# PID TERM BEHAVIOR ANALYSIS
print("\n=== PID TERM BEHAVIOR ANALYSIS ===")

# P-Term Analysis
p_max = df['P'].max()
p_min = df['P'].min()
p_range = p_max - p_min
print(f"P-Term Behavior:")
print(f"  Maximum: {p_max:.2f} (should spike with large errors)")
print(f"  Minimum: {p_min:.2f} (should approach 0 at target)")
print(f"  Range: {p_range:.2f} (shows responsiveness)")
print(f"  ✓ GOOD: P-term starts high and comes to 0 - this is correct!")

# I-Term Analysis  
i_max = df['I'].max()
i_min = df['I'].min()
i_mean = df['I'].mean()
i_std = df['I'].std()
i_range = i_max - i_min
print(f"\nI-Term Behavior:")
print(f"  Maximum: {i_max:.2f} (should stay under ±100 integral limit)")
print(f"  Minimum: {i_min:.2f}")
print(f"  Range: {i_range:.2f} (shows how much it varies)")
print(f"  Mean: {i_mean:.2f} (should be near 0 when system is balanced)")
print(f"  Std Dev: {i_std:.2f} (shows how much it varies)")

# Check if I-term follows P-term (both positive and negative)
i_positive_count = (df['I'] > 0).sum()
i_negative_count = (df['I'] < 0).sum()
i_zero_count = (df['I'] == 0).sum()
print(f"  Positive values: {i_positive_count}/{len(df)} ({i_positive_count/len(df)*100:.1f}%)")
print(f"  Negative values: {i_negative_count}/{len(df)} ({i_negative_count/len(df)*100:.1f}%)")
print(f"  Zero values: {i_zero_count}/{len(df)} ({i_zero_count/len(df)*100:.1f}%)")

if abs(i_max) >= 100 or abs(i_min) >= 100:
    print(f"  ⚠️  WARNING: I-term hitting limits (±100) - possible windup!")
elif i_range > 5:
    print(f"  ✓ GOOD: I-term actively following error direction (range: {i_range:.1f})")
else:
    print(f"  ⚠️  WARNING: I-term range too small ({i_range:.1f}) - consider increasing Ki")

# D-Term Analysis
d_max = df['D'].max()
d_min = df['D'].min()
d_mean = df['D'].mean()
d_std = df['D'].std()
print(f"\nD-Term Behavior:")
print(f"  Maximum: {d_max:.2f} (spikes during rapid changes)")
print(f"  Minimum: {d_min:.2f}")
print(f"  Mean: {d_mean:.2f} (should be near 0 on average)")
print(f"  Std Dev: {d_std:.2f} (shows noise level)")
if d_std > 2.0:
    print(f"  ⚠️  WARNING: High D-term noise - consider lowering Kd")
else:
    print(f"  ✓ GOOD: D-term noise level acceptable")

# CRITICAL DIAGNOSTIC: Check PID output distribution
print("\n=== CRITICAL DIAGNOSTIC ===")
pid_outputs = df['control']
print(f"Raw PID Output Distribution:")
print(f"  < 10: {(abs(pid_outputs) < 10).sum()}/{len(pid_outputs)} ({(abs(pid_outputs) < 10).sum()/len(pid_outputs)*100:.1f}%)")
print(f"  10-20: {((abs(pid_outputs) >= 10) & (abs(pid_outputs) < 20)).sum()}/{len(pid_outputs)} ({((abs(pid_outputs) >= 10) & (abs(pid_outputs) < 20)).sum()/len(pid_outputs)*100:.1f}%)")
print(f"  20-40: {((abs(pid_outputs) >= 20) & (abs(pid_outputs) < 40)).sum()}/{len(pid_outputs)} ({((abs(pid_outputs) >= 20) & (abs(pid_outputs) < 40)).sum()/len(pid_outputs)*100:.1f}%)")
print(f"  40-80: {((abs(pid_outputs) >= 40) & (abs(pid_outputs) < 80)).sum()}/{len(pid_outputs)} ({((abs(pid_outputs) >= 40) & (abs(pid_outputs) < 80)).sum()/len(pid_outputs)*100:.1f}%)")
print(f"  >= 80: {(abs(pid_outputs) >= 80).sum()}/{len(pid_outputs)} ({(abs(pid_outputs) >= 80).sum()/len(pid_outputs)*100:.1f}%)")

print(f"\nPID Output Range: {pid_outputs.min():.2f} to {pid_outputs.max():.2f}")
print(f"This shows what PID controller actually wants to output")

# Calculate Performance Metrics
def calculate_performance_metrics(df):
    """Calculate comprehensive PID performance metrics"""
    
    # Quantitative Error Metrics
    rmse = np.sqrt(np.mean(df['error']**2))
    mae = np.mean(np.abs(df['error']))
    max_abs_error = np.max(np.abs(df['error']))
    std_error = np.std(df['error'])
    
    # Step Response Analysis (detect step changes in target)
    target_changes = np.where(np.abs(np.diff(df['target'])) > 1.0)[0]  # Find significant target changes
    
    metrics = {
        'rmse': rmse,
        'mae': mae,
        'max_abs_error': max_abs_error,
        'std_error': std_error,
        'settling_times': [],
        'overshoots': [],
        'rise_times': [],
        'steady_state_errors': []
    }
    
    # Analyze each step response
    for i, change_idx in enumerate(target_changes):
        try:
            start_idx = change_idx + 1
            end_idx = min(start_idx + 100, len(df) - 1)  # Analyze next 100 samples or until end
            
            if end_idx - start_idx < 10:  # Need at least 10 samples
                continue
                
            step_data = df.iloc[start_idx:end_idx].copy()
            target_val = step_data['target'].iloc[0]
            
            if target_val == 0:  # Skip if target is 0 (no meaningful step)
                continue
            
            # Settling Time (time to reach and stay within 5% of target)
            settling_threshold = 0.05 * abs(target_val)
            within_threshold = np.abs(step_data['error']) <= settling_threshold
            
            if np.any(within_threshold):
                # Find last time it exceeded threshold
                last_exceed = np.where(~within_threshold)[0]
                if len(last_exceed) > 0:
                    settling_idx = last_exceed[-1] + 1
                    # Check bounds before accessing
                    if settling_idx < len(step_data):
                        settling_time = step_data['time'].iloc[settling_idx] - step_data['time'].iloc[0]
                        metrics['settling_times'].append(settling_time)
                else:
                    settling_time = 0  # Already within threshold
                    metrics['settling_times'].append(settling_time)
            
            # Overshoot (maximum deviation beyond target)
            if target_val > 0:  # Positive step
                overshoot = (step_data['position'].max() - target_val) / target_val * 100
            else:  # Negative step
                overshoot = (target_val - step_data['position'].min()) / abs(target_val) * 100
            
            overshoot = max(0, overshoot)  # Only positive overshoot
            metrics['overshoots'].append(overshoot)
            
            # Rise Time (time to go from 10% to 90% of target)
            target_10 = 0.1 * target_val
            target_90 = 0.9 * target_val
            
            if target_val > 0:
                reach_10 = np.where(step_data['position'] >= target_10)[0]
                reach_90 = np.where(step_data['position'] >= target_90)[0]
            else:
                reach_10 = np.where(step_data['position'] <= target_10)[0]
                reach_90 = np.where(step_data['position'] <= target_90)[0]
            
            if len(reach_10) > 0 and len(reach_90) > 0 and reach_90[0] < len(step_data):
                rise_time = step_data['time'].iloc[reach_90[0]] - step_data['time'].iloc[reach_10[0]]
                if rise_time > 0:  # Only add positive rise times
                    metrics['rise_times'].append(rise_time)
            
            # Steady State Error (average error in final 20% of step response)
            final_portion = step_data.iloc[int(0.8 * len(step_data)):]
            if len(final_portion) > 0:
                ss_error = np.mean(np.abs(final_portion['error']))
                metrics['steady_state_errors'].append(ss_error)
                
        except (IndexError, ValueError, KeyError) as e:
            # Skip this step response if there are any calculation errors
            print(f"Warning: Skipping step response {i} due to calculation error: {e}")
            continue
    
    return metrics

# Calculate metrics
performance_metrics = calculate_performance_metrics(df)

print("\n=== Performance Metrics ===")
print(f"RMSE (Root Mean Square Error): {performance_metrics['rmse']:.3f} mm")
print(f"MAE (Mean Absolute Error): {performance_metrics['mae']:.3f} mm")
print(f"Maximum Absolute Error: {performance_metrics['max_abs_error']:.3f} mm")
print(f"Error Standard Deviation: {performance_metrics['std_error']:.3f} mm")

if performance_metrics['settling_times']:
    print(f"Average Settling Time: {np.mean(performance_metrics['settling_times']):.3f} s")
    print(f"Average Overshoot: {np.mean(performance_metrics['overshoots']):.1f} %")
    if performance_metrics['rise_times']:
        print(f"Average Rise Time: {np.mean(performance_metrics['rise_times']):.3f} s")
    if performance_metrics['steady_state_errors']:
        print(f"Average Steady-State Error: {np.mean(performance_metrics['steady_state_errors']):.3f} mm")
else:
    print("No significant step responses detected for dynamic analysis")

# Save statistics to a text file
stats_filename = f"{save_dir}/PID_Statistics_{timestamp}.txt"
with open(stats_filename, 'w', encoding='utf-8') as f:
    f.write("=== PID Analysis Statistics ===\n")
    f.write(f"Test Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"Duration: {duration} seconds\n")
    f.write(f"Data Points: {len(df)} samples\n\n")
    
    # Basic Statistics
    f.write("=== Basic Statistics ===\n")
    f.write(f"PID Output - Min: {df['control'].min():.2f}, Max: {df['control'].max():.2f}, Mean: {df['control'].mean():.2f}\n")
    f.write(f"P Term - Min: {df['P'].min():.2f}, Max: {df['P'].max():.2f}, Mean: {df['P'].mean():.2f}\n")
    f.write(f"I Term - Min: {df['I'].min():.2f}, Max: {df['I'].max():.2f}, Mean: {df['I'].mean():.2f}\n")
    f.write(f"D Term - Min: {df['D'].min():.2f}, Max: {df['D'].max():.2f}, Mean: {df['D'].mean():.2f}\n")
    f.write(f"Error - Min: {df['error'].min():.2f}, Max: {df['error'].max():.2f}, Mean: {df['error'].mean():.2f}\n\n")
    
    # Performance Metrics
    f.write("=== Performance Metrics ===\n")
    f.write(f"RMSE (Root Mean Square Error): {performance_metrics['rmse']:.3f} mm\n")
    f.write(f"MAE (Mean Absolute Error): {performance_metrics['mae']:.3f} mm\n")
    f.write(f"Maximum Absolute Error: {performance_metrics['max_abs_error']:.3f} mm\n")
    f.write(f"Error Standard Deviation: {performance_metrics['std_error']:.3f} mm\n\n")
    
    # Dynamic Response Metrics
    f.write("=== Dynamic Response Metrics ===\n")
    if performance_metrics['settling_times']:
        f.write(f"Average Settling Time: {np.mean(performance_metrics['settling_times']):.3f} s\n")
        f.write(f"Settling Times: {[f'{t:.3f}' for t in performance_metrics['settling_times']]} s\n")
        f.write(f"Average Overshoot: {np.mean(performance_metrics['overshoots']):.1f} %\n")
        f.write(f"Overshoots: {[f'{o:.1f}' for o in performance_metrics['overshoots']]} %\n")
        if performance_metrics['rise_times']:
            f.write(f"Average Rise Time: {np.mean(performance_metrics['rise_times']):.3f} s\n")
            f.write(f"Rise Times: {[f'{t:.3f}' for t in performance_metrics['rise_times']]} s\n")
        if performance_metrics['steady_state_errors']:
            f.write(f"Average Steady-State Error: {np.mean(performance_metrics['steady_state_errors']):.3f} mm\n")
            f.write(f"Steady-State Errors: {[f'{e:.3f}' for e in performance_metrics['steady_state_errors']]} mm\n")
    else:
        f.write("No significant step responses detected for dynamic analysis\n")
    
    # PID Tuning Assessment
    f.write(f"\n=== PID Tuning Assessment ===\n")
    if performance_metrics['rmse'] < 1.0:
        f.write("[OK] Good accuracy (RMSE < 1mm)\n")
    else:
        f.write("[WARNING] Consider improving accuracy (RMSE > 1mm)\n")
    
    if performance_metrics['overshoots'] and np.mean(performance_metrics['overshoots']) > 10:
        f.write("[WARNING] High overshoot - consider reducing Kp or adding Kd\n")
    elif performance_metrics['overshoots']:
        f.write("[OK] Acceptable overshoot\n")
    
    if performance_metrics['settling_times'] and np.mean(performance_metrics['settling_times']) > 2.0:
        f.write("[WARNING] Slow settling - consider increasing Kp or Ki\n")
    elif performance_metrics['settling_times']:
        f.write("[OK] Good settling time\n")

print(f"Saved enhanced statistics: {stats_filename}")

# Save raw data as CSV for further analysis
csv_filename = f"{save_dir}/PID_Data_{timestamp}.csv"
df.to_csv(csv_filename, index=False)
print(f"Saved raw data: {csv_filename}")

# --- Individual PID Terms Plot ---
plt.figure(figsize=(12, 6))
plt.plot(df["time"], p_smooth, label="P Term", linewidth=2.0, linestyle='-', marker=None, antialiased=True, color='green')
plt.plot(df["time"], i_smooth, label="I Term", linewidth=2.0, linestyle='-', marker=None, antialiased=True, color='blue')
plt.plot(df["time"], d_smooth, label="D Term", linewidth=2.0, linestyle='-', marker=None, antialiased=True, color='purple')
plt.axhline(y=0, color='black', linestyle=':', alpha=0.5, linewidth=1)
plt.xlabel("Time (s)")
plt.ylabel("PID Term Value")
plt.title("PID Terms Over Time")
plt.legend()
plt.grid(True, alpha=0.2, linestyle='-', linewidth=0.5)
plt.tight_layout()

# Save the PID terms plot
terms_plot_filename = f"{save_dir}/PID_Terms_{timestamp}.png"
plt.savefig(terms_plot_filename, dpi=300, bbox_inches='tight')
print(f"Saved PID terms plot: {terms_plot_filename}")
plt.show()

print(f"\n=== All files saved in folder: {save_dir} ===")
print("Files created:")
print(f"1. {main_plot_filename}")
print(f"2. {terms_plot_filename}")
print(f"3. {stats_filename}")
print(f"4. {csv_filename}")