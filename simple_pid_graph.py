#!/usr/bin/env python3
"""
Simple PID Terms Visualizer
Shows P, I, D terms over 20 seconds - nothing else!
"""

import serial
import matplotlib.pyplot as plt
import numpy as np
import time
import re

def main():
    # Settings
    port = input("Enter COM port (default: COM3): ").strip() or "COM3"
    
    # Connect to Arduino
    try:
        ser = serial.Serial(port, 9600, timeout=1)
        print(f"Connected to {port}")
        time.sleep(2)
    except:
        print(f"Failed to connect to {port}")
        return
    
    # PID parameters (from your Arduino code)
    kp = 1.7
    ki = 0.0  # Currently 0
    kd = 0.0  # Currently 0
    
    # Data storage
    times = []
    errors = []
    p_terms = []
    i_terms = []
    d_terms = []
    
    # PID calculation variables
    integral = 0
    last_error = 0
    last_time = 0
    
    print("Collecting PID data for 20 seconds...")
    print("Apply force to see PID response!")
    
    start_time = time.time()
    
    # Collect data for 20 seconds
    while time.time() - start_time < 20:
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                
                # Look for Target rot and Rotations in the output
                target_match = re.search(r'Target rot:\s*([-\d.]+)', line)
                actual_match = re.search(r'Rotations:\s*([-\d.]+)', line)
                
                if target_match and actual_match:
                    # Calculate error
                    target = float(target_match.group(1))
                    actual = float(actual_match.group(1))
                    error = target - actual
                    
                    current_time = time.time() - start_time
                    dt = current_time - last_time if last_time > 0 else 0.02
                    
                    # Calculate PID terms
                    p_term = kp * error
                    
                    if dt > 0:
                        integral += error * dt
                    i_term = ki * integral
                    
                    if dt > 0 and last_error is not None:
                        derivative = (error - last_error) / dt
                    else:
                        derivative = 0
                    d_term = kd * derivative
                    
                    # Store data
                    times.append(current_time)
                    errors.append(error)
                    p_terms.append(p_term)
                    i_terms.append(i_term)
                    d_terms.append(d_term)
                    
                    # Print progress
                    print(f"Time: {current_time:.1f}s | Error: {error:.2f} | P: {p_term:.1f}")
                    
                    # Update for next iteration
                    last_error = error
                    last_time = current_time
                    
        except KeyboardInterrupt:
            break
        except:
            continue
    
    ser.close()
    
    # Create the simple graph
    if len(times) > 5:
        plt.figure(figsize=(10, 6))
        
        plt.plot(times, p_terms, 'r-', linewidth=2, label=f'P term (Kp={kp})')
        plt.plot(times, i_terms, 'g-', linewidth=2, label=f'I term (Ki={ki})')
        plt.plot(times, d_terms, 'b-', linewidth=2, label=f'D term (Kd={kd})')
        
        plt.xlabel('Time (seconds)')
        plt.ylabel('PID Terms')
        plt.title('PID Terms Over 20 Seconds')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.xlim(0, 20)
        
        # Save and show
        plt.savefig('pid_terms.png', dpi=150, bbox_inches='tight')
        print(f"\nGraph saved as 'pid_terms.png'")
        print(f"Collected {len(times)} data points")
        
        plt.show()
    else:
        print("Not enough data collected. Make sure Arduino is sending data.")

if __name__ == "__main__":
    main()
