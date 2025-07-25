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
    except Exception as e:
        print(f"Failed to connect to {port}: {e}")
        print("Try unplugging and reconnecting Arduino, or check if another program is using the port")
        return
    
    # PID parameters (from your Arduino code)
    kp = 2.3  # Updated for 15kg max force
    ki = 0.1  # Updated to match Arduino
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
                
                # Debug: Print what we're receiving
                if line:
                    print(f"Received: {line}")
                
                # Look for Target rot and Rotations in the output
                target_match = re.search(r'Target rot:\s*([-\d.]+)', line)  # ← SEARCHES for "Target rot: X.XX" pattern
                actual_match = re.search(r'Rotations:\s*([-\d.]+)', line)   # ← SEARCHES for "Rotations: X.XX" pattern
                
                if target_match and actual_match:
                    # Calculate error
                    target = float(target_match.group(1))  # ← EXTRACTS: targetRotations value from Arduino (force → target position)
                    actual = float(actual_match.group(1))  # ← EXTRACTS: rotationCounter value from Arduino (AS5600 encoder position)
                    error = target - actual
                    
                    current_time = time.time() - start_time
                    dt = current_time - last_time if last_time > 0 else 0.02
                    
                    # Calculate PID terms
                    p_term = kp * error                   # P: Proportional to current error (immediate response)
                    
                    if dt > 0:
                        integral += error * dt            # Accumulate error over time for I term
                    i_term = ki * integral                # I: Integral of accumulated error (eliminates steady-state error)
                    
                    if dt > 0 and last_error is not None:
                        derivative = (error - last_error) / dt  # Rate of change of error
                    else:
                        derivative = 0
                    d_term = kd * derivative              # D: Derivative of error (reduces overshoot)
                    
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
            print("\nStopped by user")
            break
        except Exception as e:
            # Silently continue on errors
            continue
    
    # Always close serial port
    try:
        ser.close()
        print("Serial port closed")
    except:
        pass
    


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
        
        # Use non-blocking show to prevent freezing
        plt.show(block=False)
        plt.pause(1)  # Keep window open briefly
        
        # Ask user if they want to keep the plot open
        try:
            input("Press Enter to close the plot and exit...")
        except:
            pass
        plt.close('all')  # Close all matplotlib windows
    else:
        print("Not enough data collected. Make sure Arduino is sending data.")

if __name__ == "__main__":
    main()
