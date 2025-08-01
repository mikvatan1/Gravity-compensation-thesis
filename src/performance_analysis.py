#!/usr/bin/env python3
"""
Performance Analysis for Arduino Gravity Compensation System
Analyzes the expected loop timing with various components
"""

# Component timing estimates (microseconds)
timing_estimates = {
    "I2C_read_AS5600": 125,        # ~800kHz I2C, 12-bit read
    "ADC_read": 100,               # Analog read ~100μs on Arduino
    "Math_calculations": 50,       # Float arithmetic, PID compute
    "LED_strip_update": 4320,      # 144 LEDs × 30μs per LED (WS2812B)
    "PWM_writes": 10,              # Digital/analog writes
    "Serial_output": 800,          # When printing (every 50ms)
    "Other_overhead": 50           # Loop overhead, function calls
}

print("=== Arduino Loop Performance Analysis ===\n")

# Calculate timing for different scenarios
scenarios = {
    "Minimal (no hardware)": ["Math_calculations", "Other_overhead"],
    "Hardware simulation": ["Math_calculations", "Other_overhead", "LED_strip_update"],
    "Real hardware (no serial)": ["I2C_read_AS5600", "ADC_read", "Math_calculations", 
                                   "LED_strip_update", "PWM_writes", "Other_overhead"],
    "Real hardware (with serial)": ["I2C_read_AS5600", "ADC_read", "Math_calculations", 
                                     "LED_strip_update", "PWM_writes", "Serial_output", "Other_overhead"]
}

for scenario_name, components in scenarios.items():
    total_time = sum(timing_estimates[comp] for comp in components)
    frequency = 1000000 / total_time if total_time > 0 else 0  # Convert to Hz
    
    print(f"{scenario_name}:")
    print(f"  Total loop time: {total_time} μs ({total_time/1000:.1f} ms)")
    print(f"  Loop frequency: {frequency:.1f} Hz")
    
    if total_time > 50000:  # 50ms threshold
        print(f"  ⚠️  WARNING: Loop time exceeds 50ms!")
    elif total_time > 20000:  # 20ms threshold  
        print(f"  ⚠️  CAUTION: Loop time over 20ms")
    else:
        print(f"  ✅ Good performance")
    print()

print("=== LED Strip Impact Analysis ===")
led_time = timing_estimates["LED_strip_update"]
total_without_leds = sum(timing_estimates[comp] for comp in ["I2C_read_AS5600", "ADC_read", "Math_calculations", "PWM_writes", "Other_overhead"])

print(f"LED strip update time: {led_time} μs ({led_time/1000:.1f} ms)")
print(f"Rest of loop: {total_without_leds} μs ({total_without_leds/1000:.1f} ms)")
print(f"LED impact: {(led_time/(led_time+total_without_leds))*100:.1f}% of total loop time")

print("\n=== Optimization Recommendations ===")
if led_time > total_without_leds:
    print("1. LED strip is the major bottleneck")
    print("2. Consider updating LEDs only on state changes")
    print("3. Or use a separate timer/interrupt for LED updates")
    print("4. Current approach updates 144 LEDs every loop cycle")

print(f"\n=== Your Current Setup ===")
print(f"LED count: 144 LEDs")
print(f"LED brightness: 5 (already optimized)")
print(f"I2C speed: 800kHz (optimized)")
print(f"Serial throttling: Every 50ms (optimized)")
