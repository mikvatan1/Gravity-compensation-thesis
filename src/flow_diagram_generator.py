import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import numpy as np

# Create figure and axis
fig, ax = plt.subplots(1, 1, figsize=(14, 20))
ax.set_xlim(0, 10)
ax.set_ylim(0, 24)
ax.axis('off')

# Define box styles
process_style = dict(boxstyle="round,pad=0.1", facecolor='lightblue', edgecolor='black', linewidth=1.5)
decision_style = dict(boxstyle="round,pad=0.1", facecolor='yellow', edgecolor='black', linewidth=1.5)
terminal_style = dict(boxstyle="round,pad=0.1", facecolor='lightgreen', edgecolor='black', linewidth=1.5)
input_style = dict(boxstyle="round,pad=0.1", facecolor='lightcoral', edgecolor='black', linewidth=1.5)

# Function to create a box with text
def create_box(x, y, width, height, text, style, fontsize=9):
    box = FancyBboxPatch((x-width/2, y-height/2), width, height, **style)
    ax.add_patch(box)
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize, weight='bold', wrap=True)

# Function to create arrows
def create_arrow(x1, y1, x2, y2, text=''):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', lw=1.5, color='black'))
    if text:
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mid_x + 0.3, mid_y, text, ha='center', va='center', fontsize=8, 
                bbox=dict(boxstyle="round,pad=0.1", facecolor='white', alpha=0.8))

# Create the flowchart
y_pos = 23

# START
create_box(5, y_pos, 2, 0.8, 'START\n(setup())', terminal_style)
y_pos -= 1.5

# Setup initialization
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, 'Initialize Hardware\n(Motors, Sensors, LEDs)', process_style)
y_pos -= 1.5

# Loop start
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 2, 0.8, 'LOOP START', terminal_style)
y_pos -= 1.5

# Time check
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, 'Time > 10s?', decision_style)
create_arrow(6.5, y_pos, 8.5, y_pos, 'YES')
create_box(8.5, y_pos, 2, 0.8, 'Set\nreturnToStart\n= true', process_style)
y_pos -= 1.5

# Finished check
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, 'finished\n= true?', decision_style)
create_arrow(6.5, y_pos, 8.5, y_pos, 'YES')
create_box(8.5, y_pos, 2, 0.8, 'STOP\nMOTORS\nEND', terminal_style)
y_pos -= 1.5

# LED update
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, 'Update LEDs\nif pending', process_style)
y_pos -= 1.5

# Read position
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, 'Read AS5600\nPosition Sensor', input_style)
y_pos -= 1.5

# Process position
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3.5, 0.8, 'Calculate Rotation\n& Apply Low-Pass Filter', process_style)
y_pos -= 1.5

# Manual force selection
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3.5, 1, 'Manual Force Selection\nTime < 5s: Load1\nTime ≥ 5s: Load2', input_style)
y_pos -= 1.5

# Calculate targets
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3.5, 1, 'Calculate:\na_actual = start + rotation\na_target = force/k\nerror = target - actual', process_style)
y_pos -= 1.5

# Return to start check
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, 'returnToStart\n= true?', decision_style)
create_arrow(6.5, y_pos, 8.5, y_pos, 'YES')
create_box(8.5, y_pos, 2.5, 0.8, 'Override:\nerror = start - actual', process_style)
y_pos -= 1.5

# Check if at start position
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, '|error| < 2mm\n& returnToStart?', decision_style)
create_arrow(6.5, y_pos, 8.5, y_pos, 'YES')
create_box(8.5, y_pos, 2, 0.8, 'Set\nfinished\n= true', process_style)
y_pos -= 1.5

# Main control decision
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 3, 0.8, '|error| > 2mm?', decision_style, fontsize=10)

# YES branch - Motor control
create_arrow(6.5, y_pos, 8, y_pos, 'YES')
create_box(8, y_pos - 0.5, 2, 0.8, 'Enable Motors\nSet LEDs Red', process_style)
create_box(8, y_pos - 1.5, 2.5, 0.8, 'PID Calculation\noutput = PID(target, actual)', process_style)
create_box(8, y_pos - 2.5, 2.5, 0.8, 'Set PWM Direction\nR_PWM or L_PWM', process_style)

# NO branch - Deadband
create_arrow(3.5, y_pos, 2, y_pos, 'NO')
create_box(2, y_pos - 0.5, 2, 0.8, 'Disable Motors\nSet LEDs Green', process_style)
create_box(2, y_pos - 1.5, 2.5, 0.8, 'Deadband Timer\nReset PID after 2s', process_style)

y_pos -= 3.5

# Print data
create_arrow(5, y_pos + 2.6, 5, y_pos + 0.1)
create_arrow(8, y_pos + 2.1, 5, y_pos + 0.5)
create_arrow(2, y_pos + 2.1, 5, y_pos + 0.5)
create_box(5, y_pos, 3.5, 1, 'Print Data Every 100ms\n(CSV Format)', process_style)
y_pos -= 1.5

# Loop back
create_arrow(5, y_pos + 0.9, 5, y_pos + 0.1)
create_box(5, y_pos, 2, 0.8, 'LOOP BACK', terminal_style)

# Create loop back arrow
ax.annotate('', xy=(1, 20.5), xytext=(1, y_pos),
            arrowprops=dict(arrowstyle='->', lw=1.5, color='black'))
ax.annotate('', xy=(5, 20.5), xytext=(1, 20.5),
            arrowprops=dict(arrowstyle='->', lw=1.5, color='black'))

# Add title
ax.text(5, 24.5, 'Gravity Compensation Control System - Flow Diagram', 
        ha='center', va='center', fontsize=16, weight='bold')

# Add legend
legend_elements = [
    mpatches.Patch(color='lightblue', label='Process'),
    mpatches.Patch(color='yellow', label='Decision'),
    mpatches.Patch(color='lightgreen', label='Start/End'),
    mpatches.Patch(color='lightcoral', label='Input/Sensor')
]
ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))

plt.tight_layout()
plt.savefig('main_cpp_flow_diagram.png', dpi=300, bbox_inches='tight', facecolor='white')
plt.savefig('main_cpp_flow_diagram.svg', bbox_inches='tight', facecolor='white')
plt.show()

print("Flow diagram saved as 'main_cpp_flow_diagram.png' and 'main_cpp_flow_diagram.svg'")
