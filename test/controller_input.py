import math
import pygame
import serial
import time

# --- Settings ---
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200
DEADZONE = 0.15      # Slightly wider for better centering
UPDATE_RATE = 0.05   # 20Hz is plenty (0.05s) and much more stable for Serial

# --- Universal Controller Mapping ---
# Pygame 2+ (via SDL2) standardizes most controllers to this layout.
# Physical Button Placements:
BTN_BOTTOM = 0  # Xbox 'A', PlayStation 'Cross', Switch 'B'
BTN_RIGHT  = 1  # Xbox 'B', PlayStation 'Circle', Switch 'A'
BTN_LEFT   = 2  # Xbox 'X', PlayStation 'Square', Switch 'Y'
BTN_TOP    = 3  # Xbox 'Y', PlayStation 'Triangle', Switch 'X'

# Axis Assignments:
AXIS_AZIMUTH = 0   # Left Stick X-Axis
AXIS_ELEVATION = 3 # Right Stick Y-Axis (Note: On some OS/drivers, this might be Axis 4)

# Initialize Serial with write timeouts
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05, write_timeout=0.05)
    # Important for Teensy/Windows handshake
    ser.dtr = True
    ser.rts = True
except Exception as e:
    print(f"Serial Error: {e}. Ensure VS Code Monitor is CLOSED.")
    exit()

pygame.init()
pygame.joystick.init()

# Find the First Available Controller
if pygame.joystick.get_count() == 0:
    print("No controllers found! Please plug in a controller (PS, Xbox, generic) and restart.")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Connected to Universal Controller: {controller.get_name()}")

# State tracking to prevent serial flooding
last_az = 0.0
last_el = 0.0
last_state_cmd = ""

print(f"Bridge Active. Sending data to {SERIAL_PORT}...")

try:
    while True:
        pygame.event.pump()

        # 1. Axis Input (Using mapped variables)
        az_input = -controller.get_axis(AXIS_AZIMUTH)  
        el_input = -controller.get_axis(AXIS_ELEVATION) 

        # 2. Apply Deadzone
        if abs(az_input) < DEADZONE: az_input = 0
        if abs(el_input) < DEADZONE: el_input = 0

        az_speed = az_input**3 * math.pi
        el_speed = el_input**3 * math.pi

        # 3. Check for State Buttons (Mapped to physical positions)
        current_command = None
        
        if controller.get_button(BTN_RIGHT):   # Right Button -> REMOTE
            if last_state_cmd != "REMOTE":
                current_command = "S,REMOTE\n"
                last_state_cmd = "REMOTE"
        elif controller.get_button(BTN_BOTTOM): # Bottom Button -> IDLE
            if last_state_cmd != "IDLE":
                current_command = "S,IDLE\n"
                last_state_cmd = "IDLE"
        elif controller.get_button(BTN_TOP):   # Top Button -> TRACKING
            if last_state_cmd != "TRACKING":
                current_command = "S,TRACKING\n"
                last_state_cmd = "TRACKING"
        elif controller.get_button(BTN_LEFT):  # Left Button -> CALIBRATE
            if last_state_cmd != "CALIBRATE":
                current_command = "S,CALIBRATE\n"
                last_state_cmd = "CALIBRATE"
        else:
            # If no button is pressed, allow velocity updates
            # Only send V if the stick has moved more than 1% to save bandwidth
            if abs(az_input - last_az) > 0.01 or abs(el_input - last_el) > 0.01:
                current_command = f"V,{az_speed:.3f},{el_speed:.3f}\n"
                last_az = az_input
                last_el = el_input

        # 4. Attempt to Write
        if current_command:
            try:
                ser.write(current_command.encode())
            except serial.SerialTimeoutException:
                print("TX Timeout: Teensy is busy/buffer full")
            except Exception as e:
                print(f"Write Error: {e}")

        # 5. Non-blocking Read from Teensy
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line: print(f"Teensy: {line}")
            except:
                pass
        
        time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    ser.close()
    print("\nBridge stopped.")