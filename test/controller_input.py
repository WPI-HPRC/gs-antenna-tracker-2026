import pygame
import serial
import time

# --- Settings ---
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200
DEADZONE = 0.12  # Slightly wider for better centering
UPDATE_RATE = 0.05 # 20Hz is plenty (0.05s) and much more stable for Serial

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

# Find the Pro Controller
controller = None
for i in range(pygame.joystick.get_count()):
    js = pygame.joystick.Joystick(i)
    js.init()
    if "Nintendo" in js.get_name() or "Pro Controller" in js.get_name():
        controller = js
        print(f"Connected to: {js.get_name()}")
        break

if not controller:
    print("Nintendo Pro Controller not found! Using first available.")
    if pygame.joystick.get_count() > 0:
        controller = pygame.joystick.Joystick(0)
        controller.init()
    else:
        exit()

# State tracking to prevent serial flooding
last_az = 0.0
last_el = 0.0
last_state_cmd = ""

print(f"Bridge Active. Sending data to {SERIAL_PORT}...")

try:
    while True:
        pygame.event.pump()

        # 1. Axis Input
        az_input = -controller.get_axis(0)  
        el_input = -controller.get_axis(1) 

        # 2. Apply Deadzone
        if abs(az_input) < DEADZONE: az_input = 0
        if abs(el_input) < DEADZONE: el_input = 0

        az_speed = az_input * 0.8
        el_speed = el_input * 0.8

        # 3. Check for State Buttons (A/B)
        # We only want to send these ONCE per press
        current_command = None
        if controller.get_button(1):   # 'A' Button -> IDLE
            if last_state_cmd != "IDLE":
                current_command = "S,IDLE\n"
                last_state_cmd = "IDLE"
        elif controller.get_button(0): # 'B' Button -> REMOTE
            if last_state_cmd != "REMOTE":
                current_command = "S,REMOTE\n"
                last_state_cmd = "REMOTE"
        else:
            # If no button is pressed, we allow velocity updates
            # Only send V if the stick has moved more than 1% to save bandwidth
            if abs(az_input - last_az) > 0.01 or abs(el_input - last_el) > 0.01:
                current_command = f"V,{az_speed:.3f},{el_speed:.3f}\n"
                last_az = az_input
                last_el = el_input

        # 4. Attempt to Write
        if current_command:
            try:
                ser.write(current_command.encode())
                print(f"Outgoing: {current_command.strip()}")
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