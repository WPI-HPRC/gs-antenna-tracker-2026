import pygame
import serial
import time

# --- Settings ---
SERIAL_PORT = 'COM3'  # Update this to your verified port
BAUD_RATE = 115200
DEADZONE = 0.1 # Pro Controllers sometimes have slight drift; 0.15 is safer

# Initialize Serial and Pygame
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print(f"Serial Error: {e}. Is the VS Code Serial Monitor closed?")
    exit()

pygame.init()
pygame.joystick.init()

# Find the Pro Controller specifically
controller = None
for i in range(pygame.joystick.get_count()):
    js = pygame.joystick.Joystick(i)
    js.init()
    if "Nintendo" in js.get_name() or "Pro Controller" in js.get_name():
        controller = js
        print(f"Connected to: {js.get_name()}")
        break

if not controller:
    print("Nintendo Pro Controller not found! Using first available controller.")
    if pygame.joystick.get_count() > 0:
        controller = pygame.joystick.Joystick(0)
        controller.init()
    else:
        exit()

print(f"Bridge Active. Sending data to {SERIAL_PORT}...")

try:
    while True:
        pygame.event.pump()

        # --- Axis Mapping for Nintendo Pro Controller ---
        # Axis 0: Left Stick X
        # Axis 1: Left Stick Y
        # Axis 2: Right Stick X (Azimuth)
        # Axis 3: Right Stick Y (Elevation)
        
        az_input = -controller.get_axis(0)  
        el_input = -controller.get_axis(1) # Inverted so pushing up is positive elevation

        # Apply Deadzone
        if abs(az_input) < DEADZONE: az_input = 0
        if abs(el_input) < DEADZONE: el_input = 0

        # Scale to Radians/sec
        az_speed = az_input * 0.8
        el_speed = el_input * 0.8

        # --- Button Mapping for Nintendo Pro Controller ---
        # Note: Pro Controller buttons are often mapped differently than Xbox
        # B = 0, A = 1, Y = 2, X = 3
        
        command = f"V,{az_speed:.3f},{el_speed:.3f}\n"
        
        if controller.get_button(1):   # 'A' Button
            command = "S,IDLE\n"
        elif controller.get_button(0): # 'B' Button
            command = "S,TRACKING\n"

        # Send to Teensy
        ser.write(command.encode())

        # Debugging: Print outgoing command and any incoming Teensy messages
        print(f"Outgoing: {command.strip()}")
        
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Teensy: {line}")
        
        time.sleep(0.02) # 50Hz

except KeyboardInterrupt:
    ser.close()
    print("\nBridge stopped.")