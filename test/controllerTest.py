import pygame
import serial
import time

# --- Settings ---
SERIAL_PORT = 'COM3'  # Change to your Teensy port (e.g., /dev/ttyACM0 on Linux)
BAUD_RATE = 115200
DEADZONE = 0.1

# Initialize Serial and Pygame
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found!")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()

print(f"Bridge Active. Sending data to {SERIAL_PORT}...")

try:
    while True:
        pygame.event.pump()

        # 1. Read Joysticks (Range -1.0 to 1.0)
        az_input = controller.get_axis(2)  # Right Stick X
        el_input = -controller.get_axis(3) # Right Stick Y (Inverted)

        # Apply Deadzone
        if abs(az_input) < DEADZONE: az_input = 0
        if abs(el_input) < DEADZONE: el_input = 0

        # Scale to Radians/sec (Adjust 0.8 to change max speed)
        az_speed = az_input * 0.8
        el_speed = el_input * 0.8

        # 2. Check Buttons for State Changes
        # Button A=0, B=1, X=2, Y=3 in pygame
        command = f"V,{az_speed:.3f},{el_speed:.3f}\n"
        
        if controller.get_button(0): command = "S,MANUAL\n"
        elif controller.get_button(1): command = "S,TRACKING\n"
        elif controller.get_button(3): command = "S,IDLE\n"

        # 3. Send to Teensy
        ser.write(command.encode())

        print(command)
        
        time.sleep(0.02) # 50Hz update rate

except KeyboardInterrupt:
    ser.close()
    print("Bridge stopped.")