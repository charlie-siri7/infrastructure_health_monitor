import serial
import time
import threading

# Configuration - Change COM3 to your port (e.g., '/dev/ttyUSB0' on Mac/Linux)
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

class ControlBridge:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.running = True
        
    def listen_to_sensors(self):
        """Background thread to catch telemetry from ESP32"""
        while self.running:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"TELEMETRY: {line}")
                    # In a real app, you would parse this data to update your GUI

    def send_command(self, target_angle):
        """Sends a command back to the ESP32 to move the servo"""
        command = f"SET_ANGLE:{target_angle}\n"
        self.ser.write(command.encode())
        print(f"COMMAND SENT: {command.strip()}")

# Execution
if __name__ == "__main__":
    bridge = ControlBridge()
    
    # Start listening in the background
    thread = threading.Thread(target=bridge.listen_to_sensors, daemon=True)
    thread.start()

    try:
        while True:
            # Example: User types an angle in the console to control the hardware
            val = input("Enter servo angle (0-180) or 'q' to quit: ")
            if val.lower() == 'q':
                break
            if val.isdigit():
                bridge.send_command(val)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.running = False
        print("Closing Connection...")