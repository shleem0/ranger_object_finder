import serial
import time

# Replace with your actual serial port (run `ls /dev/tty.*` to check)
SERIAL_PORT = "/dev/tty.usbmodem14101"  # Change this based on your Mac's device
BAUD_RATE = 9600  # Must match `Serial.begin(9600);` in Arduino
TIMEOUT = 1  # Optional timeout for safety

try:
    # Open serial connection
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    time.sleep(2)  # Allow time for Arduino to reset

    # Define coordinates
    P_x = 15.0
    P_y = -10.0
    P_z = -20.0

    # Format and send data
    data = f"{P_x},{P_y},{P_z}\n"
    arduino.write(data.encode())
    print(f"Sent: {data.strip()}")

    # Wait for Arduino response
    time.sleep(1)

    # Read and print response (optional)
    while arduino.in_waiting:
        print(arduino.readline().decode().strip())

    # Close connection
    arduino.close()
    print("Disconnected.")

except Exception as e:
    print(f"Error: {e}")
