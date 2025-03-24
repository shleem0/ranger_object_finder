import serial
import time

# For Raspberry Pi, change the serial port accordingly.
# Common options:
# - Built-in UART: "/dev/ttyAMA0" or "/dev/serial0"
# - USB-to-serial adapter: "/dev/ttyUSB0"
SERIAL_PORT = "/dev/ttyUSB0"  # Update this based on your setup
BAUD_RATE = 9600  # Must match Serial.begin(9600); in your Arduino code
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
    data = f"{P_x},{P_y},{P_z}"
    print(data)
    arduino.write(data.encode())
    print(f"Sent: {data.strip()}")

    # Wait for Arduino response
    time.sleep(1)

    # Read and print response (optional)
    while arduino.in_waiting:
        # print(arduino.readline().decode('CP850', errors='replace').strip())
        print(arduino.readline().decode('ascii', errors='replace').strip())

    # Close connection
    # arduino.close()
    # print("Disconnected.")

except Exception as e:
    print(f"Error: {e}")