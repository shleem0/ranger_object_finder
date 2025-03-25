import serial
import time

# For Raspberry Pi, change the serial port accordingly.
# Common options:
# - Built-in UART: "/dev/ttyAMA0" or "/dev/serial0"
# - USB-to-serial adapter: "/dev/ttyUSB0"
SERIAL_PORT = "/dev/ttyACM0"  # Update this based on your setup
BAUD_RATE = 9600  # Must match Serial.begin(9600) in your Arduino code
TIMEOUT = 1  # Optional timeout for safety

def send_coordinates(P_x, P_y, P_z = -20.0):
    """
    Sends the given coordinates (in cm) to the Arduino over a serial connection.
    """
    try:
        # Open serial connection
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2)  # Allow time for Arduino to reset

        # Format and send data
        data = f"{P_x},{P_y},{P_z}\n"
        print(data)
        arduino.write(data.encode('ascii'))
        print(f"Sent: {data}")

        # Wait for Arduino response
        time.sleep(1)

        # Read and print response (optional)
        while arduino.in_waiting:
            print(arduino.readline().decode('ascii', errors='replace').strip())

        # Optionally, close connection
        # arduino.close()
        # print("Disconnected.")

    except Exception as e:
        print(f"Error: {e}")

# Test usage
if __name__ == "__main__":
    send_coordinates(15.0, -10.0, -20.0)