import serial
import struct
import time

# Open serial port
ser = serial.Serial('/dev/ttyAMA1', baudrate=115200, timeout=1)

# Function to parse MultiWii Serial Protocol v2 messages
def parse_msp_v2(data):
    # Implement parsing logic according to MSP v2 specification
    # Extract optical flow data from the message
    pass

# Initialize lists to store data
timestamps = []
optical_flow_values = []

try:
    while True:
        # Read line from serial port
        line = ser.read_until('$')
        valueInstring = str(line)
        time.sleep(1)
        print(line)
        # Parse MSP v2 message
        # data = parse_msp_v2(line)
except KeyboardInterrupt:
    ser.close()
