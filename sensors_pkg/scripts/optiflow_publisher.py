# import serial
# import struct
# import time

# # Open serial port
# ser = serial.Serial('/dev/ttyAMA1', baudrate=115200, timeout=1)

# # Function to parse MultiWii Serial Protocol v2 messages
# def parse_msp_v2(data):
#     # Implement parsing logic according to MSP v2 specification
#     # Extract optical flow data from the message
#     pass

# # Initialize lists to store data
# timestamps = []
# optical_flow_values = []

# try:
#     while True:
#         # Read line from serial port
#         line = ser.read_until('$')
#         valueInstring = str(line)
#         time.sleep(1)
#         print(line)
#         # Parse MSP v2 message
#         # data = parse_msp_v2(line)
# except KeyboardInterrupt:
#     ser.close()

import serial
import struct
import time

# Open serial port
ser = serial.Serial('/dev/ttyAMA1', baudrate=115200, timeout=0.1)

# Function to parse MultiWii Serial Protocol v2 messages
def parse_msp_v2(data):

    # Check if the data starts with a known MSP header
    if len(data) == 0:
        print("Length of data is 0")
        return None  # Invalid message

    # Check if the data starts with a known MSP header
    if data[0] != b'$':
        print("Invalid message, data[0] not equal to $")
        return None  # Invalid message
    
    # Extract the message length if available
    if len(data) < 2:
        print("Length of data is less than 2")
        return None  # Incomplete message

    # Extract the message length
    msg_length = data[1]

    # Verify if the message length is correct
    if len(data) < msg_length + 6:
        return None  # Incomplete message

    # Extract the message ID
    msg_id = data[2]

    # Extract the data payload
    payload = data[3:3 + msg_length]

    # Parse different message types based on msg_id
    if msg_id == b'\x19':  # Example message ID, replace with actual ID
        # Parse optical flow data from the payload
        # Example: parsing two 16-bit integers representing X and Y flow
        optical_flow_data = struct.unpack('<hh', payload)
        return optical_flow_data  # Return parsed data

    # Add more message ID checks and parsing logic for other message types if needed

    return None  # Unknown message

# Initialize lists to store data
timestamps = []
optical_flow_values = []

try:
    while True:
        # Read line from serial port
        line = ser.read_until(b'$')  # Read until the end marker ($)
        #line = ser.read(14)
        #line = ser.readline()
        #line = b'X<\x00\x01\x1f\x05\x00\xff\x0c\x00\x00\x00\x06$'

        if line:
            # Parse MSP v2 message
            # for c in line:
            #     print(c, end=" ")

            print(line)

            data = None#parse_msp_v2(line)
            if data:
                # Example: Assuming the parsed data contains optical flow values
                optical_flow_values.append(data)
                timestamps.append(time.time())  # Store timestamp
                print("Optical flow data:", data)
except KeyboardInterrupt:
    ser.close()
