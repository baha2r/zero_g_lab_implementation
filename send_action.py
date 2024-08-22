import socket
import numpy as np
from create_action import return_action
import time

# Define the server address and port
server_address = ('192.168.88.11', 65432)

# Create a TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:

    # Create a NumPy array with 6 elements
    array = return_action()

    # Convert the array to bytes
    array_bytes = array.tobytes()

    # Connect to the server
    sock.connect(server_address)
    
    # Send the array size first
    sock.sendall(len(array_bytes).to_bytes(4, byteorder='big'))
    
    # Send the array bytes
    sock.sendall(array_bytes)

    time.sleep(1)