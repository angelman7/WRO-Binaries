def read_from_esp32(serial_port):
    if serial_port.inWaiting() > 0:
        data = serial_port.read()
    
    return data

def send_to_esp32(serial_port, message):
    serial_port.write(message)
    if message == "\r".encode():
        serial_port.write("\n".encode())