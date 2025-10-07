import socket

ESP32_IP = "192.168.1.110"  # replace with the IP printed by ESP32
ESP32_PORT = 3333

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ESP32_IP, ESP32_PORT))

print("Connected to ESP32, receiving data...")

try:
    while True:
        data = sock.recv(1024)  # receive up to 1024 bytes
        if not data:
            break
        print(data.decode().strip())
except KeyboardInterrupt:
    print("Disconnected.")
finally:
    sock.close()
