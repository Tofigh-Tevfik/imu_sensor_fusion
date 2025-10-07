import socket
import struct
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import time

ESP32_IP = "192.168.1.110"   # ESP32 IP
ESP32_PORT = 1234           # same as udp.begin() in ESP32

UDP_IP = "0.0.0.0"  # Listen on all interfaces for testing
UDP_PORT = 3333

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # non-blocking mode

FILTER1 = 0x04
# figure titles
match FILTER1:
    case 0x00:
        filter1 = "NoFilter"
    case 0x01:
        filter1 = "LowPass"
    case 0x02:
        filter1 = "Complementary"
    case 0x03:
        filter1 = "Madgwick"
    case 0x04:
        filter1 = "Mahony"
    case 0x05:
        filter1 = "EKF"

FILTER2 = 0x05
# figure titles
match FILTER2:
    case 0x00:
        filter2 = "NoFilter"
    case 0x01:
        filter2 = "LowPass"
    case 0x02:
        filter2 = "Complementary"
    case 0x03:
        filter2 = "Madgwick"
    case 0x04:
        filter2 = "Mahony"
    case 0x05:
        filter2 = "EKF"

config_byte = (FILTER1 << 4) | FILTER2
msg = bytes([0xAA, config_byte])
sock.sendto(msg, (ESP32_IP, ESP32_PORT))

print("Config sent, waiting for ACK...")
while True:
    try:
        data, addr = sock.recvfrom(16)
        if data[0] == 0x55:
            print("ACK received, starting streaming...")
            break
    except BlockingIOError:
        time.sleep(0.1)


print(f"Listening on {UDP_IP}:{UDP_PORT}")

# creatue new figure
fig1 = plt.figure()
# adds an axes to the current figure -> returns axes
ax1 = fig1.add_subplot(121, projection='3d')
ax2 = fig1.add_subplot(122, projection='3d')
ax1.set_xlim([-1, 1])
ax1.set_ylim([-1, 1])
ax1.set_zlim([-1, 1])
ax2.set_xlim([-1, 1])
ax2.set_ylim([-1, 1])
ax2.set_zlim([-1, 1])
# plots 3d field of arros
x1_arrow = ax1.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
y1_arrow = ax1.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
z1_arrow = ax1.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')
x2_arrow = ax2.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
y2_arrow = ax2.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
z2_arrow = ax2.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')
ax1.legend()
ax2.legend()
plt.ion() # enable interactive mode for real-time updates

while True:
    latest_data = None
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            latest_data = data
        except BlockingIOError:
            break  # no more data in buffer

    if latest_data is not None and latest_data[0] == 0xAA:
        # process only the most recent packet
        seq = struct.unpack('<I', latest_data[1:5])[0]
        ts = struct.unpack('<I', latest_data[5:9])[0]
        q1_0, q1_1, q1_2, q1_3 = struct.unpack('<ffff', latest_data[9:25])
        q2_0, q2_1, q2_2, q2_3 = struct.unpack('<ffff', latest_data[25:41])

        rot1 = R.from_quat([q1_1, q1_2, q1_3, q1_0])
        rot2 = R.from_quat([q2_1, q2_2, q2_3, q2_0])

        # rotate basis vectors
        x1_vec, y1_vec, z1_vec = rot1.apply([[1,0,0], [0,1,0], [0,0,1]])
        x2_vec, y2_vec, z2_vec = rot2.apply([[1,0,0], [0,1,0], [0,0,1]])

        # update arrows
        x1_arrow.set_segments([[[0, 0, 0], x1_vec]])
        y1_arrow.set_segments([[[0, 0, 0], y1_vec]])
        z1_arrow.set_segments([[[0, 0, 0], z1_vec]])
        x2_arrow.set_segments([[[0, 0, 0], x2_vec]])
        y2_arrow.set_segments([[[0, 0, 0], y2_vec]])
        z2_arrow.set_segments([[[0, 0, 0], z2_vec]])

        ax1.set_title(filter1)
        ax2.set_title(filter2)

        plt.draw()
        plt.pause(0.001)

    # throttle to ~50 ms refresh
    time.sleep(0.05)

