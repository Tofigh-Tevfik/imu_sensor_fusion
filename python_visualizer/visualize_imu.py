import serial
import struct
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import time


# configuring serial ports and baudrates
try:
    ser = serial.Serial('COM3', 921600, timeout=0.1)
    print("Serial port opened successfully.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

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
    ser.flushInput()
    # faced an issue of missalignment
    # to fix it I added header byte to the serial data publisher
    # waiting for header
    while True:
        header = ser.read(1)
        if header == b'\xaa':
            break

    # reading 32 bytes of data, 8 floats(1float -> 4bytes)
    data = ser.read(32)
    if len(data) == 32:
        # takes bytes and converts them to higher equivalents (8 floats thus 'ffff')
        q1_0, q1_1, q1_2, q1_3, q2_0, q2_1, q2_2, q2_3 = struct.unpack('ffffffff', data)
        # convering quaternion to roation object
        rot1 = R.from_quat([q1_1, q1_2, q1_3, q1_0])
        rot2 = R.from_quat([q2_1, q2_2, q2_3, q2_0])
        # rotate the unit vector with the measurement from IMU
        x1_vec = rot1.apply([1, 0, 0])
        y1_vec = rot1.apply([0, 1, 0])
        z1_vec = rot1.apply([0, 0, 1])
        x2_vec = rot2.apply([1, 0, 0])
        y2_vec = rot2.apply([0, 1, 0])
        z2_vec = rot2.apply([0, 0, 1])
        # update arrow position
        x1_arrow.set_segments([[[0, 0, 0], x1_vec]])
        y1_arrow.set_segments([[[0, 0, 0], y1_vec]])
        z1_arrow.set_segments([[[0, 0, 0], z1_vec]])
        x2_arrow.set_segments([[[0, 0, 0], x2_vec]])
        y2_arrow.set_segments([[[0, 0, 0], y2_vec]])
        z2_arrow.set_segments([[[0, 0, 0], z2_vec]])
        ax1.set_title(f"Quat: {q1_0:.3f}, {q1_1:.3f}, {q1_2:.3f}, {q1_3:.3f}")
        ax2.set_title(f"Quat: {q2_0:.3f}, {q2_1:.3f}, {q2_2:.3f}, {q2_3:.3f}")
        plt.draw()
        plt.pause(0.05)
    else:
        print(f"Expected 32 bytes, got {len(data)}")

