import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from networktables import NetworkTables
import time

# Initialize NetworkTables
NetworkTables.initialize(server='roborio-1756-frc.local')
table = NetworkTables.getTable("limelight")

# Data lists for all variables
x_data, y_data, z_data = [], [], []
yaw_data, pitch_data, roll_data = [], [], []

# Function to update the plot
def update_plot(frame):
    botpose = table.getNumberArray("botpose", [0.0]*6)

    x_data.append(botpose[0])
    y_data.append(botpose[1])
    z_data.append(botpose[2])
    yaw_data.append(botpose[3])
    pitch_data.append(botpose[4])
    roll_data.append(botpose[5])

    plt.cla()
    plt.subplot(3, 1, 1)
    plt.plot(x_data, label='X')
    plt.plot(y_data, label='Y')
    plt.plot(z_data, label='Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Robot Position')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(yaw_data, label='Yaw')
    plt.plot(pitch_data, label='Pitch')
    plt.plot(roll_data, label='Roll')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation')
    plt.title('Robot Orientation')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(x_data, label='X', linestyle='--')
    plt.plot(y_data, label='Y', linestyle='--')
    plt.plot(z_data, label='Z', linestyle='--')
    plt.plot(yaw_data, label='Yaw', linestyle='-.')
    plt.plot(pitch_data, label='Pitch', linestyle='-.')
    plt.plot(roll_data, label='Roll', linestyle='-.')
    plt.xlabel('Time (s)')
    plt.ylabel('Combined Data')
    plt.title('Robot Position and Orientation')
    plt.legend()
    plt.grid(True)

# Set up the plot
fig = plt.figure()
ani = FuncAnimation(fig, update_plot, interval=100)

# Start NetworkTables
NetworkTables.addConnectionListener(lambda connected, info: print(info), immediateNotify=True)

print("Waiting for NetworkTables connection...")
while not NetworkTables.isConnected():
    time.sleep(0.1)

print("Connected to NetworkTables")

# Show the plot
plt.show()
