from control import Control
import time



c = Control() # Create an instance of the Control class

Z = 35
F = 70
steps = 10

#Move forward
print("Moving forward")
for i in range(steps):
    data = ['CMD_MOVE', '1', '0', '35', '10', '0']  # command move, gait, x, y, speed, rotation
    c.run_gait(data, Z=Z, F=F)
    time.sleep(0.1)  # Adjust the sleep time as needed

#Move backward
print("Moving backward")
for i in range(steps):
    data = ['CMD_MOVE', '1', '0', '-35', '10', '0']  # command move, gait, x, y, speed, rotation
    c.run_gait(data, Z=Z, F=F)
    time.sleep(0.1)  # Adjust the sleep time as needed

c.relax(True)
print("Relaxing servos")
time.sleep(1)  # Wait for a second before ending the program
print("End of program")