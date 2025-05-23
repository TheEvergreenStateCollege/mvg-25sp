from control import Control
import time


c = Control() # Create an instance of the Control class

Z = 20
F = 90
steps = 3

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

# Test function to set servo angles
# This function is not called in the main program but can be used for testing servo angles
def test():
    c.servo.set_servo_angle(15, 90)  # Hip
    c.servo.set_servo_angle(14, 90)  # Elbow
    c.servo.set_servo_angle(13, 90)  # Wrist
