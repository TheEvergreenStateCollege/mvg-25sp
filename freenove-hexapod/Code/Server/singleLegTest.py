#need control to move robot
from control import Control
from servo import Servo
import time

c = Control()
servo = Servo()

def move_leg(legNum, hipAngle, elbowAngle, wristAngle):
	pin_map = {
		1: [15,14,13],  #Leg: [hip,elbow,wrist]
		2: [12,11,10], 
		3: [9,8,31], 
		4: [22,23,27], 
		5: [19,20,21], 
		6: [16,17,18],
	}
	
	if legNum not in pin_map:
		print(f"Invalid leg number: {legNum}")
	
	hip_pin,elbow_pin,wrist_pin = pin_map[legNum]
	c.servo.set_servo_angle(hip_pin, hipAngle)
	c.servo.set_servo_angle(elbow_pin, elbowAngle)
	c.servo.set_servo_angle(wrist_pin, wristAngle)
	
def test():
	# Note: Right side legs angles(elbow and wrist) are reversed from the left side angles
	# if (4,75,30,30) then (3,75,150,150)
	
	#for leg in range(1,7):  This was a for loop to move all the legs
		move_leg(4,75,30,30)  # This is to move just one leg (leg,hip,elbow,wrist)   
		#print(f"Tried hip on {leg}")
		print("Moved leg 4") #
		time.sleep(2)
		
test()

	
		
