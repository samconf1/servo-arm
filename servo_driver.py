import analogio
import adafruit_pca9685
import board
import time 
import busio

i2c = busio.I2C(board.D22, board.S21) #initialise i2c bus 
pca = adafruit_pca9685.PCA9685(i2c) # initialise pca9685 board

servo_poll_rate = 0.01 #update every 10ms 

class Servo:
    #called once on object initialise, sets up servo object.
    def __init__(self, servo_channel, min, max):
        self.servo_min_angle = min #servo min angle
        self.servo_max_angle = max #servo max angle 
        pca.channels[servo_channel].frequency = 50 #set channel specific pwm frequency
        self.servo_channel = servo_channel #set servo channel - relates to channel on pca board




class joystick_controlled_servo(Servo):
    def __init__(self, servo_channel, joystick_pin, min, max):
        super().__init__(servo_channel, min, max) #call parent init function
        self.joystick = analogio.AnalogIn(joystick_pin)
        self.current_angle = 90 #start at middle position
        pca.channels[self.servo_channel].duty_cycle = int(((180 + self.current_angle) / 3600) * 65535)
        
    
    def is_input_active(self):
        return bool(self.joystick.value) #returns true if joystick value is any value other than 0 

    def update_angle(self):
        #calculate new angle. current angle + joystick value scaled to servo poll rate. 
        #joystick value * 0.00025 is degrees per second, then scale by servo poll rate to get degrees per poll
        new_angle = self.current_angle + self.joystick.value * (0.00025*servo_poll_rate)
        if new_angle < self.servo_max_angle and new_angle > self.servo_min_angle: #check if new angle is within servo bounds. 
            self.current_angle = new_angle #update current angle to new angle
            pca.channels[self.servo_channel].duty_cycle = int(((180 + self.current_angle) / 3600) * 65535) #update servo duty cycle to match new angle. 
            return
    

class pot_controlled_servo(Servo):
    def __init__(self, servo_channel, pot_pin, min, max):
        super().__init__(servo_channel, min, max) #call parent init function
        self.pot = analogio.AnalogIn(pot_pin) #create pot object
        self.lastpotvalue = 0 
        self.angle_per_unit = (max - min) / 65535
    
    def is_input_active(self):
        if abs(self.lastpotvalue - self.pot.value) > 5:
            self.lastpotvalue = self.pot.value
            return True
        return False
    
    def update_angle(self):
        new_angle = self.angle_per_unit * self.lastpotvalue + self.servo_min_angle
        if new_angle < self.servo_max_angle and new_angle > self.servo_min_angle: #check if new angle is within servo bounds. 
            self.current_angle = new_angle #update current angle to new angle
            pca.channels[self.servo_channel].duty_cycle = int(((180 + self.current_angle) / 3600) * 65535) #update servo duty cycle to match new angle. 
            return


        
#create servo objects and add to list
servo_objects = []
servo_objects.append(joystick_controlled_servo(0, board.A0, 0, 180))
servo_objects.append(pot_controlled_servo(1, board.A1, 0, 180))


next_run = time.monotonic() #set start time 

while True:
    now = time.monotonic() #set current time
    if now >= next_run: #if time between now and next run is greater than poll rate, run servo update.
        for servo in servo_objects: #iterate through all servo objects.
            if servo.is_input_active(): #check if joystick is actually moved. 
                servo.update_angle() #call objects function

        next_run += servo_poll_rate #update new next run time. 






