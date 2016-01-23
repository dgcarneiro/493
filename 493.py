# apt-get install gpsd gpsd-clients python-gps
# echo BB-UART1 > /sys/devices/bone_capemgr.9/slots
# echo BB-UART2 > /sys/devices/bone_capemgr.9/slots
# echo BB-UART3 > /sys/devices/bone_capemgr.9/slots
#ls /dev/ttyO*
# gpsd -n /dev/ttyO1 -F /var/run/gpsd.sock
# gpsd -n /dev/ttyO2 -F /var/run/gpsd.sock
# gpsd -n /dev/ttyO3 -F /var/run/gpsd.sock
# cat /dev/ttyO1
# cat /dev/ttyO2
# cat /dev/ttyO3
#cgps or cgps -s

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.UART as UART
import time
import math
import thread
import cv2
import numpy as np
import serial


# Defined GPIO pins for sonar
GPIO_TRIGGER = "P8_37"
GPIO_ECHO    = "P8_38"

# Set pins as output and input
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, GPIO.LOW)

# Right side motors. Pin definitions for H-bridge control
right_wheel = "P9_14"  # controls speed of right front motor via PWM
right_wheel_dir = "P9_12"   # combine M1 pins on upper and lower motor controllers into a single pin

# Left side motors. Pin definitions for H-bridge control
left_wheel = "P9_16"  # controls speed of left front motor via PWM
left_wheel_dir = "P8_11"  # combine M2 pins on upper and lower motor controllers into a single pin

# set pin directions as output
GPIO.setup(right_wheel, GPIO.OUT)
GPIO.setup(right_wheel_dir, GPIO.OUT)
GPIO.setup(left_wheel, GPIO.OUT)
GPIO.setup(left_wheel_dir, GPIO.OUT)

PWM.start(left_wheel, 0)
PWM.start(right_wheel, 0)

 
def measure():
  # This function measures a distance
  GPIO.output(GPIO_TRIGGER, GPIO.HIGH)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER, GPIO.LOW)
  start = time.time()

  while GPIO.input(GPIO_ECHO)==0:
    start = time.time()

  while GPIO.input(GPIO_ECHO)==1:
    stop = time.time()

  elapsed = stop-start
  distance = (elapsed * 34300)/2 

#Begins camera connection
self.cap = cv2.VideoCapture(1)

def gpsd():
    gpsd = gps(mode=WATCH_ENABLE) ##  set gpsd to start gps info
    fixed = False
    while True:
        gpsd.next()
 
        if (gpsd.fix.mode != 1): #if fix mode is different than 1, 1==NO_FIX 2==2D 3==3D

            if (not fixed):
                fixed = True


				print ' GPS reading'
				print '----------------------------------------'
				print 'latitude    ' , gpsd.fix.latitude
				print 'longitude   ' , gpsd.fix.longitude
				print 'reported time and fix time    ' , gpsd.utc,' + ', gpsd.fix.time
				print 'altitude (m)' , gpsd.fix.altitude
				print 'speed error (m/s)         ' , gpsd.fix.eps
				print 'longitude error (m)        ' , gpsd.fix.epx
				print 'estimated vertical error         ' , gpsd.fix.epv
				print 'estimated timestamp error        ' , gpsd.fix.ept
				print 'speed (m/s) ' , gpsd.fix.speed
				print 'climb (m/s)       ' , gpsd.fix.climb
				print 'mode        ' , gpsd.fix.mode
				print
				print 'satellites        ' , gpsd.satellites
				time.sleep(1)
        else:
            if (fixed):
                fixed = False
				print' waiting for fix '
				sleep(1)

def gps():
	UART.setup("UART1")
	UART.setup("UART2")
	UART.setup("UART3")	
	GPS1 = serial.Serial('/dev/ttyO1', 9600)
	GPS2 = serial.Serial('/dev/ttyO2', 9600)
	GPS3 = serial.Serial('/dev/ttyO3', 9600)
	while(1):
			while GPS1.inWaiting() or GPS2.inWaiting() or GPS3.inWaiting() ==0:
					pass
			NMEA1=GPS1.readline()
			NMEA2=GPS1.readline()			
			NMEA3=GPS1.readline()			
			print NMEA

def get_img(self):
		#update current image.
		while 1:
			ret, frame = self.cap.read()
			if ret==True:#check if cap.read() returned frame
				self.img=frame
				break
	def get_lines(self):
		#function that returns a series of lines in the current image. Possibly using an input region of interest of the image.
		pass
	def wh_det(img,x1=10,x2=640,y1=470,y2=0,xit=10,yit=-10):
		tot=0;
		l1=[]
		for x in range(x1,x2,xit):
			test=3#used to ignore noise in the image. if the values goes too low the system stops incrementing in that y direction
			for y in range(y1,y2,yit):
				bgr=self.img[y,x]
				if abs(int(bgr[1])-int(bgr[2]))<14 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):#works
					tot+=1
					if test<3:
						test+=1
					else:
						test-=1
					if test<2:
						break
			l1.append(ytot)
		return l1

def forward():
	GPIO.output(right_wheel_dir, GPIO.HIGH)
	GPIO.output(left_wheel_dir, GPIO.HIGH)

def backward():
	GPIO.output(right_wheel_dir, GPIO.LOW)
	GPIO.output(left_wheel_dir, GPIO.LOW)

def spin_left():
	GPIO.output(right_wheel_dir, GPIO.HIGH)
	GPIO.output(left_wheel_dir, GPIO.LOW)

def spin_right():
	GPIO.output(right_wheel_dir, GPIO.LOW)
	GPIO.output(left_wheel_dir, GPIO.HIGH)

def speed():
	vel = int(cmd[1])*10
	PWM.set_duty_cycle(left_wheel, vel)
	PWM.set_duty_cycle(right_wheel, vel)
	
	
#scheduler runs the readings from the ir sensors and photosensors to that which will also
#view if conditions are met will set its flag to 1
def scheduler():
	#follow_readings()
	avoid_readings()

#reads flags values and when the flag is = to 1 it will run the def below it, its the decision
#maker of what action will be done and what has priority
def arbiter():
	if (avoid_output_flag ==1):
		avoid_action()
	elif (follow_output_flag ==1):
		follow_action()
	else:
		cruise()
	time.sleep(0.001)	

def readfile_coordinates():
	f = open('temp.txt', 'r')
	temp = f.readlines()
	f.close()
	coordinates = temp[line]
	
	f = open('temp.txt', 'r')
	
	

while True:
	cmd = raw_input("Command, f/b/l/r 0 .. 9, for example f5  :")
	direction = cmd[0]  # direction is the first value of two in the string
	if direction == "f":
		forward()
		speed()
	elif direction == "b":
		backward()
		speed()
	elif direction == "l":
		spin_left()
		speed()
	elif direction == "r":
		spin_right()
		speed()
	else:
		break

PWM.stop(left_wheel)
PWM.stop(right_wheel)

PWM.cleanup()
GPIO.cleanup()

