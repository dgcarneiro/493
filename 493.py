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

#Begins camera connection
self.cap = cv2.VideoCapture(1)

#values 
coordinates = 0
object_detected_flag = 0
edge_output_flag = 0
line = 0


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

def forward():
	GPIO.output(right_wheel_dir, GPIO.HIGH)
	GPIO.output(left_wheel_dir, GPIO.HIGH)

def backward():
	GPIO.output(right_wheel_dir, GPIO.LOW)
	GPIO.output(left_wheel_dir, GPIO.LOW)
	
	
def turn_left():
	GPIO.output(right_wheel_dir, GPIO.HIGH)
	GPIO.output(left_wheel_dir, GPIO.HIGH)

def turn_right():
	GPIO.output(right_wheel_dir, GPIO.HIGH)
	GPIO.output(left_wheel_dir, GPIO.HIGH)	


def spin_left():
	GPIO.output(right_wheel_dir, GPIO.HIGH)
	GPIO.output(left_wheel_dir, GPIO.LOW)

def spin_right():
	GPIO.output(right_wheel_dir, GPIO.LOW)
	GPIO.output(left_wheel_dir, GPIO.HIGH)

def speed():
	PWM.set_duty_cycle(left_wheel, 50)
	PWM.set_duty_cycle(right_wheel, 50)

 #def for sonar
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
  if distance < 20:
  	global object_detected_flag = 1


def gpsd():
    gpsd = gps(mode=WATCH_ENABLE) ##  set gpsd to start gps info
    fixed = False
    while True:
        gpsd.next()
 
        if (gpsd.fix.mode != 1): #if fix mode is different than 1, 1==NO_FIX 2==2D 3==3D

            if (not fixed):
                fixed = True

			lat = gpsd.fix.latitude
			lon = gpsd.fix.longitude
			print ' GPS reading'
			print '----------------------------------------'
			print 'latitude    ' , lat
			print 'longitude   ' , lon
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
#alternate gps def
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
		print NMEA1
		print NMEA2
		print NMEA3


	
def check_coord():
	#this is just generic values, need to tune
	if latitude > lat-0.5 and latitude < lat+0.5 and longitude > lon-0.5 and longitude < lon+0.5
		line = line+1
		pass
	elif latitude > lat-0.5 or latitude < lat+0.5
	
	
	elif longitude > lon-0.5 0r longitude < lon+0.5
	
	
	
	
def scheduler():
	while True:
		gpsd()
		#get_img()
		measure()
		
		
def cruise():
	forward()
	speed()
		



#reads flags values and when the flag is = to 1 it will run the def below it, its the decision
#maker of what action will be done and what has priority
def arbiter():
	while True:
		if (object_detected_flag ==1):   #if detects bject within 20cm
			avoid_action()
			turn_left()
			global object_detected_flag = 0
		elif (edge_output_flag ==1):
			
		else:
			cruise()
		time.sleep(0.001)
		
def avoid_():

	

def readfile_coordinates():
	global line
	f = open('temp.txt', 'r')
	temp = f.readlines(line)
	f.close()
	latitude = temp.split("")[0]
	longitude = temp.split("")[1]


while True:
	try:
		while True:
			readfile_coordinates(line)
			scheduler()
			arbiter()
			
	except KeyboardInterrupt: # pressing ctrl C stops operation and cleans up

		PWM.stop(left_wheel)	
		PWM.stop(right_wheel)
		PWM.cleanup()
		GPIO.cleanup()
		ADC.cleanup()

