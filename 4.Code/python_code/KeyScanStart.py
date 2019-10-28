#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#Definition of  motor pin 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Definition of  key
key = 8

#Set the GPIO port to BCM encoding mode
ServoPin = 23

g_LeftPos = 110
g_MidPos = 55
g_RightPos = 0

#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pins are initialized into output mode
#Key pin is initialized into input mode
def motor_init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    global delaytime
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    GPIO.setup(key,GPIO.IN)
    #Set the PWM pin and frequency is 50hz
    GPIO.setup(ServoPin,GPIO.OUT,initial=GPIO.LOW)
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(g_MidPos)
	
#advance	
def run(delaytime):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(30)
    pwm_ENB.ChangeDutyCycle(30)
    time.sleep(delaytime)

#back
def back(delaytime):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(30)
    pwm_ENB.ChangeDutyCycle(30)
    time.sleep(delaytime)

#turn left
def left(delaytime):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(30)
    pwm_ENB.ChangeDutyCycle(40)
    time.sleep(delaytime)

#turn right
def right(delaytime):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(40)
    pwm_ENB.ChangeDutyCycle(30)
    time.sleep(delaytime)

#turn left in place
def spin_left(delaytime):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(30)
    pwm_ENB.ChangeDutyCycle(30)
    time.sleep(delaytime)

#turn right in place
def spin_right(delaytime):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(30)
    pwm_ENB.ChangeDutyCycle(30)
    time.sleep(delaytime)

#brake
def brake(delaytime):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(0)
    time.sleep(delaytime)
#servo control
def servo_contrl(pos):
	pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
	
#Button detection
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
	if not GPIO.input(key):
	    time.sleep(0.01)
	    while not GPIO.input(key):
	        pass
#delay 2s	
time.sleep(2)

#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.
try:
		motor_init()
		key_scan()
		servo_contrl(g_MidPos)
		run(1)
		back(1)
		servo_contrl(g_LeftPos)
		left(2)
		servo_contrl(g_RightPos)
		right(2)
		servo_contrl(g_MidPos)
		spin_left(3)
		spin_right(3)
		brake(1)
except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
pwm_servo.stop()
GPIO.cleanup() 
