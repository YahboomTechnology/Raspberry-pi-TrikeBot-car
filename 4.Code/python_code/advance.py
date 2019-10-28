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

#Set the GPIO port to BCM encoding mode.
ServoPin = 23

g_LeftPos = 110
g_MidPos = 55
g_RightPos = 0

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pin initialization operation
def motor_init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    
    #Set the PWM pin and frequency is 50hz
    GPIO.setup(ServoPin,GPIO.OUT,initial=GPIO.LOW)
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(g_MidPos)

#advance
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    #PWM duty cycle is set to 100（0--100）
    pwm_ENA.start(20)
    pwm_ENB.start(20)
#servo control
def servo_contrl(pos):
	pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
	
#delay 2s
time.sleep(2)

#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.
try:
		motor_init()
		while True:
			servo_contrl(g_MidPos)
			run()
except KeyboardInterrupt:
		pass
pwm_ENA.stop()
pwm_ENB.stop()
pwm_servo.stop()
GPIO.cleanup()

