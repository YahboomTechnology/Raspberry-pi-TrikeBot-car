# -*- coding:UTF-8 -*-

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

#Definetion of buzzer
BEEP_PIN = 8


#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Set the GPIO port to BCM encoding mode.
GPIO.setup(BEEP_PIN, GPIO.OUT)


#whistle
try:
    while True:
        GPIO.output(BEEP_PIN, GPIO.HIGH)  
        time.sleep(1)
        
        GPIO.output(BEEP_PIN, GPIO.LOW)     
        time.sleep(1)

except:
    print "except"
GPIO.cleanup()
