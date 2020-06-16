#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import string
import serial

#Key value
run_car  = '1'  #advance key
back_car = '2'  #back key
left_car = '3'  #left key
right_car = '4' #right key 
stop_car = '8'  #stop key

left_servo_over ='6'
right_servo_over ='7'
left_servo ='1'
right_servo = '2'
init_servo = '5'
stop_servo = '8'

#State value definition
enSTOP = 0
enRUN = 1
enBACK = 2
enLEFT = 3
enRIGHT = 4

enTLEFT = 5
enTRIGHT = 6
enNODRIFT = 7

#Define Motor pins 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Define key pin
key = 8

#Define ultrasonic pins
EchoPin = 0
TrigPin = 1

#Define RGB light pins
LED_R = 22
LED_G = 27
LED_B = 24 

#Define Servo pin
ServoPin = 23

g_LeftPos = 110
g_MidPos = 55
g_RightPos = 0

#Define buzzer pin
buzzer = 8

#Define outfire pin
OutfirePin = 2

#Define IR pins
#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #Left 1 
TrackSensorLeftPin2  =  5   #Left 2
TrackSensorRightPin1 =  4   #Right 1
TrackSensorRightPin2 =  18  #Right 2

#Define
global timecount 
global count 

g_nowpos = 0

red = 0
green = 0
blue = 0
NewLineReceived = 0
InputString = ''
InputStringcache = ''
g_CarState = enSTOP
g_CarDrift = enNODRIFT
CarSpeedControl = 30 
g_num = 0
g_packnum = 0 
ReturnTemp = ''
infrared_track_value = ''
infrared_avoid_value = ''
LDR_value = ''
StartBit = 0
#Set GPIO port is BCM 
GPIO.setmode(GPIO.BCM)

#Ignore the warning message
GPIO.setwarnings(False)

def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    global pwm_rled
    global pwm_gled
    global pwm_bled
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(OutfirePin,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
    GPIO.setup(TrackSensorRightPin1,GPIO.IN)
    GPIO.setup(TrackSensorRightPin2,GPIO.IN)
    #Set pwm pin and frequency to 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    #Set the servo frequency and initial duty cycle
    #pwm_servo = GPIO.PWM(ServoPin, 50)
    #pwm_servo.start(0)
    pwm_rled = GPIO.PWM(LED_R, 1000)
    pwm_gled = GPIO.PWM(LED_G, 1000)
    pwm_bled = GPIO.PWM(LED_B, 1000)
    pwm_rled.start(0)
    pwm_gled.start(0)
    pwm_bled.start(0)
    
#Car advance   
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#Car back
def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    
#Car turn left  
def left():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl-10)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#Car turn right
def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl-10)
    
#Car spin left
def spin_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#Car spin right
def spin_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#Car stop
def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)

#Key detection
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass
                
#Ultrasonic function
def Distance():
    GPIO.output(TrigPin,GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin,GPIO.LOW)

    t3 = time.time()

    while not GPIO.input(EchoPin):
        t4 = time.time()
        if (t4 - t3) > 0.03 :
            return -1


    t1 = time.time()
    while GPIO.input(EchoPin):
        t5 = time.time()
        if(t5 - t1) > 0.03 :
            return -1

    t2 = time.time()
    time.sleep(0.01)
#    print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
    return ((t2 - t1)* 340 / 2) * 100

def Distance_test():
    num = 0
    ultrasonic = []
    while num < 5:
            distance = Distance()
            while int(distance) == -1 :
                distance = Distance()
                print("Tdistance is %f"%(distance) )
            while (int(distance) >= 500 or int(distance) == 0) :
                distance = Distance()
                print("Edistance is %f"%(distance) )
            ultrasonic.append(distance)
            num = num + 1
            time.sleep(0.01)
    print ultrasonic
    distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3])/3
    print("distance is %f"%(distance) ) 
    return distance
    
#Servo rotates to the specified angle
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)

def servo_pulse(myangle):
    global g_nowpos
    if g_nowpos == myangle:
        return
    else:
        g_nowpos = myangle

    for i in range(10):
        pulsewidth = (g_nowpos * 11) + 500
        GPIO.output(ServoPin, GPIO.HIGH)
        time.sleep(pulsewidth/1000000.0)
        GPIO.output(ServoPin, GPIO.LOW)
        time.sleep(20.0/1000-pulsewidth/1000000.0)

#Tracking 
def tracking_test():
    global infrared_track_value

    TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
    infrared_track_value_list = ['0','0','0','0']
    infrared_track_value_list[0] = str(1^ TrackSensorLeftValue1)
    infrared_track_value_list[1] =str(1^ TrackSensorLeftValue2)
    infrared_track_value_list[2] = str(1^ TrackSensorRightValue1)
    infrared_track_value_list[3] = str(1^ TrackSensorRightValue2)
    infrared_track_value = ''.join(infrared_track_value_list)
    
#Buzzer
def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.001)    
    
#RGB light
def color_led_pwm(iRed,iGreen, iBlue):
    print iRed 
    print iGreen
    print iBlue
    v_red = (100*iRed)/255
    v_green = (100*iGreen)/255
    v_blue = (100*iBlue)/255
    print v_red
    print v_green
    print v_blue
    pwm_rled.ChangeDutyCycle(v_red)
    pwm_gled.ChangeDutyCycle(v_green)
    pwm_bled.ChangeDutyCycle(v_blue)
    time.sleep(0.02)
    
def serial_data_parse():
    global NewLineReceived
    global CarSpeedControl
    global g_CarState
    global g_CarDrift
    global g_frontservopos
    global red
    global green
    global blue
    global g_LeftPos
    global g_MidPos
    global g_RightPos
   
    if (InputString.find("$TBOT,PTZ", 0, len(InputString)) != -1) and (InputString.find("POS", 0, len(InputString)) == -1):
        i = InputString.find("PTZ",  0, len(InputString)) 
        ii = InputString.find("#",  0, len(InputString))
        if ii > i:
            string = InputString[i+3:ii]
            m_kp = int(string)
            servo_appointed_detection(180 - m_kp)
            NewLineReceived = 0
            InputString.zfill(len(InputString))

    if (InputString.find("$TBOT,PTZ", 0, len(InputString)) != -1) and (InputString.find("POS", 0, len(InputString)) != -1):
        i = InputString.find("PTZ",  0, len(InputString)) 
        ii = InputString.find(",",  i, len(InputString))
        if ii > i:
            string = InputString[i+3:ii]
            m_kp = int(string)
            servo_appointed_detection(180 - m_kp)
            g_frontservopos = 180 - m_kp    
        i = InputString.find("POS",  ii, len(InputString)) 
        ii = InputString.find("#",  i, len(InputString))
        if ii > i:
            string = InputString[i+3:ii]
            m_kp = int(string)
            if m_kp == 1:
                g_LeftPos = g_frontservopos
            elif m_kp == 2:
                g_MidPos = g_frontservopos
            elif m_kp == 3:
                g_RightPos = g_frontservopos
        NewLineReceived = 0
        InputString.zfill(len(InputString))

    if (InputString.find("CLR", 0, len(InputString)) != -1):
        i = InputString.find("CLR", 0,  len(InputString)) 
        ii = InputString.find(",CLG",  0,  len(InputString))
        if ii > i:
            string = InputString[i+3:ii]
            m_kp = int(string)
            red = m_kp
        i = InputString.find("CLG",  0, len(InputString)) 
        ii = InputString.find(",CLB",  0, len(InputString))
        if ii > i:
            string = InputString[i+3:ii]
            m_kp = int(string)
            green = m_kp
        i = InputString.find("CLB",  0, len(InputString)) 
        ii = InputString.find("#",  0,  len(InputString))
        if ii > i:
            string = InputString[i+3:ii]
            m_kp = int(string)
            blue = m_kp
            print "red :%d " % red
            print green
            print blue
        color_led_pwm(red, green, blue)          
        NewLineReceived = 0
        InputString.zfill(len(InputString))
		
    if (InputString.find("$TBOT", 0, len(InputString)) == -1) and (InputString.find("#",  0, len(InputString)) != -1):
        if InputString[3] == '1':
            g_CarDrift = enTLEFT
        elif InputString[3] == '2':
            g_CarDrift = enTRIGHT
        elif InputString[3] == '8':
            g_CarDrift = enNODRIFT

        if InputString[5] == '1':
            whistle()

        if InputString[7] == '1':
            CarSpeedControl += 20
            if CarSpeedControl > 100:
                CarSpeedControl = 100
        if InputString[7] == '2':
            CarSpeedControl -= 20
            if CarSpeedControl < 20:
                CarSpeedControl = 20

        if InputString[9] == left_servo_over:
            servo_pulse(g_LeftPos)
        if InputString[9] == right_servo_over:
            servo_pulse(g_RightPos)
        if InputString[9] == init_servo:
            servo_pulse(g_MidPos)

        if InputString[11] == '1':
            color_led_pwm(255, 255, 255)
        if InputString[11] == '2':
            color_led_pwm(255, 0, 0)
        if InputString[11] == '3':
            color_led_pwm(0, 255, 0)
        if InputString[11] == '4':
            color_led_pwm(0, 0, 255)   
        if InputString[11] == '5':
            color_led_pwm(0, 255, 255) 
        if InputString[11] == '6':
            color_led_pwm(255, 0, 255) 
        if InputString[11] == '7':
            color_led_pwm(255, 255, 0) 
        if InputString[11] == '8':
            color_led_pwm(0, 0, 0)  

        if InputString[1] == run_car:
            g_CarState = enRUN
        elif InputString[1] == back_car:
            g_CarState = enBACK    
        elif InputString[1] == left_car:
            g_CarState = enLEFT
        elif InputString[1] == right_car:
            g_CarState = enRIGHT
        elif InputString[1] == stop_car:
            g_CarState = enSTOP
                
        NewLineReceived = 0
        InputString.zfill(len(InputString))    

def serial_data_postback():

    global ReturnTemp
    ReturnTemp = ''
    distance = Distance_test()
    ReturnTemp += "$TBOT,CSB"
    ReturnTemp += str(int(distance))
    ReturnTemp += ",PV12.6"
    ReturnTemp += ",GS0"
    ReturnTemp += ",LF"
    tracking_test()
    ReturnTemp += infrared_track_value
    ReturnTemp += ",HW00"
    ReturnTemp += ",GM00"
    ReturnTemp += "#"
    print ReturnTemp
    ser.write(ReturnTemp)
    
def serialEvent():
    global InputString
    global InputStringcache
    global StartBit
    global NewLineReceived
    InputString = ''
    while True:
        size = ser.inWaiting()
        if size == 0:   
            break
        else:
            while size != 0:
                serialdatabit = ser.read(1)
                size -= 1
                if serialdatabit == '$':
                    StartBit = 1
                if StartBit == 1:
                    InputStringcache += serialdatabit
                if StartBit == 1 and serialdatabit == '#':
                    NewLineReceived = 1
                    InputString = InputStringcache
                    InputStringcache = ''
                    StartBit = 0
                    size = 0
                    print InputString    
          
try:
    ser = serial.Serial("/dev/ttyAMA0", 9600, timeout = 0.001)
    print "serial.isOpen() = ",ser.isOpen()
    ser.write("serial is on!")  
    timecount = 2000
    count = 100
    init()
    while True:
        serialEvent()
        if NewLineReceived == 1:
            print "serialdata:%s" % InputString
            serial_data_parse()
            NewLineReceived = 0

        if g_CarDrift != enNODRIFT:
            if g_CarDrift == enTLEFT:
                spin_left()
                if g_CarState == enRUN:
                    servo_pulse(g_MidPos+10)                    
                elif g_CarState == enLEFT:
                    servo_pulse(g_LeftPos)                    
                elif g_CarState == enRIGHT:
                    servo_pulse(g_RightPos)                
                elif g_CarState == enBACK:
                    servo_pulse(g_MidPos+10)
                    
            elif g_CarDrift == enTRIGHT:
                spin_right()
                if g_CarState == enRUN:
                    servo_pulse(g_MidPos-10)                    
                elif g_CarState == enLEFT:
                    servo_pulse(g_LeftPos)                    
                elif g_CarState == enRIGHT:
                    servo_pulse(g_RightPos)                
                elif g_CarState == enBACK:
                    servo_pulse(g_MidPos-10)
        else:
            if g_CarState == enSTOP:
                brake()          
            elif g_CarState == enRUN:
                servo_pulse(g_MidPos)
                run()
            elif g_CarState == enLEFT:
                servo_pulse(g_LeftPos)
                left()
            elif g_CarState == enRIGHT:
                servo_pulse(g_RightPos)
                right()
            elif g_CarState == enBACK:
                servo_pulse(g_MidPos)
                back()


        timecount -= 1
        if timecount == 0:
            count -= 1
            timecount = 2000
            if count == 0:
                serial_data_postback()
                timecount = 2000
                count = 100
                
except KeyboardInterrupt:
    pass
ser.close()
pwm_ENA.stop()
pwm_ENB.stop()
pwm_rled.stop()
pwm_gled.stop()
pwm_bled.stop()
pwm_servo.stop()
GPIO.cleanup()
    
    
    
    
    
    
