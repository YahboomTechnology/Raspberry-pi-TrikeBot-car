/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         tracking.c
* @author       xiaozhen
* @version      V1.0
* @date         2017.08.16
* @brief        tracking
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>

//Definition of Pin
int Left_motor_go = 28;       //AIN2 connects to wiringPi port 28 of Raspberry pi for control Left motor forward 
int Left_motor_back = 29;     //AIN1 connects to wiringPi port 29 of Raspberry pi for control Left motor back 

int Right_motor_go = 24;      //BIN2 connects to wiringPi port 24 of Raspberry pi for control Left motor forward 
int Right_motor_back = 25;    //BIN1 connects to wiringPi port 25 of Raspberry pi for control Left motor back

int Left_motor_pwm = 27;      //PWMA connects to wiringPi port 27 of Raspberry pi for control the speed of the left motor
int Right_motor_pwm = 23;     //PWMA connects to wiringPi port 23 of Raspberry pi for control the speed of the right motor

int key = 10;                 //Key connects to wiringPi port 10 of Raspberry pi

//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      9                 21                  7                   1
const int TrackSensorLeftPin1  =  9;   //The first tracking infrared sensor pin on the left is connected to  wiringPi port 9 of Raspberry pi
const int TrackSensorLeftPin2  =  21;  //The second tracking infrared sensor pin on the left is connected to  wiringPi port 21 of Raspberry pi
const int TrackSensorRightPin1 =  7;   //The first tracking infrared sensor pin on the right is connected to  wiringPi port 7 of Raspberry pi
const int TrackSensorRightPin2 =  1;   //The second tracking infrared sensor pin on the right is connected to  wiringPi port 1 of Raspberry pi

//Define variables to store data collected by each tracking infrared pin
int TrackSensorLeftValue1;
int TrackSensorLeftValue2;
int TrackSensorRightValue1;
int TrackSensorRightValue2;

unsigned short int ServoPos_left = 110;    //Left
unsigned short int ServoPos_middle = 55;   //middle
unsigned short int ServoPos_right = 0;     //Right

int ServoPin = 4;     //Servo connects to wiringPi port 4 of Raspberry pi

/**
* Function       servo_pulse
* @author        xiaozhen 
* @date          2018.03.20
* @brief         Define a pulse function to generate the PWM value in the analog mode. 
*                The base pulse is 20ms, 
*                and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
* @param[in1]    ServPin
* @param[in2]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                     //Define the pulse width variable
  PulseWidth = (myangle * 11) + 500;  //Convert the Angle to 500-2480 pulse width
  digitalWrite(ServoPin, HIGH);      
  delayMicroseconds(PulseWidth);     
  digitalWrite(ServoPin, LOW);       
  delay(20 - PulseWidth / 1000);      //Delay remaining time 
  return;
}

/**
* Function       servo_appointed_detection
* @author        xiaozhen 
* @date          2018.03.20
* @brief         The servo rotates to the specified Angle
* @param[in]     pos
* @param[out]    void
* @retval        void
* @par History  
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 15; i++)    
  {
    servo_pulse(ServoPin, pos);
  }
}


/**
* Function       run
* @author        xiaozhen
* @date          2018.03.20
* @brief         advance
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void run(int left_speed, int right_speed)
{
  //Left motor advance 
  digitalWrite(Left_motor_go, HIGH);  
  digitalWrite(Left_motor_back, LOW); 
  softPwmWrite(Left_motor_pwm, left_speed );

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       brake
* @author        xiaozhen
* @date          2018.03.20
* @brief         brake
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void brake(int time)
{
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);

  delay(time * 100);
}

/**
* Function       left
* @author        xiaozhen
* @date          2018.03.20
* @brief         turn left(left wheel stop, right wheel advance)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void left(int left_speed, int right_speed)
{
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, left_speed);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       right
* @author        xiaozhen
* @date          2018.03.20
* @brief         turn right(right wheel stop, left wheel advance)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void right(int left_speed, int right_speed)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);  
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, left_speed);

  //Right motor stop
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       spin_left
* @author        xiaozhen
* @date          2018.03.20
* @brief         turn left in place(left wheel back, right wheel advance)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left(int left_speed, int right_speed)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH); 
  softPwmWrite(Left_motor_pwm, left_speed);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       spin_right
* @author        xiaozhen
* @date          2018.03.20
* @brief         turn right in place(left wheel advance, right wheel back)
* @param[in1]    left_speed
* @param[in2]    right_speed
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_right(int left_speed, int right_speed)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, left_speed);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       back
* @author        xiaozhen
* @date          2018.03.20
* @brief         back
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void back(int time)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, 40);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, 40);

  delay(time );
}

/**
* Function       key_scan
* @author        xiaozhen
* @date          2018.03.20
* @brief         key detection (including software-key debounce)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void key_scan()
{
	while (digitalRead(key));          //Loops this code when the key is not pressed
	while (!digitalRead(key))          //When the key is pressed
	{
		delay(10);	                   
		if (digitalRead(key)  ==  LOW) //Re-determine whether the key was pressed
		{
			delay(100);
			while (!digitalRead(key));  //Determine whether the key is released
		}
	}
}

/**
* Function       main
* @author        xiaozhen
* @date          2018.03.20
* @brief         When key is pressed, Ultrasonic obstacle avoidance mode is opened
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
	//Initialize wiringPi
	wiringPiSetup();
	
	//Initialize the motor drive IO as the output mode
    pinMode(ServoPin, OUTPUT);
   
    servo_appointed_detection(ServoPos_middle);
  
	//Initialize the motor drive IO as the output mode
	pinMode(Left_motor_go, OUTPUT);
	pinMode(Left_motor_back, OUTPUT);
	pinMode(Right_motor_go, OUTPUT);
	pinMode(Right_motor_back, OUTPUT);

	softPwmCreate(Left_motor_pwm,0,255); 
	softPwmCreate(Right_motor_pwm,0,255);
  
	//Initialize the key interface as the input mode
	pinMode(key, INPUT);

	//Initialize the tracksensor IO as the input mode
	pinMode(TrackSensorLeftPin1, INPUT);
	pinMode(TrackSensorLeftPin2, INPUT);
	pinMode(TrackSensorRightPin1, INPUT);
	pinMode(TrackSensorRightPin2, INPUT);

	key_scan();
  
	while(1)
	{
        //When the black line is detected, the corresponding indicator of the tracking module is on,and the port level is LOW.
        //When the black line is not detected, the corresponding indicator of the tracking module is off,and the port level is HIGH.
		TrackSensorLeftValue1  = digitalRead(TrackSensorLeftPin1);
		TrackSensorLeftValue2  = digitalRead(TrackSensorLeftPin2);
		TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
		TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);

        //4 tracking pins level status
        // 0 0 X 0
        // 1 0 X 0
        // 0 1 X 0
        //Turn right in place,speed is 150,delay 80ms
        //Handle right acute angle and right right angle
		if ( (TrackSensorLeftValue1 == LOW || TrackSensorLeftValue2 == LOW) &&  TrackSensorRightValue2 == LOW)
		{	
			servo_appointed_detection(ServoPos_right);
			spin_right(150, 150);
			delay(80);
		}
        //4 tracking pins level status
        // 0 X 0 0       
        // 0 X 0 1 
        // 0 X 1 0       
        //Turn right in place,speed is 150,delay 80ms   
        //Handle left acute angle and left right angle 
		else if ( TrackSensorLeftValue1 == LOW && (TrackSensorRightValue1 == LOW ||  TrackSensorRightValue2 == LOW))
		{	
	        servo_appointed_detection(ServoPos_left);
			spin_left(150, 150);
			delay(80);
		}
        // 0 X X X
        //Left_sensor1 detected black line
		else if ( TrackSensorLeftValue1 == LOW)
		{   
	        servo_appointed_detection(ServoPos_left);
			spin_left(150, 150);
		}
       // X X X 0
       //Right_sensor2 detected black line
		else if ( TrackSensorRightValue2 == LOW )
		{
			servo_appointed_detection(ServoPos_right);
			spin_right(150, 150);
		}
		//4 tracking pins level status
        // X 0 1 X
		else if ( TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == HIGH)
		{ 
			servo_appointed_detection(ServoPos_left);
			left(0, 220);
		}
		//4 tracking pins level status
        // X 1 0 X
		else if (TrackSensorLeftValue2 == HIGH && TrackSensorRightValue1 == LOW)
		{
			servo_appointed_detection(ServoPos_right);
			right(220, 0);
		}
		//4 tracking pins level status
        // X 0 0 X
		else if (TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == LOW)
		{
			servo_appointed_detection(ServoPos_middle);
			run(200, 200);
		}
		//When the level of 4 pins are 1 1 1 1 , the car keeps the previous running state.
	}
	return;
}

