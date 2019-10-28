/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         drifting.c
* @author       xiaozhen 
* @version      V1.0
* @date         2018.03.19
* @brief        drifting
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>

int ServoPin = 4;     //Servo connects to wiringPi port 4 of Raspberry pi

//Definition of Pin
int Left_motor_go = 28;       //AIN2 connects to wiringPi port 28 of Raspberry pi for control Left motor forward 
int Left_motor_back = 29;     //AIN1 connects to wiringPi port 29 of Raspberry pi for control Left motor back 

int Right_motor_go = 24;      //BIN2 connects to wiringPi port 24 of Raspberry pi for control Left motor forward 
int Right_motor_back = 25;    //BIN1 connects to wiringPi port 25 of Raspberry pi for control Left motor back

int Left_motor_pwm = 27;      //PWMA connects to wiringPi port 27 of Raspberry pi for control the speed of the left motor
int Right_motor_pwm = 23;     //PWMA connects to wiringPi port 23 of Raspberry pi for control the speed of the right motor

unsigned short int ServoPos_left = 110;    //left
unsigned short int ServoPos_middle = 55;   //middle
unsigned short int ServoPos_right = 0;     //right

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
  int PulseWidth;                    //Define the pulse width variable
  PulseWidth = (myangle * 11) + 500; //Convert the Angle to 500-2480 pulse width
  digitalWrite(ServoPin, HIGH);      
  delayMicroseconds(PulseWidth);     
  digitalWrite(ServoPin, LOW);       
  delay(20 - PulseWidth / 1000);     //Delay remaining time 
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
  for (i = 0; i <= 15; i++)     //Generate PWM, equivalent delay to ensure that it can be turned to the response angle
  {
    servo_pulse(ServoPin, pos); //Generate the PWM in the analog mode
  }
}

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         advance
* @param[in1]    LeftCarSpeedControl
* @param[in2]    RightCarSpeedControl
* @param[out]    void
* @retval        void
* @par History   
*/
void run(unsigned int LeftCarSpeedControl,unsigned int RightCarSpeedControl)
{
  //Left motor advance 
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, LeftCarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, RightCarSpeedControl);
}

/**
* Function       brake
* @author        xiaozhen
* @date          2018.03.19
* @brief         brake
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void brake()
{
	digitalWrite(Left_motor_go, LOW);
	digitalWrite(Left_motor_back, LOW);
	digitalWrite(Right_motor_go, LOW);
	digitalWrite(Right_motor_back, LOW);
}

/**
* Function       left
* @author        Danny
* @date          2017.08.16
* @brief         turn left(left wheel stop, right wheel advance)
* @param[in1]    LeftCarSpeedControl
* @param[in2]    RightCarSpeedControl
* @param[out]    void
* @retval        void
* @par History   
*/
void left(unsigned int LeftCarSpeedControl,unsigned int RightCarSpeedControl)
{
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, LeftCarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, RightCarSpeedControl);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         turn left in place(left wheel back, right wheel advance)
* @param[in1]    LeftCarSpeedControl
* @param[in2]    RightCarSpeedControl
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left(unsigned int LeftCarSpeedControl,unsigned int RightCarSpeedControl)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH); 
  softPwmWrite(Left_motor_pwm, LeftCarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, RightCarSpeedControl);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         turn right(right wheel stop, left wheel advance)
* @param[in1]    LeftCarSpeedControl:
* @param[in2]    RightCarSpeedControl
* @param[out]    void
* @retval        void
* @par History   
*/
void right(unsigned int LeftCarSpeedControl,unsigned int RightCarSpeedControl)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, LeftCarSpeedControl);

  //Right motor stop
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, LOW);  
  softPwmWrite(Right_motor_pwm, RightCarSpeedControl);
}


/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         turn right in place(left wheel advance, right wheel back)
* @param[in1]    LeftCarSpeedControl
* @param[in2]    RightCarSpeedControl
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_right(unsigned int LeftCarSpeedControl,unsigned int RightCarSpeedControl)
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, LeftCarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, RightCarSpeedControl);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         back
* @param[in1]    LeftCarSpeedControl
* @param[in2]    RightCarSpeedControl
* @param[out]    void
* @retval        void
* @par History  
*/
void back(unsigned int LeftCarSpeedControl,unsigned int RightCarSpeedControl)
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, LeftCarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH);
  softPwmWrite(Right_motor_pwm, RightCarSpeedControl);
}

/**
* Function       main
* @author        xiaozhen
* @date          2018.03.19
* @brief         drifting
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
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
	
	while(1)
	{
		servo_appointed_detection(ServoPos_middle);
		run(100,100);                       
		delay(1000);
		spin_left(200,200);                 
		delay(1000);
		servo_appointed_detection(ServoPos_middle);
		run(100,100);                       
		delay(1000);
		servo_appointed_detection(ServoPos_middle);       
		brake();                            
	}
	return;
}


