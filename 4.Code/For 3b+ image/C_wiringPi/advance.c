/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         advance.c
* @author       xiaozhen
* @version      V1.0
* @date         2018.03.19
* @brief        advance
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

int ServoPin = 4;             //Servo is connected to  wiringPi port 4 of Raspberry pi

unsigned short int ServoPos_middle = 55;   //middle

/**
* Function       servo_pulse
* @author        xiaozhen
* @date          2018.03.19
* @brief          Define a pulse function to generate the PWM value in the analog mode. 
*                The base pulse is 20ms, 
*                and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
* @param[in1]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int servoPin,int myangle)
{
	int PulseWidth;                    //Define the pulse width variable
	PulseWidth = (myangle * 11) + 500; //Convert the Angle to 500-2480 pulse width
	digitalWrite(ServoPin, HIGH);      
	delayMicroseconds(PulseWidth);     
	digitalWrite(ServoPin, LOW);       
	delay(20 - PulseWidth / 1000);     
	return;
}

/**
* Function       servo_appointed_detection
* @author        xiaozhen 
* @date          2018.03.20
* @brief         The servo rotates to the specified angle
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
* @author        xiaozhen 
* @date          2018.03.19
* @brief         advance
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   
*/
void run(int time)
{
	//Left motor advance
	digitalWrite(Left_motor_go, HIGH);    
	digitalWrite(Left_motor_back, LOW); 
    //This code is for setting the PWM of the specified pin	
	softPwmWrite(Left_motor_pwm, 200);    //pwm:0-255. The left and right wheels are slightly different

   //Right motor advance 
   digitalWrite(Right_motor_go, HIGH);   
   digitalWrite(Right_motor_back, LOW);  
   //This code is for set the PWM of the specified pin
   softPwmWrite(Right_motor_pwm, 200);   //pwm:0-255. The left and right wheels are slightly different

	delay(time * 100);     //Unit:ms
	return;
}


/**
* Function       main
* @author        xiaozhen 
* @date          2018.03.19
* @brief         advance
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
	wiringPiSetup();      
  
    //Initializethe servo IO as the output mode
    pinMode(ServoPin, OUTPUT);
   
    servo_appointed_detection(ServoPos_middle);
	
	//Initializethe motor drive IO as the output mode
	pinMode(Left_motor_go, OUTPUT);
	pinMode(Left_motor_back, OUTPUT);
	pinMode(Right_motor_go, OUTPUT);
	pinMode(Right_motor_back, OUTPUT);
  
	//int softPwmCreate(int pin,int initialValue,int pwmRange);
	softPwmCreate(Left_motor_pwm,0,255); 
	softPwmCreate(Right_motor_pwm,0,255);   
  
	delay(2000);       
  
	while(1)
	{
		servo_appointed_detection(ServoPos_middle);
		run(10);                        
	}
	return;
}
