/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         avoid_ultrasonic.c
* @author       xiaozhen
* @version      V1.0
* @date         2018.03.20
* @brief        Ultrasonic obstacle avoidance
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

//Definition of Pin
int Left_motor_go = 28;       //AIN2 connects to wiringPi port 28 of Raspberry pi for control Left motor forward 
int Left_motor_back = 29;     //AIN1 connects to wiringPi port 29 of Raspberry pi for control Left motor back 

int Right_motor_go = 24;      //BIN2 connects to wiringPi port 24 of Raspberry pi for control Left motor forward 
int Right_motor_back = 25;    //BIN1 connects to wiringPi port 25 of Raspberry pi for control Left motor back

int Left_motor_pwm = 27;      //PWMA connects to wiringPi port 27 of Raspberry pi for control the speed of the left motor
int Right_motor_pwm = 23;     //PWMA connects to wiringPi port 23 of Raspberry pi for control the speed of the right motor

int key = 10;              //buzzer is connected to  wiringPi port 10 of Raspberry pi

int EchoPin = 30;             //EchoPin is connected to  wiringPi port 30 of Raspberry pi
int TrigPin = 31;             //TrigPin is connected to  wiringPi port 31 of Raspberry pi

int ServoPin = 4;             //Servo is connected to  wiringPi port 4 of Raspberry pi


unsigned short int ServoPos_left = 110;    //left
unsigned short int ServoPos_middle = 55;   //front
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
* @brief         The servo rotates to the specified angle
* @param[in]     pos
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 15; i++)    //Generate PWM, equivalent delay to ensure that it can be turned to the response angle
  {
    servo_pulse(ServoPin, pos); //Generate the PWM in the analog mode
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
  //Left motor adavnce
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
* @param[in]
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
* @author        xiaozhen 
* @date          2018.03.20
* @brief         turn left(left wheel stop,right wheel advance)
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

  //Right motor adavnce
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW);
  softPwmWrite(Right_motor_pwm, right_speed);
}

/**
* Function       right
* @author        xiaozhen 
* @date          2018.03.20
* @brief         turn right(right wheel stop,left wheel advance)
* @param[in]     void
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
* @brief         turn left in place(left wheel back,right wheel advance)
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
* Function       spin_right
* @author        xiaozhen 
* @date          2018.03.20
* @brief         turn right in place(left wheel advance,right wheel back)
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
* Function       key_scan
* @author        Danny
* @date          2017.08.16
* @brief         key detection (including software-key debounce)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void key_scan()
{
  while (digitalRead(key));       //Loops this code when the key is not pressed
  while (!digitalRead(key))       //When the key is pressed
  {
    delay(10);	                 
    if (digitalRead(key)  ==  LOW)//Re-determine whether the key was pressed
    {
      delay(100);
      while (!digitalRead(key));  //Determine whether the key is released
    }
  }
}


/**
* Function       Distance
* @author        xiaozhen 
* @date          2018.03.20
* @brief          measure the distance by Ultrasonic
* @param[in]     void
* @param[out]    void
* @retval        float:distance
* @par History   
*/
float Distance()
{
	float distance;
	struct timeval tv1;
	struct timeval tv2;
	struct timeval tv3;
	struct timeval tv4;
	long start, stop;
	
	digitalWrite(TrigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(TrigPin, HIGH);      //Input a high level of at least 10 US to the Trig pin
	delayMicroseconds(15);
	digitalWrite(TrigPin, LOW);
    
    gettimeofday(&tv3, NULL);        
	start = tv3.tv_sec * 1000000 + tv3.tv_usec;
	while(!digitalRead(EchoPin) == 1)
	{
		gettimeofday(&tv4, NULL);    
		stop = tv4.tv_sec * 1000000 + tv4.tv_usec;
		
		if ((stop - start) > 30000)   //Maximum time value (5m)：10/340=0.03s
		{
			return -1;              
		}
	} 
	
	gettimeofday(&tv1, NULL);     
    start = tv1.tv_sec*1000000+tv1.tv_usec;
	while(!digitalRead(EchoPin) == 0)
	{
		gettimeofday(&tv3,NULL);   
		stop = tv3.tv_sec*1000000+tv3.tv_usec;
		if ((stop - start) > 30000)
		{
			return -1;
		}
	}                              
	gettimeofday(&tv2, NULL);      

	start = tv1.tv_sec * 1000000 + tv1.tv_usec;
	stop = tv2.tv_sec * 1000000 + tv2.tv_usec;

	distance = (float)(stop - start)/1000000 * 34000 / 2;
	return distance;
}

/**
* Function       bubble
* @author        xiaozhen 
* @date          2018.03.20
* @brief         Bubble sorting 
* @param[in1]    a:Ultrasonic array first address
* @param[in2]    n:Size of Ultrasonic array 
* @param[out]    void
* @retval        void
* @par History   
*/
void bubble(unsigned long *a, int n)

{
  int i, j, temp;
  for (i = 0; i < n - 1; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (a[i] > a[j])
      {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
      }
    }
  }
}

/**
* Function       Distane_test
* @author        xiaozhen 
* @date          2018.03.20
* @brief         Remove the maximum, minimum of the 5 datas, and get average values of 3 datas to improve accuracy of test
* @param[in]     void
* @param[out]    void
* @retval        float:distance
* @par History   
*/
float Distance_test()
{
  float distance;
  unsigned long ultrasonic[5] = {0};
  int num = 0;
  while (num < 5)
  {
     distance = Distance();
	 while((int)distance == -1)
	 {
		 distance = Distance();
	 }
    //Filter out data greater than 500 or smaller than 0 in the test distance
    while ( (int)distance >= 500 || (int)distance == 0)
    {
         distance = Distance();
    }
    ultrasonic[num] = distance;
    num++;
	delay(10);
  }
  num = 0;
  bubble(ultrasonic, 5);
  distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3;
  
  printf("distance:%f\n",distance);      
  return distance;
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
int main()
{
  float distance;
  
  //Initialize wiringPi
  wiringPiSetup();
  
  //Initialize the Servo IO as the output mode
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

   //Initialize ultrasonic pin
  pinMode(EchoPin, INPUT);    
  pinMode(TrigPin, OUTPUT);   

  key_scan();

  while(1)
  {
   distance = Distance_test();

   if (distance > 80)
   {
	 servo_appointed_detection(ServoPos_middle);
     run(100, 100);      
   }
   else if (distance >= 50 && distance <= 80)
   {
	 servo_appointed_detection(ServoPos_middle);
     run(80, 80);      
   }
   else if (distance < 50)
   {
	 servo_appointed_detection(ServoPos_right);
     spin_right(150,150);    
	 delay (200); 
     brake (1);
     distance = Distance_test();    
     if (distance >= 50)
     {
	   servo_appointed_detection(ServoPos_middle);
       run(100, 100);    
     }
     else if (distance < 50)
     {
	   servo_appointed_detection(ServoPos_left);
	   spin_left(150,150);        
	   delay(800);
       brake(1);
      distance =  Distance_test();  
       if (distance >= 50)
       {
		 servo_appointed_detection(ServoPos_middle);
         run(100, 100); 
       }
       else if (distance < 50)
       {
		 servo_appointed_detection(ServoPos_left);
         spin_left(150,150); 
		 delay (400);
         brake(1);
       }
     }
   }
 }
 return 0;
}

