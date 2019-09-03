/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         bluetooth_control.c
* @author       liusen
* @version      V1.0
* @date         2018.04.07
* @brief        Robot car is controlled by Bluetooth APP by Android Mobile phone
* @details      In this experiment, you need to use our bluetooth module
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringSerial.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>


#define run_car     '1'
#define back_car    '2'
#define left_car    '3'
#define right_car   '4'
#define stop_car    '8'

#define left_servo_over  '6'
#define right_servo_over  '7'
#define left_servo  '1'
#define right_servo  '2'
#define init_servo '5'
#define stop_servo  '8'


#define ON  1
#define OFF 0


#define HIGH 1
#define LOW  0

/*Car running status enumeration*/
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,

} enCarState;

enum {

  enTLEFT,
  enTRIGHT,
	enNODRIFT

}enCardriftState;

/*Car servo status enumeration*/
enum {
  enFRONTSERVOLEFT = 1,
  enFRONTSERVORIGHT,
  enSERVOUP,
  enSERVODOWN,
  enSERVOINIT,
  enSERVOLEFT,
  enSERVORIGHT,
  enSERVOSTOP,
  enSERVOFRONTINIT
} enServoState;

//Definition of Pin
int Left_motor_go = 28;       //AIN2 connects to wiringPi port 28 of Raspberry pi for control Left motor forward 
int Left_motor_back = 29;     //AIN1 connects to wiringPi port 29 of Raspberry pi for control Left motor back 

int Right_motor_go = 24;      //BIN2 connects to wiringPi port 24 of Raspberry pi for control Left motor forward 
int Right_motor_back = 25;    //BIN1 connects to wiringPi port 25 of Raspberry pi for control Left motor back

int Left_motor_pwm = 27;      //PWMA connects to wiringPi port 27 of Raspberry pi for control the speed of the left motor
int Right_motor_pwm = 23;     //PWMA connects to wiringPi port 23 of Raspberry pi for control the speed of the right motor

//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      9                  21                  7                   1
const int TrackSensorLeftPin1  =  9;   //The first tracking infrared sensor pin on the left is connected to  wiringPi port 9 of Raspberry pi
const int TrackSensorLeftPin2  =  21;  //The second tracking infrared sensor pin on the left is connected to  wiringPi port 21 of Raspberry pi
const int TrackSensorRightPin1 =  7;   //The first tracking infrared sensor pin on the right is connected to  wiringPi port 7 of Raspberry pi
const int TrackSensorRightPin2 =  1;   //The second tracking infrared sensor pin on the right is connected to  wiringPi port 1 of Raspberry pi

//Define variables to store data collected by each tracking infrared pin
int TrackSensorLeftValue1;
int TrackSensorLeftValue2;
int TrackSensorRightValue1;
int TrackSensorRightValue2;
char infrared_track_value[5] = {0};

int buzzer = 10;               //buzzer is connected to  wiringPi port 10 of Raspberry pi

unsigned int g_CarSpeedControl = 150;

int FrontServoPin = 4;

int EchoPin = 30;         //EchoPin is connected to  wiringPi port 30 of Raspberry pi
int TrigPin = 31;         //TrigPin is connected to  wiringPi port 31 of Raspberry pi

int LED_R = 3;           //LED_R is connected to  wiringPi port 3 of Raspberry pi
int LED_G = 2;           //LED_G is connected to  wiringPi port 2 of Raspberry pi
int LED_B = 5;           //LED_B is connected to  wiringPi port 5 of Raspberry pi

int OutfirePin = 8;      //Outfire motor is connected to  wiringPi port 8 of Raspberry pi
int g_BoolFire = 0;      //Outfire state


int FrontServoLeftRightPos = 55;
unsigned char g_frontservopos = 55;

unsigned short int g_LeftPos = 110;
unsigned short int g_MidPos = 55;
unsigned short int g_RightPos = 0;

int ServoFlags;
int g_ServoState = enSERVOSTOP;
int g_lednum = 0;        

int fd;

int g_num=0;
int g_packnum=0;

int serialtime = 5000;
int count = 20;

/*Serial port data Settings*/
char InputString[512] = {0};  //This array is used to store received content
int NewLineReceived = 0;      
int StartBit  = 0;            
int g_CarState = enSTOP;      //1run 2back 3left 4right 0stop
int g_CarDrift = enNODRIFT;
char ReturnTemp[512] = {0};   //This array is used to store return value

/*Mode switching*/
int g_modeSelect = 0; //0: default;  
                      //2:tracking mode 3:Ultrasonic obstacle avoidance mode
					  //4: color_led mode 
char g_motor = 0;
int position = 0;

float Distance_test();
void bubble(unsigned long *,int );
void Servo_Control_Thread(void);
void Beepdelay(int mode, int delay);
/**
* Function       servo_pulse
* @author        liusen
* @date          2018.03.24
* @brief        Define a pulse function to generate the PWM value in the analog mode. 
*                The base pulse is 20ms, 
*                and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
* @param[in1]    ServPin
* @param[in2]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int v_iServoPin, int myangle)
{
  int PulseWidth;                    //Define the pulse width variable
  PulseWidth = (myangle * 11) + 500; //Convert the Angle to 500-2480 pulse width
  digitalWrite(v_iServoPin, HIGH);      
  delayMicroseconds(PulseWidth);    
  digitalWrite(v_iServoPin, LOW);     
  delay(20 - PulseWidth / 1000);     //Delay remaining time 
  return;
}
/**
* Function       servo_appointed_detection
* @author        liusen
* @date          2018.03.24
* @brief         The servo rotates to the specified angle
* @param[in]     pos
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 20; i++)     //Generate PWM, equivalent delay to ensure that it can be turned to the response angle
  {
    servo_pulse(FrontServoPin, pos); //Generate the PWM in the analog mode
  } 
}


/**
* Function       servo_left
* @author        liusen
* @date          2018.03.24
* @brief         Camera servo turn left
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_left()
{
    int pos, i;
    pos = FrontServoLeftRightPos;
   	servo_pulse(FrontServoPin, pos); 
	pos += 1;
	FrontServoLeftRightPos = pos;
	if (FrontServoLeftRightPos >= g_LeftPos)
	{
		FrontServoLeftRightPos = g_LeftPos;
	}
}

/**
* Function       servo_right
* @author        liusen
* @date          2018.03.24
* @brief         Camera servo turn right 
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_right()
{  
    int pos, i;
    pos = FrontServoLeftRightPos;
   	servo_pulse(FrontServoPin, pos); 
	pos -= 1;
	FrontServoLeftRightPos = pos;
	if (FrontServoLeftRightPos <= g_RightPos)
	{
		FrontServoLeftRightPos = g_RightPos;
	}
}

/**
* Function       front_servo_left
* @author        liusen
* @date          2018.03.24
* @brief         front servo turn left
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void front_servo_left()
{
    servo_appointed_detection(g_LeftPos);
}

/**
* Function       front_servo_right
* @author        liusen
* @date          2018.03.24
* @brief         front servo turn right
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void front_servo_right()
{ 
      servo_appointed_detection(g_RightPos);
}
/**
* Function       servo_init
* @author        liusen
* @date          2018.03.24
* @brief         The position of the servo is initialized
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_init()
{
	int i = 0;

	for (i = 0; i < 10; i++)
	{
		servo_pulse(FrontServoPin, g_MidPos);
	}
	FrontServoLeftRightPos = g_MidPos;
}



/**
* Function       servo_front_init
* @author        liusen
* @date          2018.03.24
* @brief         servo front init
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_front_init()
{
	servo_pulse(FrontServoPin, g_MidPos);
}

/**
* Function       servo_stop
* @author        liusen
* @date          2018.03.24
* @brief         servo_stop
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_stop()
{
	
}


/**
* Function       color_led
* @author        liusen
* @date          2018.03.24
* @brief         7 different colors formed by different combinations of R,G and B
* @param[in1]    Red
* @param[in2]    Green
* @param[in3]    Blue
* @retval        void
* @par History   
*/
void color_led(int v_iRed, int v_iGreen, int v_iBlue)
{
  v_iRed == ON ? digitalWrite(LED_R, HIGH): digitalWrite(LED_R, LOW);
 
  v_iGreen == ON ? digitalWrite(LED_G, HIGH) : digitalWrite(LED_G, LOW);
  
  v_iBlue == ON ? digitalWrite(LED_B, HIGH) : digitalWrite(LED_B, LOW);
}

/**
* Function       Distance
* @author        liusen
* @date          2018.03.24
* @brief         measure the distance by Ultrasonic
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
		
		if ((stop - start) > 30000)  //Maximum time value (5m)：10/340=0.03s
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
	printf("distance: %f\n", distance);
	return distance;
}


/**
* Function       track_test
* @author        liusen
* @date          2018.03.24
* @brief         Tracking pin testing
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void track_test()
{
  //When the black line is detected,the corresponding indicator of the tracking module is on,and the port level is LOW
  //When the black line is not detected,the corresponding indicator of the tracking module is off,and the port level is HIGH
  TrackSensorLeftValue1 = digitalRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2 = digitalRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);
 
  infrared_track_value[0] = ((TrackSensorLeftValue1 == LOW)? '1' : '0');
  infrared_track_value[1] = ((TrackSensorLeftValue2 == LOW)? '1' : '0');
  infrared_track_value[2] = ((TrackSensorRightValue1 == LOW)?'1': '0');
  infrared_track_value[3] = ((TrackSensorRightValue2 == LOW)? '1': '0');
  return;
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
*  @author        Danny
* @date          2017.08.16
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
* Function       whistle
* @author        Danny
* @date          2017.08.16
* @brief         whistle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void whistle()
{
  digitalWrite(buzzer, LOW);   //sound
  delay(100);                  
  digitalWrite(buzzer, HIGH);  //no sound
  delay(1);                    

  digitalWrite(buzzer, LOW);   //sound
  delay(200);                  
  digitalWrite(buzzer, HIGH);  //no sound
  delay(2);                    
  return;
}



/**
* Function       color_led_pwm
* @author        liusen
* @date          2018.03.24
* @brief         Color_LED light the specified color
* @param[in1]    v_iRed（0-255）
* @param[in2]    v_iGreen（0-255）
* @param[in3]    v_iBlue（0-255）
* @param[out]    void
* @retval        void
* @par History   
*/
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  softPwmWrite(LED_R, v_iRed);
  softPwmWrite(LED_G, v_iGreen);
  softPwmWrite(LED_B, v_iBlue);
  return;
}

/***********Mode2:Tracking_Mode*************/
/**
* Function       Tracking_Mode
* @author        liusen
* @date          2017.07.25
* @brief         Tracking_Mode
* @param[in1]    void
* @param[out]    void
* @retval        void
* @par History   
*/

void Tracking_Mode()
{
   char *p=ReturnTemp;
   memset(ReturnTemp,0,sizeof(ReturnTemp));
   track_test();

   //4 tracking pins level status
   // 0 0 X 0
   // 1 0 X 0
   // 0 1 X 0
   //Turn right in place,speed is 150,delay 80ms
   //Handle right acute angle and right right angle 
   if ( (TrackSensorLeftValue1 == LOW || TrackSensorLeftValue2 == LOW) &&  TrackSensorRightValue2 == LOW)
   {
     servo_appointed_detection(g_RightPos);
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
      servo_appointed_detection(g_LeftPos);
			spin_left(150, 150);
     	delay(80);
   }
   // 0 X X X
   //Left_sensor1 detected black line
   else if ( TrackSensorLeftValue1 == LOW)
   {
     servo_appointed_detection(g_LeftPos);
	 spin_left(150, 150);
   }
   // X X X 0
   //Right_sensor2 detected black line
   else if ( TrackSensorRightValue2 == LOW )
   {
   	servo_appointed_detection(g_RightPos);
    spin_right(150, 150);
   }
   //4 tracking pins level status
   // X 0 1 X
   else if ( TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == HIGH)
   {
   	 servo_appointed_detection(g_LeftPos);
     left(0, 220);
   }
   //4 tracking pins level status
   // X 1 0 X  
   else if (TrackSensorLeftValue2 == HIGH && TrackSensorRightValue1 == LOW)
   {
   	 servo_appointed_detection(g_RightPos);
     right(220, 0);
   }
   //4 tracking pins level status
   // X 0 0 X
   else if (TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == LOW)
   {
   	 servo_appointed_detection(g_MidPos);
     run(200, 200);
   }

}

/**
* Function       servo_color_carstate
* @author        liusen
* @date          2017.07.26
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
 void servo_color_carstate()
{
  float distance;
  int iServoPos = 0;
  int LeftDistance = 0;    
  int RightDistance = 0; 
  int FrontDistance = 0;  
  color_led_pwm(255, 0, 0);////LED_red
  back(80,80);            //Avoid stop suddenly
  delay(80);
  brake();
  
  servo_appointed_detection(g_RightPos);
  delay(500);
  distance = Distance_test();  
  RightDistance = distance;    
  //printf("rightdistance :%d\n",RightDistance);
 
  servo_appointed_detection(g_LeftPos);
  delay(500);
  distance = Distance_test();  
  LeftDistance = distance;
  // printf("leftdistance :%d\n",LeftDistance);

  servo_appointed_detection(g_MidPos);
  delay(500);
  distance = Distance_test();
  FrontDistance = distance;
  //printf("FrontDistance:%d\n",FrontDistance);
 
  if (LeftDistance < 30 && RightDistance < 30 && FrontDistance < 30  )
  { 
    //LED_magenta
    color_led_pwm(255, 0, 255);		
		servo_appointed_detection(g_RightPos);  
		spin_right(150,150);           
		delay(600);
  }
  else if ( LeftDistance >= RightDistance) 
  {
    //LED_blue
    color_led_pwm(0, 0, 255);
    servo_appointed_detection(g_LeftPos);   
		spin_left(150,150);          
		delay(200); 
  }
  else if (LeftDistance < RightDistance ) 
  {
     //LED_magenta
     color_led_pwm(255, 0, 255);
	 servo_appointed_detection(g_RightPos);   
	 spin_right(150,150);           
	 delay(200);
  }
} 

/**
* Function       Distane_test
* @author        liusen
* @date          2018.03.24
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
    while ( distance >= 500 || (int)distance == 0)
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
* Function       bubble
* @author        liusen
* @date          2018.03.24
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

/************Mode3: Ultrasonic obstacle avoidance Mode*************/
/**
* Function       Ultrasonic_avoidMode
* @author        liusen
* @date          2017.07.26
* @brief         Ultrasonic obstacle avoidance Mode
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void Ultrasonic_avoidMode()
{
	float distance = 0;
	distance = Distance_test();        
   if (distance > 50  )   
   {
    run(250, 250);
    color_led_pwm(0, 255, 0);
  }
  else if ((distance >= 30 && distance <= 50))
  {
    run(180, 180);
  }
  else if (  distance < 30  )
  {
    servo_color_carstate();
  }	
}

/**
* Function       myrandom
* @author        liusen
* @date          2017.07.26
* @brief         random(0-255)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
int myrandom()
{
	int rand_num = 0;
	srand((int)time(0));
	rand_num = rand()%255;
	return rand_num;	
}

/***********Mode4: Color_LED mode***********/
/**
* Function       LED_Color_Mode()
* @author        liusen
* @date          2017.07.26
* @brief         Color_LED mode
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void LED_Color_Mode()
{
	 int red , green, blue;
     servo_appointed_detection(position);
	 red = myrandom();
	 green = myrandom();
	 blue = myrandom();
     color_led_pwm( red, green, blue);
     position += 10;
     if(position > g_LeftPos)
     {
       position = g_RightPos;
     }
}

/**
* Function       app_paraparking
* @author        liusen
* @date          2018.04.08    
* @brief         Mode5：paraparking
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void app_paraparking(void)
{
	color_led_pwm(255,0,0);
	Beepdelay(3, 300);	
	servo_appointed_detection(g_RightPos);
	back(60, 60);
	delay(800);

	servo_appointed_detection(g_MidPos);
	Beepdelay(2, 110);
		
	servo_appointed_detection(g_LeftPos);

	Beepdelay(12, 25);

	servo_appointed_detection(g_MidPos);
	brake();
	Beepdelay(5, 50);

	run(60, 60);
	color_led_pwm(0,255,0);
	delay(400);
	
	brake();
	g_modeSelect = 0;

}
/**
* Function       app_revparking
* @author        liusen
* @date          2018.04.08    
* @brief         Mode6： revparking
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void app_revparking(void)
{
	
	servo_appointed_detection(g_RightPos);
	back(60, 60);
	delay(1500);
	servo_appointed_detection(g_MidPos);
	Beepdelay(2, 100);
	delay(600);
	brake();
	g_modeSelect = 0;

}

/**
* Function       app_driftmode
* @author        liusen
* @date          2018.04.08    
* @brief         Mode7：driftmode
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void app_driftmode(void)
{

	servo_appointed_detection(g_MidPos);
	run(200, 200);
	delay(1000);

	servo_appointed_detection(g_MidPos+10);
	spin_left(200, 200);
	delay(1600);	

	servo_appointed_detection(g_MidPos);
	run(200, 200);
	delay(1000);
	
	brake();
	g_modeSelect = 0;

}

/**
* Function       ModeBEEP
* @author        liusen
* @date          2017.07.26
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void ModeBEEP(int mode)
{
  int i;
  pinMode(buzzer, OUTPUT);
  for (i = 0; i < mode + 1; i++)
  {
    digitalWrite(buzzer, LOW); //sound
    delay(100);
    digitalWrite(buzzer, HIGH); //no sound
    delay(100);
  }
  delay(100);
  digitalWrite(buzzer, HIGH); //no sound
}

void Beepdelay(int mode, int delaytime)
{
	int i;
	for (i = 0; i < mode; i++)
	{
		digitalWrite(buzzer, LOW); //sound
		delay(delaytime);
		digitalWrite(buzzer, HIGH); //no sound
		delay(delaytime);
	}
}

/**
* Function       BeepOnOffMode
* @author        liusen
* @date          2017.07.26
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void BeepOnOffMode()
{
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);   //sound
  delay(1000);                
  digitalWrite(buzzer, HIGH);  //no sound
}

  /**
* Function       StringFind
* @author        liusen
* @date          2018.03.24    
* @brief         StringFind
* @param[in]     pSrc:source string; pDst:Find string; v_iStartPos:The starting position of the source string
* @param[out]    void
* @retval        void
* @par History   
*/

int StringFind(const char *pSrc, const char *pDst, int v_iStartPos)  
{  
    int i, j;  
    for (i = v_iStartPos; pSrc[i]!='\0'; i++)  
    {  
        if(pSrc[i]!=pDst[0])  
            continue;         
        j = 0;  
        while(pDst[j] !='\0' && pSrc[i+j]!='\0')  
        {  
            j++;  
            if(pDst[j]!=pSrc[i+j])  
            break;  
        }  
        if(pDst[j]=='\0')  
            return i;  
    }  
    return -1;  
} 

/**
* Function       serial_data_parse
* @author        liusen
* @date          2018.03.24
* @brief         The serial port data is parsed and the corresponding action is specified
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void serial_data_parse()
{
	if(StringFind((const char *)InputString, (const char *)"INFO", 0) > 0 )
	{
		printf("$TBOT#");
		NewLineReceived = 0;  
		memset(InputString, 0x00, sizeof(InputString));
		delay(100);
		printf("$TBOT#");  
		return;	
	}

	if(StringFind((const char *)InputString, (const char *)"START", 0) > 0 )
	{
		printf("$OK#");
		//g_autoupdata = 1;
		NewLineReceived = 0;  
		memset(InputString, 0x00, sizeof(InputString));  
		return;	
	}

	if(StringFind((const char *)InputString, (const char *)"MODE", 0) > 0
		&& StringFind((const char *)InputString, (const char *)"TBOT", 0) > 0)
	{
		if(InputString[11] == '0')
		{
			brake();
			g_CarState = enSTOP;
			g_modeSelect = 1;
			BeepOnOffMode();
		}
		else
		{
		  switch (InputString[10])
        	{
           case '0': g_modeSelect = 1; ModeBEEP(0); break;
           case '1': g_modeSelect = 1; ModeBEEP(1); break;//Color LED mode
           case '2': g_modeSelect = 2; ModeBEEP(2); break;//Tracking mode
           case '3': g_modeSelect = 3; ModeBEEP(3); break;//Ultrasonic obstacle avoidance Mode
           case '4': g_modeSelect = 4; ModeBEEP(4); break;//paraparking
           case '5': g_modeSelect = 5; ModeBEEP(5); break;//revparking
           case '6': g_modeSelect = 6; ModeBEEP(6); break;//driftmode
           default: g_modeSelect = 1; break;
      		}
      		delay(1000);
      		BeepOnOffMode();
		}
	  memset(InputString, 0x00, sizeof(InputString));
      NewLineReceived = 0;
      return;	  
	}
	
	
     //Analyze the control instructions of the servo sent by the host computer and execute corresponding operation
     //For exmaple:$4WD,PTZ180# servo turn 180°
	if (StringFind((const char *)InputString, (const char *)"PTZ", 0) > 0 && 
			StringFind((const char *)InputString, (const char *)"POS", 0) < 0)
	{
		int m_kp, i, ii;

		i = StringFind((const char *)InputString, (const char *)"PTZ", 0); 
		ii = StringFind((const char *)InputString, (const char *)"#", i);
		if (ii > i)
		{
			char m_skp[5] = {0};
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			
			m_kp = atoi(m_skp);        

			servo_appointed_detection(180 - m_kp);
			g_ServoState = enSERVOSTOP;
			NewLineReceived = 0;  
			memset(InputString, 0x00, sizeof(InputString));
			return;
		}
	}
  
	if (StringFind((const char *)InputString, (const char *)"PTZ", 0) > 0 && 
			StringFind((const char *)InputString, (const char *)"POS", 0) > 0)
	{
		int m_kp, i, ii;

		i = StringFind((const char *)InputString, (const char *)"PTZ", 0); 
		ii = StringFind((const char *)InputString, (const char *)",", i);
		if (ii > i)
		{
			char m_skp[5] = {0};
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			
			m_kp = atoi(m_skp);        

			servo_appointed_detection(180 - m_kp);
			g_frontservopos = 180 - m_kp;
			g_ServoState = enSERVOSTOP;
		}
		i = StringFind((const char *)InputString, (const char *)"POS", ii); 
		ii = StringFind((const char *)InputString, (const char *)"#", i);
		if (ii > i)
		{
			char m_skp[2] = {0};
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			
			m_kp = atoi(m_skp);      

			switch(m_kp)
			{
				case 1: g_LeftPos = g_frontservopos; break;
				case 2: g_MidPos = g_frontservopos; break;
				case 3: g_RightPos = g_frontservopos; break;
				default: break;
			}
		}

		NewLineReceived = 0;  
		memset(InputString, 0x00, sizeof(InputString));
		return;

  }

    //Analyze the control instructions sent by the host computer and execute corresponding operation
    //For exmaple:$4WD,CLR255,CLG0,CLB0# color_red
	if (StringFind((const char *)InputString, (const char *)"CLR", 0) > 0)
 	{
		int m_kp, i, ii, red, green, blue;
		char m_skp[5] = {0};
		i = StringFind((const char *)InputString, (const char *)"CLR", 0);
		ii = StringFind((const char *)InputString, (const char *)",", i);
		if (ii > i)
		{			
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			red =   m_kp;
		}
		i = StringFind((const char *)InputString, (const char *)"CLG", 0);
		ii = StringFind((const char *)InputString, (const char *)",", i);
		if (ii > i)
		{
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			green =   m_kp;
		}
		i = StringFind((const char *)InputString, (const char *)"CLB", 0);
		ii = StringFind((const char *)InputString, (const char *)"#", i);
		if (ii > i)
		{
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			blue =  m_kp;
			color_led_pwm(red, green, blue);
			NewLineReceived = 0;  
			memset(InputString, 0x00, sizeof(InputString));
			return;
		}
	}

  //Analyze the control instructions sent by the host computer and execute corresponding operation
  //For exmaple:$1,0,0,0,0,0,0,0,0,0#    advance
  if (StringFind((const char *)InputString, (const char *)"TBOT", 0) == -1 &&
		                    StringFind((const char *)InputString,(const char *)"#",0) > 0)
  {
    //puts(InputString);

    if (InputString[3] == '1')      
    {
      g_CarDrift = enTLEFT;
    }
    else if (InputString[3] == '2') 
    {
      g_CarDrift = enTRIGHT;
    }
    else if	(InputString[3] == '8') 
    {
      g_CarDrift = enNODRIFT;
    }

    if (InputString[5] == '1')    
    {
    	printf("whistle\n");
      whistle();
    }

    if (InputString[7] == '1')    
    {
      g_CarSpeedControl += 20;
      if (g_CarSpeedControl > 200)
      {
        g_CarSpeedControl = 200;
      }

    }
    if (InputString[7] == '2')    
    {
      g_CarSpeedControl -= 20;
      if (g_CarSpeedControl < 100)
      {
        g_CarSpeedControl = 100;
      }
    }

    if (InputString[11] == '1')//white
    {
      g_lednum++;
      if(g_lednum == 1)
      {
				color_led_pwm(255, 255, 255);
			}
			else if(g_lednum == 2)
			{
				color_led_pwm(255,0,0);
			}
			else if(g_lednum == 3)
	    {
				color_led_pwm(0,255,0);
			}
			else if(g_lednum == 4)
			{
				color_led_pwm(0,0,255);
			}
			else if(g_lednum == 5)
			{
				color_led_pwm(255,255,0);
			}
			else if(g_lednum == 6)
			{
				color_led_pwm(0,255,255);
			}
			else if(g_lednum == 7)
			{
				color_led_pwm(255,0,255);
			}
			else
			{
			   color_led_pwm(0,0,0);
			   g_lednum=0;
			}
    }
    
    if (InputString[11] == '2')//Red
    {
      color_led_pwm(255, 0, 0);
    }
    if (InputString[11] == '3')//Green
    {
      color_led_pwm(0, 255, 0);
    }
    if (InputString[11] == '4') //Blue
    {
      color_led_pwm(0, 0, 255);
    }
		if (InputString[11] == '5')//cyan
		{
		  color_led_pwm(0, 255, 255);
		}
	  if (InputString[11] == '6')//magenta
		{
		  color_led_pwm(255, 0, 255);
		}
	  if (InputString[11] == '7')//yellow
		{
		  color_led_pwm(255, 255, 0);
		}
		if (InputString[11] == '8') 
    {
      color_led_pwm(0, 0, 0);
    }

	switch(InputString[9])
	{
		case left_servo_over: g_frontservopos = g_LeftPos; g_ServoState = enSERVOSTOP; break;
		case right_servo_over: g_frontservopos = g_RightPos;  g_ServoState = enSERVOSTOP; break;
		case left_servo: g_ServoState = enSERVOLEFT; break;
		case right_servo: g_ServoState = enSERVORIGHT; break;
		case init_servo: g_ServoState = enSERVOINIT; break;
		case stop_servo: g_ServoState = enSERVOSTOP; break;
	}

   	if (InputString[1] != '0')
		{
      switch (InputString[1])
      {
        case run_car:   g_CarState = enRUN;  break;
        case back_car:  g_CarState = enBACK;  break;
        case left_car:  g_CarState = enLEFT;  break;
        case right_car: g_CarState = enRIGHT;  break;
        case stop_car:  g_CarState = enSTOP;  break;
        //default: g_CarState = enSTOP; break;
      }
    
		}
    memset(InputString, 0x00, sizeof(InputString));       
    NewLineReceived = 0;

		if(g_CarDrift != enNODRIFT)
		{
		  	switch (g_CarDrift)
				{
		   		case enTLEFT: 
					{
						spin_left(g_CarSpeedControl, g_CarSpeedControl);
	
						switch(g_CarState)
						{
							case enRUN: g_frontservopos = g_MidPos+10; FrontServoLeftRightPos = g_MidPos+10; break;
							case enBACK: g_frontservopos = g_MidPos+10; FrontServoLeftRightPos = g_MidPos+10; break;
							case enLEFT: g_frontservopos = g_LeftPos; FrontServoLeftRightPos = g_LeftPos; break;
							case enRIGHT: g_frontservopos = g_RightPos;FrontServoLeftRightPos = g_RightPos; break; 
						}
	
					} break;
					case enTRIGHT: 
					{
					   	spin_right(g_CarSpeedControl, g_CarSpeedControl);
					   	switch(g_CarState)
						{
							case enRUN: g_frontservopos = g_MidPos-10; FrontServoLeftRightPos = g_MidPos-10; break;
							case enBACK: g_frontservopos = g_MidPos-10; FrontServoLeftRightPos = g_MidPos-10; break;
							case enLEFT: g_frontservopos = g_LeftPos; FrontServoLeftRightPos = g_LeftPos; break;
							case enRIGHT: g_frontservopos = g_RightPos; FrontServoLeftRightPos = g_RightPos; break; 
						}
	
					} break;
				
				}
		}
		else
		{
		  	switch (g_CarState)
		  	{
				case enSTOP: brake(); break;
				case enRUN: run(g_CarSpeedControl, g_CarSpeedControl);   g_frontservopos = g_MidPos; FrontServoLeftRightPos = g_MidPos; break;
				case enBACK: back(g_CarSpeedControl, g_CarSpeedControl); g_frontservopos = g_MidPos; FrontServoLeftRightPos = g_MidPos; break;
				case enLEFT: left(g_CarSpeedControl-50, g_CarSpeedControl);	g_frontservopos = g_LeftPos - 20; FrontServoLeftRightPos = g_LeftPos - 20; break;
				case enRIGHT:right(g_CarSpeedControl, g_CarSpeedControl-50); g_frontservopos = g_RightPos + 20; FrontServoLeftRightPos = g_RightPos + 20; break;

			}
		}
  }
}

/**
* Function       itoa
* @author        liusen
* @date          2018.03.24
* @brief         Converts integer Numbers into characters
* @param[in]     void
* @retval        void
* @par History   
*/
void itoa(int i, char *string)
{
	sprintf(string,"%d", i);
	return;
}

/**
* Function       serial_data_postback
* @author        liusen
* @date          2018.03.24
* @brief         The collected sensor data is transmitted by the serial port to the host computer for display
* @param[in]     void
* @retval        void
* @par History   
*/
void serial_data_postback(int fd)
{
  char *p= ReturnTemp;
  char str[25];
  float distance;
  memset(ReturnTemp,0,sizeof(ReturnTemp));
  //ultrasonic
  distance = Distance();
  if((int)distance == -1)
  {
  	  distance = 0;
  }
  strcat(p, "$TBOT,CSB");
  itoa((int)distance, str);
  strcat(p, str);
   //voltage
  strcat(p, ",PV12.6");
  //gray level
  strcat(p, ",GS0");
  //tracking
  strcat(p, ",LF");
  track_test();
  strcat(p,infrared_track_value);
  //infrared obstacle avoidance
  strcat(p, ",HW");
  //infrared_avoid_test();
  strcat(p,"00");
  //light-seeking
  strcat(p, ",GM");
  //follow_light_test();
  strcat(p, "00");
  strcat(p, "#");
  printf("ReturnTemp:%s\n",p);
  serialPrintf(fd, p);
  return;
}

/**
* Function       serialEvent
* @author        liusen
* @date          2018.03.24
* @brief         Serial port parsing data packet
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void serialEvent()
{ 
  int UartReturnCount;
  char uartvalue = 0;
  while(1)
  {
    UartReturnCount = serialDataAvail(fd);
  	if( UartReturnCount == 0)
   {
	  break;
   }
  	else if (UartReturnCount > 0 )
   {
     while(UartReturnCount--)
	 {
	   uartvalue = (char)serialGetchar(fd);
	   if(uartvalue == '$')
	   {
		 StartBit = 1;
		 g_num = 0;
	   }
	  if(StartBit == 1)
	   {
		 InputString[g_num]= uartvalue;
	   }
      if(StartBit ==1 && uartvalue == '#')
	   {
		 NewLineReceived = 1;
		 StartBit = 0;
		 g_packnum = g_num;
	   //  printf("inputstring:%s\n", InputString);
	   }
	   g_num++;
	
	   if(g_num >= 80)
	   {
		 g_num=0;
		 StartBit=0;
		 NewLineReceived=0;
	   }
     }
   }
  }
}

/**
* Function       Servo_Control_Thread
* @author        liusen
* @date          2017.11.02
* @brief         Servo_Control_Thread
* @param[in]     void
* @retval        void
* @par History   
*/

void Servo_Control_Thread(void)
{
	static int i_frontservopos = 0;
	
	if (g_frontservopos != i_frontservopos)
	{
		servo_appointed_detection(g_frontservopos);
		i_frontservopos = g_frontservopos;
		return;
	}

	switch (g_ServoState)
	{		
		case enSERVOLEFT:			
			servo_left();
			break;
		case enSERVORIGHT: 			
			servo_right();
				break;
		case enSERVOINIT:
			servo_init();
			g_ServoState = enSERVOSTOP;
			break;
		case enSERVOSTOP: 			
			servo_stop();
			break;

	}
	
}

/**
* Function       main
* @author        liusen
* @date          2018.03.24
* @brief          Analyze the control instructions sent by the host computer and execute corresponding operation
* @param[in]     void
* @retval        void
* @par History   
*/
int main()
{
  g_modeSelect = 1;

  wiringPiSetup();

  //Initialize the motor drive IO as the output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);
  softPwmCreate(OutfirePin,0,255);   
  
  //Initialize the tracksensor IO as the input mode
  pinMode(TrackSensorLeftPin1, INPUT);
  pinMode(TrackSensorLeftPin2, INPUT);
  pinMode(TrackSensorRightPin1, INPUT);
  pinMode(TrackSensorRightPin2, INPUT);
  
  //Initialize the buzzer IO as the output mode
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  
  pinMode(EchoPin, INPUT);   //Initialize the EchoPin as the input mode
  pinMode(TrigPin, OUTPUT);  //Initialize the TrigPin as the output mode

  //Initialize the RGB IO as the output mode
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  softPwmCreate(LED_R,0,255); 
  softPwmCreate(LED_G,0,255); 
  softPwmCreate(LED_B,0,255); 

  //Initialize the servo IO as the output mode
  pinMode(FrontServoPin, OUTPUT);

  servo_init();
	
  if ((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0)
  {
    fprintf(stderr, "Uable to open serial device: %s\n", strerror(errno));
		return -1;
  }
  while(1)
  {
   serialEvent();
   if (NewLineReceived)
   {	
    printf("serialdata:%s\n",InputString);
    serial_data_parse();  
		NewLineReceived = 0;
   }
   
    //Switch between different functional modes
   switch (g_modeSelect)
   {
    case 1: break; 
    case 2: Tracking_Mode(); break; 
    case 3: Ultrasonic_avoidMode();  break; 
    case 4: LED_Color_Mode(); break;  
    case 5: app_paraparking(); break;  				
		case 6: app_revparking(); break;				
		case 7: app_driftmode(); break;					
    	
   }
   Servo_Control_Thread(); 
   if(g_modeSelect == 1)
   {
   	serialtime--;
   	if(serialtime ==0)
   	{
		 count--;
		 serialtime = 5000;
		 if(count == 0)
		 {
	     	 serial_data_postback(fd);
			 serialtime = 5000;
			 count = 20;
		 }
   	}
   }
   usleep(10);
  }
 serialClose(fd); 
 return 0;
}




