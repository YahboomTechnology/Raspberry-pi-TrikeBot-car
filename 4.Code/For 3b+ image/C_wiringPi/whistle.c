/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         whistle.c
* @author       xiaozhen
* @version      V1.0
* @date         2018.03.19
* @brief        whistle
* @details
* @par History  
*/
#include <wiringPi.h>


int buzzer = 10;                //buzzer is connected to  wiringPi port 10 of Raspberry pi

/**
* Function       main 
* @author        xiaozhen 
* @date          2018.03.19
* @brief         whistle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
int  main()
{
  wiringPiSetup();    
	
   //Initialize the buzzer IO as the output mode
  pinMode(buzzer, OUTPUT);
  
  while( 1 )
  {
  digitalWrite(buzzer, LOW);   //sound
  delay(100);                  //delay 100ms
  digitalWrite(buzzer, HIGH);  //no sound
  delay(1);                    //delay 1ms

  digitalWrite(buzzer, LOW);   //sound
  delay(200);                  //delay 200ms
  digitalWrite(buzzer, HIGH);  //no sound
  delay(2);                    //delay 1ms
  
  }
  return 0;
}
