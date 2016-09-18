/*
 * libsoft.h
 *
 *  Created on: Sep 18, 2016
 *      Author: Nick
 */

typedef enum SERVO_loc{
	PA0 = 0x00,
	PA1 = 0x01
}SERVO_loc;

typedef struct SERVO_type{

	int SERVOx_num;	//Interal servo name

	SERVO_loc SERVOx_loc; //Servo location
	int SERVOx_deg;
}servo;

void init_RC(void);
void init_GPIO(void);
void init_Servos(void);
void set_Servos(void);
