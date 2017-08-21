#include <stdio.h>
#include <string.h>
#include <math.h>

#include "LPC17xx.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_uart.h"

#include "acc.h"		//accelerometer
#include "pca9532.h"	//arrays of leds - port expander
#include "light.h"		//light sensor
#include "rgb.h"		//RGB LED
#include "temp.h"		//temperature sensor
#include "led7seg.h"	//7-segment
#include "oled.h"		//LED Display
#include "rotary.h"		//rotary
#include "joystick.h"   //joystick


typedef enum{
	STABLE,
	MONITOR,
} STATE;


STATE CURRENT_STATE = STABLE;

typedef enum{
	HELP_DISPLAY,
	DEFAULT_DISPLAY,
}DISPLAY;

DISPLAY CURRENT_DISPLAY = DEFAULT_DISPLAY;

uint8_t joyState    = 0;
const uint32_t numTicksPerSec = 1000; //in ms
static const int sensor_Sampling_Period = 5000; //in ms
const uint32_t MIN_LIGHT = 50; //in Lux
static const int MAX_TEMP = 26; //in degree celcius
static const int cmes_Sampling_Period = 16000; //in ms
uint32_t sensors_timer;

uint32_t SEG_timer;

uint32_t blink_timer;
int red_count;
int blue_count;
int purple_count;

volatile uint32_t msTicks = 0;

//int8_t x = 0, y = 0, z = 0;
int display;
uint32_t light_timer;
uint32_t light;
uint32_t arrayLED_timer;

uint32_t temp_timer;
float tempReading;

int32_t temperature = 0;
uint32_t acc_timer;


uint32_t uart_timer;

uint8_t SEG_value;
int joystick_flag;
int sw3_flag;
int uart_receive_flag;
int light_interrupt_flag;
int led_blue;
int led_red;
int UART_red;
int UART_blue;
int send_UART_help;
uint32_t rotary_timer;
int count, i;
char alpha[6] = {'A', 'B', 'C', 'D', 'E', 'F'};

uint32_t lastDebounceTime;
uint32_t debounceDelay = 1000; //in ms

int32_t xoff = 0;  // for accelerometer offset
int32_t yoff = 0;
int32_t zoff = 0;

//int acc_count;

volatile int PCA_DUTY;
uint32_t prevVal;

int8_t x = 0;				//acc reading for x
int8_t y = 0;				//acc reading for y
int8_t z = 0;				//acc reading for z

//Strings for OLED
uint8_t xString[40]={};
uint8_t yString[40]={};
uint8_t zString[40]={};
uint8_t accelerometer[40]= {};	//string array for acc readings for oled
uint8_t tempStr[40]= {};	//string array for temp sensor readings for oled
uint8_t tempStrUART[40] = {};
uint8_t light_string[40]= {};			//string array for light sensor readings for oled
uint8_t lightStrUART [40] = {};
uint8_t lightStr[40] = {};
uint8_t segFlag = 0;
uint8_t countUART[40] = {};
uint8_t j = 0;
uint8_t num = 000;
char segVal = '0';
int movement_flag = 0;
volatile char help_msg[48] = {};
volatile char emer_msg_blue[48] = {};
volatile char emer_msg_red[48] = {};
volatile char msg[48] = {};
volatile char entering_msg[48] = {};

const uint32_t lightLL = 0;


volatile uint32_t oneSecondTicks, unitTicks, threeSecondsTicks,
readingResSensorsTicks, sw4PressedTicks;

//Strings for OLED
int8_t OLED_EXT_MODE[15];
int8_t OLED_X[15];
int8_t OLED_Y[15];
int8_t OLED_Z[15];
int8_t OLED_LIGHT[15];
int8_t OLED_TEMPERATURE[15];

//Initialize all protocols

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}


void init_GPIO(){
	GPIO_SetDir(2, 1<<0, 1);
	GPIO_SetDir(2, 0, 0);
    GPIO_SetDir(2, 1<<1, 1);

    //Speaker
    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

	PINSEL_CFG_Type PinCfg;
	//SW3
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Funcnum = 1;
	PinCfg.Pinmode = 0;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);

	//Speaker
	GPIO_SetDir(0, 1<<26, 1);
	GPIO_SetDir(0, 1<<27, 1); //LM4811-clk
	GPIO_SetDir(0, 1<<28, 1); //LM4811-up/dn
	GPIO_SetDir(2, 1<<13, 1); //LM4811-shutdn
	GPIO_ClearValue(0, 1<<27); //LM4811-clk
	GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
	GPIO_ClearValue(2, 1<<13); //LM4811-shutdn


    // Initialize button SW4
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

	//Initialize light interrupt
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 5;
	PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<5, 0);

	//Initialize temperature interrupt
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, 1<<2, 0);

	return;
}
//SysTick Timer
void SysTick_Handler(void){
	msTicks++;
}

static uint32_t getTicks(void){
	return msTicks;
}

int rotary_dir;

void rotary1(void){
	if(PCA_DUTY <= 0)//Increase
	{
		PCA_DUTY = 10;
}
	else if((PCA_DUTY >0) && (PCA_DUTY< 90)) //Why does the LED switch off when PCA = 100?
	{
		PCA_DUTY = PCA_DUTY + 10;

		}
	else{
		PCA_DUTY = 99;
	}

	//LPC_PWM1->MR0 = LOUDNESS;
}

void rotary2(void){
	if(PCA_DUTY <= 0){
		PCA_DUTY = 0;

	}
	else if((PCA_DUTY >0) && (PCA_DUTY<= 99)){
		PCA_DUTY = PCA_DUTY - 10;

	}

	//LOUDNESS = LOUDNESS - 1000;
	//LPC_PWM1->MR0 = LOUDNESS;
	//else
	//PCA_DUTY = 0;
}
void setRotaryFlag(int f){

	//	uint32_t currentTime = getTicks();
	//	if((currentTime - lastDebounceTime) > rotary_debounceDelay){
	//		lastDebounceTime = currentTime;
	if((f==1) && (CURRENT_STATE == MONITOR)){
		//if(checkFlag(&rotary_timer, 100)) {
		rotary_dir = rotary_read();
		if((rotary_dir == 1)) //Add debounce timer
		{
			prevVal = 1;
			rotary1();

		}
		else if((rotary_dir == 2))//Decrease
		{
			prevVal = 2;
			rotary2();
		}
		else if((rotary_dir == 0)){
			if(prevVal == 1){
				rotary1();
				//prevVal = 1;
			}
			else if(prevVal==2){
				rotary2();
				//prevVal = 2;
			}
		}
		LPC_GPIOINT->IO0IntClr = 1<<24;
		LPC_GPIOINT->IO0IntClr = 1<<25;
		setLEDs();
		//playSong(song);
		//printf("Rotary direction: %d\n", rotary_dir);
	}
}
//}

//void EINT3_IRQHandler(void){
//	//printf("Reached handler\n");
//	//if(lightReading<=MIN_LIGHT){
//	//alarm();
//	//	disableRGB();
//	if (((LPC_GPIOINT->IO0IntStatF>>24)&0x1) | ((LPC_GPIOINT->IO0IntStatF>>25)&0x1)) {
//		LPC_GPIOINT->IO0IntClr = 1<<24;
//		LPC_GPIOINT->IO0IntClr = 1<<25;//printf("Inside handler\n");
//		setRotaryFlag(1);
//		//playSong(song);
//
//	}
//
//	//setLEDs();
//}



void setLEDs(void){
	//	GPIO_ClearValue(2, 1);
	pca9532_setBlink0Duty(PCA_DUTY);
	//printf("Duty cycle: %d\n", PCA_DUTY);
	pca9532_setBlink0Leds(0xFF00);
}


void pinsel_uart3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart3;
    pinsel_uart3();

    UART_ConfigStructInit(&uartCfg);
    //supply power & setup working parameters for uart3
    UART_Init(LPC_UART3, &uartCfg);
    //enable transmit for uart3
    UART_TxCmd(LPC_UART3, ENABLE);
}
volatile char DATAREPORT[43]={'\0'}, DATA_received[64]={'\0'}, DATA_validated[23]={'\0'};

void UART3_IRQHandler(void)
{
        if(LPC_UART3->IIR & 0x4)
        {
        	if(uart_receive_flag == 1){
        		//send_UART_help==1;
        		UART_help();
        	}
        }

}

void UART_help(void){
    uint8_t data = 0;
    uint32_t len = 0;
    int i;
    uint8_t incoming = 0;
    UART_Receive(LPC_UART3, &incoming, 16, BLOCKING);

    printf("Incoming = %s\n", incoming);
   // UART_SendString(LPC_UART3, "\n");
    UART_Send(LPC_UART3, &incoming, 16, BLOCKING);
//   do
//    {       UART_Receive(LPC_UART3, &data, 30, BLOCKING);
//            if (data != '\r')
//            {
//                    len++;
//                    DATA_received[len-1] = data;
//            }
//    } while((len<22) && (data!='\r'));
//    DATA_received[len]=0;
//     UART_SendString(LPC_UART3, &DATA_received);
//
//     printf("%s\n", DATA_received);
  /*  if(DATA_received[0] == '#' && DATA_received[21] =='#' && DATA_received[1]=='N' &&
                    DATA_received[5]=='_' && DATA_received[6]=='T' && DATA_received[11]=='_' &&
                    DATA_received[12]=='L' && DATA_received[16]=='_' && DATA_received[17]=='V')
    {
            DATA_validated[0] = '_';
            DATA_validated[21]= '\0';
            for(i = 1; i<21; i++)
                    DATA_validated[i] = DATA_received[i];
    }
    else
            DATA_validated[0] = '\0';
*/
}
//This checkFlag() function is used to check whether the sampling period for a given
//event has taken place. If yes, then flag = 1 is returned
uint8_t checkFlag (uint32_t *time, uint32_t samplingPeriod){
	uint8_t flag = 0;

	if((msTicks - *time) >= samplingPeriod){
		flag = 1;
		*time = msTicks;
	}
	return flag;
}

//Operations to perform when the state = STABLE
void stableState(void){
	baseboardinitializaton();
	sw3_flag = 0;
	light_interrupt_flag = 0;
    led_red = 0;
    led_blue = 0;
    movement_flag = 0;
    UART_blue =0;
    UART_red =0;
    uart_receive_flag = 0;
    send_UART_help=0;
    light_setLoThreshold(50);
}

//Operations to perform when the state = MONITOR
void monitorState(void){

	OLED_DISPLAY();
	acc();
	SEGMENT_DISPLAY();
	sw3_flag = 1;
	emergency();

	switch(CURRENT_DISPLAY){
	case DEFAULT_DISPLAY:
		removeHelpDisplayOLED();
		break;
	case HELP_DISPLAY:
		helpfunction();
		break;
	default:
		break;
	}


	//light_enable();
	//uartTest();
}

void emergency (void){
	if((light_interrupt_flag == 1) && (movement_flag == 1))
		led_blue = 1;

	if((led_red ==1) && (led_blue  == 1)){
		//blink_RED();
		UART_red =1;
		//blink_BLUE();
		UART_blue =1;
		blink_PURPLE();
	}
	if((led_red == 0) && (led_blue  == 1)){
		blink_BLUE();
		UART_blue =1;
	}
	if((led_red ==1) && (led_blue  == 0)){
		blink_RED();
		UART_red =1;
	}
}
//Blink the RED LED at 333 ms frequency
void blink_RED(void){

	if(checkFlag(&blink_timer, 333)){
		red_count++;
	}
	if(!((red_count%2) == 0)){
			GPIO_SetValue( 2, 1<<0);
	}
	else if((red_count%2) == 0){
			GPIO_ClearValue( 2, 1<<0);
	}
}

//Blink the BLUE LED at 333 ms frequency
void blink_BLUE(void){
	if(checkFlag(&blink_timer, 333)){
		blue_count++;
	}
	if(!((blue_count%2) == 0)){
			GPIO_SetValue( 0, (1<<26) );


	}
	else if((blue_count%2) == 0){
			GPIO_ClearValue( 0, (1<<26) );


	}
}
//Blink the BLUE LED at 333 ms frequency
void blink_PURPLE(void){
	if(checkFlag(&blink_timer, 333)){
		purple_count++;
	}
	if(!((purple_count%2) == 0)){
			GPIO_SetValue( 0, (1<<26) );
			GPIO_SetValue( 2, 1<<0);
	}
	else if((purple_count%2) == 0){
			GPIO_ClearValue( 0, (1<<26) );
			GPIO_ClearValue( 2, 1<<0);
	}
}


//OLED display during MONITOR mode - incomplete
void OLED_DISPLAY(){
	//oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(25, 0, "MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK); //1st value = col; 2nd value = row
	oled_line(0,8,96,8, OLED_COLOR_WHITE);
	uint32_t lightReading;
	float tR = (temp_Sample()/10.0);
	if(temp_Sample() >450)
		led_red = 1;
	sprintf(tempStr, "Temp: %#.2f", tR);
	sprintf(tempStrUART, "%#.2f", tR);
	strcpy(lightStr, " ");

	lightReading = light_read();
	sprintf(lightStr, "Light: %d", lightReading);
	sprintf(lightStrUART,"%d", lightReading);

}

//Sample data from temperature sensor every 5 seconds
int temp_Sample(void){
	temp_init(&getTicks);
	temperature = temp_read();
	return temperature;

}

//Sample data from light sensor every 5 seconds
//uint32_t light_Sample(void){
//	light_init();
//	light_enable();
//	return light_read();
//}
int prevMagnitude = 0;
int magnitude = 0;
int magdiff = 0;
int mag_count = 0;
void acc(void) {


		  //	if(checkFlag(&acc_timer, 500)){
	    		//sampling x	, y, z to observe its behaviour
				acc_read(&x, &y, &z);
				x = x+xoff;
				y = y+yoff;
				z = z+zoff;

				magnitude = sqrt((x*x) + (y*y) + (z*z));
				if(mag_count == 0){
					prevMagnitude = magnitude;
					mag_count++;
				}
				magdiff = magnitude - prevMagnitude;
				prevMagnitude = magnitude;
				if (magdiff>abs(10))
					movement_flag = 1;
				else
					movement_flag = 0;
				//prevMagnitude = magnitude;
				strcpy(accelerometer, " ");
	    		sprintf(accelerometer,"%d %#3d %#3d",x,y,z);
	    		sprintf(xString,"%d",x);
	    		sprintf(yString,"%d",y);
	    		sprintf(zString,"%d",z);

}
/*void movement_detection(void){

  		if(acc_count >= 5){
  			movement_flag=1;
  		printf("GPIO Interrupt 2.10\n");
  		}
  		else
  			movement_flag=0;
  		acc_count =0;

}*/
void check_SW4(void){
	uint32_t currTime = getTicks();
	if((currTime - lastDebounceTime) > debounceDelay){
	lastDebounceTime = currTime;
	if(!((GPIO_ReadValue(1) >> 31) & 0x01)){

		if(CURRENT_STATE== STABLE){
			oled_putString (21, 15, "Entering", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			oled_putString (24, 25, "Monitor", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			oled_putString (28, 35, "Mode", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			Timer0_Wait(1000);
			sprintf(entering_msg, "Entering MONITOR Mode");
			strcat(entering_msg, "\r\n");
			UART_SendString(LPC_UART3, entering_msg);
			CURRENT_STATE= MONITOR;
			oled_clearScreen(OLED_COLOR_BLACK);

		}
		else{
			oled_clearScreen(OLED_COLOR_BLACK);
			oled_putString (0, 35, "Entering Stable", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString (40, 45, "Mode", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			Timer0_Wait(1000);
			CURRENT_STATE = STABLE;
		}
	}

	}
}

void baseboardinitializaton (void){

		/*function to clear and off everthing before start of program*/

		led7seg_setChar(' ', FALSE);
		oled_clearScreen(OLED_COLOR_BLACK);
		pca9532_setLeds(0,0xffff);
		GPIO_ClearValue(0,1<<26);
		GPIO_ClearValue(2,1<<0);
		//GPIO_ClearValue( 2, (1<<1) );
		count=0;
		SEG_value = 0;


}
// 7 Segment Display
void SEGMENT_DISPLAY(void)
{
//char alpha[6] = {'A', 'B', 'C', 'D', 'E', 'F'};

if(checkFlag(&SEG_timer, 1000)){

if(count < 16){

	if(count>=10){
		led7seg_setChar(alpha[i], FALSE);
		if((alpha[i]=='A') || (alpha[i]=='F')){
			oled_putString(18, 20, (uint8_t *) tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(18, 30, (uint8_t *) lightStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK); //*Line changed*
			oled_putString(18,10,(uint8_t*)accelerometer,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			if(alpha[i]=='F'){

				uartTest();

			}
		}
		count++;
		i++;
	}
	else{
		count++;
		led7seg_setChar('0'+SEG_value, FALSE);
		if(SEG_value==5){
			oled_putString(18, 20, (uint8_t *) tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(18, 30, (uint8_t *) lightStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(18,10,(uint8_t*)accelerometer,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		}
		SEG_value++;
	}
}

else{
	count = 0;
	i=0;
	SEG_value = 0;
	led7seg_setChar('0'+SEG_value, FALSE);
}

}
return;
}

//Help display on the OLED
void helpDisplayOLED(void){
	oled_rect(17,38,80,61, OLED_COLOR_WHITE);
	oled_putString (20, 41, "Need help?", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString (25, 51, "Yes", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString (60, 51, "No", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}
void removeHelpDisplayOLED(void){
	oled_rect(17,38,80,61, OLED_COLOR_BLACK);
	oled_putString (20, 41, "Need help?", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
	oled_putString (25, 51, "Yes", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
	oled_putString (60, 51, "No", OLED_COLOR_BLACK, OLED_COLOR_BLACK);
	joystick_flag = 0;
}

void calculateCountUART(void){
	sprintf(countUART, "%03d", num);
	num++;
	//j++;
}
//UART test code
void uartTest(void){
		sprintf(emer_msg_blue, "Movement in darkness was Detected.");
		strcat(emer_msg_blue, "\r\n");
		sprintf(emer_msg_red, "Fire was Detected.");
		strcat(emer_msg_red, "\r\n");

	    if((UART_blue == 1) && (UART_red==0)){
	    	UART_SendString(LPC_UART3, emer_msg_blue);
	    	//UART_SendString(LPC_UART3, help_msg);
	    	//strcpy(countUART, " ");
	    }
	    else if((UART_red == 1) && (UART_blue==0)){
	    	UART_SendString(LPC_UART3,emer_msg_red) ;
	    	//UART_SendString(LPC_UART3, help_msg);
	    	//strcpy(countUART, " ");
	    }
	    else if ((UART_red == 1) && (UART_blue==1)){
	    	UART_SendString(LPC_UART3,emer_msg_red) ;
	    	UART_SendString(LPC_UART3, emer_msg_blue);
	    	//UART_SendString(LPC_UART3, help_msg);
	    }

		calculateCountUART();
		sprintf(msg, "%s_-_T%s_L%s_AX%s_AY%s_AZ%s",countUART, tempStrUART, lightStrUART, xString, yString, zString);
		strcat(msg, "\r\n");
		UART_SendString(LPC_UART3, msg);
		strcpy(countUART, " ");


}


void help_joystick()
{
	int right = 0;
	int left = 0;
	int joystick_dir;
	joystick_dir = joystick_read();


	  if(joystick_dir == 0x10){
	  	 right++;
	  	 left = 0;
	  }
	  if(joystick_dir == 0x08) {
	  	left++;
	  	right = 0;
	  }
	  if(right > 0){
		  display = 2;
		  oled_putString (60, 51, "No", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		  right = 0;
		  CURRENT_DISPLAY= DEFAULT_DISPLAY;

	  }

	  if(left > 0){
		   display = 3;
		   oled_putString (25, 51, "Yes", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		   left = 0;
		sprintf(help_msg, "Help Required");
		strcat(help_msg, "\r\n");
		UART_SendString(LPC_UART3, help_msg);
	//	uart_receive_flag == 1;
		CURRENT_DISPLAY= DEFAULT_DISPLAY;

	  }
	}

void EINT3_IRQHandler(void)
{
	//Light interrupt
	if((LPC_GPIOINT->IO2IntStatF>>5) & 0x1)
	{
		//clears light interrupt

		//GPIO_SetValue( 2, 1<<0);
			/*if (movement_flag == 1){
				led_blue == 1;
				printf("Light Interrupt: %d\n", light_read());
			}*/

		light_interrupt_flag = 1;
		//led_blue = 1;

	    light_setLoThreshold(0); // below this number will interrupt

		LPC_GPIOINT->IO2IntClr |= 1<<5;
		light_clearIrqStatus();
	}
	if ((LPC_GPIOINT->IO2IntStatF>>10)& 0x1) //SW3 innterrupt
	{
		if(sw3_flag == 1){
     //   printf("GPIO Interrupt 2.10\n");
        if(CURRENT_DISPLAY== DEFAULT_DISPLAY){
        	CURRENT_DISPLAY= HELP_DISPLAY;

        }
        else
        	CURRENT_DISPLAY= DEFAULT_DISPLAY;
		}
		sw3_flag == 0;
        // Clear GPIO Interrupt P2.10
        LPC_GPIOINT->IO2IntClr |= 1<<10;
	}



    //      joystick

		if (((LPC_GPIOINT->IO2IntStatF>>4)&0x1) |((LPC_GPIOINT->IO0IntStatF>>16)&0x1))
		{
			if(joystick_flag ==1){
				help_joystick();
			}
				joystick_flag = 0;
				LPC_GPIOINT->IO0IntClr |= 1<<16;
				LPC_GPIOINT->IO2IntClr |= 1<<4;

		}
		if (((LPC_GPIOINT->IO0IntStatF>>24)&0x1) | ((LPC_GPIOINT->IO0IntStatF>>25)&0x1)) {
				LPC_GPIOINT->IO0IntClr = 1<<24;
				LPC_GPIOINT->IO0IntClr = 1<<25;//printf("Inside handler\n");
				setRotaryFlag(1);
				//playSong(song);

			}


}
void helpfunction(void){
	helpDisplayOLED();
	joystick_flag=1;
}

//Main function
int main(void)
{
	//uint32_t threshold = 20;
		init_i2c();
	    init_ssp();
	    init_GPIO();
	    rgb_init();
	    pca9532_init();
	    joystick_init();
	    acc_init();
	    oled_init();
	    led7seg_init();
	    init_uart();
	    SysTick_Config(SystemCoreClock/1000);
	    uint8_t nextState[10] = {};
	    led_red = 0;
	    led_blue = 0;
	    PCA_DUTY = 0;
	    send_UART_help=0;

	    UART_blue =0;
	    UART_red =0;
	    joystick_flag = 0;
	    sw3_flag = 0;
	    uart_receive_flag = 0;
	    light_interrupt_flag = 0;

	    rotary_timer = msTicks;
	    prevVal = 0;

	    light_setRange(LIGHT_RANGE_1000);
	    light_setLoThreshold(50); // below this number will interrupt
	    light_setHiThreshold(972); // above this number will interrupt
		light_setIrqInCycles(LIGHT_CYCLE_1);
		light_clearIrqStatus();

	   // NVIC_SetPriority(EINT3_IRQn,2);

		//EINT3 Configuration
		LPC_SC->EXTINT = 1;


	    //SW3 Interrupt
		LPC_GPIOINT->IO2IntClr |= 1 << 10; //SW3
		LPC_GPIOINT->IO2IntEnF |= 1 << 10; //switch


		//Light interrupt
		light_enable();
	    LPC_GPIOINT->IO2IntClr |= 1<<5; // IntClr - clearing the interrupt after it has been processed
	    LPC_GPIOINT->IO2IntEnF |= 1<<5; //IO2 as it is port 2, and 1<<5 as pin 5, F- falling edge
	   // light_enable();


	    //Joystick interrupt
		LPC_GPIOINT->IO0IntClr |= 1<<16;
		LPC_GPIOINT->IO2IntClr |= 1<<4;
        LPC_GPIOINT->IO0IntEnF |= 1<<16; //right
        LPC_GPIOINT->IO2IntEnF |= 1<<4;  //left

        //rotary
		LPC_GPIOINT->IO0IntClr = 1<<24;
		LPC_GPIOINT->IO0IntEnF |= 1<<24;
		LPC_GPIOINT->IO0IntClr = 1<<25;
		LPC_GPIOINT->IO0IntEnF |= 1<<25;

		//UART interrupt config
		UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);
		NVIC_EnableIRQ(UART3_IRQn);


        NVIC_ClearPendingIRQ(EINT3_IRQn);
        //NVIC_SetPriorityGrouping(4);
        //NVIC_SetPriority(EINT3_IRQn, NVIC_EncodePriority(4,2,0));
        NVIC_EnableIRQ(EINT3_IRQn); // Enable EINT3

        uint8_t btnSW4 = 0;
		baseboardinitializaton();
		num = 0;

	count = 0;
	i=0;
	SEG_value = 0;

	sensors_timer = msTicks;

	SEG_timer = msTicks;

	blink_timer = msTicks;

	temp_timer = msTicks;

	light_timer = msTicks;

	arrayLED_timer = msTicks;

	uart_timer = msTicks;

	uint8_t button = 0;


	light_init();





	while(1){
		check_SW4();
		switch(CURRENT_STATE){
		case STABLE:
			stableState();

			break;
		case MONITOR:
			monitorState();

			break;
		default:
			break;
		}
	}
	return 0;
}