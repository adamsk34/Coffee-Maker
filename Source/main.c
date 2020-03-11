#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "codec.h"

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#define DEBOUNCE_WINDOW 20
#define DOUBLE_PRESS_WINDOW 500
#define LONG_PRESS_WINDOW 3000
#define SERVO_POSITION_NEUTRAL 600
#define SERVO_POSITION_ESPRESSO 1100
#define SERVO_POSITION_MILK 1800
#define SERVO_POSITION_CHOCOLATE_MILK 2200
double  NOTEFREQUENCY = 0.015;	
double NOTEAMPLITUDE = 500.0;		

void vButtonEventGenerator(void *pvParameters);
void vButtonListener(void *pvParameters);
void vDPDisableSinglePress(void *pvParameters);
void vIdle(void *pvParameters);
void vShowCoffeeSelected(void *pvParameters);
void vWaitIfSinglePressed(void *pvParameters);
void vWaitIfLongPressed(void *pvParameters);
void vServeEspresso(void *pvParameters);
void vServeLatte(void *pvParameters);
void vServeMocha(void *pvParameters);
void vCountDown(void *pvParameters);
void vCountDownEspresso(void *pvParameters);
void vCountDownLatte(void *pvParameters);
void vCountDownMocha(void *pvParameters);

void doublePressButtonEvent(void);
void longPressButtonEvent(void);
void pressButtonEvent(void);
void singlePressButtonEvent(void);
void unpressButtonEvent(void);

void nextCoffeeType(void);
void initDebounceTimer(void);
void enableDebounceInterrupt(void);

void setSysTick(void);
void InitServos (void);
void InitPWMTimer4(void);
void SetupPWM(void);

typedef struct {
		float tabs[8];
		float params[8];
		uint8_t currIndex;
} fir_8;

int getCoffeeTime(void);
void playSound(int);
float updateFilter(fir_8* theFilter, float newValue);
void initFilter(fir_8* theFilter);
void changeCoffeeTime(void);// increase time of selected coffee type by 1
void resetCoffeeTime(void); //change time of selected coffee type to be 0 

enum states {
	cyclingCoffeeTypes,
	countDown,
	programming//to program time
};
int currState = cyclingCoffeeTypes;

enum debounceStates {
	startDebounce,
	activeDebounce,
	doneDebounce
};
int currDebounceState = doneDebounce;

enum coffeeTypes {
	mochaCoffee,
	espressoCoffee,
	latteCoffee
};

enum buttonStates {
	buttonDown,
	buttonUp
};
int currButtonState = buttonUp;// accounts for debounce

int coffeeSelected = mochaCoffee;

int pressButtonOccurred = 0;// false
int unpressButtonOccurred = 0;// false

// double press disable single press
int dpDisableSinglePress = 0;// false
// long press disable single press
int lpDisableSinglePress = 0;// false

//volatile uint32_t msTicks;// Counts 1ms timeTicks

TickType_t ticksLastPress = NULL;
TickType_t ticksLastUnpress = NULL;

GPIO_InitTypeDef GPIO_Initstructure;
TIM_TimeBaseInitTypeDef timer_InitStructure;

int espressoTime = 5;
int milkTime = 5;
int chocoMilkTime = 6;

//int countDown = 0;//false; It is 1(true) if start to count down

//====gobal value for sound function================
GPIO_InitTypeDef GPIO_InitStructure;
volatile uint32_t sampleCounter = 0;
volatile int16_t sample = 0;
double sawWave = 0.0;
float filteredSaw = 0.0;
fir_8 filt;
//===================================================

int main(void) {
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
	enableDebounceInterrupt();
  
	setSysTick();
	InitServos();
	InitPWMTimer4();
	SetupPWM();
	
	//xTaskCreate(vCountDown, (const char*)"Countdown Task",
		//STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL);
	
	xTaskCreate( vIdle, (const char*)"Idle Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	xTaskCreate( vButtonListener, (const char*)"Button Listener Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	xTaskCreate( vShowCoffeeSelected, (const char*)"LEDs Show Selected Coffee Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	xTaskCreate( vButtonEventGenerator, (const char*)"Button Event Generator Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	vTaskStartScheduler();
}

void nextCoffeeType() {
	if(coffeeSelected == espressoCoffee) {
		coffeeSelected = latteCoffee;
	} else if(coffeeSelected == latteCoffee) {
		coffeeSelected = mochaCoffee;
	} else if(coffeeSelected == mochaCoffee) {
		coffeeSelected = espressoCoffee;
	}
}

void vButtonListener(void *pvParameters) {
	uint8_t button_pin = 0;
	uint8_t late_button_pin = 0;// what the value of `button_pin` was, last loop iteration

	for(;; late_button_pin = button_pin) {
		button_pin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		
		if(button_pin) {// true if the button is currently being pressed
			if(!late_button_pin) {// true if button_pin JUST turned on
				initDebounceTimer();
			}
		} else {// true if the button is not currently being pressed
			if(late_button_pin) {// true if button_pin JUST turned off
				initDebounceTimer();
			}
		}
	}
}

void vWaitIfSinglePressed(void *pvParameters) {
	
	vTaskDelay(DOUBLE_PRESS_WINDOW);

	if(!dpDisableSinglePress) {// check if a double press has not happened
		singlePressButtonEvent();
	}
	
	vTaskDelete(NULL);
}

void vWaitIfLongPressed(void *pvParameters) {	
	TickType_t origTicksLastUnpress = ticksLastUnpress;
	
	vTaskDelay(LONG_PRESS_WINDOW);
	
	// check if button was released during delay ^
	if(ticksLastUnpress < ticksLastPress && origTicksLastUnpress == ticksLastUnpress) {
		longPressButtonEvent();
		lpDisableSinglePress = 1;// true
	}
	
	vTaskDelete(NULL);
}

// double press disable single press
void vDPDisableSinglePress(void *pvParameters) {
	vTaskDelay(DOUBLE_PRESS_WINDOW);
	
	dpDisableSinglePress--;
	
	vTaskDelete(NULL);
}

void vButtonEventGenerator(void *pvParameters) {
	while(1) {
		if(pressButtonOccurred) {
			pressButtonOccurred = 0;// false
			pressButtonEvent();
			if(ticksLastPress != NULL && ticksLastPress > xTaskGetTickCount() - DOUBLE_PRESS_WINDOW) {
				doublePressButtonEvent();
				dpDisableSinglePress++;
				
				xTaskCreate( vDPDisableSinglePress, (const char*)"Double Press (Temporarily) Disables Single Press Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
			}
			ticksLastPress = xTaskGetTickCount();
			
			
			
			xTaskCreate( vWaitIfLongPressed, (const char*)"Wait If Long Pressed Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
		} else if(unpressButtonOccurred) {
			unpressButtonOccurred = 0;// false
			unpressButtonEvent();
			ticksLastUnpress = xTaskGetTickCount();
			
			if(!dpDisableSinglePress && !lpDisableSinglePress) {
				xTaskCreate( vWaitIfSinglePressed, (const char*)"Wait If Single Pressed Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
			}
			
			lpDisableSinglePress = 0;// false
		}
	}
}

void vShowCoffeeSelected(void *pvParameters) {
	while(1) {
		if(currState ==cyclingCoffeeTypes &&coffeeSelected == mochaCoffee) {
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOn(LED_ORANGE);// orange on
			STM_EVAL_LEDOff(LED_RED);
		} else if(currState == cyclingCoffeeTypes && coffeeSelected == espressoCoffee) {
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOff(LED_ORANGE);
			STM_EVAL_LEDOn(LED_RED); // red on
		} else if(currState == cyclingCoffeeTypes && coffeeSelected == latteCoffee) {
			STM_EVAL_LEDOn(LED_BLUE);// blue on
			STM_EVAL_LEDOff(LED_ORANGE);
			STM_EVAL_LEDOff(LED_RED);
		}
		else{
			continue;
		}
		
	}
}

void vIdle(void *pvParameters) {
	while(1) {
		vTaskDelay(500);
		STM_EVAL_LEDToggle(LED_GREEN);
	}
}

void vServeEspresso(void *pvParameters) {
	
	TIM4->CCR1 = SERVO_POSITION_NEUTRAL;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_ESPRESSO;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_NEUTRAL;
	vTaskDelay(1000);
	
	vTaskDelete(NULL);
}

void vServeLatte (void *pvParameters) {
	
	TIM4->CCR1 = SERVO_POSITION_NEUTRAL;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_ESPRESSO;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_MILK;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_NEUTRAL;
	vTaskDelay(1000);
	
	vTaskDelete(NULL);
}

void vServeMocha(void *pvParameters) {
	
	TIM4->CCR1 = SERVO_POSITION_NEUTRAL;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_ESPRESSO;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_CHOCOLATE_MILK;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_NEUTRAL;
	vTaskDelay(1000);
	
	vTaskDelete(NULL);
}

// ************************************** Button Events **************************************

// double press (not single press, not long press)
// only called once for each double press
void doublePressButtonEvent() {
	
	if(currState == cyclingCoffeeTypes) {
		currState = countDown;
		switch(coffeeSelected) {
			case espressoCoffee:
				xTaskCreate( vServeEspresso, (const char*)"Serve Espresso Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				xTaskCreate( vCountDownEspresso, (const char*)"CountDown Espresso Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				break;
			case latteCoffee:
				xTaskCreate( vServeLatte, (const char*)"Serve Latte Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				xTaskCreate( vCountDownLatte, (const char*)"CountDown Latte Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				break;
			case mochaCoffee:
				xTaskCreate( vServeMocha, (const char*)"Serve Mocha Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				xTaskCreate( vCountDownMocha, (const char*)"CountDown Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				break;
		}
	}
}

// long press (not single press, not double press)
// only called once for each long press
void longPressButtonEvent() {
	if(currState == programming){
		currState = cyclingCoffeeTypes;
	}
	else{
		currState = programming;
	}
}

// single press (not double press, not long press)
// only called once for each single press
void singlePressButtonEvent() {
	switch(currState) {
		case cyclingCoffeeTypes:
			nextCoffeeType();
			break;
		case programming:
			changeCoffeeTime();
			break;
		case countDown:
			currState = cyclingCoffeeTypes;
	}
}

// very recently, button was pressed down (accounts for debouncing)
// this event can be a part of a single, double, or long press
// only called once for each press
void pressButtonEvent() {

}

// very recently, finger taken off button (accounts for debouncing
// this event can be a part of a single, double, or long press
// only called once for each unpress
void unpressButtonEvent() {
	
}

// ************************************** Servos **************************************

void setSysTick(void){
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1){};
    }
}

void InitServos (void){ 
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    //Initialize PB6 (TIM4 Ch1) and PB7 (TIM4 Ch2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
}

void InitPWMTimer4(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //TIM4 Clock Enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  // Time Base Configuration for 50Hz
  TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = 84 -1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM4, ENABLE);
}

void SetupPWM(void) {
    
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //Set output capture as PWM mode
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Enable output compare
  TIM_OCInitStructure.TIM_Pulse = 0; // Initial duty cycle at 0%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // HIGH output compare active
  // Set the output capture channel 1 and 2 (upto 4)
  TIM_OC1Init(TIM4, &TIM_OCInitStructure); // Channel 1
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC2Init(TIM4, &TIM_OCInitStructure); // Channel 2
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
}

// ************************************** Timers **************************************

void initDebounceTimer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
      
  timer_InitStructure.TIM_Prescaler = 40000;
  timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timer_InitStructure.TIM_Period = DEBOUNCE_WINDOW;
  timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timer_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timer_InitStructure);
  TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	currDebounceState = startDebounce;
}

void enableDebounceInterrupt()
{
		NVIC_InitTypeDef nvicStructure;
		nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
		nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
		nvicStructure.NVIC_IRQChannelSubPriority = 1;
		nvicStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvicStructure);
}

// Debounce window is over, checks where button is, assumes that is valid
void TIM2_IRQHandler()
{
	uint8_t button_pin;
	//Checks whether the TIM2 interrupt has occurred or not
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		button_pin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		if(button_pin) {
			if(currDebounceState == activeDebounce) {
				if(currButtonState == buttonUp) {// true if finger VERY recently pushed button down
					pressButtonOccurred = 1;// true
				}
				currDebounceState = doneDebounce;// false
				currButtonState = buttonDown;
			} else if(currDebounceState == startDebounce) {
				currDebounceState = activeDebounce;
			}
		} else {
			if(currDebounceState == activeDebounce) {
				if(currButtonState == buttonDown) {// true if finger VERY recently taken off button
					unpressButtonOccurred = 1;// true
				}
				currDebounceState = doneDebounce;// false
				currButtonState = buttonUp;
			} else if(currDebounceState == startDebounce) {
				currDebounceState = activeDebounce;
			}
		}
		
	}
}

void changeCoffeeTime(){
	if(coffeeSelected == latteCoffee){
		milkTime ++;
	}
	else if(coffeeSelected == espressoCoffee){
		espressoTime ++;
	}
	else if(coffeeSelected == mochaCoffee){
		chocoMilkTime ++;
	}
}

void resetCoffeeTime(){
	if(coffeeSelected == latteCoffee){
		milkTime = 0;
	}
	else if(coffeeSelected == espressoCoffee){
		espressoTime = 0;
	}
	else if(coffeeSelected == mochaCoffee){
		chocoMilkTime = 0;
	}
}




//*************************CountDown Timer**************************************
int timeCounter = 0;
int i = 0;
void vCountDown(void *pvParameters)
{
	for(;;)
	{
		if(currState){
			int typeToMake = coffeeSelected;
			timeCounter = getCoffeeTime() *2;//get times to blinking
			for(i = 0; i < timeCounter; i++){
				if(typeToMake == latteCoffee){
					STM_EVAL_LEDToggle(LED_BLUE);
				}
				else if(typeToMake == mochaCoffee){
					STM_EVAL_LEDToggle(LED_ORANGE);
				}
				else if(typeToMake == espressoCoffee){
					STM_EVAL_LEDToggle(LED_RED);
				}
				vTaskDelay( 1000 / portTICK_RATE_MS );
			}
			playSound(coffeeSelected);
			currState = cyclingCoffeeTypes;
		}
	}
}


void vCountDownLatte(void* pvParameters){
	int i = 0; 
	int timeRequired = getCoffeeTime()*2;
	for(i = 0; i< timeRequired; i++){
		STM_EVAL_LEDToggle(LED_BLUE);
		vTaskDelay(1000/portTICK_RATE_MS);
	}
	playSound(coffeeSelected);
	currState = cyclingCoffeeTypes;
	vTaskDelete(NULL);
}

void vCountDownMocha(void* pvParameters){
	int i = 0; 
	int timeRequired = getCoffeeTime()*2;
	for(i = 0; i< timeRequired; i++){
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(1000/portTICK_RATE_MS);
	}
	playSound(coffeeSelected);
	currState =cyclingCoffeeTypes;
	vTaskDelete(NULL);
}

void vCountDownEspresso(void* pvParameters){
	int i = 0; 
	int timeRequired = getCoffeeTime()*2;
	for(i = 0; i< timeRequired; i++){
		STM_EVAL_LEDToggle(LED_RED);
		vTaskDelay(1000/portTICK_RATE_MS);
	}
	playSound(coffeeSelected);
	currState = cyclingCoffeeTypes;
	vTaskDelete(NULL);
}


int getCoffeeTime(void){
	int result = 0;
	if(coffeeSelected == mochaCoffee){
			result = espressoTime + chocoMilkTime;
	}
	else if(coffeeSelected == latteCoffee){
			result = espressoTime + milkTime;
	}
	else if(coffeeSelected == espressoCoffee){
			result = espressoTime;
	}
	else{
			result = -1;
	}
	return result;
}

//*************************Sound************************************************
//5 beeps for mocha
//7 beeps for espresso
//6 beeps for lattee
void playSound(int coffeeType){
		int beep = 0;
		int wait = 0;
		int beepTimes = 6;
		int frequence = 0;
		int interval = 2000000;
	
		if(coffeeType == mochaCoffee){
			NOTEFREQUENCY = 0.015*5;
			beepTimes = 5;
		}
		else if(coffeeType == espressoCoffee){
			NOTEFREQUENCY = 0.015 * 10;
			beepTimes = 7;
		}
		else if(coffeeType == latteCoffee){
			NOTEFREQUENCY = 0.015;
			beepTimes = 6;
		}
		while(beepTimes >0){
				SystemInit();
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
				beep = 0;
				codec_init();
				codec_ctrl_init();
				I2S_Cmd(CODEC_I2S, ENABLE);
				initFilter(&filt);
				while(beep != interval)
				{
						if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
						{
								SPI_I2S_SendData(CODEC_I2S, sample);
								if (sampleCounter & 0x00000001)
								{
										sawWave += NOTEFREQUENCY;
										if (sawWave > 1.0)
												sawWave -= 2.0;

										filteredSaw = updateFilter(&filt, sawWave);
										sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
								}
								sampleCounter++;
						}
						beep++;
				}
				stop();
				beepTimes --;
				wait = frequence * interval;
				while(wait > 0){
					wait --;
				}
		}
}

// a very crude FIR lowpass filter
float updateFilter(fir_8* filt, float val)
{
		uint16_t valIndex;
		uint16_t paramIndex;
		float outval = 0.0;

		valIndex = filt->currIndex;
		filt->tabs[valIndex] = val;

		for (paramIndex=0; paramIndex<8; paramIndex++)
		{
				outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
		}

		valIndex++;
		valIndex &= 0x07;
		filt->currIndex = valIndex;
		return outval;
}

void initFilter(fir_8* theFilter)
{
		uint8_t i;

		theFilter->currIndex = 0;

		for (i=0; i<8; i++)
				theFilter->tabs[i] = 0.0;

		theFilter->params[0] = 0.01;
		theFilter->params[1] = 0.05;
		theFilter->params[2] = 0.12;
		theFilter->params[3] = 0.32;
		theFilter->params[4] = 0.32;
		theFilter->params[5] = 0.12;
		theFilter->params[6] = 0.05;
		theFilter->params[7] = 0.01;
}



