#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#define DEBOUNCE_WINDOW 20
#define DOUBLE_PRESS_WINDOW 500
#define LONG_PRESS_WINDOW 3000
#define SERVO_POSITION_NEUTRAL 600
#define SERVO_POSITION_ESPRESSO 1100
#define SERVO_POSITION_MILK 1800
#define SERVO_POSITION_CHOCOLATE_MILK 2200

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

enum states {
	cyclingCoffeeTypes
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
		if(coffeeSelected == mochaCoffee) {
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOn(LED_ORANGE);// orange on
			STM_EVAL_LEDOff(LED_RED);
		} else if(coffeeSelected == espressoCoffee) {
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOff(LED_ORANGE);
			STM_EVAL_LEDOn(LED_RED); // red on
		} else if(coffeeSelected == latteCoffee) {
			STM_EVAL_LEDOn(LED_BLUE);// blue on
			STM_EVAL_LEDOff(LED_ORANGE);
			STM_EVAL_LEDOff(LED_RED);
		}
	}
}

void vIdle(void *pvParameters) {
	while(1);
}

void vServeEspresso(void *pvParameters) {
	
	TIM4->CCR1 = SERVO_POSITION_NEUTRAL;
	vTaskDelay(1000);
	
	TIM4->CCR1 = SERVO_POSITION_ESPRESSO;// espresso ingredient
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
		
		switch(coffeeSelected) {
			case espressoCoffee:
				xTaskCreate( vServeEspresso, (const char*)"Serve Espresso Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				break;
			case latteCoffee:
				xTaskCreate( vServeLatte, (const char*)"Serve Latte Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				break;
			case mochaCoffee:
				xTaskCreate( vServeMocha, (const char*)"Serve Mocha Task",
					STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
				break;
		}
	}
}

// long press (not single press, not double press)
// only called once for each long press
void longPressButtonEvent() {
	
}

// single press (not double press, not long press)
// only called once for each single press
void singlePressButtonEvent() {
	switch(currState) {
		case cyclingCoffeeTypes:
			nextCoffeeType();
			break;
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
