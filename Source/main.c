#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#define DEBOUNCE_WINDOW 20

void vButton(void *pvParameters);
void vIdle(void *pvParameters);
void vShowCoffeeSelected(void *pvParameters);

void singlePressButtonOccurred(void);
void pressButtonOccurred(void);
void unpressButtonOccurred(void);

void nextCoffeeType(void);
void initDebounceTimer(void);
void enableDebounceInterrupt(void);

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
	
	xTaskCreate( vIdle, (const char*)"Idle Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	xTaskCreate( vButton, (const char*)"Button Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	xTaskCreate( vShowCoffeeSelected, (const char*)"LEDs Show Selected Coffee Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	initDebounceTimer();
	
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

// single press (not double press, not long press)
// only called once for each single press
void singlePressButtonOccurred() {
	switch(currState) {
		case cyclingCoffeeTypes:
			nextCoffeeType();
			break;
	}
}

// very recently, button pressed down (accounts for debouncing)
// only called once for each press
void pressButtonOccurred() {
	
}

// very recently, finger taken off button (accounts for debouncing)
// only called once for each unpress
void unpressButtonOccurred() {
	// TODO: should wait 500ms to make sure not a double press
	singlePressButtonOccurred();
}

void vButton(void *pvParameters) {
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
					pressButtonOccurred();
				}
				currDebounceState = doneDebounce;// false
				currButtonState = buttonDown;
			} else if(currDebounceState == startDebounce) {
				currDebounceState = activeDebounce;
			}
		} else {
			if(currDebounceState == activeDebounce) {
				if(currButtonState == buttonDown) {// true if finger VERY recently taken off button
					unpressButtonOccurred();
				}
				currDebounceState = doneDebounce;// false
				currButtonState = buttonUp;
			} else if(currDebounceState == startDebounce) {
				currDebounceState = activeDebounce;
			}
		}
		
	}
}








// TODO: use xTaskGetTickCount
// https://stackoverflow.com/questions/44696004/freertos-getting-the-current-time
// Consider using portTICK_RATE_MS
// 		Example has this code "vTaskDelay( 200 / portTICK_RATE_MS );"









