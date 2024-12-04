/*
 * dance_library.h
 *
 *  Created on: Nov 18, 2024
 *      Author: zender
 */

#include <stdio.h>
#include <string.h>

#ifndef INC_DANCE_LIBRARY_H_
#define INC_DANCE_LIBRARY_H_

#define TIM3_ADDR 0x40000400 //timer 3 base register
#define TIM4_ADDR 0x40000800 //timer 4 base register
#define TIM_CCR1_OFFSET 0x34 //capture/compare register 1
#define TIM_CCR2_OFFSET 0x38 //capture/compare register 2
#define CCR_MASK 0xFFFF

uint32_t * tim3_ccr2 = (uint32_t *)(TIM3_ADDR + TIM_CCR2_OFFSET); // pointer to right motor CCR
uint32_t * tim4_ccr1 = (uint32_t *)(TIM4_ADDR + TIM_CCR1_OFFSET); // pointer to left motor CCR

int bool_stop = 0;

struct table_entry {
	char genre[50];
	void (*fun_ptr)(int);
};

/*
 * Stops Zumo by setting motor speeds to zero
 */
void stop(){
	  *tim3_ccr2 &= ~CCR_MASK;
	  *tim3_ccr2 |= 0; // sets right motor speed
	  *tim4_ccr1 &= ~CCR_MASK;
	  *tim4_ccr1 |= 0; // sets left motor speed

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // sets right motor direction
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // sets left motor direction
}

/*
 * Moves Zumo forward in a straight line
 * Arguments: bpm - Determines the linear velocity of the Zumo
 */
void straight_forward(int bpm) {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= bpm; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= bpm; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // sets left motor direction

  for(int i = 0; i < 5000000; i++);
  stop();
}

/*
 * Moves Zumo backward in a straight line
 * Arguments: bpm - Determines the linear velocity of the Zumo
 */
void straight_backward(int bpm) {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= bpm; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= bpm; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1); // sets left motor direction

  HAL_Delay(2000);
  stop();
}

/*
 * Spins Zumo in place clockwise
 * Arguments: bpm - Determines the angular velocity of the Zumo
 */
void spin_cw(int bpm) {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= bpm; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= bpm; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // sets left motor direction

  HAL_Delay(2000);
  stop();
}

/*
 * Spins Zumo in place counter-clockwise
 * Arguments: bpm - Determines the angular velocity of the Zumo
 */
void spin_ccw(int bpm) {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= bpm; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= bpm; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1); // sets left motor direction

  HAL_Delay(2000);
  stop();
}

/*
 * Turns Zumo left while also moving forward
 * Arguments: bpm - Determines the overall velocity of the Zumo
 */
void turn_left() {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= 199; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= 49; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // sets left motor direction

  HAL_Delay(2000);
  stop();
}

/*
 * Turns Zumo right while also moving forward
 * Arguments: bpm - Determines the overall velocity of the Zumo
 */
void turn_right() {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= 49; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= 199; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // sets left motor direction

  HAL_Delay(2000);
  stop();
}

/*
 * Discretely spins Zumo in place clockwise for a given number of seconds
 * Arguments:
 * 		bpm - Determines the angular velocity of the Zumo while spinning
 * 		seconds - How long to spin for
 */
void discrete_spin_cw(int bpm, int seconds) {
	for (int i = 0; i < 2 * seconds; i++) {
		spin_cw(bpm);
		HAL_Delay(400); // delay for 0.4 seconds
		stop();
		HAL_Delay(100); // delay for 0.1 seconds
	}
}

/*
 * Discretely spins Zumo in place counter clockwise for a given number of seconds
 * Arguments:
 * 		bpm - Determines the angular velocity of the Zumo while spinning
 * 		seconds - How long to spin for
 */
void discrete_spin_ccw(int bpm, int seconds) {
	for (int i = 0; i < 2 * seconds; i++) {
		spin_ccw(bpm);
		HAL_Delay(400); // delay for 0.4 seconds
		stop();
		HAL_Delay(100); // delay for 0.1 seconds
	}
}

void groovy_dance() {
	while (bool_stop == 0) {
		discrete_spin_cw(100, 10);
		discrete_spin_ccw(100, 10);
		turn_right();
		HAL_Delay(10000);
		turn_left();
		HAL_Delay(10000);
	}
	bool_stop = 1;
}

#endif /* INC_DANCE_LIBRARY_H_ */
