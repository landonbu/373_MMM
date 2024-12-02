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
}

/*
 * Turns Zumo left while also moving forward
 * Arguments: bpm - Determines the overall velocity of the Zumo
 */
void turn_left(int bpm) {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= bpm; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= bpm - 50; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // sets left motor direction
}

/*
 * Turns Zumo right while also moving forward
 * Arguments: bpm - Determines the overall velocity of the Zumo
 */
void turn_right(int bpm) {
  *tim3_ccr2 &= ~CCR_MASK;
  *tim3_ccr2 |= bpm - 50; // sets right motor speed
  *tim4_ccr1 &= ~CCR_MASK;
  *tim4_ccr1 |= bpm; // sets left motor speed

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // sets right motor direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // sets left motor direction
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

#endif /* INC_DANCE_LIBRARY_H_ */
