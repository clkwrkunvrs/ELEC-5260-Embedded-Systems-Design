/*====================================================*/
/* Victor P. Nelson */
/* ELEC 3040/3050 - Lab 1, Program 1 */
/* Toggle LED1 while button pressed, with short delay inserted */
/*====================================================*/
#include "STM32L1xx.h" /* Microcontroller information */
/* Define global variables */
static uint8_t count1 = 0,count2 = 0; /* determines where in the count the program is */
/*---------------------------------------------------*/
/* Initialize GPIO pins used in the program */
/* PA0 = push button */
/* PC8 = blue LED, PC9 = green LED */
/*---------------------------------------------------*/
void PinSetup () {
 /* Configure PA0 as input pin to read push button */
 RCC->AHBENR |= 0x01; /* Enable GPIOA clock (bit 0) */
 GPIOA->MODER &= ~(0x000006); /*Clear PA[2:1] Mode bits (extraneous in this case) */
 GPIOA->MODER &= ~(0x00000006); /* General purpose input mode for sw1 & sw2 (PA1 AND PA2) */
 RCC->AHBENR |= 0x04; /* Enable GPIOC clock (bit 2) */
 GPIOC->MODER &= ~(0x00FFFF); /*Clear PC[7:0] Mode bits*/
 GPIOC->MODER |= (0x00005555); /* General purpose output mode*/
}

/* Function to count up and down from 0 to 9 and repeat */
void counting(char direction) {
	//COUNT1
	if((direction == 0x04) & (count1 == 0x0)) count1 = 9;
	else if((direction == 0x0) && (count1 == 9)) count1 = 0;
	else direction ? count1 -- : count1++;
	//COUNT2
	if((direction == 0x04) & (count2 == 0x0)) count2 = 9;
	else if((direction == 0x0) && (count2 == 9)) count2 = 0;
	else direction ? count2 ++ : count2--;
		}
/*----------------------------------------------------------*/
/* Delay function - do nothing for about 1/2 second */
/*----------------------------------------------------------*/
void delay () {
 int i,j,n;
 for (i=0; i<20; i++) { //outer loop
 for (j=0; j<9500; j++) { //inner loop
 n = j; //dummy operation for single-step test
	} //do nothing
 }
}
/*------------------------------------------------*/
/* Main program */
/*------------------------------------------------*/
int main(void) {
 unsigned char sw1,sw2; //state of SW1 and SW2
 PinSetup(); //Configure GPIO pins

 /* Endless loop */
 while (1) { //Can also use: for(;;) {
	 sw1 = GPIOA->IDR & 0x02; //Read GPIOA and mask all but bit 1 (PA1)
	  /*when sw1 = 1, call the counting function, then output the binary value
	 of the count to the LEDs at PC[3:0] */
	 while(sw1) {
		  sw1 = GPIOA->IDR & 0x02; //Read GPIOA and mask all but bit 1 (PA1)
		  sw2 = GPIOA->IDR & 0x04; //Read GPIOA and mask all but bit 2 (PA2)
			counting(sw2);
			//this might blow up because count is only 8 bits and ODR is 16; hence casting it as a short
			GPIOC->ODR = ((((uint16_t)count2)<<4) |(uint16_t)count1) & 0x00FF; //AND the count bits with 0x00FF to get binary value of count1 and count2
			//alternative could be a loop with (count >> k) & 1. Also, don't care about upper ODR bits for this project
		  delay(); //1/2 second delay to count to next number
	 }
 } /* repeat forever */
}
