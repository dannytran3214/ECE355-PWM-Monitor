#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "assert.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx.h"
#include "math.h"
// ----------------------------------------------------------------------------
// ---------Defines------------------------------------------------------------
#define myTIM2_PRESCALER ((uint16_t)0x0000) // Clock prescaler for TIM2 timer: noprescaling
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)// Maximum setting for overflow
#define MAX_Resistance ((int)5000)
#define MIN_Resistance ((int) 1)
#define myTIM3_PRESCALER ((uint16_t) 0xBB80) // Clock prescaler for TIM3 timer: 48Mhz/48K
(0xBB80 = 48000) = 1Khz prescaling
#define myTIM3_PERIOD ((uint32_t) 0xFFFFFFFF) // Maximum possible setting for overflow
// ----------------------------------------------------------------------------
// ---------Global Variables---------------------------------------------------
volatile uint32_t Resistance = 0;
volatile uint32_t Frequency =0;
volatile uint32_t edge = 0;
// ----------------------------------------------------------------------------
// --------Declarations--------------------------------------------------------
void myADC_Init(void); // Initializes the ADC itself
void myGPIO_Init(void); //Initializes GPIO for ADC
uint32_t getADC(void);// Fetches the ADC value and converts it
void myTIM3_Init(void);
void wait(int);
int Res_Conversion (uint32_t Input);
void myDAC_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void lcd_Init(void);
void lcd_write(int, int); //for writing data on LCD
void lcd_cmd(uint16_t); //send command to LCD
void lcd_send(char);// send ASCII code to LCD
// ----------------------------------------------------------------------------
// ----- main() ---------------------------------------------------------------
// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
int main(int argc, char* argv[]){
uint32_t res = 0 ;
myGPIO_Init(); // Initializes all GPIO pins, PA0 1 4, PC 1 PB 4 5 6 7 8 9 10 11 12 13 14 15
myADC_Init(); // Initializes the ADC to measure the POT resistance
myTIM3_Init(); // Initialize timer TIM3
myDAC_Init(); // Initialize the DAC
myEXTI_Init(); /* Initialize EXTI to measured the frequency of the signal*/
myTIM2_Init(); /* Initialize timer TIM2 to time the period of the signal*/
lcd_Init();
while (1)
{ wait(1000);
res = getADC(); // Gets the value from the POT and returns a number from 0 - 4095
Resistance = Res_Conversion(res); // converts the (0-4095 value to the equivalent
resistance 0- 5000 ohms)
DAC->DHR12R1 = ADC1->DR; //DAC output toward 4N35 Octocoupler
wait(100);
lcd_write(Frequency,Resistance);
}
}
void myGPIO_Init(){
//PA1 -> digital -> used to generate interrupts to measure frequency.
//PA4 -> analog -> used for DAC to change the frequency of the oscillator
RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* Enables clock for GPIOA peripheral */
//PA1
GPIOA->MODER &= ~(GPIO_MODER_MODER1); //Timer Input
GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1); // No pull up/down
//PC1 -> analog -> Used to measure Pot's resistance
// Enable clock for GPIOC
RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
GPIOC->MODER &= ~(GPIO_MODER_MODER1); // Analog input
GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1); // No pull up/down
//PA4 DAC output
GPIOA->MODER &= ~(GPIO_MODER_MODER4); //Analog output
GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4); // No pull up/down
//Initializes PBs
RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // start GPIOB clock
GPIOB->MODER = (GPIOB->MODER & 0xFFFFFF00)| (0x55551500);
GPIOB->PUPDR = (GPIOB->PUPDR & 0xFFFFFF00)| (0X0<<8); /* Ensure no pull-up/pull-down for
PB */
}
uint32_t getADC(){
ADC1->CR |= ADC_CR_ADSTART//start ADC conversion
/*wait until the conversion has finished*/
while(!(ADC1->ISR & ADC_ISR_EOC)){};
ADC1->ISR &= ~(ADC_ISR_EOC); // resets conversion flag
return ((ADC1 ->DR) &ADC_DR_DATA); // Returns on ADC1's data masking the rest
}
void myADC_Init(){
RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable clock for ADC
/*Configuration of ADC for continuous conversion and overrun modes */
ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);
/*Sets ADC channel to be C1 */
ADC1->CHSELR = ADC_CHSELR_CHSEL11;
/*Enable ADC and wait for it to complete
ADC1->CR |= ADC_CR_ADEN;
while (!(ADC1->ISR & ADC_ISR_ADRDY)) {};
}
int Res_Conversion(uint32_t res){
float Percent = (float)(Input / 4095.0); // Converts the value read from a 0 - 4095 value to a
percent
int Res = (int)(Percent * MAX_Resistance); // Scales the percent by the maximum resistance
possible
return Res; // Returns the equivalent resistance for a given input
}
void myDAC_Init(){
RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); //Enable DAC clock
DAC->CR |= DAC_CR_EN1; //enable DAC channel 1 corresponding to PA4
}
void myTIM2_Init(){
/* Enable clock for TIM2 peripheral */
// Relevant register: RCC->APB1ENR
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /*TIM2 enable*/
/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
* enable update events, interrupt on overflow only */
// Relevant register: TIM2->CR1
TIM2->CR1 = ((uint16_t) 0x008C); /*buffer auto-reload, count up, stop on overflow,
enable update events, interrupt on overflow only*/
/* Set clock prescaler value */
TIM2->PSC = myTIM2_PRESCALER;
/* Set auto-reloaded delay */
TIM2->ARR = myTIM2_PERIOD;
/* Update timer registers */
// Relevant register: TIM2->EGR
TIM2->EGR = ((uint16_t) 0x0001);
/* Assign TIM2 interrupt priority = 0 in NVIC */
// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
NVIC_SetPriority(TIM2_IRQn, 0);
/* Enable TIM2 interrupts in NVIC */
// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
NVIC_EnableIRQ(TIM2_IRQn);
/* Enable update interrupt generation */
// Relevant register: TIM2->DIER
TIM2->DIER |= TIM_DIER_UIE;
}
void myEXTI_Init() {
/* Map EXTI1 line to PA1 */
// Relevant register: SYSCFG->EXTICR[0]
SYSCFG->EXTICR[0] = ((uint32_t) 0x00000080); /*SETS EXTI 1 to be bound to PA 1pin*/
/* EXTI1 line interrupts: set rising-edge trigger */
// Relevant register: EXTI->RTSR
EXTI->RTSR = ((uint32_t) 0x00000002); /*Sets rising edge trigger for TR1 and TR 2*/
/* Unmask interrupts from EXTI1 line */
// Relevant register: EXTI->IMR
EXTI->IMR = ((uint32_t) 0x00000002); /*Unmasks MR1 leaving the rest masked*/
/* Assign EXTI1 interrupt priority = 0 in NVIC */
// Relevant register: NVIC->IP[1], or use NVIC_SetPriorit
NVIC_SetPriority(EXTI0_1_IRQn, 0); /*EXTREAMLY UNSURE*/
/* Enable EXTI1 interrupts in NVIC */
// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
NVIC_EnableIRQ(EXTI0_1_IRQn); /*Enables interupts on External IRQ lines?*/
}
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler() {
/* Check if update interrupt flag is indeed set */
if ((TIM2->SR & TIM_SR_UIF) != 0) {
trace_printf("\n*** Timer Oveflow (555) ***\n");
/* Clear update interrupt flag */
// Relevant register: TIM2->SR
TIM2->SR &= ~(TIM_SR_UIF); /*Flips the first bit from 1 to 0*/
/* Restart stopped timer */
// Relevant register: TIM2->CR1
TIM2->CR1 |= TIM_CR1_CEN; /*Toggles the first bit (hopefully) */
}
}
void myTIM3_Init(){
 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable clock for TIM3 peripheral - Relevant register:
RCC->APB1ENR
 TIM3->CR1 = ((uint16_t) 0x006C); // Configure TIM3: buffer auto-reload, count up, stop on
overflow, enable update events, interrupt on overflow only
 TIM3->PSC = myTIM3_PRESCALER; // Set clock prescaler value
 TIM3->EGR |= ((uint16_t) 0x0001); // Update timer registers - Relevant register: TIM3->EGR
}
// Use tim3 to wait for a certain amount of clock cycles
void wait(int ms){
 TIM3->CNT = ((uint32_t) 0x00000000); // Clear the timer
 TIM3->ARR = ms; // Set the timeout value from parameter
 TIM3->EGR |= ((uint16_t) 0x0001); // Update registers
 TIM3->CR1 |= ((uint16_t) 0x0001); // Start the timer
 while((TIM3->SR & 0x0001) == 0); // Wait until timer hits the desired count
 TIM3->SR &= ~((uint16_t) 0x0001); // Clear update interrupt flag
 TIM3->CR1 &= ~0x0001; // Stop timer (TIM3->CR1)
}
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler(){
float frequency;
int count;
if ((EXTI->PR & EXTI_PR_PR1) != 0){
if (edge == 0) { //if first edge
TIM2 -> CNT = 0X00000000; // clear counter register
edge = 1;
TIM2 -> CR1 = TIM_CR1_CEN; // start timer
}
else {
TIM2 -> CR1 &= ~(TIM_CR1_CEN); // DISABLE COUNTER
edge = 0;
count = TIM2->CNT; //Put TIM2 Counter value into count
frequency = (float)SystemCoreClock / (float)count; // calculate frequency
Frequency = (int) frequency;
}
EXTI->PR |= EXTI_PR_PR1;}
}
void lcd_Init()
{ GPIOB->BRR = ((uint16_t) 0xFFF0);
lcd_cmd((uint16_t)0x38);//DL = 1, N = 1, F = 0 DDRAM using 8bits,2 lines of 8 characters
lcd_cmd((uint16_t)0xC);//(D = 1, C = 0, B = 0 display on, no cursor, no blinking
lcd_cmd((uint16_t)0x6);//I/D = 1, S = 0 DDRAM address is auto-incremented after each access
no shift
lcd_cmd((uint16_t)0x1);//clear display
}
void lcd_cmd(uint16_t cmd){
uint16_t temp_port;
GPIOB->BRR = GPIO_Pin_5;//Set RS to 0
GPIOB->BRR = GPIO_Pin_6;//Set RW to 0
temp_port = GPIOB->ODR &~((uint16_t) 0xFF00);//Assign port B to a temporary port, clear 15:8.
GPIOB->ODR = temp_port|((uint16_t)cmd<<8); //put data on output Port
GPIOB->BSRR = GPIO_Pin_4;//Set bit 4 to 1 ENB
wait(20);
while(((GPIOB->IDR)&GPIO_Pin_7) == 0){};//Wait for input from LCD Pin 7
GPIOB->BRR |= GPIO_Pin_4;//Set bit 4 to 0 ENB
while(((GPIOB->IDR)&GPIO_Pin_7) != 0){};//wait for input from LCD Pin 7
wait(50);
}
void lcd_send(char a){
uint16_t temp_port;
GPIOB->BSRR = GPIO_Pin_5;//Set RS to 1
GPIOB->BRR = GPIO_Pin_6;//Set RW to 0
temp_port = GPIOB->ODR & ~((uint16_t) 0xFF00);//Assign port B to a temporary port, clear
15:8.
GPIOB->ODR = temp_port|((uint16_t)a<<8); //put data on output Port
GPIOB->BSRR = GPIO_Pin_4;//Set bit 4 to 0 ENB
while(((GPIOB->IDR)& GPIO_Pin_7) == 0){};//Wait for input from LCD Pin 7
GPIOB->BRR = GPIO_Pin_4;//Set bit 4 to 1 ENB
while(((GPIOB->IDR)& GPIO_Pin_7) != 0){};//wait for input from LCD Pin 7
wait(50);
}
void lcd_write(int freq, int res){
uint8_t digit = 0;
int temp_freq = freq;
int i = 0;
int temp_res = res;
lcd_cmd(0x0080);//send cursor to fist position
lcd_send('F');//Print F
lcd_send(':');//Print :
for(i=3 ;i>=0 ;i--){ //print out the value for frequency on LCD
temp_freq = freq/pow(10,i);
digit = 48 + temp_freq%10;
lcd_send((uint8_t)digit);//print digit value
}
lcd_send('H');//Print H
lcd_send('z');//Print z
lcd_cmd(0x00C0);//Send cursor to second position
lcd_send('R'); //Print R
lcd_send(':');//Print :
for(i=3 ;i>=0 ;i--){// print the value of resistance on LCD
temp_res = res/pow(10,i);
digit = 48+temp_res%10;
lcd_send((uint8_t) digit);
}
lcd_send('O');//Print o
lcd_send('h');// Print h
lcd_send('m');// Print m
}
#pragma GCC diagnostic pop
// ----------------------------------------------------------------------------