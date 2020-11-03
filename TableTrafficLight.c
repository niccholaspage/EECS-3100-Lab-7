/*******************************************************************
 * TableTrafficLight.c
 * Instructor: ***fill this in***
 * Runs on LM4F120/TM4C123
 * Index implementation of a Moore finite state machine to operate a traffic light.  
 * Authors: Daniel Valvano,
 *					Jonathan Valvano,
 * 					Thomas Royko
 * Student: ***fill this in***
 * Section: ***fill this in***
 * Date:    ***fill this in***
 *
 * east/west red light connected to PB5
 * east/west yellow light connected to PB4
 * east/west green light connected to PB3
 * north/south facing red light connected to PB2
 * north/south facing yellow light connected to PB1
 * north/south facing green light connected to PB0
 * pedestrian detector connected to PE2 (1=pedestrian present)
 * north/south car detector connected to PE1 (1=car present)
 * east/west car detector connected to PE0 (1=car present)
 * "walk" light connected to PF3 (built-in green LED)
 * "don't walk" light connected to PF1 (built-in red LED)
 *******************************************************************/

#include "TExaS.h"
#include "inc\tm4c123gh6pm.h"

#include "SysTick.h"

// Label input/output ports
#define LIGHT                   (*((volatile uint32_t *)0x400050FC))
#define SENSOR                  (*((volatile uint32_t *)0x4002401C))
#define GPIO_PORTF_OUT          (*((volatile uint32_t *)0x40025028))

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

struct State {
	uint32_t Out;				// 8-bit output
	uint32_t Time;			// 10 ms
	uint8_t Next[8];		// 3 inputs means 2^3 = 8
};

typedef const struct State STyp;

#define goS	0
#define goW	1
#define walkStopSW 2

// Port I/O
// PE2-0 = inputs (south sensor, west sensor, and walk button)
// PB5-0 = traffic light outputs
// PF3 & PF1 = walking LEDs

//State machine output:
// Bit 7 - PF3 - green LED, walk light
// Bit 6 - PF1 - red LED, don't walk light
// Bit 5 - PB5 - South Red
// Bit 4 - PB4 - South Yellow
// Bit 3 - PB3 - South Green
// Bit 2 - PB2 - West Red
// Bit 1 - PB1 - West Yellow
// Bit 0 - PB0 - West Green

STyp FSM[3] = {
	{0x61, 5, {goS, goS, goS, goS, goS, goS, goS, goS}}, // goS
	{0x4C, 5, {goS, goS, goS, goS, goS, goS, goS, goS}}, // goW
	{0xA4, 5, {walkStopSW, walkStopSW, walkStopSW, walkStopSW, walkStopSW, walkStopSW, walkStopSW, walkStopSW}}, // walkStopSW
};

volatile unsigned long delay;

void Init_GPIO_PortsEBF(void) {
	SYSCTL_RCGC2_R |= 0x32;        // activate clock for Port E, Port F, and Port B
	delay = SYSCTL_RCGC2_R; // allow time for clock to stabilize
  GPIO_PORTB_DIR_R |= 0x3F;         // make PB5-0 out
  GPIO_PORTB_AFSEL_R &= ~0x3F;      // disable alt funct on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;         // enable digital I/O on PB5-0
                                    // configure PB5-0 as GPIO
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFF000000) + 0x00000000;
  GPIO_PORTB_AMSEL_R &= ~0x3F;      // disable analog functionality on PB5-0

  GPIO_PORTE_DIR_R &= ~0x07;        // make PE2-0 in
  GPIO_PORTE_AFSEL_R &= ~0x07;      // disable alt funct on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;         // enable digital I/O on PE2-0
                                    // configure PE2-0 as GPIO
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFFFFF000) + 0x00000000;
  GPIO_PORTE_AMSEL_R &= ~0x07;      // disable analog functionality on PE1-0
	
	// Port F initialization
	GPIO_PORTF_DIR_R |= 0x0A;					// make PF3 and PF1 outputs
	GPIO_PORTF_AFSEL_R &= 0x0A;				// disable ALT functions on PF5-0
	GPIO_PORTF_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF0F0F) + 0x00000000;
	GPIO_PORTF_AMSEL_R &= ~0x0A;			// disable analog functionality on PF3 and PF1
}

int main(void){
  uint8_t n; // state number
  uint32_t Input;
	// activate grader and set system clock to 80 MHz
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210, ScopeOff);
	SysTick_Init();
	Init_GPIO_PortsEBF();
	
	n = walkStopSW; // Initial state: green south, west red
  
  EnableInterrupts();

  while(1){
    LIGHT = FSM[n].Out;             // set traffic lights to current state's Out value
		// set the walk & don't walk lights to current state's Out value, modified
		// we only care about the left two bits, so we shift to the right 6.
		// This results in the ending 2 bits being the values for PF3 and PF1,
		// so we have to shift left one bit, putting PF1 in its correct place.
		// Finally, we take the original output, shift it to the right 7 bits,
		// then back to the left 3 bits to finalize PF3.
		GPIO_PORTF_OUT = (FSM[n].Out > 6 < 1) | (FSM[n].Out > 7 < 3);
    SysTick_Wait10ms(FSM[n].Time);  // wait 10 ms * current state's Time value
    Input = SENSOR;                 // get new input from car detectors
    n = FSM[n].Next[Input];         // transition to next state
  }
}

