
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 4 Pushbuttons and 4 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "tm4c123gh6pm.h"


//a2 right led
//a3 left led
//a4 middle green
//a5 magnet

#define RightRED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define LeftRED_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define MiddleGREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define MagnetGREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))

// REQUIRED: correct these bitbanding references for green and yellow LEDs (temporary to guarantee compilation)
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4))) //pd0
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4))) //pd1
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))//pd2
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) //pd3

//#define PUSH_BUTTON_1  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) //pc4
//#define PUSH_BUTTON_2  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) //pc5
#define PUSH_BUTTON_3  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) //pc6
#define PUSH_BUTTON_4  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) //pc7

#define onboardGREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_QUEUE_SIZE 10
struct semaphore
{ int windex;
  int rindex;
  unsigned int count;
  unsigned int queueSize;
  int processQueue[MAX_QUEUE_SIZE]; // store task index here
} *s, keyPressed, keyReleased, flashReq;

// task
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  char name[16];                 // name of task used in ps command
  void* semaphore;               // pointer to the semaphore that is blocking the thread
  uint8_t skipcount;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];
void magnet();
void bluetooth();
void magnetControl();
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//magnet variables
uint16_t V1[4];
bool MagValid;                                     // check for valid data
float Md;                                       // Updated value from median
uint32_t MagCount,Npcount;                                 // No Magnet detected Count

////////////////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t max=80;
char str[max+1]={NULL};
uint8_t position[10]={NULL};
uint8_t parsecount;
uint8_t ascstr[max+1];
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Btupdate =false;
char Btdir;

int getSP()
{
__asm(" MOV R0,R13 ");
return;
}


int putSP(int g)
{
__asm(" MOV R13,R0 ");
__asm(" SUB R13,#0x8 ");
return;
}

isCommand(char fieldName[10],int minArgs)
{
	char *str1 = &str[position[0]];
	char *str2=fieldName;
	if (!strcmp(str1,str2))
	{
		if((parsecount-1)== minArgs)
			return true;
		else
			return false;
	}
	else
		return false;
}

isAlpha(int fieldNo)
{
	 uint8_t i;
	 uint8_t j=0;
	 char *argType = &str[position[fieldNo]];
	 for(i=0; i< strlen(argType); i++)
	 	{
	 		if ((ascstr[position[fieldNo]+i])  >= 65 && (ascstr[position[fieldNo]+ i]) <= 90)
	 		j++;
	 	}
	 	if (j == strlen(argType))
	 		 return true;
	  	 else
	  		return false;
}


isNumber(int fieldNo)
 {
	 uint8_t i;
	 uint8_t j=0;
	 char *argType;
	 argType=&str[position[fieldNo]];
	for(i=0; i< strlen(argType); i++)
	{
		if ((ascstr[position[fieldNo]+i])  >= 48 && (ascstr[position[fieldNo]+ i]) <= 57)
		j++;
	}
	if (j == strlen(argType))
		 return true;
	 else
		return false;
 }

uint8_t getNumber(int fieldNo)
{
	char *arg = &str[position[fieldNo]];
	uint8_t i;
	i=atoi(arg);
	return i;
}

char* getAlpha(int fieldNo)
{
	 char* arg;
	 arg= &str[position[fieldNo]];
	 return arg;
}
//-----------------------------------------------------------------------------
// RTOS Kernel
//-----------------------------------------------------------------------------

void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
  // REQUIRED: systick for 1ms system timer
  NVIC_ST_RELOAD_R = 39999;
  NVIC_ST_CURRENT_R = 0X1;
  NVIC_ST_CTRL_R |= 0x7; //0111
}

int rtosScheduler()
{
  // REQUIRED: Implement prioritization to 16 levels
  bool ok;
  bool stateready;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
  stateready = (tcb[task].state == STATE_READY);
  if(stateready)
  	{
  	  if(tcb[task].skipcount == 0)
  	  {
  		  tcb[task].skipcount = tcb[task].priority;
  		ok = true;
  	  }
  	  else tcb[task].skipcount--;
    }
  }

  return task;
}


bool createThread(_fn fn, char name[], int priority)
{
  bool ok = false;
  uint8_t i = 0;
  bool found = false;
  // REQUIRED: store the thread name
  // REQUIRED: take steps to ensure a task switch cannot occur
  // save starting address if room in task list
  if (taskCount < MAX_TASKS)
  {
    // make sure fn not already in list (prevent reentrancy)
    while (!found && (i < MAX_TASKS))
    {
      found = (tcb[i++].pid ==  fn);
    }
    if (!found)
    {
      // find first available tcb record
      i = 0;
      while (tcb[i].state != STATE_INVALID) {i++;}
      tcb[i].state = STATE_READY;
      tcb[i].pid = fn;
      strcpy(tcb[i].name, name);

      // REQUIRED: preload stack to look like the task had run before
//      stack[i][255]= 0; //faker3
//      stack[i][254]= 0; //fakelr
      stack[i][255]= fn; //r14
      stack[i][254]= 11; //r11
      stack[i][253]= 10; //r10
      stack[i][252]= 9; //r9
      stack[i][251]= 8; //r8
      stack[i][250]= 7; //r7
      stack[i][249]= 6; //r6
      stack[i][248]= 5; //r5
      stack[i][247]= 4; //r4
      tcb[i].sp = &stack[i][247]; // REQUIRED: + offset as needed for the pre-loaded stack
      tcb[i].priority = priority;
      tcb[i].skipcount = priority;
      tcb[i].currentPriority = priority;
      // increment task count
      taskCount++;
      ok = true;
    }
  }
  // REQUIRED: allow tasks switches again
  return ok;
}

// REQUIRED: modify this function to destroy a process
void destroyProcess(_fn fn)
{
	 uint8_t task = 0xFF;
	  bool ok = false;
	  while (!ok)
	  {
	    task++;
	    if (task >= MAX_TASKS)
	      task = 0;
	    ok = (tcb[task].pid == fn);
	  }
//	  int ik= strcmp(tcb[task].name, "Idle");
	  if(tcb[task].state == STATE_BLOCKED)
	  {
		  uint8_t i=0;
	       s =tcb[task].semaphore;
		  while(s->processQueue[i] != task)
		  {
		  i++;
		  }
		  s->processQueue[i]= -1;
		  tcb[task].state = STATE_INVALID;
		  tcb[task].pid = 0;
		  taskCount--;
	  }

		  else if(((tcb[task].state == 1) || (tcb[task].state == 3) ) && strcmp(tcb[task].name, "Idle"))
		  {
		  tcb[task].state = STATE_INVALID;
	      tcb[task].pid = 0;
	      taskCount--;
		  }
	      else
	    	  putsUart0("\n\r Can not delete");
}

void rtosStart()
{
  // REQUIRED: add code to call the first task to be run, restoring the preloaded context
  _fn fn;
  taskCurrent = rtosScheduler();
  // Add code to initialize the SP with tcb[task_current].sp;
  // Restore the stack to run the first process
  	putSP(tcb[taskCurrent].sp); //change SP to taskCurrent's sp
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");   //LR
	__asm(" BX LR ");
}

void init(void* p, int count)
{
  s = p;
  s->count = count;
  s->queueSize = 0;
  s->rindex = 0;
  s->windex = 0;
}

// REQUIRED: modify this function to yield execution back to scheduler
void yield()
{
	// push registers, call scheduler, pop registers, return to new function
	__asm(" POP {R3} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP(); //R13

	taskCurrent = rtosScheduler();

	putSP(tcb[taskCurrent].sp); //change SP to next task's sp
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");   //LR
	__asm(" BX LR ");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses
void sleep(uint32_t tick)
{
	tcb[taskCurrent].ticks= tick;
	tcb[taskCurrent].state= STATE_DELAYED;
	__asm(" POP {R3} ");
	// push registers, set state to delayed, store timeout, call scheduler, pop registers,
	// return to new function (separate unrun or ready processing)
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP(); //R13

	taskCurrent = rtosScheduler();

	putSP(tcb[taskCurrent].sp); //change SP to next task's sp
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");   //LR
	__asm(" BX LR ");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler
void wait(void* pSemaphore)
{
	s = pSemaphore;
	if((s->count) == 0)
	tcb[taskCurrent].semaphore = pSemaphore;
	__asm(" POP {R3} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP();
	
	if((s->count) > 0)
	(s->count)--;
	else
	{
	tcb[taskCurrent].state = STATE_BLOCKED;
//	tcb[taskCurrent].semaphore = pSemaphore;
	s->processQueue[s->windex] = taskCurrent;
	s->windex = (s->windex +1) % MAX_QUEUE_SIZE;
	s->queueSize = (s->queueSize +1) % MAX_QUEUE_SIZE;
	taskCurrent = rtosScheduler();
	}

	putSP(tcb[taskCurrent].sp);
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");
	__asm(" BX LR ");

}

// REQUIRED: modify this function to signal a semaphore is available
void post(void* pSemaphore)
{

	s = pSemaphore;
	__asm(" POP {R3} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp= getSP();

	s->count++;

	if( (s->count > 0) && (s->queueSize > 0) )
		{
			if(s->processQueue[s->rindex] == -1)
			s->rindex = (s->rindex +1) % MAX_QUEUE_SIZE;
			if(tcb[s->processQueue[s->rindex]].state == STATE_BLOCKED)
			{
			tcb[s->processQueue[s->rindex]].state = STATE_READY;
			s->rindex = (s->rindex +1) % MAX_QUEUE_SIZE;
			s->queueSize = (s->queueSize -1);
			s->count--;
		    }
		}

	putSP(tcb[taskCurrent].sp);
	__asm(" POP {R4-R11} ");
	__asm(" POP {R14} ");
	__asm(" BX LR ");

}

// REQUIRED: modify this function to add support for the system timer
void systickIsr()
{  uint8_t i;
	for(i=0; i < MAX_TASKS; i++)
	{
		if( tcb[i].state == STATE_DELAYED)
		{
		if((tcb[i].ticks)!= 0)
		(tcb[i].ticks)--;
		else
		tcb[i].state = STATE_READY;
		}
	}
}


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for orange, red, green, and yellow LEDs
    //           4 pushbuttons, and uart
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    //b4,5,e4,5 PWM      //c4,5 Uart1 e0,1,2,3 adc

    // Configure ONBOARD LED
       GPIO_PORTF_DIR_R |= 0x8;
       GPIO_PORTF_DR2R_R |= 0x8;
       GPIO_PORTF_DEN_R |= 0x8;


    // Configure Taillight LED
    GPIO_PORTA_DIR_R |= 0x9C;
    GPIO_PORTA_DR2R_R |= 0x9C;
    GPIO_PORTA_DEN_R |= 0x9C;

    // Configure LED and pushbutton pins
    GPIO_PORTC_DIR_R &= ~0xC0;  // bits 4,5,6,7 are inputs, other pins unchanged //Now only BIT 6,7 IP so F0--->C0
    GPIO_PORTD_DIR_R |= 0x0F;   // bits 1,2,3,4 are outputs, other pins unchanged
    GPIO_PORTC_DR2R_R = 0xC0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DR2R_R = 0x0F;
    GPIO_PORTC_DEN_R = 0xC0;  // enable LEDs and pushbuttons
    GPIO_PORTD_DEN_R = 0x0F;
    GPIO_PORTC_PUR_R = 0xC0;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0 | SYSCTL_RCGCUART_R1;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

        // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

      // Configure UART1 pins
    GPIO_PORTC_AFSEL_R |= 0x30;
    GPIO_PORTC_DEN_R |= 0X30;
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xFF00FFFF)+0x00220000;
    GPIO_PORTC_AMSEL_R &= ~0x30;
     // Configure UART1 to 9600 baud, 8N1 format (must be 3 clocks from clock enable and config writes)     UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 260;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 26;                               // round(fract(r)*64)=45
    UART1_LCRH_R |= UART_LCRH_WLEN_8  | UART_LCRH_FEN  ; // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R |= UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;// | UART_CTL_LBE ; // enable TX, RX, and module

     //    UART1_IM_R |= UART_IM_RXIM;                       // turn-on RX interrupt
     //    NVIC_EN0_R |= 1 << (INT_UART1 - 16);               // turn-on interrupt 22 (UART1)



    // Enabling Port E as input
       GPIO_PORTE_DIR_R &= 0xF0; //&= 0XF0;

  // Configure three backlight LEDs
     GPIO_PORTB_DIR_R |= 0x30;   // make bit5 an output
     GPIO_PORTB_DR2R_R |= 0x30;  // set drive strength to 2mA
     GPIO_PORTB_DEN_R |= 0x30;   // enable bit5 for digital
     GPIO_PORTB_AFSEL_R |= 0x30; // select auxilary function for bit 5
     GPIO_PORTB_PCTL_R = GPIO_PCTL_PB4_M0PWM2 | GPIO_PCTL_PB5_M0PWM3; // enable PWM on bit 5
     GPIO_PORTE_DIR_R |= 0x30;   // make bits 4 and 5 outputs
     GPIO_PORTE_DR2R_R |= 0x30;  // set drive strength to 2mA
     GPIO_PORTE_DEN_R |= 0x30;   // enable bits 4 and 5 for digital
     GPIO_PORTE_AFSEL_R |= 0x30; // select auxilary function for bits 4 and 5
     GPIO_PORTE_PCTL_R = GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5; // enable PWM on bits 4 and 5


     SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
     __asm(" NOP");                                   // wait 3 clocks
     __asm(" NOP");
     __asm(" NOP");
     __asm(" NOP");
     SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
     SYSCTL_SRPWM_R = 0;                              // leave reset state
     PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
     PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2


     PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                      // output 3 on PWM0, gen 1b, cmpb
     PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 2 on PWM0, gen 1a, cmpa
     PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                      // output 4 on PWM0, gen 2a, cmpa
     PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                      // output 5 on PWM0, gen 2b, cmpb
     PWM0_1_LOAD_R = 2048;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
     PWM0_2_LOAD_R = 2048;
     PWM0_INVERT_R = PWM_INVERT_PWM2INV | PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV;
                                                      // invert outputs for duty cycle increases with increasing compare values

     PWM0_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
     PWM0_1_CMPA_R = 0;
     PWM0_2_CMPB_R = 0;                               // green off
     PWM0_2_CMPA_R = 0;  // blue off


     PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
     PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2
     PWM0_ENABLE_R = PWM_ENABLE_PWM3EN |PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN;

     // Configure AN0 as an analog input
            SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking

            GPIO_PORTE_AFSEL_R |= 0x0F;                      // select alternative functions for AIN0, AIN1, AIN2, AIN3 (PE3, PE2, PE1, PE0)
            GPIO_PORTE_DEN_R &= ~0x0F;                       // turn off digital operation on pin PE3, PE2, PE1, PE0
            GPIO_PORTE_AMSEL_R |= 0x0F;                      // turn on analog operation on pin PE3, PE2, PE1, PE0

            ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
            ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
            ADC0_EMUX_R = ADC_EMUX_EM1_PROCESSOR;            // select SS1 bit in ADCPSSI as trigger
            ADC0_SSMUX1_R = 0x3210;                          // set first sample to AIN0, AIN1, AIN2, AIN3
            ADC0_SSCTL1_R = ADC_SSCTL1_END3;                 // mark first sample as the end
            ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char tempc)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = tempc;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* tempstr)
{
	uint8_t l;
    for (l = 0; l < strlen(tempstr); l++)
	  putcUart0(tempstr[l]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while(UART0_FR_R & UART_FR_RXFE)
	{yield();}
	return UART0_DR_R & 0xFF;
}
char getcUart1()
{
    while(UART1_FR_R & UART_FR_RXFE)
    {yield();}
    return UART1_DR_R & 0xFF;
}

void readAdc0Ss1()
{
    int i;
    ADC0_PSSI_R |= ADC_PSSI_SS1;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS1 is not busy
    for(i=0;i<4;++i)
        V1[i] = ADC0_SSFIFO1_R;                         // get single result from the FIFO
}



// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}


void shell()
{
  while (true)
  {
      // REQUIRED: add processing for the shell commands here

      uint8_t i;
      uint8_t j;
      uint8_t k;
      parsecount=0;
      uint8_t posDelim[80];
      uint8_t delimCount=0;
      char type[10];

      memset(&str[0], 0, sizeof(str));
      uint8_t count=0;
//    putsUart0(" \n \r Enter your Command:");

        while(count < max)
        {
        uint8_t c = getcUart1();
        putcUart0(c);
        if (c == 8)
        {
            if (count>0)
                count--;
        }
        else if (c == 13 || c == 46) //46-->Dot-->EOS
        {
            str[count]= NULL;
            break;
        }
        else if (c >= 32 && c != 46)
        {
            str[count++]=c;
            if (count == max)
            {
                str[count]=NULL;
                break;
            }
        }
    }

    memset(&position[0], 0, sizeof(position));
        for(j=0;j<=strlen(str); j++)
        {
//              str[j]=toupper(str[j]);
            ascstr[j]=str[j];
        }
        if ((ascstr[0] >= 65 && ascstr[0]<= 90)  || (ascstr[0] == 95/*_*/) || (ascstr[0] >= 97 && ascstr[0]<= 122) || (ascstr[0] == 38 /*&*/) || (ascstr[0] == 43/*+*/))
        {
                type[parsecount] = 'a';
                position[parsecount] = 0;
                parsecount++;
        }
            else if (ascstr[0] >= 48 && ascstr[0]<= 57 )
            {
                type[parsecount] = 'n';
                position[parsecount] = 0;
                parsecount++;
            }
        for (i=0; i< strlen(str); i++)
        {
            if(( ascstr[i] >= 32 && ascstr[i]<= 37 )|| ( ascstr[i] >= 39 && ascstr[i]<= 42 /*43 is + exception*/) || (ascstr[i] >= 44 && ascstr[i]<= 47) || ( ascstr[i] >= 58 && ascstr[i]<= 64 )|| ( ascstr[i] >= 91 && ascstr[i]<= 94) || (ascstr[i] == 96) || ( ascstr[i] >= 123 && ascstr[i]<= 255 ))
            {
                    posDelim[delimCount]=i;
                    delimCount++;
                    if((ascstr[i+1] >= 65 && ascstr[i+1]<= 90 )|| (ascstr[i+1] >=97 && ascstr[i+1] <= 122) || (ascstr[i+1] == 43/*+*/) )
                    {
                        type[parsecount] = 'a';
                        position[parsecount] = i+1;
                        parsecount++;
                    }
                    else if( ascstr[i+1] >= 48 && ascstr[i+1]<= 57)
                    {
                        type[parsecount] = 'n';
                        position[parsecount] = i+1;
                        parsecount++;
                    }
            }
        }

        for(k=0; k<delimCount; k++)
        {
            str[posDelim[k]]=NULL;
        }


             char displayRxd[80]={NULL};
             uint8_t validCommand= false; //bool
             int temi;
             char *istr1 = &str[position[0]];
             uint8_t teni;
             uint8_t si;

             if(isCommand("AT+W",0))
             {
                 Btupdate = true;
                 Btdir = 's';
                 validCommand= true;
             }

             else if(isCommand("AT+X",0))
              {
                      Btupdate = true;
                      Btdir = 'x';
                      validCommand= true;
              }

             else if(isCommand("AT+A",0))
              {
                     Btupdate = true;
                     Btdir = 'l';
                     validCommand= true;
              }

             else if(isCommand("AT+D",0))
              {
                     Btupdate = true;
                     Btdir = 'r';
                     validCommand= true;
              }
             else if(isCommand("AT+S",0))
                {
                       Btupdate = true;
                       Btdir = 'b';
                       validCommand= true;
                }

           if(!validCommand)
           putsUart0("\n \r Syntax Error \n");


  }
}



void magnet()
{
 // REQUIRED: add code to read the Hall effect sensors that maintain lane centering and report the lane position relative to center
    uint64_t sum[4];
        float  V[4],D[4],x;
        int i,j,k,b=2,temp[4];
    //  char str[100];
        while(1)
            {
            // *** resetting the variables to initial values ***
                    for (i=0;i<4;++i)
                        sum[i]=0;

                    MagValid = false;

                    // Taking 16 samples for each Analog pin
                    for (i=0;i<16;++i)
                    {
                        readAdc0Ss1();

                        for (j=0;j<4;++j)
                            sum[j]+=V1[j];
                    }

                    // Averaging
                    for (i=0;i<4;++i)
                        sum[i]=sum[i]/16;

                    // converting the discrete voltage value to Analog voltage & Distance
                    for(i=0;i<4;++i)
                    {
                    x=100;
                    temp[i]=i;

                    // *** Calculating Analog voltage ***
                    if(sum[i]>1252 && sum[i]<1290)
                        V[i]=1.03;
                    else
                        V[i] = (3.30*sum[i])/4095.0;

                    // *** Calculating Distance from Analog voltage using Distance equaations from excel sheet ***
                    if(V[i]<=0.051)
                    {
                        D[i] = -22727*pow(V[i],2) + 2379.1*V[i] - 61.618;
                    }
                    else if(V[i]>0.051 && V[i]<=0.775)
                    {
                        D[i] = 0.5505*V[i] + 0.5796;
                    }
                    else if(V[i]>0.775 && V[i]<=1.02)
                    {
                        D[i] = 383.45*pow(V[i],3) - 982.14*pow(V[i],2) + 839.46*V[i] - 238.17;
                    }
                    else if(V[i]>1.02 && V[i]<=1.04)
                        D[i] = 10;
                    else if(V[i]>1.04 && V[i]<=1.2)
                        D[i] = -1543.3*pow(V[i],3) + 5316.6*pow(V[i],2) - 6105.9*V[i] + 2339.1;
                    else if(V[i]>1.2 && V[i]<=2.004)
                        D[i] = -10.357*pow(V[i],4) + 64.107*pow(V[i],3) - 146.97*pow(V[i],2) + 146.9*V[i]  - 52.939;
                    else if(V[i]<0.01 && V[i]>2.004)
                        D[i] = 0;

                    // calibrating the first sensor depending on experiments. Other sensors doesn't require any calibration.
                    if(i==0)
                        D[i] += 0.2;
                    }

                    // *** Sorting ***
                    for(i=0;i<3;++i)
                        for(j=0;j<(3-i);++j)
                            if(V[temp[j]]<V[temp[j+1]])
                            {
                            k=temp[j+1];
                            temp[j+1]=temp[j];
                            temp[j]=k;
                            }

            //  sprintf(str,"\n\n\r\t%2.4f\t\t%2.4f\t\t%2.4f\t\t%2.4f",V[0],V[1],V[2],V[3]);
            //      putsUart0(str);
            //
            //  sprintf(str,"\t\t\t\t%2.4f\t\t%2.4f\t\t%2.4f\t\t%2.4f",D[0],D[1],D[2],D[3]);
            //          putsUart0(str);

                    // ******** Calculating Distance from the Median ***********
                        x = (V[temp[0]] - 1.03);
                    if(x>=0.005)
                    {
                        x =D[temp[0]]-1.25;

                        if(temp[0]==0)
                                {
                                    if(temp[1]==1 || (sum[temp[1]]-sum[1]<=10))
                                        x = -((3*b)/2-x/2);

                                    else
                                        x = -((3*b)/2+x-0.3);
                                }

                        else if(temp[0]==1)
                                {
                                    if(temp[1]==0)
                                        x = -(b/2+x-0.3);

                                    else if(temp[1]==2)
                                        x = -b/2+x;

                                    else
                                        x = 100;
                                }

                        else if(temp[0]==2)
                                {
                                    if(temp[1]==1)
                                        x = b/2-x/4;

                                    else if(temp[1]==3)
                                        x = b/2+x;
                                    else
                                        x= b/2;
                                }

                        else
                                {
                                    if(temp[1]==2)
                                        x=(3*b)/2-x;
                                    else
                                        x=(3*b)/2+x;
                                }

                if(x>=-5 && x<=5)
                {
                    MagValid = true;
                    Md = x;
                    MagCount = 0;
                    for (i=0;i<4;++i)
                    {
                        if(V[i]<1.02)
                        {
                            ++Npcount;
                            break;
                        }
                    }
                }

                }

                else
                {
                    MagValid = false;
                    ++MagCount;
                }

//                sprintf(str,"\n\n\n\r%2.2f\n",x);
//                putsUart0(str);
//                waitMicrosecond(0X001E8480);

          yield();
            }
       }
void magnetControl()
{
    bool firstUpdate = true;
//    float iirMd;
//    float alpha = 0.99;
    float prevMd=0;
    while(true)
    {
    if(MagValid)
   {
        onboardGREEN_LED ^= 1;
      if (firstUpdate == true)
         {
          prevMd = Md;
         MagCount = 0;
         firstUpdate = false;
         }
       else
          { //after atleast one magnet was detected
            if ( abs(prevMd - Md) <= 2 ) //very large deviation in successive values
                {
                    MagValid = true;
                    MagCount = 0;
                    prevMd = Md;
                }
            else
                MagValid = false;
          }
    }
     if(MagValid)
     {
         onboardGREEN_LED = 1;
//give appropriate pwm values here
         if( Md >= 3.4 && Md <= 4.5)
          {
             //*****************RIGHT********************
                             PWM0_2_CMPA_R = 0;
//                             PWM0_2_CMPB_R =1650;
                             PWM0_2_CMPB_R =(int)( 2047 - (Md*94.5));
             //******************LEFT********************
                             PWM0_1_CMPA_R = 2047 ; //pb4 PWM2
                             PWM0_1_CMPB_R = 0; //pb5 PWM3
//
            //very acute right
                             sleep(50);
          }
          if( Md >= 2 && Md <= 3.4)
          {
              //*****************RIGHT********************
                              PWM0_2_CMPA_R = 0;
//                              PWM0_2_CMPB_R =1750;
                              PWM0_2_CMPB_R = (int)(2047 - (Md*110));
              //******************LEFT********************
                              PWM0_1_CMPA_R = 2047 ; //pb4 PWM2
                              PWM0_1_CMPB_R = 0; //pb5 PWM3
                              sleep(50);
       //more right
          }
          if( Md >= 1 && Md <= 2)
          {
              //*****************RIGHT********************
                              PWM0_2_CMPA_R = 0;
                              PWM0_2_CMPB_R =(int)(2047 - (Md*121.33));

              //******************LEFT********************
                              PWM0_1_CMPA_R = 2047 ; //pb4 PWM2
                              PWM0_1_CMPB_R = 0; //pb5 PWM3
                              sleep(50);
       //more right
          }
         if( Md >= -1 && Md <= 1)
         {
             //straight
             //*****************RIGHT********************
                               PWM0_2_CMPA_R = 0;
                               PWM0_2_CMPB_R =1640;

             //******************LEFT***************
                               PWM0_1_CMPA_R = 1730 ;
                               PWM0_1_CMPB_R = 0;
                               sleep(50);
         }
         if( Md >= -2 && Md <= -1)
         {
             //*****************RIGHT********************
                           PWM0_2_CMPA_R = 0;
                           PWM0_2_CMPB_R =2047;

             //******************LEFT********************
                           PWM0_1_CMPA_R = (int)(2047 + (Md*94.52));
                           PWM0_1_CMPB_R = 0;
                           sleep(50);

    //left
         }
         if( Md >= -3.8 && Md <= -2)
         {
             //*****************RIGHT********************
                                     PWM0_2_CMPA_R = 0;
                                     PWM0_2_CMPB_R =1850;

                       //******************LEFT********************
                                     PWM0_1_CMPA_R = (int)(2010 + (Md*102.4));
                                     PWM0_1_CMPB_R = 0;
                                     sleep(50);

              //left
                            //more left
         }
         if( Md >= -4.5 && Md <= -3.8)
         {
             //*****************RIGHT********************
                                     PWM0_2_CMPA_R = 0;
                                     PWM0_2_CMPB_R =1850;

                       //******************LEFT********************
                                     PWM0_1_CMPA_R = (int)(2010 + (Md*122.44));
                                     PWM0_1_CMPB_R = 0;
                                     sleep(50);

              //left                    //very acute left
         }



 }
     else if(MagCount > 3000 && firstUpdate != true)
        {
            Btupdate = true; //stop
            Btdir = 'x';
            MagCount = 0;
        }
    if(Npcount >= 2)
                 {
                     Btupdate = true; //stop
                     Btdir = 'x';
                     Npcount = 0;
                     sleep(500);  //use binomial backoff to start or BT Shell command
                     MagValid = true;
                     Btdir = 's';
                 }
yield();
    }
}


void bluetooth()
{  bool rev = false;
    bool Btfirst = true;

  while(true)
  {
      if(Btupdate)
      {
          if(Btfirst == true )
          {
              if(Btdir == 's')
              {
          //*****************RIGHT********************
                            PWM0_2_CMPA_R = 0;
                            PWM0_2_CMPB_R =1915;

          //******************LEFT***************
                            PWM0_1_CMPA_R = 2047 ; //val here for forward
                            PWM0_1_CMPB_R = 0;
                            Btfirst =false;
                            sleep(5);
              }
              else if(Btdir == 'b')
              {
                        //*****************RIGHT********************
                                          PWM0_2_CMPA_R = 1750; //1950;
                                          PWM0_2_CMPB_R =0;

                        //******************LEFT***************
                                          PWM0_1_CMPA_R = 0 ;
                                          PWM0_1_CMPB_R = 1850; //2047;
                                          Btfirst =false;
                                          sleep(50);
               }
          }


          if(Btdir == 's')  //straight
          {       MiddleGREEN_LED=0;
                  LeftRED_LED = 0;
                  RightRED_LED =0;

//*****************RIGHT********************
                  PWM0_2_CMPA_R = 0;
                  PWM0_2_CMPB_R =1850;

//******************LEFT***************
                  PWM0_1_CMPA_R = 2010 ;
                  PWM0_1_CMPB_R = 0;
                  Btupdate = false;
                  Btdir = NULL;
          }

          else if(Btdir == 'b')  //reverse
          {        rev = true;
                   MiddleGREEN_LED = 1;
//*****************RIGHT********************
                  PWM0_2_CMPA_R = 2010;
                  PWM0_2_CMPB_R =0;

//******************LEFT***************
                  PWM0_1_CMPA_R = 0 ;
                  PWM0_1_CMPB_R = 1850;
                  Btupdate = false;
                  Btdir = NULL;
                  Btfirst = true;
          }

          else if(Btdir == 'x') //stop
            { MiddleGREEN_LED = 0;
              LeftRED_LED = 1;
              RightRED_LED = 1;
              sleep(250);
              if(rev != true)
              {
              int i=0;
              while(i < 100)
              {
//*****************RIGHT********************
                PWM0_2_CMPA_R = 0;
                PWM0_2_CMPB_R = 1850 - i;

//******************LEFT***************
                PWM0_1_CMPA_R = 2010 - i ;
                PWM0_1_CMPB_R = 0;
                i= i+10;
              }
              i=0;

//*****************RIGHT********************
              PWM0_2_CMPA_R = 0;
              PWM0_2_CMPB_R = 0;

//******************LEFT***************
              PWM0_1_CMPA_R = 0;
              PWM0_1_CMPB_R = 0;

              Btfirst = true;
              Btupdate = false;
              Btdir = NULL;
              LeftRED_LED = 0;
              RightRED_LED = 0;
              }
          else
          {
          int i=0;
          while(i < 80)
          {
//*****************RIGHT********************
            PWM0_2_CMPA_R = 1640 - i;
            PWM0_2_CMPB_R = 0;

//******************LEFT***************
            PWM0_1_CMPA_R = 0;
            PWM0_1_CMPB_R = 1750 - i;
            i= i+10;
          }
          i=0;

//*****************RIGHT********************
          PWM0_2_CMPA_R = 0;
          PWM0_2_CMPB_R = 0;

//******************LEFT***************
          PWM0_1_CMPA_R = 0;
          PWM0_1_CMPB_R = 0;

          Btfirst = true;
          Btupdate = false;
          Btdir = NULL;
          LeftRED_LED = 0;
          RightRED_LED = 0;
          rev = false;
          }
            }

          else if(Btdir == 'l') //left
          {
              LeftRED_LED = 1;

//*****************RIGHT********************
              PWM0_2_CMPA_R = 0;
              PWM0_2_CMPB_R =2020;

//******************LEFT********************
              PWM0_1_CMPA_R = 1770;
              PWM0_1_CMPB_R = 0;

      sleep(3000);
//*****************RIGHT********************
              PWM0_2_CMPA_R = 0;
              PWM0_2_CMPB_R =1850;

//******************LEFT********************
              PWM0_1_CMPA_R = 2010 ;
              PWM0_1_CMPB_R = 0;
              Btupdate = false;
              Btdir = NULL;

              LeftRED_LED = 0;
          }

          else if(Btdir == 'r') //right
                {
              RightRED_LED = 1;
//*****************RIGHT********************
                PWM0_2_CMPA_R = 0;
                PWM0_2_CMPB_R =1770;

//******************LEFT********************
                PWM0_1_CMPA_R = 2020 ; //pb4 PWM2
                PWM0_1_CMPB_R = 0; //pb5 PWM3

                sleep(3000);

//*****************RIGHT********************
                PWM0_2_CMPA_R = 0;
                PWM0_2_CMPB_R =1850;

//******************LEFT********************
                PWM0_1_CMPA_R = 2010 ;
                PWM0_1_CMPB_R = 0;

                Btupdate = false;
                Btdir = NULL;
                RightRED_LED = 0;
                }
          }
    yield();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

	// Initialize hardware
	initHw();
	rtosInit();


	// Add required idle process
//	ok =  createThread(idle, "Idle", 15);
//	 Add other processes
	ok = createThread(shell, "Shell", 12);
    ok &= createThread(bluetooth, "Bluetooth", 15);
   ok &= createThread(magnet, "Magnet", 8);
   ok &= createThread(magnetControl, "MagnetControl", 15);


	// Start up RTOS
	if (ok)
	  rtosStart(); // never returns
	else
	  RED_LED = 1;

    return 0;
    // don't delete this unreachable code
    // if a function is only called once in your code, it will be
    // accessed with two goto instructions instead of call-return,
    // so any stack-based code will not function correctly
    yield(); sleep(0); wait(0); post(0);
}


