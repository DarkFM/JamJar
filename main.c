
#include <stdint.h>     // allows us to use unint32_t that are in the macro headers
#include <tm4c123gh6pm.h>
#include <stdbool.h>
#include <stdio.h>
#include <arm_math.h>
#include <math.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

//void PortF_Interrupt_Init(void);
void GPIOPortF_Handler(void);
void SysTick_init(void);
//void SysTick_Handler(void);
void ADC0SS1_Handler(void);
void ADC1SS1_Handler(void);
void ADC0_SS1_Init(void);
void ADC1_SS1_Init(void);
void SSI_0_Init(void);
void delay(unsigned long time);
void PortF_init(void);
void PortE_Init(void);
void PortD_Init(void);
void PortB_Init(void);
void PortC_Init(void);
void SysTick_wait_time(unsigned long time);
void sample_rate(unsigned long rate);
void PLL_Init(void);
void PWM0_Init(void);
void PWM0_0_Handler(void);
void WTIMER0(void);
void WTIMER0_Period_Init(void);

unsigned long led;
unsigned int time_10ms = 160000;
volatile static unsigned long OCT_RANGE_SEL;
volatile static unsigned long OCT_SHIFT_SEL;
volatile static unsigned long SAMPLE_RATE;
volatile static unsigned long POT4;
volatile static uint16_t LFO_SAMPLE;
volatile static uint16_t POT5;
//volatile static unsigned long POT_SAMPLES;
//volatile static int int_count = 0;

//volatile unsigned long temp_rate;
float temp_rate;
volatile static unsigned long store;
unsigned long LOAD_VAL = 6500;        // MAX value is 2^16 = 65536
unsigned long LOAD_CMPA;
unsigned long LOAD_CMPB;
unsigned long old_sample = 0.0;
//float old_rate = 0.0;
bool stable_sample = false;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


int main()
{
 // NVIC_FPCC_R |= (1u << 31) | (1u << 30);
LOAD_CMPA = (LOAD_VAL/2);
LOAD_CMPB = (LOAD_VAL*75)/(100);
/**********************  INITIALIZERS  *********************/
  ROM_FPULazyStackingEnable();
  ROM_FPUEnable();
 // ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  PLL_Init();  
  PortB_Init(); 
  PortC_Init();
  PortD_Init();
  PortE_Init();
  PortF_init();
  SysTick_init(); 
  PWM0_Init();
  ADC0_SS1_Init();
  ADC1_SS1_Init();
  SSI_0_Init();
  //WTIMER0(); 
 // WTIMER0_Period_Init();
  
/**********************************************************/


  while(1){
  //sample_rate(SAMPLE_RATE);

    
  }
  
  
}


// used for ADC sampling ->  ADC0 = (PE2, PE1, PE0), ADC1 = (PE3)
void PortE_Init(void)
{
  SYSCTL_RCGCGPIO_R |= (1 << 4);        // enable clock port E
    delay(1000);
  GPIO_PORTE_DIR_R &=  ~((1 << 1) | (1 << 2) | (1 << 0)| (1 << 3));       // set pin 0,1,2 input
  GPIO_PORTE_AFSEL_R |= (1 << 1) | (1 << 2) | (1 << 0)| (1 << 3);      // making sure alternate funtionality is selected
  GPIO_PORTE_DEN_R &= ~((1 << 1) | (1 << 2) | (1 << 0)| (1 << 3));        // DISABLE digital functionality on pin, sets it up for analog mode
  GPIO_PORTE_AMSEL_R |= (1 << 1) | (1 << 2) | (1 << 0)| (1 << 3);        // ENABLE ANALOG MODE FOR PE0,1.2
}

// used to ADC sampling -> PD3, PD2
void PortD_Init(void)
{
  // enable the clock to approporiate GPIO ports
  SYSCTL_RCGCGPIO_R |= (1 << 3);       // enable clock for  D
    delay(1000);
  GPIO_PORTD_DIR_R &=  ~((1 << 3) | (1 << 2));       // set pin 3 input
  GPIO_PORTD_AFSEL_R |= (1 << 3) | (1 << 2);      // making sure alternate funtionality is selected
  GPIO_PORTD_DEN_R &= ~((1 << 3) | (1 << 2));        // DISABLE digital functionality on pin, sets it up for analog mode
  GPIO_PORTD_AMSEL_R |= (1 << 3)  | (1 << 2);        // ENABLE ANALOG MODE FOR PD3  
}

// USED FOR PWMO -> PB6
void PortB_Init(void)
{
  SYSCTL_RCGCGPIO_R |= (1u << 1);        // enable clock for port B pin6
    delay(1000);
  GPIO_PORTB_DIR_R |= (1 << 6) | (1 << 7);       // set pin 4 = Output
  GPIO_PORTB_AMSEL_R &= 0x00;   // disable analog function
  GPIO_PORTB_AFSEL_R |= (1 << 6)| (1 << 7);      // making sure alternate funtionality is selected
  GPIO_PORTB_DEN_R |= (1 << 6)| (1 << 7);      // allow digital functionality on pin, sets it up for analog mode
  GPIO_PORTB_PCTL_R |= (4 << 24)| (4 << 28);       // Select M0 PWM0 function
 
}


// USED FOR WTIMER0
void PortC_Init(void)
{
  SYSCTL_RCGCGPIO_R |= (1u << 2);        // enable clock for port F
  //SysTick_wait_time(10000);// allow for clock to stabilize
  delay(1000);
  GPIO_PORTC_DIR_R |= (1 << 4) | (1 << 5);        // set pin 4,5 to output
  GPIO_PORTC_AMSEL_R &= 0x00;   // disable analog function
  GPIO_PORTC_DEN_R |= (1 << 4) | (1 << 5)| (1 << 6);       // enable digital functionality on pin
  GPIO_PORTC_AFSEL_R |= (1 << 4) | (1 << 5);      // making sure alt funtionality is selected 
  GPIO_PORTC_PCTL_R |= (7 << 16) | (7 << 20);
  
  GPIO_PORTC_PDR_R |= (1 << 6);         // allows for positive logic on the pin
  GPIO_PORTC_ADCCTL_R |= (1 << 6);     // CONFIGURE PC6 TO TRIGGER ADC
  
//  To prevent false interrupts, the following steps should be taken when re-configuring GPIO
//edge and interrupt sense registers:
//1. Mask the corresponding port by clearing the IME field in the GPIOIM register.
  GPIO_PORTC_IM_R &= ~(1 << 6);
  
//2. Configure the IS field in the GPIOIS register and the IBE field in the GPIOIBE register.
  GPIO_PORTC_IS_R &= ~(1 << 6); // edge detect
  
//3. Clear the GPIORIS register.
  GPIO_PORTC_RIS_R |= (1 << 6);
  
//4. Unmask the port by setting the IME field in the GPIOIM register
    GPIO_PORTC_IM_R |= (1 << 6);

}


 // configuring ADC0, SS1, AIN1-4, -> PE2, PE1, PE0, PD3
// used by PWM
void ADC0_SS1_Init(void) {
  
  
  /***************************ADC INITIALIZATION******************************/
  // Enable the ADC clock using the RCGCADC register 
 
  SYSCTL_RCGCADC_R |= (1 << 0);        //SELECTS ADC0
  delay(1000);
  /**** configure the sample sequencer ***/

  // ensure the sample sequencer is diabled by clearing the corrsponding ASENn bit
  ADC0_ACTSS_R &= ~(1 << 1); // disable ss1 first. we are using SS1 cause we need 4 samples
  
  // configure trigger event for the sample sequencer in the ADCEMUX register
  ADC0_EMUX_R |= (0x6 << 4);//(0x6 << 4); //(0xF << 4);  // TRIGGER EVENT
 
  ADC0_TSSEL_R &= ~(0x3 << 4);  // select PWM0 and GEN0
  
  
  //For each sample in the sample sequence, configure the corresponding input source in the ADCSSMUXn register.
  ADC0_SSMUX1_R  |= (0x1 << 0) | (0x2 << 4) | (0x3 << 8) | (0x4 << 12); // MUX1(for sequancer 1) is given the value of 1, 2, 3, 4
                                        // which corresponds to (AIN1, 2, 3, 4)-> (PE2,1,0 and PD3)
  
  //For each sample in the sample sequence, configure the sample control bits 
  ADC0_SSCTL1_R |= (1 << 14) | (1 << 13);  //|  (1 << 10) | (1 << 6) | ( 1 << 2 ); // set end of sequence and enable interrupt so
                                       // to trigger at end of ADC conversion instead of by pooling
                                        // sets interrupt for each sample of the sequence
  
  //If interrupts are to be used, set the corresponding MASK bit in the ADCIM register.
  ADC0_IM_R |= (1 << 1);        // allow interrupts to be sent for sequencer 3 
 
  // set the sampling rate to 1ksps (1 kilo samples per sec)
  //ADC0_PC_R |= 0x1;
  
  //assign priority. IRQ = 15
  ADC0_SSPRI_R |= (2 << 4);    // ASSIGN priority of 2 to SS1
  NVIC_PRI3_R |= (0x5u << 29);    // Used a mask to set the priority to 5
  NVIC_EN0_R |= (1 << 15);      // Set bit 15 as the IRQ for ADC0SS1 interrupt is 15 in the vector table
   
    //Enable the sample sequencer logic by setting the corresponding ASENn bit
  //in the ADCACTSS register.
  ADC0_ACTSS_R |= (1 << 1); // enable ADC sequencer 1
}


// configuring ADC1 SS2, AIN0=(PE3) AND AIN5=(PD2)
// handles NE555 timer trigger
void ADC1_SS1_Init(void)
{
  /***************************ADC_1 INITIALIZATION******************************/
  // Enable the ADC clock using the RCGCADC register
  SYSCTL_RCGCADC_R |= (1 << 1);        //SELECTS ADC1
    delay(1000);
  
  /**** configure the sample sequencer ***/
  
  // ensure the sample sequencer is diabled by clearing the corrsponding ASENn bit
  ADC1_ACTSS_R &= ~(1 << 1); // disable ss1 first. 
  
  // configure trigger event for the sample sequencer in the ADCEMUX register
  ADC1_EMUX_R |= (0x4 << 4);    // use external trigger
                //(0x5 << 4);  // TRIGGER FROM PWM_GEN0
                                
  //For each sample in the sample sequence, configure the corresponding input source in the ADCSSMUXn register.
  ADC1_SSMUX1_R  |= (0x0 << 0) | (0x5 << 4); // select AIN0 & AIN5 as the sample input for this sequencer
  
  //For each sample in the sample sequence, configure the sample control bits 
  ADC1_SSCTL1_R |= (1 << 5) | (1 << 6); // set end of sequence and enable interrupt so
                                       // to trigger at end of ADC conversion instead of by pooling
  
  //If interrupts are to be used, set the corresponding MASK bit in the ADCIM register.
  ADC1_IM_R |= (1 << 1);        // allow interrupts to be sent for sequencer 2 
 
  //assign priority. IRQ = 49
  ADC1_SSPRI_R |= (1 << 4);    // ASSIGN priority of 1 to SS1
  NVIC_PRI12_R |= (NVIC_PRI12_INT49_M & 3u << 13);    // Used a mask to set the priority to 3
  NVIC_EN1_R |= (1 << 17);      // Set bit 17 as the IRQ 49
  
  //Enable the sample sequencer logic by setting the corresponding ASENn bit
  //in the ADCACTSS register.
  ADC1_ACTSS_R |= (1 << 1); // enable ADC sequencer 1
}


// handles the interrupt for the PWM ADC tigger (POT Samples)
void ADC0SS1_Handler(void){
  
  ADC0_ISC_R |= (1 << 1);       // Acknowledge interrupt by clearing interrupt

  OCT_RANGE_SEL = ADC0_SSFIFO1_R;// PE2 -> AIN1
  OCT_SHIFT_SEL = ADC0_SSFIFO1_R;// PE1 -> AIN2
  SAMPLE_RATE = ADC0_SSFIFO1_R;// PE0 (AKA -PE0) -> AIN3
  POT4 = ADC0_SSFIFO1_R; // PD3 -> AIN4
  
   
  GPIO_PORTF_DATA_R ^= (1 << 3); // green 
}


// handles LFO sampling from TIMER ****not anymore
// handles 555 timer trigger
void ADC1SS1_Handler(void)
{
  ADC1_ISC_R |= (1 << 2);       // Acknowledge interrupt by clearing interrupt
  
  LFO_SAMPLE = ADC1_SSFIFO1_R;  // AIN0 -> PE3
  POT5 = ADC1_SSFIFO1_R;
  //GPIO_PORTF_DATA_R = 0;
  GPIO_PORTF_DATA_R ^= (1 << 2);// led blue 
}





void SysTick_init(void) {
  NVIC_ST_CTRL_R &= (0u << 0);  // turn off enable bit before we use the systick
  NVIC_ST_CTRL_R |= (0x05);// turn on enable bit and also set to PIOSC //CLK_SRC to system clock 16Mhz
  
}

void SysTick_wait_time(unsigned long time) {    // 1 = 6.25E-8 secs
  
  // setting the reload register so counter can count down from this value
  NVIC_ST_RELOAD_R = time-1; // if time is 16 million. this means the counter will take 1 sec to count to zero if system clock is 16MHz
  NVIC_ST_CURRENT_R  = 0;       // This clears the current register
  while((NVIC_ST_CTRL_R & 0x00010000) == 0){   // checks if the counter is done counting.
  }  
}


void PortF_init(void) {
    
  SYSCTL_RCGCGPIO_R |= (1u << 5);        // enable clock for port F
  //SysTick_wait_time(10000);// allow for clock to stabilize
  delay(1000);

  GPIO_PORTF_DIR_R |= ((1 << 1) | (1 << 2) | (1 << 3));        // set pin 1,2,3 to output
  GPIO_PORTF_DEN_R |= ((1 << 1) | (1 << 2) | (1 << 3));        // enable digital functionality on pin
  GPIO_PORTF_AFSEL_R &= ((0 << 1) | (0 << 2) | (0 << 3));      // making sure GPIO funtionality is selected 
}


void PLL_Init(void)
{
  // NEED TO ACTIVATE PLL TO USE ADC. this sets it to 80MHz
  

  // 0) Use RCC2
  SYSCTL_RCC2_R |=  0x80000000;  // USERCC2
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |=  0x00000800;  // BYPASS2, PLL bypass
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)   // clear XTAL field, bits 10-6
                 + 0x00000540;   // 10101, configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~0x00000070;  // configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~0x00002000;
  // 4) set the desired system divider
  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000)  // clear system clock divider
                  + (4<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~0x00000800;
  
  
  
  
}

void sample_rate(unsigned long rate){        // takes the trimpot sampled value

  if(abs(old_sample-rate) > 10.0){
      old_sample = rate;
      stable_sample = true;
      if(old_sample != 0)
        temp_rate =  20000.0 * ( (float)old_sample / 4095.0);
      else
        temp_rate = 4.884;      // makes freq 1 HZ @old_sample = 1
  }
  
  //if (abs((int)old_rate - (int)temp_rate) > 17 ){
  if (stable_sample){  
      //old_rate = temp_rate;
    stable_sample = false;
    // PWM0_0_CTL_R &= ~(1 << 0);   // enables the PWM0 GEN block 0
    // PWM0_ENABLE_R &= ~((1 << 1)| (1 << 0));
     
    //old_rate = 40000.0/old_rate;
    temp_rate = 80000000.0/temp_rate;
    WTIMER0_TAILR_R = (int)temp_rate;
    
     
//        PWM0_0_LOAD_R = ((int)old_rate-1);
//      int compA =  ((int)old_rate * 75) /100;
//      int compB =  ((int)old_rate * 50 )/100;
//      PWM0_0_CMPA_R = compA;  // helps set the duty cycle of pwmA
//      PWM0_0_CMPB_R = compB;        // helps set the duty cycle for pwmB
//      
      //PWM0_0_CTL_R |= (1 << 0);   // enables the PWM0 GEN block 0
      //PWM0_ENABLE_R |= ((1 << 1)| (1 << 0));
  }
  
  
  
  
 
  
      GPIO_PORTF_DATA_R &= 0;
  if (OCT_RANGE_SEL >= 2000 ){//&& POT2 < 4 && POT3 < 4 && POT4 < 4){
  GPIO_PORTF_DATA_R &= 0;
  GPIO_PORTF_DATA_R |= (1 << 3); // green 
  //GPIO_PORTF_DATA_R = 0;  
  }
  if ( OCT_SHIFT_SEL >= 4000 ){//POT1 < 4 && POT2 >= 4 && POT3 < 4 && POT4 < 4){
     // GPIO_PORTF_DATA_R &= 0;  
     // GPIO_PORTF_DATA_R |= (1 << 2);// blue

  }
 if (SAMPLE_RATE >= 2000){
    GPIO_PORTF_DATA_R &= 0;
    GPIO_PORTF_DATA_R |= (1 << 1);      // red
  
  }
 if (POT4 >= 2000){
    //GPIO_PORTF_DATA_R &= 0;
   
   // GPIO_PORTF_DATA_R |= (1 << 3) | (1 << 2)  | (1 << 1); // white
 
  }
  
}



// Used to trigger ADC0 that samples the POTS @500HZ
void PWM0_Init(void){

    //5. Configure the Run-Mode Clock Configuration (RCC) register in the System Control module
  //to use the PWM divide (USEPWMDIV) and set the divider (PWMDIV) to divide by 2 (000).

    SYSCTL_RCC_R |= (1 << 20);  // enables the PWM clock divisor
    SYSCTL_RCC_R &= ~0x000E0000;      // specifies value to divide the clock by. here its 80/2 = 40MHz
    SYSCTL_RCC_R = (0x5 << 17); // actually divides by 64-> 80/64 = 1.25Mhz
    
  // 1. Enable the PWM clock by writing a value of
  //0x0010.0000 to the RCGC0 register in the System Control
  SYSCTL_RCGC0_R |= (0x1 << 20);
  delay(1000);           // delay 
  
 // 2. Enable the clock to the appropriate GPIO module via the RCGC2 register in the System Control
// module (see page 464).

  //3. In the GPIO module, enable the appropriate pins for their alternate function using the
//GPIOAFSEL register. To determine which GPIOs to configure, see Table 23-4 on page 1344.
  
  //4. Configure the PMCn fields in the GPIOPCTL register to assign the PWM signals to the appropriate
//pins (see page 688 and Table 23-5 on page 1351).

  //6. Configure the PWM generator for countdown mode with immediate updates to the parameters.
    // Write the PWM0CTL register with a value of 0x0000.0000.
    PWM0_CTL_R = 0x0;   // clear the register
    //PWM0_CTL_R |= (1 << 1); // set to count-up/down mode
    //¦ Write the PWM0GENA register. reg 44
    PWM0_0_GENA_R |= (3 << 4) | (2 << 6);//(3 << 2) | (2 << 6);   // controls generation of pwmA for gen module 0 of pwm module 0
                                            // high on load, low on compA-down
    //¦ Write the PWM0GENB register
    PWM0_0_GENB_R |= (3 << 8) | (2 << 10);//(3 << 10) | (2 << 2);      //high on compB-down, low on load

  //    7. Set the period. 
    PWM0_0_LOAD_R = LOAD_VAL-1; //40000-1;//639;        // desired-freq(sec) / pwm-clk-divider-on-sys-clk-period(sec)
 
  //8. Set the pulse width of the MnPWM0 pin for a 25% duty cycle.
   
    PWM0_0_CMPA_R = LOAD_CMPA;//200000;//320;  // helps set the duty cycle of pwmA

  //9. Set the pulse width of the MnPWM1 pin for a 75% duty cycle.
    //¦ Write the PWM0CMPB register with a value of 0x0000.0063.
    PWM0_0_CMPB_R = LOAD_CMPB;//30000;//479;        // helps set the duty cycle for pwmB
    
    // sets when to trigger interrupt and also can setup ADC trigger
    PWM0_0_INTEN_R |= (1 << 9); //compareA-down  //trigger interrupt to ADC
   //PWM0_INTEN_R |= (1 << 0);   // enable interrupt for gen block 0
    // PWOGEN0 = IRQ-> 26
    //NVIC_PRI6_R |= (0x2 << 21); // set priority to 2
   // NVIC_EN0_R |= (1 << 26);    // enable this interrupt
    
    
  //10. Start the timers in PWM generator 0.
//¦ Write the PWM0CTL register with a value of 0x0000.0001.
    PWM0_0_CTL_R |= (1 << 0) |(1<< 1);   // enables the PWM0 GEN block 0

  //11. Enable PWM outputs.
    //¦ Write the PWMENABLE register with a value of 0x0000.0003.
  //PWM0_ENUPD_R |= (3 << 8); //enable M0PWM4 corresponding with pin
  
  // use PWNENABLE to drive the PWM to output pins. 
  PWM0_ENABLE_R |= (1 << 1)| (1 << 0);
}


// this doesnt work
void PWM0_0_Handler(void)
{
  PWM0_ISC_R |= (1 << 0);       // ack interrupt by clearing it
  GPIO_PORTF_DATA_R |= (1 << 1);
  
}




void delay(unsigned long time){
  while(time--){}
   //return var;
}



// using PC4 -> MUX 7 in AFSEL -> 32/64 Wide Timer / PWM 0*****************THIS DOESNT WORK
void WTIMER0(void){
  
  SYSCTL_RCGCWTIMER_R |= (1 << 0);      //enable clock for WTIMER0
  
 // 1. Ensure the timer is disabled (the TnEN bit is cleared) before making any changes.
  WTIMER0_CTL_R &= ~0x1;
  
//2. Write the GPTM Configuration (GPTMCFG) register with a value of 0x0000.0004.
  WTIMER0_CFG_R = 0x4;  // this selects the 32-bit timer config
  
//3. In the GPTM Timer Mode (GPTMTnMR) register, set the TnAMS bit to 0x1, the TnCMR bit to
//0x0, and the TnMR field to 0x2.
  WTIMER0_TAMR_R |= (1<< 3) | (0x2 << 0);
  
//4. Configure the output state of the PWM signal (whether or not it is inverted) in the TnPWML field
//of the GPTM Control (GPTMCTL) register.
  WTIMER0_CTL_R &= ~((1 << 14) | (1 << 6)); // Make sure outputs A & B are NOT inverted
  WTIMER0_CTL_R |= (1 << 5);    // TIMERA triggers ADC sample
                            // the timer must be configured for one-shot or periodic time-out mode
                            // to produce an ADC trigger assertion
  
//5. If a prescaler is to be used, write the prescale value to the GPTM Timer n Prescale Register
//(GPTMTnPR).
     
  
//6. If PWM interrupts are used, configure the interrupt condition in the TnEVENT field in the
//GPTMCTL register and enable the interrupts by setting the TnPWMIE bit in the GPTMTnMR
//register. Note that edge detect interrupt behavior is reversed when the PWM output is inverted
//(see page 737).
  
  
//7. Load the timer start value into the GPTM Timer n Interval Load (GPTMTnILR) register.
   WTIMER0_TAILR_R = 800000;       // approx 1 sec
  
//8. Load the GPTM Timer n Match (GPTMTnMATCHR) register with the match value.
   WTIMER0_TAMATCHR_R = 4000000;        // this adjusts the duty cycle
  
//9. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable the timer and begin
//generation of the output PWM signal.
  WTIMER0_CTL_R |= (1 << 0);
  
}


// using PC4 -> MUX 7 in AFSEL -> 32/64 Wide Timer / PWM 0
// using periodic count down timer
// uses PLL so 1 = 1.25e-8 secs
void WTIMER0_Period_Init(void)
{
   SYSCTL_RCGCWTIMER_R |= (1 << 0);      //enable clock for WTIMER0
   delay(1000);
//  The GPTM is configured for One-Shot and Periodic modes by the following sequence:
//1. Ensure the timer is disabled (the TnEN bit in the GPTMCTL register is cleared) before making
//any changes.
    WTIMER0_CTL_R &= ~(0x1 | (1 << 8));
  
//2. Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0000.
    WTIMER0_CFG_R = 0x0; 
  
  //3. Configure the TnMR field in the GPTM Timer n Mode Register (GPTMTnMR):
//a. Write a value of 0x1 for One-Shot mode.
//b. Write a value of 0x2 for Periodic mode.
    WTIMER0_TAMR_R |= (1<< 3) | (0x2 << 0);
   // WTIMER0_TBMR_R |= (1<< 3) | (0x2 << 0);
  
//4. Optionally configure the TnSNAPS, TnWOT, TnMTE, and TnCDIR bits in the GPTMTnMR register
//to select whether to capture the value of the free-running timer at time-out, use an external
//trigger to start counting, configure an additional trigger or interrupt, and count up or down.

  
  //5. Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
    WTIMER0_TAILR_R = 8000000;
    //WTIMER0_TBILR_R = 80000000;
    WTIMER0_CTL_R &= ~((1 << 14) | (1 << 6)); // Make sure outputs A & B are NOT inverted
    WTIMER0_CTL_R |= (1 << 5); //| (1 << 13);    // WTIMER0_A triggers ADC sample
  
  //6. If interrupts are required, set the appropriate bits in the GPTM Interrupt Mask Register
//(GPTMIMR).

  
  //7. Set the TnEN bit in the GPTMCTL register to enable the timer and start counting.
    WTIMER0_CTL_R |= (1 <<0); //| (1 << 8);        // Enable WTIMERO_A&B
  
  //8. Poll the GPTMRIS register or wait for the interrupt to be generated (if enabled). In both cases,
//the status flags are cleared by writing a 1 to the appropriate bit of the GPTM Interrupt Clear
//Register (GPTMICR).
  
  
//If the TnMIE bit in the GPTMTnMR register is set, the RTCRIS bit in the GPTMRIS register is set,
//and the timer continues counting. In One-Shot mode, the timer stops counting after the time-out
//event. To re-enable the timer, repeat the sequence. A timer configured in Periodic mode reloads
//the timer and continues counting after the time-out event.

   
    
  
}

/*--------------------------------------------------------- SPI INITALIZATION -----------------------------------------------------------------------*/
void SSI_0_Init(void) {
 
  SYSCTL_RCGCSSI_R |= (0x1 << 0); // SSI Module 0 Run Mode Clock Gate Controll: ENABLED 
  delay(1000);
  SYSCTL_RCGCGPIO_R |= (0x1 << 0); // I/O Run Mode Clock Gate Controll: PORT A ENABLED
  delay(1000);
  GPIO_PORTA_AFSEL_R |= (0x1 << 1) | (0x1 << 2) | (0x1 << 3); // Set Pins PA 2,3,4 to SSI mode
  GPIO_PORTA_PCTL_R |= (0x2 << 4) | (0x2 << 8) | (0x2 << 12); // GPIO port controll 
  GPIO_PORTA_DEN_R |= (0x1 << 1) | (0x1 << 2) | (0x1 << 3);
/* *******In addition, the drive strength,
drain select and pull-up/pull-down functions must be configured. Refer to “General-Purpose
Input/Outputs (GPIOs)” on page 649 for more information.
  
  Note: Pull-ups can be used to avoid unnecessary toggles on the SSI pins, which can take the
slave to a wrong state. In addition, if the SSIClk signal is programmed to steady state
High through the SPO bit in the SSICR0 register, then software must also configure the
GPIO port pin corresponding to the SSInClk signal as a pull-up in the GPIO Pull-Up
Select (GPIOPUR) register. ****** */
  SSI0_CR1_R |= (0x1 << 1) | (0x0 << 2 ); // SSI OPERATIONS ENABLED, SET AS MASTER 
  SSI0_CC_R |= (0x5 << 0); //PIOSC CLOCK SELECTED 
  SSI0_CPSR_R |= (0x2 << 0 ); // SSI0_CLK = SysCLK / (CPDVSR *(1+SCR)) SPDVSR = 2-254
  SSI0_CR0_R |= (0x7 << 0)|(0x0 << 4)|(0x1 << 8); //SELECT SSI FRAM FORMAT | SSI CLOCK PHASE | SSI0_CLK = SysCLK / (CPDVSR *(1+SCR)); SCR = 0-255 (Bits 8-15)         
  /*¦ 
  Serial clock rate (SCR)
¦ Desired clock phase/polarity, if using Freescale SPI mode (SPH and SPO)
¦ The protocol mode: Freescale SPI, TI SSF, MICROWIRE (FRF)
¦ The data size (DSS)*/
  
}



