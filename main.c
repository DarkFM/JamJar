//#include <lm4f120h5qr.h>
#include <stdint.h>     // allows us to use unint32_t that are in the macro headers
#include <tm4c123gh6pm.h>
#include <stdio.h>

//void PortF_Interrupt_Init(void);
void GPIOPortF_Handler(void);
void SysTick_init(void);
//void SysTick_Handler(void);
void ADC0SS1_Handler(void);
void ADC0_SS1_Init(void);
void delay(unsigned long time);
void PortF_init(void);
void SysTick_wait_time(unsigned long time);
void UART_Init(void);
void Send_Char(unsigned char c);
char Receive_Char(void);
void print_string(unsigned char *c);

unsigned long led;
unsigned int time_10ms = 160000;
volatile static unsigned long POT1;
volatile static unsigned long POT2;
volatile static unsigned long POT3;
volatile static unsigned long POT4;
volatile static unsigned long POT_SAMPLES;




int main()
{

  ADC0_SS1_Init();

    UART_Init();
    PortF_init();
    SysTick_init();


  
  while(1){
  /*  
    
    
    
    
  //print_string("AIN1 \t");
  Send_Char(POT1);
  print_string("\r");
  
  //print_string("AIN2 \t");
  Send_Char(POT2);
  print_string("\r");
  
  //print_string("AIN3 \t");
  Send_Char(POT3);
  print_string("\r");
  
  //print_string("AIN4 \t");
  Send_Char(POT4);
  print_string("\r");
  
  SysTick_wait_time(16000000);
  ADC0_PSSI_R |= (1<<1); // ACTIVATE SS1
 
  ADC0_ACTSS_R ^= (1 << 1);
*/
   
  }
  
  
}
/*
void PortF_Interrupt_Init(void){
  //(a) The clock for the port must be enabled. 
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
  
  //(c) The appropriate pins must be enabled as inputs. 
  GPIO_PORTF_DIR_R = (0u << 4); // makes pin 4 an input pin
  
  GPIO_PORTF_DIR_R =  (1u << 1) | (1u << 2) | (1u << 3);// MAKE PF1,2,3 OUTPUT
  GPIO_PORTF_DEN_R =  (1u << 1) | (1u << 2) | (1u << 3);// MAKE PF1,2,3 DIGITAL FUNCTION
  
  GPIO_PORTF_DEN_R |= (1u << 4); // enables digital function for pin 4
  GPIO_PORTF_PUR_R |= (1u << 4); // enable pull up resistor for negative logic, and cause PF4 requires it
  GPIO_PORTF_AFSEL_R &= ~(1 << 4);      // disable alternate function on pin 4
  GPIO_PORTF_PCTL_R &= ~(0xF << 16);    // select GPIO as alternate function for pin 4
  
  //(d) We must specify whether to trigger on the rise, the fall, or both edges. In this case we will trigger on the rise of PF4.
  GPIO_PORTF_IS_R &= ~(0x10);   // clear the is bit for PF4 to enable edge detection
  GPIO_PORTF_IEV_R |= (0u << 4);        // sets detection to falling edge
  
  //(e) It is good design to clear the trigger flag during initialization
  
  GPIO_PORTF_ICR_R |= (1u << 4);        // clear the interrupt flag first
  
  // Arm the interrupt
  GPIO_PORTF_IM_R |= (1 << 4); // this arms the interrupt for pin 4
  
  //assign priority
  NVIC_PRI7_R |= (NVIC_PRI7_INT30_M & (0X00600000));    // Used a mask to set the priority to 3 -> 0b0110
  NVIC_EN0_R |= (1 << 30);      // Set bit 30 as the IRQ for PORTF interrupt is 30 in the vector table
   
}



void GPIOPortF_Handler(void){
  GPIO_PORTF_ICR_R = (1u << 4);      // acknowledge flag4 by clearing interrupt in RIS register by writing to IC register  
 
  delay(160000);
  GPIO_PORTF_DATA_R |= (1 << 1) | (1 << 2) | (1 << 3); // TOGGLE ON THE WHITE LED. 
  
}

*/




void ADC0_SS1_Init(void) {
  
  // NEED TO ACTIVATE PLL TO USE ADC. this sets it to 80MHz
  
  // Using RCC2 b/c it provids more options
  SYSCTL_RCC2_R |= (1 << 31) | (1 << 11);   // use RCC2 instead of RCC and set BYPASS2
  // Specify the crystal freq and oscillator source
    SYSCTL_RCC_R &= ~(SYSCTL_RCC_XTAL_M);       // clear XTAL
    SYSCTL_RCC_R = 0x00000540; // configure for 16 MHz crystal
    SYSCTL_RCC2_R &= ~(0x7 << 4);  // configure for main oscillator source
  //  activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~0x00002000;
  //  set the desired system divider
  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000)  // clear system clock divider
                  + (4<<22);      // configure for 80 MHz clock
  //  wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit
  // enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~0x00000800;
  
  
  
  /***************************ADC INITIALIZATION******************************/
  // Enable the ADC clock using the RCGCADC register 
  // configuring ADC0, SS1, AIN1-4, -> PE2, PE1, PE0, PD3
  SYSCTL_RCGCADC_R |= (1 << 0);        //SELECTS ADC0
  
  // enable the clock to approporiate GPIO ports
  SYSCTL_RCGCGPIO_R |= (1 << 4) | (1 << 3);       // enable clock for port E & D
  
  GPIO_PORTE_DIR_R &=  ~((1 << 1) | (1 << 2) | (1 << 0));       // set pin 0,1,2 input
  GPIO_PORTE_AFSEL_R |= (1 << 1) | (1 << 2) | (1 << 0);      // making sure alternate funtionality is selected
  GPIO_PORTE_DEN_R &= ~((1 << 1) | (1 << 2) | (1 << 0));        // DISABLE digital functionality on pin, sets it up for analog mode
  GPIO_PORTE_AMSEL_R |= (1 << 1) | (1 << 2) | (1 << 0);        // ENABLE ANALOG MODE FOR PE0,1.2

  GPIO_PORTD_DIR_R &=  ~(1 << 3);       // set pin 3 input
  GPIO_PORTD_AFSEL_R |= (1 << 3);      // making sure alternate funtionality is selected
  GPIO_PORTD_DEN_R &= ~(1 << 3);        // DISABLE digital functionality on pin, sets it up for analog mode
  GPIO_PORTD_AMSEL_R |= (1 << 3);        // ENABLE ANALOG MODE FOR PD3
  
  
  /**** configure the sample sequencer ***/

  
  // ensure the sample sequencer is diabled by clearing the corrsponding ASENn bit
  ADC0_ACTSS_R &= ~(1 << 1); // disable ss1 first. we are using SS1 cause we need 4 samples
  
  // configure trigger event for the sample sequencer in the ADCEMUX register
  ADC0_EMUX_R |=(0xF << 4);  // continuosly sample for SS1
                                //(IGNORE)trigger event is handled by software setting bit in ADCPSSI 
  
  //For each sample in the sample sequence, configure the corresponding input source in the ADCSSMUXn register.
  ADC0_SSMUX1_R  |= (0x1 << 0) | (0x2 << 4) | (0x3 << 8) | (0x4 << 12); // MUX1(for sequancer 1) is given the value of 1, 2, 3, 4
                                        // which corresponds to (AIN1, 2, 3, 4)-> (PE2,1,0 and PD3)
  
  //For each sample in the sample sequence, configure the sample control bits 
  ADC0_SSCTL1_R |= (1 << 14) | (1 << 13); // set end of sequence and enable interrupt so
                                       // to trigger at end of ADC conversion instead of by pooling
  
  //If interrupts are to be used, set the corresponding MASK bit in the ADCIM register.
  ADC0_IM_R |= (1 << 1);        // allow interrupts to be sent for sequencer 3 
 
  // set the sampling rate to 1ksps (1 kilo samples per sec)
  ADC0_PC_R |= 0x1;
  
  //assign priority. IRQ = 15
  ADC0_SSPRI_R |= (1 << 4);    // ASSIGN priority of 1 to SS1
  NVIC_PRI3_R |= ((1 << 29));    // Used a mask to set the priority to 1 -> 0b0010
  NVIC_EN0_R |= (1 << 15);      // Set bit 15 as the IRQ for ADC0SS1 interrupt is 15 in the vector table
   
    //Enable the sample sequencer logic by setting the corresponding ASENn bit
  //in the ADCACTSS register.
  ADC0_ACTSS_R |= (1 << 1); // enable ADC sequencer 1
}



void ADC0SS1_Handler(void){
  
  ADC0_ISC_R |= (1 << 1);       // Acknowledge interrupt by clearing interrupt
  POT_SAMPLES = ADC0_SSFIFO1_R;
  //POT1 = ADC0_SSFIFO1_R;
  //POT2 = ADC0_SSFIFO1_R;
  //POT3 = ADC0_SSFIFO1_R;
  //POT4 = ADC0_SSFIFO1_R;
  POT1 = POT_SAMPLES & (0x7);  // PE2 -> AIN1
  POT2 = ( (POT_SAMPLES & (0x38)) >> 3);  // PE1 -> AIN2
  POT3 = ( (POT_SAMPLES & (0x1C0) ) >> 6); // PE0 (AKA -PE0) -> AIN3
  POT4 = ( (POT_SAMPLES & (0xE00) ) >> 9);      // PD3 -> AIN4
  

  GPIO_PORTF_DATA_R = 0;  
    
  if (POT1 >= 4 && POT2 < 4 && POT3 < 4 && POT4 < 4){
  GPIO_PORTF_DATA_R |= (1 << 3);  
  //GPIO_PORTF_DATA_R = 0;  
  }
  else if (POT1 < 4 && POT2 >= 4 && POT3 < 4 && POT4 < 4){
      GPIO_PORTF_DATA_R &= 0;  
      GPIO_PORTF_DATA_R |= (1 << 2);

  }
  else if (POT1 < 4 && POT2 < 4 && POT3 >= 4 && POT4 < 4){
    GPIO_PORTF_DATA_R &= 0;
    GPIO_PORTF_DATA_R |= (1 << 1);
  
  }
  else if (POT1 < 4 && POT2 < 4 && POT3 < 4 && POT4 >= 4){
    GPIO_PORTF_DATA_R &= 0;
    //GPIO_PORTF_DATA_R |= (1 << 3) | (1 << 2);
 
  }
  else// (POT1 <= 4 && POT2 <= 5 && POT3 <= 4 && POT4 <= 4)
  GPIO_PORTF_DATA_R &= 0;
}
 

void delay(unsigned long time){
  int i = 0;
  while(i++ < time);
}


void UART_Init(void){            // should be called only once

  //1. Enable the UART module using the RCGCUART register (see page 342).
  SYSCTL_RCGCUART_R |= (1u << 0);       // enable UART0 for USB COM to pc
  
  //2. Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 338). To find out which GPIO port to enable, refer to Table 23-5 on page 1344.
  SYSCTL_RCGCGPIO_R |= (1u << 0);       // enable clock for port A
  
  //3. Set the GPIO AFSEL bits for the appropriate pins (see page 668). To determine which GPIOs toconfigure, see Table 23-4 on page 1337.
  GPIO_PORTA_AFSEL_R |= (1u << 0) | (1u << 1);  // select alternate functions for PA0&1
  
  //4. Configure the GPIO current level and/or slew rate as specified for the mode selected (seepage 670 and page 678).

  //5. Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate pins (see page 685 and Table 23-5 on page 1344)
  GPIO_PORTA_PCTL_R |= (1u << 0) | (1u << 4);   // Select UART functionality for PA0&1
  
  // enable digital functions for the pins
  GPIO_PORTA_DEN_R |= (1u << 0) | (1u << 1);    // allow digital levels 0/3.3 on the pins
  
  // set the baud rate on the UART: clk(Hz)/16/(baud rate)
  UART0_CTL_R &= ~(1u << 0);    // disable UART before modify
  UART0_FBRD_R |= 11u;        // assign 11 -> gotten from decimal of result: 64*dec
  UART0_IBRD_R |= 104u;        // assign 104 -> integer part of result from eqn above
  
  // Configure the UART clock source by writing to the UARTCC register
  UART0_CC_R   |= UART_CC_CS_SYSCLK;    //System clock (based on clock source and divisor factor)
  
  //set word length of the FIFO in UART and enable FIFO
  UART0_LCRH_R |= (1u << 6) | (1u << 5);        // select word length of 8 bits
  UART0_LCRH_R |= UART_LCRH_FEN;                // UART Enable FIFOs
  
  //Enable the UART
  UART0_CTL_R |= (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN); // enable receive, transmit, and uart
  
}




void Send_Char(unsigned char c){        // send char to pc
  
  // make sure any previous transmission has ended
  while((UART0_FR_R & UART_FR_TXFF) != 0);   // WAIT IF FIFO IS  EMPTY
    UART0_DR_R = c;
  

}

char Receive_Char(void){        // reviece data from pc
  
  char c;
  // make sure fifo is not empty to begin recieving
  while( (UART0_FR_R & UART_FR_RXFE) != 0);      // wait if the Fifo is empty
  c = UART0_DR_R;  
  return c;
  
}

void print_string(unsigned char *c)     // sends a string to the pc
{
  while(*c){
    Send_Char(*(c++));
  }
}

void SysTick_init(void) {
  NVIC_ST_CTRL_R &= (0u << 0);  // turn off enable bit before we use the systick
  NVIC_ST_CURRENT_R  = 0;       // This clears the current register
  NVIC_ST_CTRL_R |= (0x05);  // turn on enable bit and also set CLK_SRC to system clock 16Mhz
  
}

void SysTick_wait_time(unsigned long time) {
  
  // setting the reload register so counter can count down from this value
  NVIC_ST_RELOAD_R = time-1; // if time is 16 million. this means the counter will take 1 sec to count to zero if system clock is 16MHz
  while((NVIC_ST_CTRL_R & 0x00010000) == 0){   // checks if the counter is done counting.
  }  
}


void PortF_init(void) {
    
  SYSCTL_RCGCGPIO_R |= (1u << 5);        // enable clock for port F
  //SysTick_wait_time(10000);// allow for clock to stabilize

  GPIO_PORTF_DIR_R |= ((1 << 1) | (1 << 2) | (1 << 3));        // set pin 1,2,3 to output
  GPIO_PORTF_DEN_R |= ((1 << 1) | (1 << 2) | (1 << 3));        // enable digital functionality on pin
  GPIO_PORTF_AFSEL_R &= ((0 << 1) | (0 << 2) | (0 << 3));      // making sure GPIO funtionality is selected
  
  
}
