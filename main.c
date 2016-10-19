//#include <lm4f120h5qr.h>
#include <stdint.h>     // allows us to use unint32_t that are in the macro headers
#include <tm4c123gh6pm.h>
#include <stdio.h>

void UART_Init(void);
void PortF_init(void);
char Receive_Char(void);
void Send_Char(unsigned char c);
void print_string(unsigned char *c);


int main()
{ 
    UART_Init();
    PortF_init();
    char c;
    
  while(1){
    
    print_string("Please enter either, \'b\', \'r\', \'g\': \n\r");
    c = Receive_Char(); // wait for user to enter a value
    Send_Char(c);       // send this value to pc via UART;
    print_string("\n");
    switch(c) { 
    case 'r':
      GPIO_PORTF_DATA_R &= 0;   //CLEAR BITS
      GPIO_PORTF_DATA_R |= (1u << 1);   // make led red
      if(GPIO_PORTF_DATA_R & (1u << 1) != 0)
         GPIO_PORTF_DATA_R &= ~(1u << 1);
      break;
    case 'g':
      GPIO_PORTF_DATA_R &= 0;
      GPIO_PORTF_DATA_R |= (1u << 3); // make led green
        if(GPIO_PORTF_DATA_R & (1u << 3) != 0)
         GPIO_PORTF_DATA_R &= ~(1u << 3);
      break;
    case 'b':
      GPIO_PORTF_DATA_R &= 0;
      GPIO_PORTF_DATA_R |= (1u << 2);   // make led blue
        if(GPIO_PORTF_DATA_R & (1u << 2) != 0)
         GPIO_PORTF_DATA_R &= ~(1u << 2);
      break;
    default:
      GPIO_PORTF_DATA_R |= (1u << 1)| (1u << 2)| (1u << 3);   // make led white
      break;
  
    }
  }
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


void PortF_init(void) {
    
  SYSCTL_RCGCGPIO_R |= (1u << 5);        // enable clock for port F
  //SysTick_wait_time(10000);// allow for clock to stabilize

  GPIO_PORTF_DIR_R |= ((1 << 1) | (1 << 2) | (1 << 3));        // set pin 1,2,3 to output
  GPIO_PORTF_DEN_R |= ((1 << 1) | (1 << 2) | (1 << 3));        // enable digital functionality on pin
  GPIO_PORTF_AFSEL_R &= ((0 << 1) | (0 << 2) | (0 << 3));      // making sure GPIO funtionality is selected
  
  
}

void Send_Char(unsigned char c){        // send char to pc
  
  // make sure any previous transmission has ended
  while((UART0_FR_R & UART_FR_TXFF) != 0);   // WAIT IF FIFO IS FULL EMPTY
    UART0_DR_R = c;
  

}

char Receive_Char(void){        // reviece data from pc
  
  char c;
  // make sure fifo is not empty to begin recieving
  while( (UART0_FR_R & UART_FR_RXFE) != 0);      // wait if the Fifo is empty
  c = UART0_DR_R;  
  return c;
  
}

void print_string(unsigned char *c)
{
  while(*c){
    Send_Char(*(c++));
  }
}
