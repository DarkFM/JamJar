//#include <lm4f120h5qr.h>
#include <stdint.h>     // allows us to use unint32_t that are in the macro headers
#include <tm4c123gh6pm.h>
#include <stdio.h>

void UART_Init(void);
void PortF_init(void);
char Receive_Char(void);
void Send_Char(unsigned char c);
void print_string(unsigned char *c);
void SysTick_init(void);
void SysTick_wait_time(unsigned long time);

#define char_size 30
#define time_delay 4000000
    unsigned char command_byte;
    unsigned char para1, para2;
/*
    unsigned char *cmd_names[8] = {  
    " OFF", "ON", "AFTR", "CHG", "PROG", "CHAF", "PTCH", "SYS"};
    unsigned char *sys_names[8] = {
    "  EX", "F/4", "SPOS", "SSEL", "0xF4", "0xF5", "TUNE", "EOX"};
  */ 
    
    
/*      WE WILL USE PD7-> TXD ------ PD6-> RXD*/

int main()
{ 
    PortF_init();
    SysTick_init();
    UART_Init();

    unsigned char command_store;
    unsigned char channel_store;


  while(1){
    

     
      command_byte = Receive_Char();
      command_store = command_byte & 0xf0;
      channel_store = command_byte & 0x0f;
     // GPIO_PORTF_DATA_R |= (1u << 2) | (1u << 3);   // make led purpule
      SysTick_wait_time(time_delay);
      GPIO_PORTF_DATA_R &= ~(1u << 2) | (1u << 3);   // turn off


    
      if(channel_store == 0x0){
       // print_string("Ch-");
        //Send_Char(channel_store);
        //print_string("HEX\n");
      }
      //print_string(cmd_names[command_store >>4 & 0x7]);
     // print_string("\n");
      switch(command_store) {
        //these commands have two parameters
            case 0x80:
            case 0x90:
            case 0xa0:
            case 0xb0:
            case 0xe0:
                para1 = Receive_Char();
                
                GPIO_PORTF_DATA_R &= 0;   //CLEAR BITS
                GPIO_PORTF_DATA_R |= (1u << 1) | (1u << 2) | (1u << 3);   // make led white
                SysTick_wait_time(time_delay);
                //if(GPIO_PORTF_DATA_R & (1u << 1) != 0)
                    //GPIO_PORTF_DATA_R &= ~((1u << 1) | (1u << 2) | (1u << 3));
                
                para2 = Receive_Char();
               // Send_Char(para1);
               // print_string("HEX/n");
               // Send_Char(para2);
               // print_string("HEX/n");
                
                GPIO_PORTF_DATA_R &= 0;   //CLEAR BITS
                GPIO_PORTF_DATA_R |= (1u << 2);   // make led blue
                SysTick_wait_time(time_delay);
                // GPIO_PORTF_DATA_R &= 0;   //CLEAR BITS
                //if(GPIO_PORTF_DATA_R & (1u << 2) != 0)
                //    GPIO_PORTF_DATA_R &= ~(1u << 2);
                break;
            
        //These ones have only one parameter
            case 0xc0:
            case 0xd0:
                para1 = Receive_Char();
                //Send_Char(para1);
                //print_string("HEX/n");
                
                GPIO_PORTF_DATA_R &= 0;   //CLEAR BITS
                GPIO_PORTF_DATA_R |= (1u << 3);   // make led green
                SysTick_wait_time(time_delay);
                // GPIO_PORTF_DATA_R &= 0;   //CLEAR BITS
                // if(GPIO_PORTF_DATA_R & (1u << 3) != 0)
                 //   GPIO_PORTF_DATA_R &= ~(1u << 3);
                break;
                
        //Handle system ones specially
            case 0xf0:
                //print_string(sys_names[command_store & 0x7]);
                //print_string("/n");
                break;
      default:
        GPIO_PORTF_DATA_R &= 0;   //CLEAR BITS
        SysTick_wait_time(time_delay);
        break;

   
    }
}
}




void UART_Init(void){            // should be called only once

  //1. Enable the UART module using the RCGCUART register (see page 342).
  SYSCTL_RCGCUART_R |= (1u << 2);       // enable UART2 
  
  //2. Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 338). To find out which GPIO port to enable, refer to Table 23-5 on page 1344.
  SYSCTL_RCGCGPIO_R |= (1u << 3);       // enable clock for port D
  
  //3. Set the GPIO AFSEL bits for the appropriate pins (see page 668). To determine which GPIOs toconfigure, see Table 23-4 on page 1337.
  GPIO_PORTD_AFSEL_R |= (1u << 7) | (1u << 6);  // select alternate functions for PD7&6
  
  //4. Configure the GPIO current level and/or slew rate as specified for the mode selected (seepage 670 and page 678).

  //5. Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate pins (see page 685 and Table 23-5 on page 1344)
  GPIO_PORTD_PCTL_R |= (1u << 28) | (1u << 24);   // Select UART functionality for PD7&6
  
  // enable digital functions for the pins
  GPIO_PORTD_DEN_R |= (1u << 7) | (1u << 6);    // allow digital levels 0/3.3 on the pins
  
  // set direction of pins
  GPIO_PORTD_DIR_R |= (0u << 6);
  
  // set the baud rate on the UART: clk(Hz)/16/(baud rate)
  UART2_CTL_R &= ~(1u << 0);    // disable UART before modify
  UART2_FBRD_R |= 0u;        // assign 11 -> gotten from decimal of result: 64*dec
  UART2_IBRD_R |= 32u;        // assign 104 -> integer part of result from eqn above
  
  // Configure the UART clock source by writing to the UARTCC register
  UART2_CC_R   |= UART_CC_CS_SYSCLK;    //System clock (based on clock source and divisor factor)
  
  //set word length of the FIFO in UART and enable FIFO
  UART2_LCRH_R |= (1u << 6) | (1u << 5);        // select word length of 8 bits
  UART2_LCRH_R |= UART_LCRH_FEN;                // UART Enable FIFOs
  
  // enable UART interrupt for recieve and transmit
 // UART2_IFLS_R |= (1 << 4); // trigger when RX fifo is 1/2 full
 // UART2_IFLS_R |= (1 << 1); //trigger when TX FIFO is 1/2 full
  
  
  // set the corresponding bits in the interrupt Mask to allow interrupts
  //UART2_IM_R |= (1u << 4); // allows an interrupt to be sent when RXRIS bit
                            // in the UARTRIS register(read-only) is set
  //UART2_IM_R |= (1u << 5); // allows intterupt to be sent when transmitting
 
  // assign interrupt priority. IQR=33, So use PRI8 -> Interrupt 32-35
  //NVIC_PRI8_R |= (NVIC_PRI8_INT33_M & (1u << 14)); // SET priority to 2
  // enable the interrupt to be serviced
 // NVIC_EN1_R |= (1u << 1);      // sets interrupt 33 to be enabled
  
  //Enable the UART
  UART2_CTL_R |= (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN); // enable receive, transmit, and uart
  
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
  while((UART2_FR_R & UART_FR_TXFF) != 0);   // WAIT IF FIFO IS FULL EMPTY
    UART2_DR_R = c;
  

}

char Receive_Char(void){        // reviece data from pc
  
  char c;
  // make sure fifo is not empty to begin recieving
  while( (UART2_FR_R & UART_FR_RXFE) != 0);      // wait if the Fifo is empty
  c = UART2_DR_R;  
  return c;
  
}

void print_string(unsigned char *c)     // sends a string to the pc
{
  while(*c){
    Send_Char(*(c++));
  }
}

/*
// triggers when FIFO is 1/2 full
void UART0_Handler(void){
 //acknowledge the recieve interrupt
  //if(UART2_RIS_R&(1<< 4) !=0){
    UART0_ICR_R |= (1<<4);      // acknowledge interrupt
    GPIO_PORTF_DATA_R ^= (1u << 2);   // TOGGLE blue LED
    command_byte = UART0_DR_R;  // store the char from FIFO
 // }
}
*/

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





/*

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

*/


/*
static void HardFault_Handler( void )
{
__asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}
*/