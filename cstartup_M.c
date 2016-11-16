/**************************************************
 *
 * This file contains an interrupt vector for Cortex-M written in C.
 * The actual interrupt functions must be provided by the application developer.
 *
 * Copyright 2007 IAR Systems. All rights reserved.
 *
 * $Revision: 66254 $
 *
 **************************************************/

#pragma language=extended
#pragma segment="CSTACK"

extern void __iar_program_start( void );

extern void NMI_Handler( void );
extern void HardFault_Handler( void );
extern void MemManage_Handler( void );
extern void BusFault_Handler( void );
extern void UsageFault_Handler( void );
extern void SVC_Handler( void );
extern void DebugMon_Handler( void );
extern void PendSV_Handler( void );
extern void SysTick_Handler( void );

extern void GPIOPortA_Handler( void );
extern void GPIOPortB_Handler( void );
extern void GPIOPortC_Handler( void );
extern void GPIOPortD_Handler( void );
extern void GPIOPortE_Handler( void );
extern void UART0_Handler( void );
extern void UART1_Handler( void );
extern void SSI0_Handler( void );
extern void I2C0_Handler( void );
extern void PWM0Fault_Handler( void );
extern void GPIOPortF_Handler( void );
extern void ADC0SS0_Handler( void);
extern void ADC0SS1_Handler( void);
extern void ADC0SS2_Handler( void);
extern void ADC0SS3_Handler( void);

extern void ADC1SS0_Handler( void);
extern void ADC1SS1_Handler( void);
//extern void ADC2SS2_Handler( void);
//extern void ADC3SS3_Handler( void);

extern void PWM0_0_Handler( void);
extern void PWM0_1_Handler( void);
extern void PWM0_2_Handler( void);


extern void PWM1_0_Handler( void);
extern void PWM1_1_Handler( void);
extern void PWM1_2_Handler( void);
extern void PWM1_3_Handler( void);




typedef void( *intfunc )( void );
typedef union { intfunc __fun; void * __ptr; } intvec_elem;

// The vector table is normally located at address 0.
// When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
// If you need to define interrupt service routines,
// make a copy of this file and include it in your project.
// The name "__vector_table" has special meaning for C-SPY, which
// is where to find the SP start value.
// If vector table is not located at address 0, the user has to initialize
// the  NVIC vector table register (VTOR) before using interrupts.


#pragma location = ".intvec"
const intvec_elem __vector_table[] =
{
  { .__ptr = __sfe( "CSTACK" ) },
  __iar_program_start,

  NMI_Handler,
  HardFault_Handler,
  MemManage_Handler,
  BusFault_Handler,
  UsageFault_Handler,
  0,
  0,
  0,
  0,
  SVC_Handler,
  DebugMon_Handler,
  0,
  PendSV_Handler,
  SysTick_Handler,
  GPIOPortA_Handler,
  GPIOPortB_Handler,
  GPIOPortC_Handler,
  GPIOPortD_Handler,
  GPIOPortE_Handler,
  UART0_Handler,
  UART1_Handler,
  SSI0_Handler,
  I2C0_Handler,
  PWM0Fault_Handler,
  PWM0_0_Handler,
  PWM0_1_Handler,
  PWM0_2_Handler,
  0,
  ADC0SS0_Handler,
  ADC0SS1_Handler,
  ADC0SS2_Handler,
  ADC0SS3_Handler,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  GPIOPortF_Handler,
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  ADC1SS0_Handler, 
  ADC1SS1_Handler,
  0,//ADC1SS2_Handler,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0,
  0,
  0,
  0,
  PWM1_0_Handler,
  PWM1_1_Handler,
  PWM1_2_Handler,
  PWM1_3_Handler
  


};

#pragma call_graph_root = "interrupt"
__weak void NMI_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void HardFault_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void MemManage_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void BusFault_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void UsageFault_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void SVC_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void DebugMon_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void PendSV_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void SysTick_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void GPIOPortA_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void GPIOPortB_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void GPIOPortC_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void GPIOPortD_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void GPIOPortE_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void UART0_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void UART1_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void SSI0_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void I2C0_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void GPIOPortF_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void ADC0SS0_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void ADC0SS1_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void ADC0SS2_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void ADC0SS3_Handler( void ) { while (1) {} }

#pragma call_graph_root = "interrupt"
__weak void ADC1SS0_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void ADC1SS1_Handler( void ) { while (1) {} }
//#pragma call_graph_root = "interrupt"
//__weak void ADC1SS2_Handler( void ) { while (1) {} }




#pragma call_graph_root = "interrupt"
__weak void PWM0Fault_Handler( void ) { while (1) {} }

#pragma call_graph_root = "interrupt"
__weak void PWM0_0_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void PWM0_1_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void PWM0_2_Handler( void ) { while (1) {} }

#pragma call_graph_root = "interrupt"
__weak void PWM1_0_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void PWM1_1_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void PWM1_2_Handler( void ) { while (1) {} }
#pragma call_graph_root = "interrupt"
__weak void PWM1_3_Handler( void ) { while (1) {} }





void __cmain( void );
__weak void __iar_init_core( void );
__weak void __iar_init_vfp( void );

#pragma required=__vector_table
void __iar_program_start( void )
{
  __iar_init_core();
  __iar_init_vfp();
  __cmain();
}
