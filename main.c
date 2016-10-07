#include <lm4f120h5qr.h>
/*
int counter;
int main()
{
  int *p;
  p = &counter;
  while (*p < 21){
    ++(*p);
  }
  p = (int*)0x20000002u;
  *p = 0xDEADBEEF;
  return 0;
}
*/

void wait(void);

int main()
{
  SYSCTL -> RCGCGPIO = 0x20;
  
  GPIOF->DIR = 0xf;
  
  GPIOF->DEN = 0Xe;
    
    
  while(1){
    
    GPIOF->DATA = 0x02;
    wait();
    GPIOF->DATA = 0X04;
    wait();
    GPIOF->DATA = 0X8;
    wait();
    
  }

}


void wait(void){
  
  int clock_counter = 0;
  while(clock_counter++ < 1000000);
    
}