#include <avr/interrupt.h>
#include <typedefs.h>

volatile u16 time_ms;

void wait(u08);
void tinywait(u08);
void timing_init0(u08);
void timing_init2(u08);
void wait_ms(u16);
