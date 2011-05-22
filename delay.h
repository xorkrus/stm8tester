// פאיכ delay.h
#ifndef DELAY_H
#define DELAY_H

#ifndef F_CPU
   #error "F_CPU not defined!"
#endif

#define US(x) \
(unsigned int)((((x*(F_CPU/1000000.0))<=8)*8 \
+((x*(F_CPU/1000000.0))>8)*(x*(F_CPU/1000000.0)) - 4)/4)

#define MS(x) US(x*1000)

@inline void delay(unsigned int del)
{
volatile unsigned int tmp;
tmp = del;
#asm
$N:
        decw x
        jrne $L
        nop
#endasm 
}
#endif // #ifndef DELAY_H
