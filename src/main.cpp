#ifndef F_CPU
#define F_CPU 16000000UL
#endif // Set system clock frequency



#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>



// These  are my  version of  the bit  manipulation functions,  so you
// don't need to know the order of precedence rules.

// bitSet() Sets bit n of register reg

// bitClear() Sets bit n to 0 in register reg

// Note bigCheck()  and bitRead() do  the same thing, they  return 0/1
// depending  on   what  n   bit  value   we  require   from  register
// reg. bigCheck() however has extra parentheses for safety.

// bitInverse() Toggles of flips bit n in register reg.

#define bitSet(reg, n) (reg |= (1 << n))
#define bitRead(reg, n) ((reg >> n) & 1)
#define bitCheck(reg, n) ((reg) >> (n)) & 1)
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitInverse(reg, n) (reg ^= (1 << n))


#define start_tc2              (TCCR2B = (TCCR2B & 0b11111000) | 0b010) // prescaler = 8
#define stop_tc2               (TCCR2B &= 0b11111000)

// Timer2/Counter2 compare match overflow interrupt enable and disable
#define enable_tc2_interrupt   (TIMSK2 |=  (1 << OCIE2A))
#define disable_tc2_interrupt  (TIMSK2 &= ~(1 << OCIE2A))


//#define pin_oc2b    PC2  // for atmega328p
#define pin_trigger PC2
#define pin_echo PD6

// TC2 F = F_CPU/prescaler
// TC2 tick_period = 1/F
// So TC2 tick period = prescaler/F_CPU
// Prescaler 8 gives good resolution for TC2 tick period as 8/16MHz = 0.5us
#define prescalerTC2 8

/// / Total _time_ taken for the counter to count from 0->TOP-1 again.
// // For the fast PWM TOP signal.
// float timer2_overflow_time_us;

// // T=1/F, the perioud of each clock tick.  Where is F/prescaler.
// //float clock_tc0;
// float period_of_tick;

// // Kai's original code, causes my_delay_us() to hang as num_timer2_overflow overflows:
// //volatile unsigned int num_timer2_overflow = 0;
// //volatile unsigned int num_timer2_overflow_max_sonar = 0;
// //
// Avoids counter overflow in my_delay_us() for 1s delay.
volatile uint32_t  num_timer2_compare_matches = 0;



// Function Prototypes
void my_delay_us(unsigned long x);
void config_tc2(void);



// Timer2 overflow, which is really compare match A overflows in CTC mode.
ISR(TIMER2_COMPA_vect)
{
    num_timer2_compare_matches++;
}



void config_tc2(void)
{
    
    // Configure Timer/Counter2 in CTC mode.

    // Mode: CTC, TOP = OCR2A
    
    // Stop Timer 2 while we configure it for safety.
    stop_tc2;

    // Normal port operation, CTC mode selected using WGM21 = 1
    TCCR2A = 0;
    TCCR2B = 0;

    // CTC mode: WGM22:0 = 010
    // WGM22 is in TCCR2B, WGM21 and WGM20 are in TCCR2A
    TCCR2A |= (1 << WGM21);

    // Set TOP value for CTC mode
    OCR2A = 255;

    // Start counter from 0
    TCNT2 = 0;

    // Clear pending compare-match A flag
    TIFR2 = (1 << OCF2A);

    // Disable compare-match interrupt initially
    disable_tc2_interrupt;

    // Start Timer2
    start_tc2;
}



int main(void){

    // Configure Sonar Pins
    bitSet(DDRC, pin_trigger);    // HC-SR04 trigger pin as output
    bitClear(PORTC, pin_trigger); // HC-SR04 trigger pin set to low
    bitClear(DDRD, pin_echo);     // HC-SR04 echo as input
    bitClear(PORTD, pin_echo);    // Disable internal pull-up resistor
    
    config_tc2(); // configure TC2, Timer/Counter 2
    
    sei(); // Enable global interrupts

    while (1) {

        bitClear(PORTC, pin_trigger);
        my_delay_us(10UL);
        bitSet(PORTC, pin_trigger);
        my_delay_us(10UL);
        
    }

        
    
    return 0;
}



///////////////////////////////////////////////////////////////////////////////
//                           User defined functions                          //
///////////////////////////////////////////////////////////////////////////////

void my_delay_us(unsigned long x)
{
    // Delay x us
    
    // Remember we are trying to  avoid floating point calculations as
    // these are very slow on the atmega328p.

    unsigned long total_ticks;
    unsigned long num_complete_ctc_periods;
    unsigned char remainder_ticks;

    
    // total_ticks = x / period_of_one_tick_in_us
    //
    // period_of_one_tick = 1 / F
    // F = F_CPU / prescaler
    // period_of_one_tick = 1 / (F_CPU / prescaler)
    // period_of_one_tick = prescaler / F_CPU
    //
    // period_of_one_tick_us = (prescaler / F_CPU) * 1000000UL
    // period_of_one_tick_us = (prescaler * 1000000UL) / F_CPU
    //
    // total_ticks = x / ((prescaler * 1000000UL) / F_CPU)
    // total_ticks = x * F_CPU / (prescalerTC2 * 1000000UL);
    // == total_ticks = x * (F_CPU / prescalerTC2 / 1000000UL);

    total_ticks = x * F_CPU / (prescalerTC2 * 1000000UL);
        

    // Remember  that TC2  is an  8 bit  Timer/Counter so  its maximum
    // count is 256 values.
    num_complete_ctc_periods = total_ticks / 256UL;
    remainder_ticks = total_ticks % 256UL;

    // Delay for complete number of of Timer2 compare matches.
    if (num_complete_ctc_periods > 0){
        num_timer2_compare_matches = 0;
        TCNT2 = 0;
        TIFR2 = (1 << OCF2A);

        enable_tc2_interrupt;
        while (num_timer2_compare_matches < num_complete_ctc_periods)
        {
            // wait
        }
        disable_tc2_interrupt;
    }

    // Delay for the remainder number of Timer2 ticks
    TCNT2 = 0;
    while (TCNT2 < remainder_ticks){
        // wait
    }
}
