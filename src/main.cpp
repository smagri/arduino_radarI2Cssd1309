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

// Total _time_ taken for the counter to count from 0->TOP-1 again.
// For the fast PWM TOP signal.
float timer2_overflow_time_us;

// T=1/F, the perioud of each clock tick.  Where is F/prescaler.
//float clock_tc0;
float period_of_tick;

// Kai's original code, causes my_delay_us() to hang as num_timer2_overflow overflows:
//volatile unsigned int num_timer2_overflow = 0;
//volatile unsigned int num_timer2_overflow_max_sonar = 0;
//
// Avoids counter overflow in my_delay_us() for 1s delay.
volatile uint32_t  num_timer2_overflows = 0;



// Function Prototypes
void my_delay_us(unsigned long x);
void config_tc2(void);

// Timer overflow,  number of overflows  for the current fast  PWM TOP
// signal.  A flag gets set in hardware then the ISR gets called.
//
// ISR(TIMER2_OVF_vect)
// {
//     num_timer2_overflows++;
// }

// Timer2 overflow, which is really compare match A overflows in CTC mode.
ISR(TIMER2_COMPA_vect)
{
    num_timer2_overflows++;
}



void config_tc2(void)
{
    
    // Configure Timer/Counter2 in CTC mode.

    // Mode: CTC, TOP = OCR2A
    // Clock: F_CPU / prescalerTC2

    // With:
    //     F_CPU = 16 MHz
    //     prescalerTC2 = 8

    // Timer2 tick time:

    //     tick = 8 / 16 MHz
    //          = 0.5 us

    // OCR2A = 255 gives a compare-match period of:

    //     (OCR2A + 1) ticks
    //     = 256 ticks

    //     256 * 0.5 us = 128 us

    // So  my_delay_us()  function can  count  max  128 us  using  the
    // TIMER2_COMPA  interrupt, TCNT2  is then  used directly  for the
    // remaining ticks.
    
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

    // Calculate Timer2 tick period in seconds
    period_of_tick = (float)prescalerTC2 / (float)F_CPU;

    // Time for one complete CTC cycle, in microseconds
    timer2_overflow_time_us =
        ((float)(OCR2A + 1) * period_of_tick) * 1.0e6;

    // Start Timer2
    start_tc2;
}

// void config_tc2(void){
    
//     period_of_tick = 1.0f/(float)(F_CPU/prescalerTC2);

//     // Fast PWM, TOP = OCR2A (Mode 7)
//     //
//     // Set waveform generation mode: WGM02:0 = 111
//     TCCR2A |= (1 << WGM21) | (1 << WGM20);
//     TCCR2B |= (1 << WGM22);

//     // We want PWM output on OC2B in non-inverting mode. OCR2A will be
//     // used for myTOP, proportional period  of PWM frequency and OCR2B
//     // for duty cycle.
//     //
//     // We  also require  Clear OC0B  on  compare match.   Set OC0B  at
//     // BOTTOM.
//     //
//     TCCR2A |= (1 << COM2B1);

//     // Prescaler need to be set to a value for required Fpwm such that
//     // TOP  doesn't   exceed  its   range.   Prescaler  =   128.   See
//     // notes:26/4/2026 pg1.   However, we want start_tc2  to start the
//     // timers very  close together  to get the  desired effect  on the
//     // oscilloscope.  Setting  the prescaler  here starts  the counter
//     // timing.
//     //
//     //TCCR2B |= (1 << CS22) | (1 << CS20);

//     // Setting TOP  for phase  correct PWM  TOP Mode  of TC0.   From a
//     // positive linear mapping  of last three digits of  my student id
//     // card to TOP value range.
//     //
//     // TCO is an 8 bit counter so it's range is from 0-255
//     //
//     myTOPtc2 = (uint8_t)linear_mapping(1723, 1000, 2000, 100, 255);

//     // myToptc2  is =  myToctc0  - 1.   This is  the  outcome for  the
//     // TOP/myTOP value when you make  Ftc0=Ftc2 as required.  Refer to
//     // notes: 29/4/2026 pg3.
//     //
//     // OCROA   is  proportional   to   the  Period/Frequency(see   the
//     // calculation of the Fpwm signal)  of the PWM signal.  The larger
//     // the TOP  value is it means  the timer takes longer  to complete
//     // one  PWM cycle,  so  the frequency  is  lower.  Conversly,  the
//     // smaller the  TOP means the  timer completes one PWM  faster, so
//     // the frequency is higher.
//     //
//     //
//     myTOPtc2--;
//     usart_send_string("myTOPtc2=");
//     usart_send_num(myTOPtc2, 4, 0);
//     usart_send_byte('\n');

//     OCR2A = myTOPtc2;  // This sets the Period/Frequency of the PWM signal.
//     OCR2B = 106;        // This sets the Duty Cycle of the PWM signal on OCOB/PD5

    
//     // timer2_overflow_time_us=(total_number_of_ticks * period_of_tick) * 1.0e6
//     //
//     // In our fast PWM TOP mode the timer/counter counts as such:
//     //
//     // 0 -> 1 -> 2 -> ... -> TOP -> 0 -> 1 -> ...,
//     //
//     // Note that the the reset to 0 is part of the cycle, resulting in
//     // TOP+1 transitions for the period.
//     //
//     timer2_overflow_time_us = ((float)myTOPtc2 + 1.0f) * period_of_tick * 1.0e6;

// }



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
// void my_delay_us(unsigned long x){

//     // Delays, determined with a hardware timer/counter for x us.

    
//     // NOTE: that a float assigned  to an integer returns the interger
//     // value with no decimal pont values.
        
//     // Number of complete Timer2 overflow periods needed.
//     unsigned long num_complete_overflows = ( (float)x / timer2_overflow_time_us );

//     // Remaining time after complete overflows, in microseconds.
//     float remainder_time_us =
//         (  (float)x - ((float)num_complete_overflows*timer2_overflow_time_us) );

//     // Convert remaining time into Timer2 ticks.
//     unsigned char remainder_time_ticks =
//         ( remainder_time_us / (period_of_tick * 1.0e6) );

//     //  Delay for complete number of overflows of Timer2
//     if (num_complete_overflows > 0){
//         num_timer2_overflows = 0;
//         TCNT2 = 0;

//         // // TODO: Test  Clears  and pending  overflow flag  so the  ISR doen't  run
//         // immediately and have the delay overflow too short.
//         //TIFR2 = (1 << TOV2);

//         enable_tc2_interrupt;
//         while (num_timer2_overflows < num_complete_overflows);
//         disable_tc2_interrupt;
//     }

//     // Delay for remainder number of ticks
//     TCNT2 = 0;
//     while (TCNT2 < remainder_time_ticks);
// }
// void my_delay_us(unsigned long x)
// {
    
//     // Delay for approximately x microseconds using Timer2.
    
//     // This avoids  floating-point maths,  which is  very slow  on the
//     // ATmega328P and was causing delays to become much longer.

//     // Also remember that Timer2 is an 8bit register so its max value is 255

//     // Assumptions:
//     //     F_CPU = 16 MHz
//     //     Timer2 prescaler = 8
//     // Therefore Timer2 clock frequency is:
//     //     16 MHz / 8 = 2 MHz
//     // So each Timer2 tick takes:
//     //     1 / 2 MHz = 0.5 us

//     unsigned long total_ticks;
//     unsigned long num_complete_ctc_periods;
//     unsigned char remainder_ticks;

// /*
//     Convert microseconds to Timer2 ticks.

//     Formula:

//         total_ticks = x * F_CPU / (prescalerTC2 * 1000000)

//     where:

//         x            = delay in microseconds
//         F_CPU        = CPU clock frequency
//         prescalerTC2 = Timer2 prescaler
//         1000000      = number of microseconds in 1 second
// */
//     total_ticks = (uint32_t)(((uint64_t)x * F_CPU) / ((uint64_t)prescalerTC2 * 1000000UL));


 
//     /*
//         In CTC mode with OCR2A = 255, Timer2 counts:

//             0, 1, 2, ..., 255

//         That is 256 ticks per complete CTC cycle.

//         One complete CTC period is therefore:

//             256 ticks * 0.5 us = 128 us

//         For delays longer than 128 us, we count how many complete
//         CTC compare-match periods are needed.
//     */
//     num_complete_ctc_periods = total_ticks / 256UL;

//     /*
//         After counting the complete 256-tick periods, there may be
//         some leftover ticks still needed.

//         For example, for 10 us:

//             total_ticks = 20
//             num_complete_ctc_periods = 20 / 256 = 0
//             remainder_ticks = 20

//         So for a 10 us delay, the function only waits directly for
//         TCNT2 to reach 20.
//     */
//     remainder_ticks = total_ticks % 256UL;

//     /*
//         If the requested delay is longer than one complete CTC period,
//         use the Timer2 compare-match interrupt to count full periods.

//         Note:
//         The variable name num_timer2_overflows is kept from your code,
//         but in CTC mode it is really counting compare-match events,
//         not normal Timer2 overflows.
//     */
//     if (num_complete_ctc_periods > 0)
//     {
//         // Reset the software counter
//         num_timer2_overflows = 0;

//         // Start Timer2 counting from zero
//         TCNT2 = 0;

//         /*
//             Clear any pending Timer2 compare-match A flag.

//             This prevents an old compare-match flag from immediately
//             triggering the interrupt and making the delay too short.
//         */
//         TIFR2 = (1 << OCF2A);

//         /*
//             Enable Timer2 compare-match A interrupt.

//             This requires that you have:

//                 ISR(TIMER2_COMPA_vect)

//             and that global interrupts have been enabled using:

//                 sei();
//         */
//         enable_tc2_interrupt;

//         /*
//             Wait until the required number of complete CTC periods
//             has occurred.
//         */
//         while (num_timer2_overflows < num_complete_ctc_periods)
//         {
//             // wait here
//         }

//         /*
//             Disable the Timer2 compare-match interrupt after the
//             full-period part of the delay is complete.
//         */
//         disable_tc2_interrupt;
//     }

//     /*
//         Now delay for the remaining number of Timer2 ticks.

//         For my_delay_us(10), this is the important part:

//             remainder_ticks = 20

//         So Timer2 waits until TCNT2 reaches 20.

//         Since each tick is 0.5 us:

//             20 ticks * 0.5 us = 10 us
//     */
//     TCNT2 = 0;

//     while (TCNT2 < remainder_ticks)
//     {
//         // wait here
//     }
// }
// void my_delay_us(unsigned long x)
// {
//     uint32_t total_ticks;
//     uint32_t num_complete_ctc_periods;
//     uint16_t remainder_ticks;
//     uint16_t ctc_period_ticks;

//     /*
//         Timer2 clock frequency:

//             timer2_clock_hz = F_CPU / prescalerTC2

//         Example:

//             F_CPU = 16000000
//             prescalerTC2 = 8

//             timer2_clock_hz = 16000000 / 8
//                             = 2000000 Hz

//         Since there are 1,000,000 microseconds in one second:

//             timer ticks needed =
//                 delay_us * timer2_clock_hz / 1000000

//         For x = 10 us:

//             total_ticks = 10 * 2000000 / 1000000
//                         = 20 ticks
//     */

//     total_ticks =
//         (uint32_t)(((uint64_t)x * (F_CPU / prescalerTC2)) / 1000000UL);

//     /*
//         In CTC mode, Timer2 counts from 0 to OCR2A.

//         So one complete CTC period has:

//             OCR2A + 1 ticks

//         Example:

//             OCR2A = 255
//             ctc_period_ticks = 256
//     */
//     ctc_period_ticks = (uint16_t)OCR2A + 1;

//     /*
//         Split the requested delay into:

//             1. complete CTC periods
//             2. remaining ticks
//     */
//     num_complete_ctc_periods = total_ticks / ctc_period_ticks;
//     remainder_ticks = total_ticks % ctc_period_ticks;

//     /*
//         Delay for complete CTC periods.

//         This section is not used for my_delay_us(10), because 10 us
//         only needs 20 ticks, which is less than one full 256-tick
//         CTC period.
//     */
//     if (num_complete_ctc_periods > 0)
//     {
//         num_timer2_overflows = 0;
//         TCNT2 = 0;

//         // Clear pending compare-match A flag
//         TIFR2 = (1 << OCF2A);

//         // Enable Timer2 compare-match A interrupt
//         enable_tc2_interrupt;

//         while (num_timer2_overflows < num_complete_ctc_periods)
//         {
//             // wait
//         }

//         disable_tc2_interrupt;
//     }

//     /*
//         Delay for the remaining ticks.

//         For my_delay_us(10), with prescalerTC2 = 8:

//             remainder_ticks = 20

//         Timer2 tick = 0.5 us

//             20 ticks * 0.5 us = 10 us
//     */
//     TCNT2 = 0;

//     while (TCNT2 < remainder_ticks)
//     {
//         // wait
//     }
// }
// void my_delay_us(unsigned long x)
// {
//     /*
//         Delay for approximately x microseconds using Timer2 in CTC mode.

//         Uses:

//             F_CPU
//             prescalerTC2
//             OCR2A

//         Formula:

//             total_ticks = x / time_of_one_tick

//         But written using integer maths as:

//             total_ticks = x * F_CPU / (prescalerTC2 * 1000000)

//         Example:

//             F_CPU = 16000000
//             prescalerTC2 = 8
//             x = 10 us

//             total_ticks = 10 * 16000000 / (8 * 1000000)
//                         = 20 ticks

//         Since each Timer2 tick is 0.5 us:

//             20 ticks = 10 us
//     */

//     uint32_t total_ticks;
//     uint32_t num_complete_ctc_periods;
//     uint16_t remainder_ticks;
//     uint16_t ctc_period_ticks;

//     /*
//         Convert requested microseconds into Timer2 ticks.

//         This is the important line.

//         It is the same idea as:

//             total_ticks = x * F_CPU / (prescalerTC2 * 1000000)

//         The uint64_t casts stop x * F_CPU from overflowing.
//     */
//     total_ticks =
//         (uint32_t)(((uint64_t)x * F_CPU) /
//         ((uint64_t)prescalerTC2 * 1000000UL));

//     /*
//         In CTC mode, Timer2 counts from 0 to OCR2A.

//         So if OCR2A = 255:

//             ctc_period_ticks = 256
//     */
//     ctc_period_ticks = (uint16_t)OCR2A + 1;

//     /*
//         Split the required delay into:

//             1. complete CTC periods
//             2. leftover ticks
//     */
//     num_complete_ctc_periods = total_ticks / ctc_period_ticks;
//     remainder_ticks = total_ticks % ctc_period_ticks;

//     /*
//         Delay for complete CTC periods.

//         For my_delay_us(10), this section should be skipped because:

//             total_ticks = 20
//             ctc_period_ticks = 256
//             num_complete_ctc_periods = 0
//     */
//     if (num_complete_ctc_periods > 0)
//     {
//         num_timer2_overflows = 0;

//         TCNT2 = 0;

//         // Clear pending Timer2 compare-match A flag
//         TIFR2 = (1 << OCF2A);

//         // Enable Timer2 compare-match A interrupt
//         enable_tc2_interrupt;

//         // Wait until enough complete CTC periods have occurred
//         while (num_timer2_overflows < num_complete_ctc_periods)
//         {
//             // wait
//         }

//         // Disable Timer2 compare-match A interrupt
//         disable_tc2_interrupt;
//     }

//     /*
//         Delay for the remaining ticks.

//         For my_delay_us(10), with:

//             F_CPU = 16000000
//             prescalerTC2 = 8

//         this gives:

//             remainder_ticks = 20

//         So Timer2 waits for 20 ticks:

//             20 * 0.5 us = 10 us
//     */
//     TCNT2 = 0;

//     while (TCNT2 < remainder_ticks)
//     {
//         // wait
//     }
// }
