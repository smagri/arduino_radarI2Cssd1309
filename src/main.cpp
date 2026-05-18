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
#define bitSet(reg, n)      ((reg) |=  (1 << (n)))
#define bitRead(reg, n)     (((reg) >> (n)) & 1)
#define bitCheck(reg, n)    (((reg) >> (n)) & 1)
#define bitClear(reg, n)    ((reg) &= ~(1 << (n)))
#define bitToggle(reg, n)   ((reg) ^=  (1 << (n)))
#define bitInverse(reg, n)  ((reg) ^=  (1 << (n)))


#define start_tc2              (TCCR2B = (TCCR2B & 0b11111000) | 0b010) // prescaler = 8
#define stop_tc2               (TCCR2B &= 0b11111000)

// Timer2/Counter2 compare match overflow interrupt enable and disable
#define enable_tc2_interrupt   (TIMSK2 |=  (1 << OCIE2A))
#define disable_tc2_interrupt  (TIMSK2 &= ~(1 << OCIE2A))


//#define pin_oc2b    PC2  // for atmega328p
#define pin_trigger PC2     // Arduino Uno digital analog pin 2
#define pin_echo PD6        // Arduino Uno digital pin 6
#define pin_servo_pwm PB2   // OC1B = Arduino Uno digital pin 10

// TC2 F = F_CPU/prescaler
// TC2 tick_period = 1/F
// So TC2 tick period = prescaler/F_CPU
// Prescaler 8 gives good resolution for TC2 tick period as 8/16MHz = 0.5us
#define prescalerTC2 8


#define SERVO_MIN_PULSE_WIDTH 2000 // 1ms pulse measured on the oscilloscope.
#define SERVO_MAX_PULSE_WIDTH 4000 // 2ms pulse measured on the oscilloscope.

// Drive from a angle greater than 0deg and less than 180deg, so we
// don't overdirve servo.
#define SERVO_MIN_ANGLE 15  // degrees
#define SERVO_MAX_ANGLE 165 // degrees

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

volatile float tLow = 0;
volatile float tHigh = 0;

volatile float tFall = 0;
volatile float tRise = 0;

volatile bool echo_signal_high_detected = 0;

volatile float Tov;
volatile float Tclk_tc1;

volatile uint16_t numOV = 0;
uint16_t icr1;


// Function Prototypes

// Debugging USART functions
void usart_debugging(void);
void usart_init(float baud_rate);
void usart_flush(void);
void usart_read_string(char *ptr_to_str);
void usart_send_byte(unsigned char data);
void usart_send_string(const char *ptr_to_str);
void usart_send_num(float num, char num_int, char num_decimal);


void config_tc2(void);
void config_tc1(void);
void my_delay_us(unsigned long x);
float sonar();
void drive_servo(void);
float linear_mapping(float x, float x1, float x2, float y1, float y2);



// Timer2 overflow, which is really compare match A overflows in CTC mode.
ISR(TIMER2_COMPA_vect)
{
    num_timer2_compare_matches++;
}



ISR(TIMER1_OVF_vect)
{
    numOV++;
}



ISR(TIMER1_CAPT_vect)
{
    icr1 = ICR1 + 1;

    float tmp = numOV * Tov + icr1 * Tclk_tc1;

    // HC-SR04 ultrasonic sensor echo signal on Analog Comaparator(AC)
    // positive pin AIN0,  3.3V on negative pin AIN1.   tFall and tLow
    // belong to ACO(Analog Comaparator Output) of the Input capture.
    
    if (!bitCheck(TCCR1B, ICES1))
    {
        // Falling edge of comparator output
        tFall = tmp;

        // AC(Analog  Comaparator) output  high  duration  for PWM  on
        // Positive AC Input.
        //
        tHigh = tFall - tRise;
        
        // AC(Analog  Comaparator)  output  low duration  for  PWM  on
        // Negative AC Input.
        //
        //tLow = tFall - tRise;
        echo_signal_high_detected = 1;
    }
    else
    {
        // Rising edge of comparator output
        tRise = tmp;

        // AC(Analog  Comaparator) output  low  duration  for PWM  on
        // Positive AC Input.
        //
        tLow = tRise - tFall;
        
        // AC(Analog  Comaparator) output  high  duration  for PWM  on
        // Negative AC Input.
        //
        // tHigh = tRise - tFall;
    }

    // Toggle between rising-edge and falling-edge capture next.
    bitToggle(TCCR1B, ICES1);
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


void config_tc1(void)
{

    ///////////////////////////////////////////////////////////////////////////
    //          TC1 is firstly used for AC and IC.                //
    ///////////////////////////////////////////////////////////////////////////

    // The Analog Comparator Output(AC0) is connected to the TC1 Input
    // Capture unit  by setting ACIC in  ACSR in main().  See  pg8 and
    // pg17 of  leacture notes for  diagrams and dataheet  for setting
    // ACSR.  AIN0 is  connected to the echo signal  of the ultrasonic
    // sensor with  a 3.3V referece  signal on AIN1.  So  AC0 measures
    // the length of the echo signal.
        

    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0;

    bitSet(TCCR1B, ICES1);   // Start by capturing rising edge
    bitSet(TCCR1B, ICNC1);   // Enable input capture noise cancellation

    bitSet(TIMSK1, TOIE1);   // Enable Timer1 overflow interrupt
    bitSet(TIMSK1, ICIE1);   // Enable Timer1 input capture interrupt

    // CHECK do we need to do this? Clear input capture flag
    // TIFR1 |= (1 << ICF1);
    
    // Timer1 prescaler = 1024.
    // For TC1:
    // CS12:CS11:CS10 = 101 means prescaler 1024.

    TCCR1B |= 0b101;

    float f = 16.0e6 / 1024.0; // Frequency hardware will run at

    // Tov = (number of ticks) * (period of one tick)
    //
    // Remember in this mode, Normal Mode,  tc1 is a 16bit register
    //
    Tov = 65536.0 / f;       // Timer1 overflow period in seconds
    Tclk_tc1 = 1.0 / f;      // Timer1 tick period in seconds


    ///////////////////////////////////////////////////////////////////////////
    //                 Fast PWM TOP mode add-on for servo PWM                //
    ///////////////////////////////////////////////////////////////////////////

    // Switch from running TC1 in normal mode to Fast PWM TOP mode.

    
        // We use Fast PWM mode 15, where TOP = OCR1A.

    //   We do not use Fast PWM mode 14, where TOP = ICR1, because ICR1
    //   is  already being  used  by  the Input  Capture  unit for  the
    //   HC-SR04 echo timing.
    //
    //   Fast PWM mode 15:
    //       WGM13:WGM12:WGM11:WGM10 = 1 1 1 1
    //       TOP = OCR1A
    //       PWM output = OC1B
    //       Duty/pulse width = OCR1B
    //
    //   OC1B is on PB2, which is Arduino Uno digital pin 10.
    

    // Fast PWM mode 15: WGM13:0 = 1111, TOP = OCR1A.
    bitSet(TCCR1A, WGM10);
    bitSet(TCCR1A, WGM11);
    bitSet(TCCR1B, WGM12);
    bitSet(TCCR1B, WGM13);

   
    // Non-inverting PWM on OC1B.
    //
    // COM1B1:COM1B0 = 1 0
    //
    // This means:
    //     OC1B goes HIGH at BOTTOM
    //     OC1B goes LOW when TCNT1 == OCR1B
    //
    // Therefore OCR1B controls the servo pulse width.
   
    bitSet(TCCR1A, COM1B1);
    bitClear(TCCR1A, COM1B0);


    // Servo PWM period = 20ms  = 50Hz.  Thus the smallest prescaler=8
    // such that  TOP=OCR1A=40000 does not exceed  the 65535(max count
    // for TC1 16bit register).
    //
    // T = 1/F  = 1 / (F_CPU  / prescaler) = prescaler  / F_CPU.  Thus
    // one tick is 8 / 16 x 10^6 = 5us.
    //
    // TOP = OCR1A = 40000 - 1, as we count from 0.
    OCR1A = 39999;

    // OCR1B controls the pulse width on OC1B/PB2/D10.
    OCR1B = 3000;   // Start servo near centre position.


    // Earlier code started Timer1 with prescaler = 1024:
    //
    //     TCCR1B |= 0b101;
    //
    // For servo PWM, as detiled earlier we need prescaler = 8 instead:
    //     CS12:CS11:CS10 = 0 1 0

    // So first clear the old clock-select bits, then set CS11.
    // This does not clear ICNC1, ICES1, WGM13, or WGM12.
    TCCR1B &= 0b11111000;   // Clear CS12:CS11:CS10 only.
    bitSet(TCCR1B, CS11);   // Start TC1 with prescaler = 8.

    
    // Update the timing variables used by the Input Capture ISR.
    //
    // In this Fast PWM TOP mode, Timer1 no longer overflows at 65536.
    // It reaches OCR1A, then resets to 0.

    // Therefore:
    //     Tov      = (OCR1A + 1) / timer_clock
    //     Tclk_tc1 = 1 / timer_clock
    
    f = 16.0e6 / 8.0; // Frequency hardware will run at = F_CPU/prescaler

    // Toverflow = (Max Timer Ticks * Time for One tick).
    Tov = (OCR1A + 1) / f;  // 20 ms overflow/TOP period
    Tclk_tc1 = 1.0 / f;     // 0.5 us Timer1 one tick period in seconds.
    
}



int main(void){
    
    // Configure Sonar Pins
    bitSet(DDRC, pin_trigger);    // HC-SR04 trigger pin as output
    bitClear(PORTC, pin_trigger); // HC-SR04 trigger pin set to low
    bitClear(DDRD, pin_echo);     // HC-SR04 echo as input
    bitClear(PORTD, pin_echo);    // Disable internal pull-up resistor
    //bitSet(PORTD, pin_echo);    // Disable internal pull-up resistor

    // Analog Comparator input pins
    // PD6 = AIN0 = positive input, echo sinal of HC-SR04
    // PD7 = AIN1 = negative input, 3.3V
    bitClear(DDRD, PD6);
    bitClear(DDRD, PD7);

    // Servo PWM pin OC1B/PB2/D10 as output.
    bitSet(DDRB, pin_servo_pwm);
    
    
    usart_init(9600);

    // Connect Analog Comparator Output to Timer1 Input Capture.
    bitSet(ACSR, ACIC);

    // configure TC1, Timer/Counter 2 for AC + IC of HC-SR04 echo signal.                      
    config_tc1(); // configure TC1, Timer/Counter 1
    config_tc2(); // configure TC2, Timer/Counter 2

    
    sei(); // Enable global interrupts

    
    while (1) {


        drive_servo();
        

        sonar();

        
        
    } // end: while(1)

        
    
    return 0;
}



///////////////////////////////////////////////////////////////////////////////
//                           User defined functions                          //
///////////////////////////////////////////////////////////////////////////////
void drive_servo(void)
{

    // Drive the servo  all the way forward and back  again wrt angle,
    // then repeat.

    // T of tick, Ttick = 1/F = 1/(F_CPU/prescaler) = prescaler/F_CPU 
    
    // With Ttick=0.5, period of one tick:
    //
    //  0.5 ms ~= 1000 ticks
    //  1.0 mv ~= 2000    
    //  1.5 ms ~= 3000 ticks
    //  2.0 mv ~= 4000
    //  2.5 ms ~= 5000 ticks
    //
    // OCR1B controls the pulse width on OC1B/PB2/D10.


    
    // Debugging:
    //
    // Sweep from 0 degrees to 180 degrees
    // for (unsigned int pulse_width_ticks=1000; pulse_width_ticks<=5000;
    //      pulse_width_ticks+=200){
        
    //     OCR1B = pulse_width_ticks;
    //     _delay_ms(100); // let the servo settle
    // }

    // _delay_ms(1000);

    // // Sweep from 180 degrees to 0 degrees
    // for (unsigned int pulse_width_ticks=5000; pulse_width_ticks>=1000;
    //      pulse_width_ticks-=200){

    //     OCR1B = pulse_width_ticks;
    //     _delay_ms(100);
    // }
    // _delay_ms(1000);

    // Lower limit, 1ms pulse measured on the oscilloscope.
    // OCR1B = 2000;
    // Centre, 1.5ms puls
    //OCR1B = 3000;
    // Upper limit, 2ms pulse measured on the oscilloscope.
    //OCR1B = 4000;

    static unsigned char angle = SERVO_MIN_ANGLE;
    static unsigned char moving_forward = 1;
    unsigned char angle_step = 5;

    // Sweep the servo from 1ms pulse  to 2ms pulse, has been measured
    // on the oscilloscope.  Increment angle by 5  degrees every time,
    // so sonar measures a distance about every 5 degrees.
    
    // Work out the next angle for the next call to drive_servo().
    if (moving_forward){
        // We are sweeping from SERVO_MIN_ANGLE toward SERVO_MAX_ANGLE
        angle += angle_step;

        // If we have reached the maximum angle, change direction
        if (angle >= SERVO_MAX_ANGLE){
            angle = SERVO_MAX_ANGLE;
            moving_forward = 0;
        }
    }
    else{
        // We are sweeping from SERVO_MAX_ANGLE back toward SERVO_MIN_ANGLE
        angle -= angle_step;

        // If we have reached the minimum angle, change direction
        if (angle <= SERVO_MIN_ANGLE){
            angle = SERVO_MIN_ANGLE;
            moving_forward = 1;
        }
    }
    
    // Linearly map sevo angle to pulse with ranges.
    OCR1B = (uint16_t)linear_mapping(angle, SERVO_MIN_ANGLE , SERVO_MAX_ANGLE,
                                     SERVO_MIN_PULSE_WIDTH,
                                     SERVO_MAX_PULSE_WIDTH);

    
    // usart_send_string("OCR1B=");
    // usart_send_num(OCR1B, 4, 2);
    // usart_send_byte('\n');
    // usart_send_string("angle=");
    // usart_send_num(angle, 4, 2);
    // usart_send_byte('\n');

    _delay_ms(100); // let the servo settle at it's new angle

}


float sonar(void){

    // Measure HCR-S04  ultrasonic sensor's length of  the echo signal
    // going  from  rising edge  to  falling  edge.  Using  this  then
    // measure and display, in the serial monitor, the distance to the
    // object.
    
    // Debugging my_delay_us() to use in HC-SR04 trigger pulse
    // bitClear(PORTC, pin_trigger);
    // my_delay_us(10UL);
    // bitSet(PORTC, pin_trigger);
    // my_delay_us(10UL);
        

    // Trigger the sonar
    bitClear(PORTC, pin_trigger);
    //my_delay_us(2);
    _delay_us(2);
    bitSet(PORTC, pin_trigger);
    //my_delay_us(11);
    _delay_us(11);
    bitClear(PORTC, pin_trigger);
        
    // We  disable interrupts  here as  we don't  want the  Timer1
    // Input Capture ISR to change them while we are using them.
    //
    // cli();
    // tHigh_copy = tHigh;
    // tLow_copy = tLow;
    // sei();

    // Wait for falling edge capture, with a timeout
    unsigned long timeout = 30000;
    while (!echo_signal_high_detected && timeout > 0) {
        //my_delay_us(1);
        _delay_us(1);
        timeout--;
    }

    float tHigh_copy; // Echo signal high time from AC+TC.
    float tLow_copy;  // Echo signal low time from AC+TC.
    float distance_to_object;
    float time_echo_signal_is_high;
    // Velocity of sound about 343m/s at 25degreesC.
    float velocity_of_sound = 343;

    if (echo_signal_high_detected){

        // We  disable interrupts  here as  we don't  want the  Timer1
        // Input Capture ISR to change them while we are using them.
        //
        cli();
        tHigh_copy = tHigh;
        tLow_copy = tLow;
        echo_signal_high_detected = 0;
        sei();
            


        // Note: Velocity  = Distance/Time,  so,
        // D = (time_echo_signal_is_high * velocity_of_sound/2).
        // Where D is the distance_to_the_obstacle in m.

        // Remember that the Distance value returned by the ultrasonic
        // sensor is 2D,  the round trip distance  of the trigger/echo
        // signals.
        
        // Which   implies   that time_echo_signal_is_high = 2D/V.
        //
        // The velocity of a sound wave is 346m/s@25degreesC.
    
        time_echo_signal_is_high = tHigh_copy;
        distance_to_object = ((time_echo_signal_is_high * velocity_of_sound) / 2);
    

        // Debugging, send values to serial monitor
        // usart_send_string("Echo Signal High=");
        // usart_send_num((tHigh_copy*1000000), 8, 2);
        // usart_send_string("us\n");

        // Dispaly on serial monitor the distance2object in cm's
        usart_send_string("Distance to object=");
        usart_send_num((distance_to_object*100.0), 6, 3);
        usart_send_string("cm\n");
        _delay_ms(100);

    }
    else{
        usart_send_string("No echo detected\n");
    }

    // Wait about 60 ms for hardware to reset before next sonar ping
//        my_delay_us(60000UL);
    _delay_us(60000UL);
}



void my_delay_us(unsigned long x){
    
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



float linear_mapping(float x, float x1, float x2, float y1, float y2){

    // Linear mapping equation: slope = (y2 -y1)/(x2-x1)
    //
    // We have y vs x 
    //
    // The slope is essentialy the gradient of the straight line, also
    // known as  the rate of change.   The rate of change  in a linear
    // equation is constant.  So when  we use linear mapping, we apply
    // a  constant rate  of change  between  the input  range and  the
    // output range.

    // To get greater y more quickly you need to increase the slope by
    // increasion y2.  

    // By swapping y1 and y2 the slope becomes -ve(as y2>y1) so we get
    // a swapped around effect, or -ve linear mapping.


    // Clamp input so it stays in  sensor range.
    //
    // 1. Clamp the input to the lower bound
    if (x <= x1) return y1;
    
    // 2. Clamp the input to the upper bound
    if (x >= x2) return y2;

    
    // Note: You lose precision with integer division because integers
    // cannot store  fractional parts. When two  integers are divided,
    // the result is truncated (the decimal part is discarded).
    
    // 3. Perform Linear Interpolation (Lerp)
    // Formula: y = y1 + (x - x1) * (slope)
    // where slope = (y2 - y1) / (x2 - x1)
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}



void usart_init(float baud_rate){


    // Setting the baud rate register:
    //
    // Fosc, internal Crystal Arduino r3 = 16MHz
    // ubrr0 = (Fosc/prescale factor * baud rate) - 1
    // U2Xn=0, prescale factor is 2/4/2=16
    // U2Xn=1, prescale factor is 2/4=8
    
    float ubrr0 = ( (F_CPU / (16.0 * baud_rate)) - 1);
    int ubrr0a = (int)ubrr0;

    // Round up if neccessary to the next integer.
    if (ubrr0 - ubrr0a >= 0.5) {
        ubrr0a = ubrr0a + 1;
    }

    // Set baud rate
    UBRR0 = ubrr0a;
    
    // Enable transmitter and receiver, and the Interrupt Service
    // Routine on the RX buffer.
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    
    // Set frame format: 8 bit data, 1 stop bit, no parity
    UCSR0C = (0 << UMSEL01) | (0 << UMSEL00) |  // Async mode
             (0 << UPM01) | (0 << UPM00) |        // No parity
             (0 << USBS0) |                       // 1 stop bit  
             (1 << UCSZ01) | (1 << UCSZ00);       // 8-bit character

}


void usart_flush(void){
    char dummy;

    // Check if the UCSROA(USART Control  and Status Register) has the
    // RXC0 flag  set.  If flag set  then there is unread  data in the
    // receive buffer.  Read what's in  the RX buffer which clears the
    // buffer.
    while (bitCheck(UCSR0A, RXC0))
        dummy = UDR0;

    ((void)dummy); // To suppress warning
}



void usart_read_string(char *ptr){

    // UDR0 is the  TX/RX data I/O register of the  arduino.  When the
    // flag RXC0  is set in  the UCR0A(Control and Status  Register A)
    // data is ready  to be read from the RX  buffer(UDR0).  When RXC0
    // is cleared in the UCR0A register the RX buffer(UDR0), is empty.

    char tmp;

    // Continue  reading bytes/characters  from the  RX buffer  of the
    // arduino MCU till EOL is found.   The data comes from the serial
    // monitor on my PC as ASCII characters.
    while (1) {

        // Block/wait till the user sends something.  Kai's code
        while (!bitCheck(UCSR0A, RXC0))
            ;            

        tmp = UDR0; // TX/RX I/O register, or data buffer.

        // Echo immediately to see if RXC0 os beomg set/
        // usart_send_byte(tmp);

        // Make sure your Serial Monitor has: Line Ending: CRLF (or at
        // least LF due to this code.).
        //
        // Some terminals send \r, some \n and some both.
        //
        // If this byte is found we are at the end of our string.
        if ( (tmp == '\r') || (tmp == '\n') ){
            // Terminate string with string terminator as per C
            // programming language spec.
            *ptr = '\0';
            //flag_read_done = 1;
            return;
        }
        else{
            // Otherwise save the character in the usart_buffer.
            *ptr++ = tmp;
        }

        
    } // end: while(1)
        
} // end: usart_read_string()



void usart_send_byte(unsigned char data){

    // Send a byte  when the UDRE0(USART data register  empty flag) id
    // is set.  In the UCSR0A (UART contorl or status register A).  It
    // means that the USART data register is empty.
    
    while (!bitCheck(UCSR0A, UDRE0))
        ;
    UDR0 = data; 
}




// check if const is really needed, cgpt says c++ compiler is strictor
// than c compiler.
void usart_send_string(const char *pstr){
    
    // note:  const means  a  read-only-string, prevents  accidentally
    // modifying  string   literals(which  causes   crashes).   String
    // literals in c  are a sequence of characters  enclosed in double
    // quotes, that is, an constant  array of characters that are null
    // terminated, ie '\0' terminated.

    // Send each byte, one at a  time, till the string terminator '\0'
    // is  sent.  '\0'  is literally  the numeric  value 0  written in
    // character form(NULL ACSII character).
    while (*pstr != '\0') {
        usart_send_byte(*pstr);
        pstr++;
    }
}



void usart_send_num(float num, char num_int, char num_decimal){

    // Send a number from MCU to PC by converting it to a string.

    // dtostrf(num,  width, precision,  string) converts  float to  an
    // ASCII string.  We add a string  terminator '\0' so the PC knows
    // when we are at the end of a string).

    
    char str[20];
    if (num_decimal == 0)
        dtostrf(num, num_int, num_decimal, str);
    else
        dtostrf(num, (num_int+num_decimal+1), num_decimal, str);
    str[num_int+num_decimal+1] = '\0';
    usart_send_string(str);
        
}
