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


void my_delay_us(unsigned long x);
void config_tc2(void);
void config_tc1(void);
float sonar(float tHigh);




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

    // HC-SR04 echo  signal on  Comaparator(AC) positive pin,  3.3V on
    // negative pin  tFall and  tLow belong to  ACO(Analog Comaparator
    // Output) of the Input capture.
    
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
    /*
        TC1 is used for AC and Input Capture.

        The  Analog Comparator  output(AC0)  is connected  to the  TC1
        Input Capture unit by setting ACIC in ACSR in main().  See pg8
        and  pg17 of  leacture  notes for  diagrams  and dataheet  for
        setting ACSR.
        
    */

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

    usart_init(9600);

    // Connect Analog Comparator output to Timer1 Input Capture
    bitSet(ACSR, ACIC);

    // configure TC1, Timer/Counter 2 for AC + IC of HC-SR04 echo signal                       
    config_tc1();
    config_tc2(); // configure TC2, Timer/Counter 2

    
    sei(); // Enable global interrupts

    float tHigh_copy;
    float tLow_copy;
    float distance2object;
    
    while (1) {

        // Debugging my_delay_us() to use in HC-SR04 trigger pulse
        // bitClear(PORTC, pin_trigger);
        // my_delay_us(10UL);
        // bitSet(PORTC, pin_trigger);
        // my_delay_us(10UL);
        

        // Trigger the sonar
        bitClear(PORTC, pin_trigger);
        my_delay_us(2);
        bitSet(PORTC, pin_trigger);
        my_delay_us(11);
        bitClear(PORTC, pin_trigger);
        
        // We  disable interrupts  here as  we don't  want the  Timer1
        // Input Capture ISR to change them while we are using them.
        //
        cli();
        tHigh_copy = tHigh;
        tLow_copy = tLow;
        sei();

        // Wait for falling edge capture, with a timeout
        unsigned long timeout = 30000;
        while (!echo_signal_high_detected && timeout > 0) {
            my_delay_us(1);
            timeout--;
        }

        if (echo_signal_high_detected){

            // We  disable interrupts  here as  we don't  want the  Timer1
            // Input Capture ISR to change them while we are using them.
            //
            cli();
            tHigh_copy = tHigh;
            tLow_copy = tLow;
            echo_signal_high_detected = 0;
            sei();
            
            distance2object = sonar(tHigh_copy);

            // Debugging, send values to serial monitor
            usart_send_string("Echo Signal High=");
            usart_send_num((tHigh_copy*1000000), 8, 2);
            usart_send_string("us\n");

            usart_send_string("Distance to object=");
            usart_send_num((distance2object*100.0), 6, 3);
            usart_send_string("cm\n");
            _delay_ms(1000UL);

        }
        else{
            usart_send_string("No echo detected\n");
        }
        // Wait about 60 ms for hardware to reset before next sonar ping
        my_delay_us(60000UL);

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



float sonar(float tHigh_copy){

    float time_echo_signal_is_high;
    float distance_to_object;
    float velocity_of_sound = 343; // Velocity of sound about 343m/s
                                   // at 25degreesC.

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
    

    return distance_to_object;
}
