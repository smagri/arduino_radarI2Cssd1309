#ifndef F_CPU
#define F_CPU 16000000UL
#endif // Set system clock frequency


// Arduino libraries  for ssd1309 OLED  display, allowed to use  as it
// was not covered in the course.
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>



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
#define pin_servo_pwm PB2   // OC1B, Arduino Uno digital pin 10
#define pin_led_pwm PD5     // OC0B, Arduino Uno digital pin 5

// External Interrupt INT1 , to trigger system start
#define pin_int1_interrupt PD3
// External Interrupt INT1 , to trigger system stop
#define pin_int0_interrupt PD2

// TC2 F = F_CPU/prescaler
// TC2 tick_period = 1/F
// So TC2 tick period = prescaler/F_CPU
// Prescaler 8 gives good resolution for TC2 tick period as 8/16MHz = 0.5us
//#define prescalerTC2 8
#define prescalerTC2 64
#define prescalerTC0 8


// RX recive buffer size for debugging
#define BUFFER_SIZE 50

#define SERVO_MIN_PULSE_WIDTH 2000 // 1ms pulse measured on the oscilloscope.
#define SERVO_MAX_PULSE_WIDTH 4000 // 2ms pulse measured on the oscilloscope.


// Drive from a angle greater than 0deg and less than 180deg, so we
// don't overdirve servo.
// For testing:
//#define SERVO_MIN_ANGLE 15  // degrees
//#define SERVO_MAX_ANGLE 165 // degrees
#define SERVO_MIN_ANGLE 0  // degrees
#define SERVO_MAX_ANGLE 180 // degrees

#define ADC_MIN 0
#define ADC_MAX 1023

#define OLED_CONTRAST_MIN 0
#define OLED_CONTRAST_MAX 255

// Radar display settings:
//
//#define SCREEN_WIDTH   128
//#define SCREEN_HEIGHT  64
//#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

// U8g2 SSD1309 I2C constructor.
// Uses small page buffer to save SRAM on Arduino Uno.
// Preconfigured for a 128x64 pixel display
U8G2_SSD1309_128X64_NONAME2_1_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE
);

// If the screen stays blank or looks wrong, try this constructor instead:
// U8G2_SSD1309_128X64_NONAME0_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


// =============================================================
// Radar display settings
// =============================================================

//#define OLED_I2C_ADDRESS        0x3C

//#define RADAR_SCREEN_WIDTH      128
//#define RADAR_SCREEN_HEIGHT     64

#define RADAR_ORIGIN_X          64
#define RADAR_ORIGIN_Y          63

// Leave some top space for text.
//#define RADAR_RADIUS            51
// Leave some space at the top for angle/distance text
#define RADAR_RADIUS_PIXELS 48

// Change this to match your selected radar range.
#define RADAR_MAX_DISTANCE_CM   100.0f

#define PI_FLOAT 3.14159265f    

// On an Arduino Uno, the SSD1309 128×64 display needs a 1024-byte RAM
// framebuffer.   Normal  variables,  USART buffer,  Adafruit  object,
// stack, and  plain C string literals  all share the Uno’s  tiny 2 KB
// SRAM.  If x.begin() cannot allocate  the display buffer, it returns
// false, which sends the program oled_init() into failed mode and the
// state machine  does not run.  This  uses an AVR technique  to store
// the c string literals in flash not SRAM and avoid this problem.
//    
#define usart_send_string_flash(str) usart_send_string_flash_real(PSTR(str))



/// / Total _time_ taken for the counter to count from 0->TOP-1 again.
// // For the fast PWM TOP signal.
float timer2_overflow_time_us;

// // T=1/F, the perioud of each clock tick.  Where is F/prescaler.
// //float clock_tc0;
float period_of_tick_tc2;

// // Kai's original code, causes my_delay_us() to hang as num_timer2_overflow overflows:
// //volatile unsigned int num_timer2_overflow = 0;
// //volatile unsigned int num_timer2_overflow_max_sonar = 0;
// //
// Avoids counter overflow in my_delay_us() for 1s delay.
volatile uint32_t  num_timer2_compare_matches = 0;
volatile uint32_t  num_timer2_overflows = 0;

volatile float tLow = 0;
volatile float tHigh = 0;

volatile float tFall = 0;
volatile float tRise = 0;

volatile bool echo_signal_high_detected = 0;

volatile float Tov;
volatile float Tclk_tc1;

volatile uint16_t numOV = 0;
uint16_t icr1;

volatile unsigned char flag_system_start = 0;
volatile bool flag_system_stop = 0;

// Current ADC value read in ADC_vect ISR.
volatile uint16_t adc_cur = 0;

// Counts the number  of ms, each 1ms  is a CTC compare  match of TC2.
// Where TC2 prescaler is 64.
volatile uint32_t system_time_ms = 0;


// Our program buffer  that stores TX/RX data for the  Arduino that we
// want to transmit from MCU->PC or recive data from the PC->MCU.
//
// all elements of usart_buffer are initially assgined to 0.
char usart_buffer[BUFFER_SIZE] = {0};
char *ptr_to_usart_buffer = usart_buffer;

// Flags for respective  main state machine modes  to print debugging,
// or  not, data  as it  is calculated.   Input is  taken from  the PC
// serial  monitor and  recived by  the MCU  via an  interrupt service
// routine, hence this is a non-blocking uart RX buffer read.
bool usart_debugging_mode_object_distance = 0;
bool usart_debugging_mode_angle = 0;
bool usart_debugging_mode_adc = 0;

// Set to volatile as can be changed in the ISR.  Indicates the RX
// xfer of string from PC to MCU is complete or not.
volatile bool flag_rx_done = 0; // Initialised to not complete.

// radar/ultrasonic sensor current angle and distance to object
float cur_radar_angle;
float cur_radar_distance_to_object_cm;


// Setup State Types
typedef enum{
    IDLE_MODE,
    SERVO_MODE,
    SONAR_MODE
} State;

volatile State state_current;



// Function Prototypes

// Debugging USART functions
void usart_debugging(void);
void usart_init(float baud_rate);
void usart_flush(void);
void usart_read_string(char *ptr_to_str);
void usart_send_byte(unsigned char data);
void usart_send_string(const char *ptr_to_str);
void usart_send_string_flash_real(const char *ptr_to_str);
void usart_send_num(float num, char num_int, char num_decimal);
void print_usart_debugging_mode_menu(void);
void set_user_required_usart_debugging_mode(int8_t user_choice);
void usart_clear_print(void);
    
void config_tc2(void);
void config_tc1(void);
void config_tc0(void);
void interrupt_init(void);
void my_delay_us(uint16_t delay_us);
//void my_delay_us(unsigned long x);
float sonar(void);
float drive_servo(void);
float linear_mapping(float x, float x1, float x2, float y1, float y2);
void led_pwm_on(void);
void led_pwm_off(void);
void adc_init(void);
uint8_t get_adc_to_oled_contrast(uint16_t adc_raw);
void oled_set_contrast(uint8_t oled_contrast);


//  OLED radar display functions:
float clamp_float(float value, float minimum, float maximum);
void radar_point_from_radius(float angle_deg, float radius_pixels, int *x, int *y);
void radar_point_from_distance(float angle_deg, float distance_cm, int *x, int *y);
void draw_radar_arc(int radius_pixels);
void draw_radar_grid(void);
void draw_radar_text(float angle_deg, float distance_cm, bool object_detected);
void draw_radar_sweep(float angle_deg, float distance_cm, bool object_detected);
void radar_display_update(float angle_deg, float distance_cm, bool object_detected);
void oled_init(void);
void oled_contrast_test(void);
void oled_clear(void);


ISR(ADC_vect){

    // System ADC  value reading ISR  routine.  These ADC  values from
    // will be linearly  mapped to OLED screen contrast.
    
    //usart_send_string("dbg: in ISR\n");
    
    // Triggers when interrupt conversion is complete.
    adc_cur = ADC; // Read ADC data register.
}



ISR(USART_RX_vect){

    // We have recived the interrupt indicating the RX buffer contains
    // data.  Remember first we need to enable the interrupt for, data
    // in RX  buffer, with the  RXCIE0 flag  in UCSR0B.

    // We have used  this routine for the system to  set its debugging
    // state by the user sending values to the USART.
    
    char tmp = UDR0;

    //usart_send_string("\ndbg: USART_RX_vect(): in ISR\n");

    if (flag_rx_done == 0){
        // If we  have reached the end  of string sent from  the PC to
        // the MCU. Terminate the string as per C standard.  Otherwise
        // put the UDRO RX data into our usart_buffer.
        if ( (tmp == '\r') || (tmp == '\n') ){
            *ptr_to_usart_buffer = '\0';
            flag_rx_done = 1; // xfer from PC to MCU complete
        }
        else{
            *ptr_to_usart_buffer++ = tmp;
        }
    }
    
}



ISR(INT0_vect){

    // System External interrupt INT0 triggered  when a push button is
    // pressed that is connected to a pin on the atmega328p MCU.  This
    // will  put the  state  machine into  IDLE_MODE disabling  system
    // operation.   INT1 ISR  has been  configured to  be triggerd  on
    // falling edge, ie high to low, when the button is pressed.
    
     // crude debouncing // TODO: implement proper debounceing code.
    _delay_ms(10);
    if (!bitRead(PIND, pin_int0_interrupt))
        flag_system_stop = 1;

    //usart_send_string("\ndbg: INT0_vect(): in ISR\n");
}



ISR(INT1_vect){

    // System External interrupt INT1 triggered  when a push button is
    // pressed that is connected to a pin on the atmega328p MCU.  This
    // will  put the  state machine  into action.   INT1 ISR  has been
    // configured to be triggerd on falling edge, ie high to low, when
    // the button is pressed.
    
     // crude debouncing // TODO: implement proper debounceing code.
    _delay_ms(10);
    if (!bitRead(PIND, pin_int1_interrupt))
        flag_system_start = 1;

    //usart_send_string("\ndbg: INT1_vect(): in ISR\n");
}


// Timer2 overflow, which is really compare match A overflows in CTC mode.
// ISR(TIMER2_COMPA_vect)
// {
//     num_timer2_overflows++;
//     //num_timer2_compare_matches++;
// }
ISR(TIMER2_COMPA_vect)
{
    system_time_ms++;
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



// void config_tc2(void)
// {
    
//     // Configure Timer/Counter2 in CTC mode.

//     // Mode: CTC, TOP = OCR2A
    
//     // Stop Timer 2 while we configure it for safety.
//     stop_tc2;

//     // Normal port operation, CTC mode selected using WGM21 = 1
//     TCCR2A = 0;
//     TCCR2B = 0;

//     // CTC mode: WGM22:0 = 010
//     // WGM22 is in TCCR2B, WGM21 and WGM20 are in TCCR2A
//     TCCR2A |= (1 << WGM21);

//     // Set TOP value for CTC mode
//     OCR2A = 255;

//     // Start counter from 0
//     TCNT2 = 0;

//     // Clear pending compare-match A flag
//     TIFR2 = (1 << OCF2A);

//     period_of_tick_tc2 = 1.0f/(float)(F_CPU/prescalerTC2);
    
//     // timer2_overflow_time_us=(total_number_of_ticks * period_of_tick) * 1.0e6
//     //
//     // In our fast PWM TOP mode the timer/counter counts as such:
//     //
//     // 0 -> 1 -> 2 -> ... -> TOP -> 0 -> 1 -> ...,
//     //
//     // Note that the the reset to 0 is part of the cycle, resulting in
//     // TOP+1 transitions for the period.
//     //
//     timer2_overflow_time_us = ((float)OCR2A + 1.0f) * period_of_tick_tc2 * 1.0e6;
     
//     // Disable compare-match interrupt initially
//     disable_tc2_interrupt;

//     // Start Timer2
//     start_tc2;
// }
void config_tc2(void)
{
    /*
     * Configure Timer/Counter2 as a 1 ms system tick timer.
     *
     * Mode: CTC
     * TOP:  OCR2A
     *
     * F_CPU = 16 MHz
     * Prescaler = 64
     *
     * Timer clock frequency(f) = 16 MHz / 64 = 250 kHz
     * Timer tick(period of one tick=(1/f))  = 4 us
     *
     * For 1 ms CTC period:
     *
     * 1 ms(Toverflow) = (OCR2A + 1) × 4 µs
     * OCR2A + 1 = 1000 / 4
     * OCR2A + 1 = 250
     * OCR2A = 249
     * OCR2A = 249 because the counter counts from 0 to OCR2A
     */

    // Stop Timer2 while configuring it.
    // Clear CS22:CS20 bits.
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));

    // Clear Timer2 control registers.
    TCCR2A = 0;
    TCCR2B = 0;

    // CTC mode: WGM22:0 = 010
    // WGM21 is in TCCR2A.
    TCCR2A |= (1 << WGM21);

    
    // Set compare  value for  1 ms interrupt.   That is,  CTC compare
    // match every 1ms.
    //
    // See subject notes for CTC mode, where T_tick=period of one tick
    // Toverflow = (OCR2A+1) * T_tick
    // Toverflow/T_tick = (OCR2A+1)
    //
    // Remember from above Toverflow=1ms is required:
    // OCR2A = ((Toverflow/T_tick) - 1)
    // OCR2A = (0.001s / (prescaler/F_CPU)) - 1
    // OCR2A = ((1/1000s * F_CPU) / prescaler) - 1
    OCR2A = (uint8_t)((F_CPU / prescalerTC2 / 1000UL) - 1);

    // Start counting from 0.
    TCNT2 = 0;

    // Clear any pending Timer2 compare match A flag.
    // Interrupt flags are cleared by writing a 1 to them.
    TIFR2 = (1 << OCF2A);

    // Enable Timer2 compare match A interrupt.
    TIMSK2 |= (1 << OCIE2A);

    // Start Timer2 with prescaler = 64.
    // For Timer2: CS22:CS21:CS20 = 100 gives prescaler /64.
    TCCR2B |= (1 << CS22);
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



void config_tc0(void){

    // Drive LED continuously with Phase Correct PWM TOP Mode.
    

    // In our phase correct PWM TOP mode we need to * 2 as the counter
    // overflows after it counts from BOTTOM to TOP and back downwards
    // to BOTTOM again, that is:
    //
    // 0 -> 1 -> 2 -> ... -> TOP -> ... -> 2 -> 1 ->  0
    

    // Timer0 Phase Correct PWM, TOP = OCR0A  (Mode 5)
    //
    // Set waveform generation mode: WGM02:0 = 101
    //
    // We want PWM output on OC0B in non-inverting mode. OCR0A will be
    // used for myTOP,  or period of PWM frequency and  OCROB for duty
    // cycle.
    //
    // We  also   require  clear  OC0B   on  compare  match   when  up
    // counting. Set OCOB on compare match when counting down.
    //
    // Prescaler need to be set to a value for required Fpwm such that
    // TOP   doesn't    exceed   its   range.     Prescaler=64.    See
    // notes:26/4/2026 pg2.
    //
    // Also, setting the  clock select bits, ie  setting the prescaler
    // means the timers are started  here.
    
    TCCR0A = (1 << COM0B1) | (1 << WGM00);
    //TCCR0B = (1 << WGM02);
    TCCR0B = (1 << WGM02) | (1 << CS00) | (1 << CS01);


    // Setting TOP for phase correct PWM TOP Mode of TC0 to drive the
    // LED.
    //
    // TCO is an 8 bit counter so it's range is from 0-255
    //

    // OCROA  is  proportional  to  the Period/Frequency  of  the  PWM
    // signal(see equation for  Fpwm signal) The larger  the TOP value
    // is it means  the timer takes longer to complete  one PWM cycle,
    // so  the frequency  is lower.   Conversly, the  smaller the  TOP
    // means the timer  completes one PWM faster, so  the frequency is
    // higher.
    //
    OCR0A = 255; // Maximum for this 8 bit register

    // This sets the Duty Cycle of the PWM signal on OCOB/PD5, to 90%.
    OCR0B = 229;

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
    
    // Enable pin for INT1 as an input.  Then enable the pullup
    // resistor which sets the port pin HIGH.
    bitClear(DDRD, pin_int1_interrupt);
    bitSet(PORTD, pin_int1_interrupt);

    // Enable pin for INT0 as an input.  Then enable the pullup
    // resistor which sets the port pin HIGH.
    bitClear(DDRD, pin_int0_interrupt);
    bitSet(PORTD, pin_int0_interrupt);

    bitSet(DDRD, pin_led_pwm);    // LED pin configure as an output
    bitClear(PORTD, pin_led_pwm); // LED off to start off with
    
    // Connect Analog Comparator Output to Timer1 Input Capture.
    bitSet(ACSR, ACIC);

    // Potentiometer ADC input pin: ADC0 = A0 = PC0
    bitClear(DDRC, PC0);     // A0/PC0 as input
    bitClear(PORTC, PC0);    // no internal pull-up
    


    usart_init(9600);


    // Config TC0 in Phase Correct PWM for LED
    config_tc0();
    led_pwm_off(); // Turn LED off initially
    
    // Configure  TC1,  Timer/Counter 2  for  AC  + IC  of
    // HC-SR04 echo signal.  And Fast PWM Mode for driving
    // the servo
    //
    config_tc1(); // configure TC1, Timer/Counter 1

    // Configure TC2  as a simple  counter in CTC  mode to
    // generating the  11us trigger pulse for  the HCR-S04
    // ultrasonic sensor.
    //
    config_tc2(); // configure TC2, Timer/Counter 2

    // Configure  the external  interrupts INT1  and INT2.
    // When  INTI is  pressed the  control system  starts,
    // when INI0 is pressed the system stops.
    //
    interrupt_init();

    // Initialise the ADC0 register mapped to OLED contrast.
    adc_init();
                
    sei(); // Enable global interrupts

    // Initalise U8g2  ssd1309 arduino librarys to  just configure I2C
    // not the whole  of init() as this interfers  with my TC0/TC1/TC2
    // configurations.
    
    oled_init();
    //oled_contrast_test();
        
    // ADSC sets  the ADC to  start conversions.  Setting this  bit is
    // the  "software trigger"  that  tells the  hardware to  actually
    // begin the  electrical process  of sampling  the voltage  on the
    // pin.   As soon  as  the conversion  is  finished, the  hardware
    // automatically  clears  it  to  0.   This  is  known  as  single
    // conversion mode  as the  conversion occurs  only once  for each
    // time ADSC is set.
    bitSet(ADCSRA, ADSC);
    my_delay_us(10000UL);
    
    //usart_send_string("dbg: in INIT_FINISHED\n");

    //usart_clear_print();
    usart_send_string_flash("REBOOT!!!\n");
    usart_flush();
    print_usart_debugging_mode_menu();
    
    state_current = IDLE_MODE;


    while (1){

        usart_debugging();

        uint16_t adc_raw;
        uint8_t oled_contrast;

        // So ISR  does not corrupt  the current adc value  we disable
        // interrupts while accessing the ISR set adc value.
        cli();
        adc_raw = adc_cur;
        sei();

        oled_contrast =  get_adc_to_oled_contrast(adc_raw);
        oled_set_contrast(oled_contrast);
        

        if (usart_debugging_mode_adc){
            usart_send_string_flash(">adc_raw:");
            usart_send_num(adc_raw, 4, 0);
            usart_send_string_flash("\n"); 

            usart_send_string_flash(">oled_contrast:");
            usart_send_num(oled_contrast, 3, 0);
            usart_send_string_flash("\n");
        }
        
        // Setup  state  machine  mode  transitions.
        if (flag_system_start){

            // usart_send_string("\ndbg: flag_system_start\n");
            // usart_send_num(flag_system_start, 1, 0);
            // usart_send_string("\n");
            
            flag_system_start = 0;

            led_pwm_on();
            
            state_current = SERVO_MODE;
        }


        if (flag_system_stop){

            // usart_send_string("\ndbg: flag_system_start\n");
            // usart_send_num(flag_system_start, 1, 0);
            // usart_send_string("\n");
            
            flag_system_stop = 0;

            led_pwm_off();

            oled_clear();
    
            state_current = IDLE_MODE;
        }

        
        switch (state_current){


            case IDLE_MODE: {

                //usart_send_string("dbg: in IDLE_MODE\n");
                my_delay_us(1000000UL);

                break;
            }
            case SERVO_MODE: {

                //usart_send_string("dbg: in SERVO_MODE\n");

                cur_radar_angle = drive_servo();
                state_current = SONAR_MODE;

                break;
            }
            case SONAR_MODE: {

                //usart_send_string("dbg: in SONAR_MODE\n");

                cur_radar_distance_to_object_cm = sonar();
                bool object_detected = (cur_radar_distance_to_object_cm >= 0.0f);
                
                if (!object_detected) {
                    cur_radar_distance_to_object_cm = 0.0f;
                }

                // Update ssd1309 OLED
                radar_display_update(cur_radar_angle, cur_radar_distance_to_object_cm,
                                     object_detected);
                
                state_current = SERVO_MODE;
                break;
            }
            default:
                break;
        }


        
    } // end: while(1)

        
    
    return 0;
}



///////////////////////////////////////////////////////////////////////////////
//                           User defined functions                          //
///////////////////////////////////////////////////////////////////////////////
   
uint8_t get_adc_to_oled_contrast(uint16_t adc_raw){

    // Single conversion mode is active, so conversion only occurs
    // once  everytime ADSC  is  set.  Get  the  current value  of
    // the ADC via ISR 
    bitSet(ADCSRA, ADSC);


    // Linear Mapping of current ADC reading to OLED contrast.
    uint8_t oled_contrast = linear_mapping(adc_raw, ADC_MIN, ADC_MAX,
                                      OLED_CONTRAST_MIN, OLED_CONTRAST_MAX);

    if (oled_contrast < 0) oled_contrast = 0;
    if (oled_contrast > 255) oled_contrast = 255;

        
    return oled_contrast; // contrast value for ssd1309 is from 0->255
}



void led_pwm_on(void){

    // Connect OC0B/PD5 to Timer0 PWM output
    bitSet(TCCR0A, COM0B1);
    bitClear(TCCR0A, COM0B0);
}



void led_pwm_off(void)
{
    // Disconnect OC0B/PD5 from Timer0 PWM hardware
    bitClear(TCCR0A, COM0B1);
    bitClear(TCCR0A, COM0B0);

    // Now PORTD controls PD5 again, so force LED off
    bitClear(PORTD, pin_led_pwm);
}



// Debugging USART functions //////////////////////////////////////////////////

void usart_debugging(void){

    // Sets mode of debugging according to request from user PC serial
    // monitor.  Option for no debugging  also exists.  It can also be
    // used with vscode Teleplot.
    //
    // PC byte to MCU input reading  is into the USART RX register and
    // is non-blocking as it is under USART_RX ISR control.
    
    uint8_t user_choice; // User choice for debugging mode.
    
    // Nothing to do if we haven't recived an input from the PC serial
    // monitor yet, via the USART_RX interrupt service routine.
    if (!flag_rx_done)
        return;

    // Xfer from  PC to MCU is  complete and via ISR.   We can
    // now process that data.
        
    // Check if it's  exactly one digit 1–4 by  checking the ASCII
    // numbers.  Also,  reject non single character  entries, such
    // decimals and outside range ascii characters.
    if ( (usart_buffer[0]<'0')
         || (usart_buffer[0]>'4')
         || (usart_buffer[1]!='\0') ){
        usart_send_string_flash("Invalid selection. Try again\n");

        // Ignore this  byte and get  ready for the  next byte
        // from the RX buffer.
    }
    else{
        // Convert input string into an integer.
        //
        // note: atoi() expects a null - terminated string.
        user_choice = atoi(usart_buffer);
        // usart_send_string("dbg: main(): User_choice is: ");
        // usart_send_num(user_choice, 1, 0);
        // usart_send_byte('\n'); // send EOL, CRLF
        // Set debugging mode flags.
        set_user_required_usart_debugging_mode(user_choice);
    }
    
        
    // Get the next byte from the RX buffer.
    ptr_to_usart_buffer = usart_buffer;
    flag_rx_done = 0;
    usart_flush();
        
    print_usart_debugging_mode_menu();

    return;
}


void print_usart_debugging_mode_menu(void){

    // Output the menu for debugging operations.
    usart_send_byte('\n');
    usart_send_string_flash("\nEnter a number for usart debugging required:\n");
    usart_send_string_flash("0. Switch off debugging mode, no data from MCU to PC.\n");
    usart_send_string_flash("1. Send sonar object distance measurments to PC.\n");
    usart_send_string_flash("2. Send sonar angle to PC.\n");
    usart_send_string_flash("3. Both data in mode 1 and 2 sent to PC.\n");
    usart_send_string_flash("4. ADC values sent from MCU to PC.\n");
}


void set_user_required_usart_debugging_mode(int8_t user_choice){

    // Set program state for debugging operations.

    switch (user_choice) {
        case 0: {
            usart_send_string_flash("\nDebugging mode has been turned off.\n");
            usart_debugging_mode_object_distance = 0;
            usart_debugging_mode_angle = 0;
            usart_debugging_mode_adc = 0;
            break;
        }
        case 1: {
            usart_send_string_flash("\nSending sonar object distance measurments");
            usart_send_string_flash(" to PC via Serial Monitor\n");
            usart_debugging_mode_object_distance = 1;
            break;
        }
        case 2: {
            usart_send_string_flash("\nSending sonar angle measurements");
            usart_send_string_flash(" to PC via Serial Monitor\n");
            usart_debugging_mode_angle = 1;
            usart_debugging_mode_object_distance = 0;
            break;
        }
        case 3: {
            usart_send_string_flash("\nSending both data measurements from mode 1 and 2");
            usart_send_string_flash(" to PC via Serial Monitor\n");
            usart_debugging_mode_object_distance = 1;
            usart_debugging_mode_angle = 1;
            break;
        }
        case 4: {
            usart_send_string_flash("\nSending ADC values to PC via Teleplot\n");
            usart_debugging_mode_adc = 1;
            break;
        }   
        default:
            break;
    }

}



float drive_servo(void)
{

    // Drive the servo  all the way forward and back  again wrt angle,
    // then repeat.

    // T of tick, Ttick = 1/F = 1/(F_CPU/prescaler) = prescaler/F_CPU 
    
    // With Ttick=0.5, period of one tick:
    //
    //  0.5 ms ~= 1000 ticks
    //  1.0 ms ~= 2000    
    //  1.5 ms ~= 3000 ticks
    //  2.0 ms ~= 4000
    //  2.5 ms ~= 5000 ticks
    //
    // OCR1B controls the pulse width on OC1B/PB2/D10.


    
    // Debugging:
    //
    // Sweep from 0 degrees to 180 degrees
    // for (unsigned int pulse_width_ticks=1000; pulse_width_ticks<=5000;
    //      pulse_width_ticks+=200){
        
    //     OCR1B = pulse_width_ticks;
    //     my_delay_us(100000UL); // let the servo settle
    // }

    // my_delay_us(1000000UL);

    // // Sweep from 180 degrees to 0 degrees
    // for (unsigned int pulse_width_ticks=5000; pulse_width_ticks>=1000;
    //      pulse_width_ticks-=200){

    //     OCR1B = pulse_width_ticks;
    //     my_delay_us(100000UL);
    // }
    // my_delay_us(1000000UL);

    // Lower limit, 1ms pulse measured on the oscilloscope.
    // OCR1B = 2000;
    // Centre, 1.5ms pulse
    //OCR1B = 3000;
    // Upper limit, 2ms pulse measured on the oscilloscope.
    //OCR1B = 4000;

    //usart_send_string("dbg: in drive_servo()\n");
    
    static unsigned char angle = SERVO_MIN_ANGLE;
    static unsigned char moving_forward = 1;
    unsigned char angle_step = 1;

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

    if (usart_debugging_mode_angle){
     usart_send_string_flash("Current Angle:");
     usart_send_num(angle, 5, 2);
        usart_send_string_flash("\n");
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

    my_delay_us(200000UL); // let the servo settle at it's new angle

    return angle;
}



float sonar(void){

    // Measure HCR-S04  ultrasonic sensor's length of  the echo signal
    // going  from  rising edge  to  falling  edge.  Using  this  then
    // measure and display, in the serial monitor, the distance to the
    // object.
    
    // Debugging my_delay_us() to use in HC-SR04 trigger pulse
    // while (1) {
    //     bitC lear(PORTC, pin_trigger);
    //     my_delay_us(10000UL);
    //     bitSet(PORTC, pin_trigger);
    //     my_delay_us(60000UL);       
    // }


    //usart_send_string("dbg: in sonar()\n");

    // Trigger the sonar
    bitClear(PORTC, pin_trigger);
    my_delay_us(2UL);
    bitSet(PORTC, pin_trigger);
    my_delay_us(11UL);
    bitClear(PORTC, pin_trigger);


    // Wait for falling edge capture, with a timeout
    unsigned long timeout = 30000;
    while (!echo_signal_high_detected && timeout > 0) {
        my_delay_us(1);
        timeout--;
    }

    float distance_to_object_cm = -1.0f;  // -1 means no valid echo
    float tHigh_copy; // Echo signal high time from AC+TC.
    float tLow_copy;  // Echo signal low time from AC+TC.
    float distance_to_object_m; // distance to object in metres
    float time_echo_signal_is_high;
    // Velocity of sound about 343m/s at 25degreesC.
    float velocity_of_sound = 343;

    if (echo_signal_high_detected){

        // We  disable interrupts  here as  we don't  want the  Timer1
        // Input Capture ISR to change them while we are using them.
        // //
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
        distance_to_object_m = ((time_echo_signal_is_high * velocity_of_sound) / 2);
        distance_to_object_cm = distance_to_object_m * 100.0f;  // convert to cm


        // Debugging, send values to serial monitor
        // usart_send_string("Echo Signal High=");
        // usart_send_num((tHigh_copy*1000000), 8, 2);
        // usart_send_string("us\n");


        // Print debugging info to serial monitor
        if (usart_debugging_mode_object_distance){
            
            usart_send_string_flash("Distance to Object:");
            usart_send_num((distance_to_object_cm), 6, 3);
            usart_send_string_flash("cm\n");
            
        //my_delay_ms(100000UL);
        }

    }
    else{
        if (usart_debugging_mode_object_distance){
            usart_send_string_flash("No echo detected\n");
        }
    }

    // Wait about 60 ms for hardware to reset before next sonar ping
    my_delay_us(60000UL);
    
    // return distance in cm for OLED and serial monitor display
    return (distance_to_object_cm);
}



// Original routine following Kai's demo codes.
//
// void my_delay_us(unsigned long x){

//     // Delays, determined with a hardware timer/counter for x us.

//     // This  version  of  my_delay_us()  is limited  by  the  size  of
//     // unsigned long. Hence x_max=2^32-1=4,294,967,295 us=4,294.967295
//     // s=71.5828 minutes=1.193hours
    
    
//     // NOTE: that a float assigned  to an integer returns the interger
//     // value with no decimal pont values.
        
//     // Number of complete Timer2 overflow periods needed.
//     unsigned long num_complete_overflows = ( (float)x / timer2_overflow_time_us );

//     // Remaining time after complete overflows, in microseconds.
//     float remainder_time_us =
//         (  (float)x - ((float)num_complete_overflows*timer2_overflow_time_us) );

//     // Convert remaining time into Timer2 ticks.
//     unsigned char remainder_time_ticks =
//         ( remainder_time_us / (period_of_tick_tc2 * 1.0e6) );

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
void my_delay_us(uint16_t delay_us)
{

    // Blocking microsecond delay using Timer2's current counter value.
    // 
    // IMPORTANT: This function  does NOT create a true  1 us hardware
    // timer tick.  It just determines the closest multiple of 4us the
    // for the delay_us required, thus  the resolution of this routine
    // is 4us,  4us being Ttick, ie  period of one timer  tick.  Given
    // prescaler is 64 and F_CPU=16Mkz.
    //
    // it F = F_CPU/prescaler
    // Ttick = prescaler/ F_CPU = 64/16x10^6 = 4us
    //
    // As stated earlier That means  the requested delay is rounded UP
    // to the next available 4 us Timer2 tick.
    //
    //
    // Also, this function should still  only be used for short delays
    // as it still is a busy wait algorithim.  For long delays, us the
    // 1  ms system  tick as  waiting  1ms for  the ISR  to occur  and
    // comparing TCNT2 values  is not a busy wait delay,  as you would
    // be  just  asking  has  enough  time  passed  yet  as  the  only
    // calculation.  For  longer delays, interrupts or  other code can
    // cause the  loop to miss  Timer2 count  changes. If it  misses a
    // change, the function does not know  it missed one; it only sees
    // that the value is different.   So long delays may become longer
    // than expected.

    // If the requested delay is 0 us, do nothing.
    if (delay_us == 0){
        return;
    }


    // Therefore,  as calculated  earlier Ttick_us  = Ttick*1000000UL.
    // However we write it this way due to integer division truncating
    // the decimal point so we do the multiplication first.
    // 
    // ie      Ttick_us=1/F=0.000004*10^6=(prescaler/F_CPU)*1000000UL,
    // integer division  would happen first and  would make 0.000004=0
    // thus Ttick = 0*1000000UL.
    //
    // So we write it like this, to avoid this problem:
    const uint8_t TC2_TICK_US = (uint8_t)((prescalerTC2 * 1000000UL) / F_CPU);


    // Convert the requested microsecond delay into Timer2 counter ticks.
    //
    // Because Timer2 only advances once every 4 us, we must round UP.
    //
    // Mathematical trick for rounding up is used here.
    //
    // The formula:
    //
    // required_ticks = (delay_us + TC2_TICK_US - 1) / TC2_TICK_US
    //
    // Examples, remember  interger division drops the  decimal point
    // without any rounding:
    //
    //       my_delay_us(1)   -> waits about 4 us
    //       my_delay_us(4)   -> waits about 4 us
    //       my_delay_us(5)   -> waits about 8 us
    //       my_delay_us(8)   -> waits about 8 us
    //       my_delay_us(11)  -> waits about 12 us
    //       my_delay_us(12)  -> waits about 12 us
    uint16_t required_ticks = (delay_us + TC2_TICK_US - 1) / TC2_TICK_US;

    
    // This variable  counts how  many Timer2 counter  increments have
    // passed.   It  is  uint16_t  rather than  uint8_t  so  that  the
    // function can  delay for more  than 255 Timer2 ticks  if needed.
    //
    uint16_t ticks_done = 0;

    // Read  the  current value  of  the  Timer2 counter.   Timer2  is
    // already running because config_tc2() started it.
    uint8_t previous_count = TCNT2;


    // Wait until enough Timer2  counter increments have occurred(one
    // counter tick is about 4us with the current prescaler chosen).
    while (ticks_done < required_ticks)
    {
        uint8_t current_count = TCNT2;

        // If TCNT2 has changed since the previous read, then at least
        // one Timer2 tick has passed.
        // 
        // If an interrupt happens during this function, the delay may
        // become slightly  longer, but that  is okay for  the HC-SR04
        // trigger pulse  because the pulse  needs to be at  least the
        // required length.
        // 
        if (current_count != previous_count)
        {
            previous_count = current_count;
            ticks_done++;
        }
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



void adc_init(void){

    // ADMUX = ADC Multiplexer Selection Register
    //
    ADMUX = 0;              // Clear all bits first (good practice)
    bitSet(ADMUX, REFS0);   // REFS0 = 1 => Vref = AVcc (5V)

    // By default ADC is used:
    // MUX3..0 = 0000 => Select ADC0, which is A0 / PC0
    // No MUX bits need to be set for ADC0
    //bitSet(ADMUX, MUX0);    // MUX0 = 1 => Select ADC1 (A1/PC1) as input channel


    // DIDR0 = Digital Input Disable Register
    //
    // ADC0D = 1, Disable digital input buffer on ADC0 pin
    //
    // We do this to reduce power consumption; electrical noise on the
    // analog pin; and improves ADC accuracy.
    bitSet(DIDR0, ADC0D);


    // ADCSRA = ADC Control and Status Register A
    //
    ADCSRA = 0;   // Clear register first

    // We need to decrease the CPU clock for the ADC hardware to work
    // properly.  We use a prescaler to do this.  Prescaler set to
    // 128.
    
    // ADC clock = 16  MHz / 128 = 125 kHz.   The recommended range of
    // ADC  operation  is  50-200Hz,   to  give  stable  and  accurate
    // readings.
    bitSet(ADCSRA, ADPS2);
    bitSet(ADCSRA, ADPS1);
    bitSet(ADCSRA, ADPS0);


    // ADIE = ADC Interrupt Enable.
    //
    // When   conversion  is   complete  the   ISR(ADC_vect)  executes
    // automatically.  It allows non-blocking ADC reading.
    bitSet(ADCSRA, ADIE);

    
    // ADEN = ADC Enable
    bitSet(ADCSRA, ADEN);
}



void interrupt_init(void){

    // INIT1 external interrupt configuration:
    //
    // Clear INT1 sense bits first
    EICRA &= ~((1 << ISC11) | (1 << ISC10));

    // Triggered on Rising edge on INT1
    //EICRA |= (1 << ISC11) | (1 << ISC10);
    
    // INT1 ISR triggerd on falling edge, ie high(pullup configured)
    // to low, when the button is pressed.
    EICRA |= (1 << ISC11);

    // Clear any pending INT1 flag
    EIFR |= (1 << INTF1);

    // Enable external interrupt INT1
    EIMSK |= (1 << INT1);


    // Port C's PCINT10  MCU pin used so enable  PCIE1_vect pin change
    // interrupt.   Remember there  is only  one pin  change interrupt
    // allowed per port  on the atmega328p MCU.  Also  it will trigger
    // on the falling high->low and  rising low->high of a push button
    // press.
    //
    // PCICR |= (1 << PCIE1); // Set Pin Change Inerrupt Control Register.

    // PCMSK1 |= (1 << PCINT10); //Set Pin Change Mask Register 1

    
    // INIT1 external interrupt configuration:
    //
    // Clear INT0 sense control bits first
    EICRA &= ~((1 << ISC01) | (1 << ISC00));

    // INT1 ISR triggerd on falling edge, ie high(pullup configured)
    // to low, when the button is pressed.
    // ISC01 = 1, ISC00 = 0
    EICRA |= (1 << ISC01);

    // Clear any pending INT0 interrupt flag
    EIFR |= (1 << INTF0);

    // Enable external interrupt INT0
    EIMSK |= (1 << INT0);


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



void usart_send_string_flash_real(const char *flash_str){

    // AVR method to store strings  in flash instead of SRAM.  Arduino
    // has 2kB of SRAM and the screen takes up 1kB on its own.
    
    char c;

    while ((c = pgm_read_byte(flash_str++)) != '\0')
    {
        usart_send_byte(c);
    } 
}

// check if const is really needed, c++ compiler is strictor than c compiler.
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



///////////////////////////////////////////////////////////////////////////////
//                        OLED ssd1309 radar display functions               //
///////////////////////////////////////////////////////////////////////////////

float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


// Angle convention:
//
// 0 degrees   = right side
// 90 degrees  = straight up
// 180 degrees = left side
void radar_point_from_radius(float angle_deg, float radius_pixels, int *x, int *y)
{
    float angle_rad = angle_deg * PI_FLOAT / 180.0f;

    *x = RADAR_ORIGIN_X + (int)(radius_pixels * cos(angle_rad));
    *y = RADAR_ORIGIN_Y - (int)(radius_pixels * sin(angle_rad));
}


void radar_point_from_distance(float angle_deg, float distance_cm, int *x, int *y)
{
    distance_cm = clamp_float(distance_cm, 0.0f, RADAR_MAX_DISTANCE_CM);

    float radius_pixels =
        (distance_cm / RADAR_MAX_DISTANCE_CM) * RADAR_RADIUS_PIXELS;

    radar_point_from_radius(angle_deg, radius_pixels, x, y);
}


void draw_radar_arc(int radius_pixels)
{
    int prev_x;
    int prev_y;

    radar_point_from_radius(0.0f, radius_pixels, &prev_x, &prev_y);

    for (int angle = 2; angle <= 180; angle += 2)
    {
        int x;
        int y;

        radar_point_from_radius((float)angle, radius_pixels, &x, &y);

        u8g2.drawLine(prev_x, prev_y, x, y);

        prev_x = x;
        prev_y = y;
    }
}


void draw_angle_label(int angle_deg)
{
    char text[5];

    snprintf(text, sizeof(text), "%d", angle_deg);

    int x;
    int y;

    // Put labels slightly inside the outer arc.
    radar_point_from_radius(
        (float)angle_deg,
        // orig: angles inside radar grid RADAR_RADIUS_PIXELS - 8,
        RADAR_RADIUS_PIXELS + 6,
        &x,
        &y
    );

    int text_width = u8g2.getStrWidth(text);

    x = x - (text_width / 2);

    // Small corrections so labels do not hit the screen edge.
    if (angle_deg == 0)
    {
        x -= 1;
        y -= 3;
    }
    else if (angle_deg == 180)
    {
        x += 1;
        y -= 3;
    }
    else if (angle_deg == 90)
    {
        y += 7;
    }
    else
    {
        y += 3;
    }

    u8g2.drawStr(x, y, text);
}


void draw_radar_grid(void)
{
    // Baseline
    u8g2.drawHLine(
        RADAR_ORIGIN_X - RADAR_RADIUS_PIXELS,
        RADAR_ORIGIN_Y,
        RADAR_RADIUS_PIXELS * 2
    );

    // Distance arcs
    draw_radar_arc(RADAR_RADIUS_PIXELS / 3);
    draw_radar_arc((RADAR_RADIUS_PIXELS * 2) / 3);
    draw_radar_arc(RADAR_RADIUS_PIXELS);

    u8g2.setFont(u8g2_font_4x6_tr);

    // Angle sweep/grid lines every 30 degrees
    for (int angle = 0; angle <= 180; angle += 30)
    {
        int x;
        int y;

        radar_point_from_radius(
            (float)angle,
            RADAR_RADIUS_PIXELS,
            &x,
            &y
        );

        u8g2.drawLine(
            RADAR_ORIGIN_X,
            RADAR_ORIGIN_Y,
            x,
            y
        );

        draw_angle_label(angle);
    }

    // Centre point
    u8g2.drawDisc(RADAR_ORIGIN_X, RADAR_ORIGIN_Y, 2);
}



void draw_radar_text(float angle_deg, float distance_cm, bool object_detected)
{
    char text[24];

    u8g2.setFont(u8g2_font_5x7_tr);

    // Left side: Angle:180 plus manually drawn degree symbol
    snprintf(text, sizeof(text), "Angle:%3d", (int)angle_deg);
    u8g2.drawStr(0, 7, text);

    // Draw degree symbol manually as a tiny circle.
    // This avoids UTF-8/font problems.
    int degree_x = u8g2.getStrWidth(text) + 2;
    u8g2.drawCircle(degree_x, 2, 1);

    // Right side: Range value
    if (object_detected == true)
    {
        snprintf(text, sizeof(text), "Range:%3dcm", (int)distance_cm);
    }
    else
    {
        snprintf(text, sizeof(text), "Range:---");
    }

    u8g2.drawStr(65, 7, text);
}



void draw_radar_sweep(float angle_deg, float distance_cm, bool object_detected)
{
    int sweep_x;
    int sweep_y;

    radar_point_from_radius(
        angle_deg,
        RADAR_RADIUS_PIXELS,
        &sweep_x,
        &sweep_y
    );

    // Current sweep line
    u8g2.drawLine(
        RADAR_ORIGIN_X,
        RADAR_ORIGIN_Y,
        sweep_x,
        sweep_y
    );

    if (object_detected == true)
    {
        int object_x;
        int object_y;

        radar_point_from_distance(
            angle_deg,
            distance_cm,
            &object_x,
            &object_y
        );

        // Filled dot
        u8g2.drawDisc(object_x, object_y, 2);

        // Outer ring
        u8g2.drawCircle(object_x, object_y, 4);
    }
}


void radar_display_update(float angle_deg, float distance_cm, bool object_detected)
{
    angle_deg = clamp_float(angle_deg, 0.0f, 180.0f);

    u8g2.firstPage();

    do
    {
        draw_radar_grid();
        draw_radar_sweep(angle_deg, distance_cm, object_detected);
        draw_radar_text(angle_deg, distance_cm, object_detected);

    } while (u8g2.nextPage());
}


void oled_clear(void)
{
    u8g2.firstPage();

    do
    {
        // Draw nothing. This clears the screen.
    } while (u8g2.nextPage());
}


// ------------------------------------------------------------
// OLED init function
// Call this once from your setup/init code
// ------------------------------------------------------------
void oled_init(void)
{
    Wire.begin();
    Wire.setClock(100000UL);

    // U8g2 expects the 8-bit I2C address.
    // 7-bit address 0x3C becomes 0x78.
    u8g2.setI2CAddress(SCREEN_ADDRESS << 1);

    u8g2.begin();
    u8g2.enableUTF8Print();

    // Contrast range is usually 0 to 255.
    u8g2.setContrast(180);

    oled_clear();
}


void oled_set_contrast(uint8_t oled_contrast)
{
    u8g2.setContrast(oled_contrast);
}


void oled_contrast_test(void)
{
    while (1)
    {
        u8g2.setContrast(1);
        my_delay_us(2000000UL);

        u8g2.setContrast(255);
        my_delay_us(2000000UL);
    }
}
