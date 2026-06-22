Started  with,  The  Introductory  Embedded  Systems  code,  that  was
submitted for marking, final-project-48434_submitted.

Intially  it was  converted from  using  the ssd1306  to ssd1309  OLED
screen, with some OLED variable display enhancements.

The most  significant change is the  main while loop now  remains more
responsive  and  is a  truer  embedded  design.  No  _delay_ms()  used
throught. These  were replaced  by, my_delay_ms(),  which continuously
reads TC2,  in Compare Match Mode,  creating a 1ms tick.   Then during
the state machine while loop it uses  the 1ms ISR time and the elapsed
time  to  create  a  condition for  another  state  machine  function.
my_delay_us() was replaced buy still  a blocking function but since it
is  only  a  small  delay  I am  deeming  it  ok.   my_delay_us()  has
resolution of  4us to keep the  embedded system sane.  It  just counts
required us delays as TC2 ticks as a multiple of 4us.
