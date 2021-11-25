## Config of .ioc

# LCD Eq4

i2c4 cm7 standrard mode 

Timer 13
  Clock Source: Internal Clock
  Wanted Freq. 1/200ms 
  Prescaler: 47999
  Counter period: 1000
  
 GPIO
  PB10, PB11, PE6, PE15 Output No pull-up no pull-down
  PE7, PE8, PE10, PE12 Pull-up

# RPM - PWM Eq4
Timer 5 - RPM
  Clock Source: Internal Clock
  Prescaler: 0
  Counter period: 4294967295
  Wanted Freq. Max
Timer 1 - PWM
  Clock Source: Internal Clock
  Prescaler: 239
  Counter period: 100
  Channel 2: PWM Generation CH2
GPIO
  PB7 - Ext Interrupt 7. No pull-up no pull-down, Rising edge
 
