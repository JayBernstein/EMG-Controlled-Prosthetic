from machine import Pin,UART,PWM
import time, rp2
led = Pin("LED", Pin.OUT)
extraLEDPin = Pin(2, Pin.OUT)
inpin = Pin(3, Pin.IN)
uart = UART(0,9600)
led.value(1)
pwm0 = PWM(Pin(4), freq=2000, duty_u16=32768)
pwm1 = PWM(Pin(5), freq=2000, duty_u16=32768)
pwm2 = PWM(Pin(6), freq=2000, duty_u16=32768)
pwm3 = PWM(Pin(7), freq=2000, duty_u16=32768)
pwm4 = PWM(Pin(8), freq=2000, duty_u16=32768)

while 1>0:
#     uart.write('a')
    if uart.any():
        commandin = uart.readline()
        print(commandin)
        if commandin in (b'\x0D', b'\x0A'):
            continue
        else:
#             command = commandin.replace('\r', '').replace('\n', '')
            command = int.from_bytes(commandin, "big")
            pwm0.duty_u16(command*255)      # set the duty cycle of channel A, range 0-65535
            pwm1.duty_u16(command*255)      # set the duty cycle of channel A, range 0-65535
            pwm2.duty_u16(command*255)      # set the duty cycle of channel A, range 0-65535
            pwm3.duty_u16(command*255)      # set the duty cycle of channel A, range 0-65535
            pwm4.duty_u16(command*255)      # set the duty cycle of channel A, range 0-65535
            print(pwm0.duty_u16())         # get the current duty cycle of channel A, range 0-65535
    
        