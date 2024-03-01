from gpiozero import LED,Button 
from time import sleep
import keyboard

#GPIO pin values
button1_pin = 1 # main button -- if button was used 
button2_pin = 2 # mode-switching button -- if button was used 
led1_pin = 3 #on-off LED
led2_pin = 4 #blinking freq LED

button1 = Button(button1_pin)
button2 = Button(button2_pin)
led1 = LED(led1_pin)
led2 = LED(led2_pin)

mode = True #mode for button 1

def slow_freq(led):
    led.value = 1
    sleep(1)
    led.value = 0
    sleep(1)
    
def med_freq(led):
    led.value = 1
    sleep(0.5)
    led.value = 0
    sleep(0.5)
    
def fast_freq(led):
    led.value = 1
    sleep(0.1)
    led.value = 0
    sleep(0.1)
    
def toggle_LED(led):
    if led.is_on():
        led.off()
    else :
        led.on() 
        
def led_freq_switch(led,freq):
    match freq:
        case 1:
            slow_freq(led)
        case 2:
            med_freq(led)
        case 3:
            fast_freq(led)


#doing the main loop
paused = False #for pausing blinking rotation
freq = 1 

while True:
        
    if button2.is_pressed() or keyboard.is_pressed('2'):
        mode = not mode
    
    if mode: #toggle LED1
        if button1.is_pressed() or keyboard.is_pressed('1'):
            toggle_LED(led1)    
    else:
        if button1.is_pressed() or keyboard.is_pressed('1'):
            paused = not paused
            
    #for auto blinking 
    if not paused:
        #auto blink
        if freq == 3:
            freq = 1
        led_freq_switch(led2,freq)
        sleep(3)
        freq += 1
    else:
        #freezes on current blink
        led_freq_switch(led2,freq)
        