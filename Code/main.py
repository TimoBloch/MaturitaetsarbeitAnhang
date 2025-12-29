from machine import Pin, PWM, I2C
import time
import math
from pca import PCA9685
from servo import Servos

# setup pwm output for test

ch1IN  = Pin(15, Pin.IN)   #setup pins for remote control channels
ch2IN  = Pin(14, Pin.IN)
ch3IN  = Pin(13, Pin.IN)
ch4IN  = Pin(12, Pin.IN)
ch5IN  = Pin(11, Pin.IN)
ch6IN  = Pin(10, Pin.IN)

sda = Pin(0)                    #setup I2C for servo control
scl = Pin(1)
id = 0
i2c = I2C(id=id, sda=sda, scl=scl)
pca = PCA9685(i2c=i2c)
servo = Servos(i2c=i2c)


pwm_max = 65535

mot1F = Pin(19, Pin.OUT)
mot1B = Pin(18, Pin.OUT)
mot1EN = PWM(Pin(17))
mot1EN.freq(1000)
mot1EN.duty_u16(int(pwm_max*0.5))

mot2F = Pin(21, Pin.OUT)
mot2B = Pin(20, Pin.OUT)
mot2EN = PWM(Pin(16))
mot2EN.freq(1000)
mot2EN.duty_u16(int(pwm_max*0.5))





def get_remote():             #get controller channel states and return them
    for k in range(2):
        while ch1IN.value() == 0:
          pass
        start = time.ticks_us()
        while ch1IN.value() == 1:
            pass
        end = time.ticks_us()
        ch1_duty = end - start
    
    for k in range(2):
        while ch2IN.value() == 0:
          pass
        start = time.ticks_us()
        while ch2IN.value() == 1:
            pass
        end = time.ticks_us()
        ch2_duty = end - start
    
    for k in range(2):
        while ch3IN.value() == 0:
          pass
        start = time.ticks_us()
        while ch3IN.value() == 1:
            pass
        end = time.ticks_us()
        ch3_duty = end - start
    
    for k in range(2):
        while ch4IN.value() == 0:
          pass
        start = time.ticks_us()
        while ch4IN.value() == 1:
            pass
        end = time.ticks_us()
        ch4_duty = end - start
    
    for k in range(2):
        while ch5IN.value() == 0:
          pass
        start = time.ticks_us()
        while ch5IN.value() == 1:
            pass
        end = time.ticks_us()
        ch5_duty = end - start
    
    for k in range(2):
        while ch6IN.value() == 0:
          pass
        start = time.ticks_us()
        while ch6IN.value() == 1:
            pass
        end = time.ticks_us()
        ch6_duty = end - start
    
    return ch1_duty, ch2_duty, ch3_duty, ch4_duty, ch5_duty, ch6_duty


servo1_deg = 90
servo2_deg = 90#set standard values for servos
servo4_deg = 90

armfrontX = 5
armfrontY = 0
while True:   #main function
    #print(mot1EN.duty_u16(), mot2EN.duty_u16())
    ch1_duty, ch2_duty, ch3_duty, ch4_duty, ch5_duty, ch6_duty = get_remote()      
    #print (str(ch1_duty),str(ch2_duty),str(ch3_duty),str(ch4_duty),str(ch5_duty),str(ch6_duty)
    
    precision = int((ch5_duty-1000)/334)+1
    
    if ch6_duty > 1700:
        mode = "arm"
    else:
        mode = "drive"
        
    if mode == "arm":
        mot1F.low()
        mot1B.low()
        mot2B.low()
        mot2F.low()
        
        servo1_change = (0.25/precision)*((ch1_duty-1500)/1000*180)
        if abs(servo1_change) >= 1:
            servo1_deg += servo1_change  #entfernt veränderungen durch flux
        
        armfrontY = (((ch3_duty-1000)/3000)*15)
        armfrontX += (ch2_duty/1500-1)
        
        if armfrontX <= 0:
            armfrontX = 0.1
        elif armfrontX > 17:
            armfrontX = 17
        
        dist_c = math.sqrt(armfrontY**2+ armfrontX**2)
        if dist_c == 0:
            dist_c = 0.1
        
        servo2_deg = (math.acos(dist_c**2/(20*dist_c))+math.atan(armfrontY/armfrontX))*90
        servo3_deg = 180-(math.acos((200-dist_c**2)/200)*90)
        print(ch3_duty)
    
        servo4_deg += (0.25/precision)*((ch4_duty-1500)/1000*180)
        
        
        if servo1_deg >180:
            servo1_deg = 180
        elif servo1_deg <0:
            servo1_deg = 0
            
        if servo2_deg >180:
            servo2_deg = 180
        elif servo2_deg <0:
            servo2_deg = 0
            
        if servo4_deg >180:
            servo4_deg = 180
        elif servo4_deg <0:
            servo4_deg = 0
        
        servo.position(index=0, degrees=servo1_deg)
        servo.position(index=1, degrees=servo2_deg)
        servo.position(index=2, degrees=servo3_deg)
        servo.position(index=3, degrees=servo4_deg)
        
        
        
        
        
    else:                    #driving mode
        max_speed = (ch3_duty-1000)/1000 #speed as; (0.xx)
        if max_speed > 1:
            max_speed = 1
        elif max_speed < 0:
            max_speed = 0
            
        speed = int(ch2_duty-1500)/500*max_speed
        print(speed)
        diroff = int(ch1_duty-1500)/500   #directional offset (right side positive)
        if speed >= 0.1:  #going forward   mot1 = R mot2 = L
            mot1F.high()
            mot1B.low()
            mot2F.high()
            mot2B.low()
            if diroff >0.1:
                Rduty = speed-(diroff*speed)
                
                if Rduty<0:
                    Rduty = 0
                elif Rduty > 1:
                    Rduty = 1
                    
                mot1EN.duty_u16(int(Rduty*pwm_max))
                mot2EN.duty_u16(int(speed*pwm_max))
                
            elif diroff <-0.1:
                Lduty = speed-(abs(diroff)*speed)
                if Lduty<0:
                    Lduty = 0
                elif Lduty > 1:
                    Lduty = 1
                    
                mot1EN.duty_u16(int(speed*pwm_max))
                mot2EN.duty_u16(int(Lduty*pwm_max))
            else:
                mot1EN.duty_u16(int(speed*pwm_max))
                mot2EN.duty_u16(int(speed*pwm_max))
            
        elif speed <= -0.1:#going back
            speed = abs(speed)
            mot1F.low()
            mot1B.high()
            mot2F.low
            mot2B.high()
            
            if diroff >0.1:
                Rduty = speed-(diroff*speed)
                print(Rduty)
                if Rduty>0:
                    Rduty = 0
                elif Rduty < -1:
                    Rduty = -1

                    
                mot1EN.duty_u16(int(abs(Rduty)*pwm_max))
                mot2EN.duty_u16(int(speed*pwm_max))
                
            elif diroff <-0.1:
                Lduty = speed-(abs(diroff)*speed)
                print(Lduty)
                if Lduty>0:
                    Lduty = 0
                elif Lduty<-1:
                    Lduty = -1
                    
                mot1EN.duty_u16(int(speed*pwm_max))
                mot2EN.duty_u16(int(abs(Lduty)*pwm_max))
            else:
                mot1EN.duty_u16(int(speed*pwm_max))
                mot2EN.duty_u16(int(speed*pwm_max))
        else:
            mot1F.low()
            mot1B.low()
            mot2B.low()
            mot2F.low()
            mot1EN.duty_u16(0)
            mot1EN.duty_u16(0)

            
        
            
            
            
            
        
    
    

    
    
    
    
    
        
    
    
    
    
    