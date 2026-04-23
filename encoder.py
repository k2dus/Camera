from machine import Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff
import micropython
import math
from encoder import Encoder
micropython.alloc_emergency_exception_buf(100)
#Initialize left and right motor encoders

left_encoder = Encoder(pin_a = 2, pin_b = 3)
right_encoder = Encoder(pin_a = 4, pin_b = 5)
dist_travelled = 0

while(1): 
    delta_L = left_encoder.delta()
    delta_R = right_encoder.delta()
    dist_travelled = ((delta_L + delta_R) / 2) + dist_travelled
    print("dist travelled(cm): ", (math.pi * 4 * (dist_travelled/1920)))
        