from machine import Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff
import micropython
micropython.alloc_emergency_exception_buf(100)

LFWD = PWM(Pin(2))
RFWD = PWM(Pin(4))
RREV = PWM(Pin(5))
LREV = PWM(Pin(3))
freq = 1000
LFWD.freq(freq)
RFWD.freq(freq)
LREV.freq(freq)
RREV.freq(freq)

RSideInt = Pin(14, Pin.IN, Pin.PULL_UP)
LSideInt = Pin(15, Pin.IN, Pin.PULL_UP)

last_ldir = None
last_rdir = None
reverse_delay_ms = 300

overcurrentFlagRight = False
overcurrentFlagLeft = False
oc_count_r = 0
oc_count_l = 0
OC_THRESHOLD = 150  # must see fault for 150 consecutive 1ms ticks

def check_overcurrent():
    global overcurrentFlagRight, overcurrentFlagLeft
    global oc_count_r, oc_count_l
# 
#     if RSideInt.value() == 1:  # fault = pin pulled low
#         oc_count_r += 1
#         if oc_count_r >= OC_THRESHOLD:
#             overcurrentFlagRight = True
#     else:
#         oc_count_r = 0
#         overcurrentFlagRight = False

    if LSideInt.value() == 1:
        oc_count_l += 1
        if oc_count_l >= OC_THRESHOLD:
            overcurrentFlagLeft = True
    else:
        oc_count_l = 0
        overcurrentFlagLeft = False

def stop_all():
    LFWD.duty_u16(0)
    RFWD.duty_u16(0)
    LREV.duty_u16(0)
    RREV.duty_u16(0)

def move(speed, ldirection, rdirection):
    global last_ldir, last_rdir

    if (last_ldir is not None and ldirection != last_ldir) or \
       (last_rdir is not None and rdirection != last_rdir):
        stop_all()
        sleep_ms(reverse_delay_ms)

    if speed == 0:
        stop_all()
        return

    duty = int(speed * 65535)

    if overcurrentFlagRight:
        LFWD.duty_u16(0)
        LREV.duty_u16(0)
        print("Right Side Overcurrent")
    else:
        if ldirection:
            LFWD.duty_u16(duty)
        else:
            LREV.duty_u16(duty)

    if overcurrentFlagLeft:
        RFWD.duty_u16(0)
        RREV.duty_u16(0)
        print("Left Side Overcurrent")
    else:
        if rdirection:
            RFWD.duty_u16(duty)
        else:
            RREV.duty_u16(duty)

    last_ldir = ldirection
    last_rdir = rdirection

i = 0
did_forward = False

try:
    last_action = 0
    while True:
        now = ticks_ms()
        check_overcurrent()
        sleep_ms(1)

        if overcurrentFlagRight or overcurrentFlagLeft:
            stop_all()
            print("Overcurrent - stopped")
            continue

        if ticks_diff(now, last_action) > 500:
            last_action = now
            i += 1
            if i > 40:
                pass
            elif i > 20:
                if not did_forward:
                    move(0.5, True, True)
                    print("forward")
                    did_forward = True
            else:
                move(0, False, False)
                print("Time till Start:", i)

except KeyboardInterrupt:
    stop_all()