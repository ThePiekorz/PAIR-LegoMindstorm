#!/usr/bin/env python3
from os import kill
from ev3dev.ev3 import *
from time import sleep

czujnik = ColorSensor()
czujnik.mode = 'COL-REFLECT'

# funkcja służy do kalibracji czujnika kolorów
def Calibrate():

    print('Poloz mnie na czarna tasme')
    input('Wait')
    czarny = czujnik.value()

    print('Poloz mnie na bialym')
    input('Wait')
    bialy = czujnik.value()

    return (bialy+czarny)/2


# print(Calibrate())
# cal = Calibrate()

cal = 20   #domyślna wartość kalibracyjna
alfa = 250
speed = 350
k = 12
ki = 0.4
kd = 5
l = LargeMotor('outA')
r = LargeMotor('outB')

print("Run forest")
time.sleep(10)

# speed = 50 alfa = 400 czas przejazdu = 45s
def dwustanowy():
    czujnik.mode = 'COL-COLOR'
    while czujnik.value() != 5:
        czujnik.mode = 'COL-REFLECT'
        e = cal - czujnik.value()  #odchył
        if e>0:
            l.run_forever(speed_sp=speed+alfa)
            r.run_forever(speed_sp=speed)
        else:
            l.run_forever(speed_sp=speed)
            r.run_forever(speed_sp=speed+alfa)
        czujnik.mode = 'COL-COLOR'

    l.stop()
    r.stop()
# speed = 275 k = 15 czas przejazdu = 43s
def proporcjonalny():
    czujnik.mode = 'COL-COLOR'
    while czujnik.value() != 5:
        czujnik.mode = 'COL-REFLECT'
        e = cal - czujnik.value()
        l.run_forever(speed_sp=speed + k * e)
        r.run_forever(speed_sp=speed - k * e)
        czujnik.mode = 'COL-COLOR'
    l.stop()
    r.stop()
# speed = 350 k = 12 ki = 0.4 kd = 5 czas przejazdu = 30s
def PID():
    czujnik.mode = 'COL-COLOR'
    stara = 0
    suma = 0
    while czujnik.value() != 5:
        czujnik.mode = 'COL-REFLECT'
        e = cal - czujnik.value()
        suma+=e
        korekta = k * e + kd * (e-stara) + ki * suma
        l.run_forever(speed_sp = speed + korekta)
        r.run_forever(speed_sp = speed - korekta)
        stara = e
        czujnik.mode = 'COL-COLOR'
    l.stop()
    r.stop()

# dwustanowy()
# proporcjonalny()
# PID()