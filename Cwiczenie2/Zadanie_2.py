#!/usr/bin/env python3
from multiprocessing.spawn import old_main_modules
from os import kill
from unicodedata import name
from ev3dev.ev3 import *
import time
import math

class Robot():
    def __init__(self, promien=2.8, L=12, N=360):
        """Inicjalizacja robota"""
        self.l = LargeMotor('outD')
        self.r = LargeMotor('outA')
        self.promien = promien
        self.L = L
        self.N = N
        self.n_l_old = self.l.position
        self.n_r_old = self.r.position
        self.x = 0
        self.y = 0
        self.fi = 0
        print("Run Forest")
        time.sleep(8)

    def calc(self):
        """Funkcja służącą do ciągłego wykonywania obliczeń pozycji"""
        delta_n_R = self.l.position - self.n_l_old
        delta_n_L = self.r.position - self.n_r_old
        self.n_l_old = self.l.position
        self.n_r_old = self.r.position

        d_r = 2 * math.pi * self.promien * (delta_n_R/self.N)
        d_l = 2 * math.pi * self.promien * (delta_n_L/self.N)
        d_c = (d_r + d_l)/2

        self.x += d_c * math.cos(self.fi) #pełen model robota unicycle
        self.y += d_c * math.sin(self.fi)
        self.fi += ((d_r-d_l)/self.L)

    def angle_to_go(self, xg, yg):
        """Funkcja obliczająca kąt"""
        psi = math.atan2(yg - self.y, xg - self.x)
        return psi

    def go_to_goal(self,xg,yg,speed=400,k=250):
        """Funkcja odpowiedzialna za jazdę"""
        e = self.angle_to_go(xg,yg) - self.fi
        e = math.atan2(math.sin(e),math.cos(e))
        self.l.run_forever(speed_sp=speed+k*e)
        self.r.run_forever(speed_sp=speed-k*e)
   
    def check_range(self,xg,yg,drift=6):
        """Funkcja sprawdzająca położenie od pobranych
        punktów z uwzględnieniem dopuszczalnej granicy błędu(drift)
        """
        if ((xg-drift <= self.x <= xg+drift) and (yg-drift <= self.y <= yg + drift)):
            print('Check')
            return False
        else:
            return True
   
    def stop(self):
        """Funkcja STOP"""
        self.l.stop()
        self.r.stop()





    def move_to_goal(self,xg,yg):
        """Funkcja końcowa, która sprawdza położenie robota
         oraz wywołuje jazdę do celu z uwzględnieniem pobranych punktów.
        """
        while self.x > 700:  #awaryjna wartość
            self.stop()
            break
        while (self.check_range(xg,yg)):
            self.calc()                    #wywołanie pozycji
            print(self.x, self.y, self.fi) #prezentacja położenia
            self.go_to_goal(xg,yg)         #wywołanie jazdy
        self.stop()

if __name__ == '__main__':
    robo = Robot()
    waypoints = [[200,0],[125,100],[125,200]] #punkty docelowe
    for goal in waypoints:                    #przejście przez punkty
        print(goal)
        xg = goal[0]
        yg = goal[1]
        robo.move_to_goal(xg,yg)
    print("Stop")
    robo.stop()