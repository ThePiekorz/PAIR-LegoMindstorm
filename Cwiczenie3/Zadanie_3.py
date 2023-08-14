#!/usr/bin/env python3
from multiprocessing.spawn import old_main_modules
from os import kill
from unicodedata import name
from ev3dev.ev3 import *
import time
import math


class Robot():
    def __init__(self, promien=2.8, L=12, N=360):
        """Inicjalizacja robota, jego silników i sensorów z zerowymi wartościami"""
        self.l = LargeMotor('outC')
        self.r = LargeMotor('outB')
        self.lcz = InfraredSensor('in2')
        self.scz = UltrasonicSensor('in3')
        self.rcz = InfraredSensor('in4')
        self.promien = promien
        self.L = L
        self.N = N
        self.n_l_old = self.l.position
        self.n_r_old = self.r.position

        self.x = 0
        self.y = 0
        self.fi = 0

        print("Run Forest")
        time.sleep(5)

    def calc(self):
        """Funkcja służącą do ciągłego wykonywania obliczeń pozycji"""
        delta_n_R = self.l.position - self.n_l_old
        delta_n_R = self.l.position - self.n_l_old
        delta_n_L = self.r.position - self.n_r_old
        self.n_l_old = self.l.position
        self.n_r_old = self.r.position

        d_r = 2 * math.pi * self.promien * (delta_n_R/self.N) #pełen model robota unicycle
        d_l = 2 * math.pi * self.promien * (delta_n_L/self.N)
        d_c = (d_r + d_l)/2

        self.x += d_c * math.cos(self.fi)
        self.y += d_c * math.sin(self.fi)
        self.fi += ((d_r-d_l)/self.L)

    def angle_to_go(self, xg, yg):
        """Funkcja obliczająca kąt robota do celu"""
        psi = math.atan2(yg - self.y, xg - self.x)
        return psi

   
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
        while self.x > 700:
            self.stop()
            break
        while (self.check_range(xg,yg)):
            self.calc()
            print(self.x, self.y, self.fi)
            self.go_to_goal(xg,yg)
        self.stop()
   
    def dist_to_obstacle(self):
        """Funkcja ma na celu zwrócenie wartości sensorów
        Lewego, Prawego i Środkowego
        """
        distL = self.lcz.value() * 0.7
        distS = self.scz.value() * 0.1
        distR = self.rcz.value() * 0.7
        return [distL,distR,distS]
   
    def avoid_obstacle(self):
        """Funkcja ma na celu zwrócenie współrzędnych przeszkody
        zaimplementowanych(i już przekształconych) wzorów
         """
        x_c = self.dist_to_obstacle()[2] * math.cos(self.fi) + 5*math.cos(self.fi) + self.x
        y_c = self.dist_to_obstacle()[2] * math.sin(self.fi) + 5*math.sin(self.fi) + self.y

        x_l = self.dist_to_obstacle()[0] * (0.5 * math.cos(self.fi) - 0.5 * math.sqrt(3) * math.sin(self.fi)) - 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.x
        y_l = self.dist_to_obstacle()[0] * (0.5 * math.sin(self.fi) + 0.5 * math.sqrt(3) * math.cos(self.fi)) + 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.y

        x_r = self.dist_to_obstacle()[1] * (0.5 * math.sqrt(3) * math.sin(self.fi) + 0.5 * math.cos(self.fi)) - 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.x
        y_r = self.dist_to_obstacle()[1] * (0.5 * math.sin(self.fi) - 0.5 * math.sqrt(3) * math.cos(self.fi)) + 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.x

        x_obst = (x_c - self.x) + (x_r - self.x) + (x_l - self.x)
        y_obst = (y_c - self.y) + (y_r - self.y) + (y_l - self.y)

        return [y_obst, x_obst]
   
    def angle_to_obst(self):
        """Funkcja zwacająca kąt do przeszkody"""
        psi = math.atan2(self.avoid_obstacle()[0],self.avoid_obstacle()[1])
        return psi
       
    def away_from_obstacle(self, speed=400,a=40, k=500):
        """Finalna funkcja wywołująca jazdę z omijaniem przeszkody"""
        e = self.angle_to_obst() - self.fi
        distL, distC, distR = self.dist_to_obstacle()[0], self.dist_to_obstacle()[2], self.dist_to_obstacle()[1]
        if distL < a or distR < a :
            self.l.run_forever(speed_sp=speed-k*e)
            self.r.run_forever(speed_sp=speed+k*e)
        elif distC < a:
            self.stop()   #zbawienna funkcja stop
            time.sleep(1)
            # print(k,e,distC)   #wartości sprawdzające
            self.l.run_forever(speed_sp=speed-k*e)

        else:
            self.l.run_forever(speed_sp=speed)
            self.r.run_forever(speed_sp=speed)


if __name__ == '__main__':
    robo = Robot()
    start_time = time.clock()
    while time.clock() - start_time < 30:  #zegar uruchamia program na 30s
        robo.away_from_obstacle()
    robo.stop()