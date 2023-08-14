#!/usr/bin/env python3
from multiprocessing.spawn import old_main_modules
from re import I
from ev3dev.ev3 import *
from time import sleep
import math
LM=LargeMotor('outC')
PM=LargeMotor('outB')



class robot():
    def __init__(self,R=2.9,L=12,N=360) -> None:
        """Inicjalizacja robota, jego silników i sensorów z zerowymi wartościami"""
        self.LM=LargeMotor('outC')
        self.PM=LargeMotor('outB')
        self.R=R
        self.L=L
        self.N=N
        self.n_l_old=self.LM.position
        self.n_r_old=self.PM.position

        self.x=0
        self.y=0
        self.fi=0

        self.psi=0
        self.drift=10
        self.L_czujnik=InfraredSensor('in2')
        self.R_czujnik=InfraredSensor('in4')
        self.C_czujnik=UltrasonicSensor('in3')

        print("Run Forest")
        time.sleep(10)
       
    def calk_position(self):
        """Funkcja służącą do ciągłego wykonywania obliczeń pozycji"""
        self.n_l=self.LM.position
        self.n_r=self.PM.position

        delta_n_r=self.n_r-self.n_r_old
        delta_n_l=self.n_l-self.n_l_old

        self.n_l_old=self.n_l
        self.n_r_old=self.n_r

        d_r=2*math.pi*self.R*(delta_n_r/self.N)
        d_l=2*math.pi*self.R*(delta_n_l/self.N)
        d_c=(d_r+d_l)/2

        self.x+=d_c*math.cos(self.fi)
        self.y+=d_c*math.sin(self.fi)
        self.fi+=(d_r-d_l)/self.L

    def dist_to_obstacle(self):
        """Funkcja ma na celu zwrócenie wartości sensorów
        Lewego, Prawego i Środkowego
        """

        distL=self.L_czujnik.value() * 0.7
        distR=self.R_czujnik.value() * 0.7
        distC=self.C_czujnik.value() * 0.1
        return [distL,distR,distC]

    def avoid_obstacle(self):
        """Funkcja ma na celu zwrócenie współrzędnych przeszkody
        zaimplementowanych(i już przekształconych) wzorów
         """

        x_c=self.dist_to_obstacle()[2] * math.cos(self.fi) + 5 * math.cos(self.fi) + self.x
        y_c=self.dist_to_obstacle()[2] * math.sin(self.fi) + 5*math.sin(self.fi) +self.y

        x_l=self.dist_to_obstacle()[0] * (0.5 * math.cos(self.fi) - 1/2 * math.sqrt(3) * math.sin(self.fi)) - 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.x
        y_l=self.dist_to_obstacle()[0] * (0.5 * math.sin(self.fi) + 1/2 * math.sqrt(3) * math.cos(self.fi)) + 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.y
       
        x_r=self.dist_to_obstacle()[1] * (1/2 * math.sqrt(3) * math.sin(self.fi) + 0.5 * math.cos(self.fi)) - 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.x
        y_r=self.dist_to_obstacle()[1] * (0.5 * math.sin(self.fi) - 1/2 * math.sqrt(3) * math.cos(self.fi)) + 8 * math.sin(self.fi) + 8 * math.cos(self.fi) + self.y

        x_obst=(x_c-self.x)+(x_r-self.x)+(x_l-self.x)
        y_obst=(y_c-self.y)+(y_r-self.y)+(y_l-self.y)
        return x_obst,y_obst

    def angle_to_obstacle(self):
        """Funkcja zwracająca kąt do przeszkody"""

        return math.atan2(self.avoid_obstacle()[1],self.avoid_obstacle()[0])

    def angle_to_go(self,xg,yg):
        """Funkcja zwracająca kąt do celu"""
        return math.atan2(yg-self.y,xg-self.x)
   
    def go_to_goal(self,xg,yg,speed=300,k=200):
        """Funkcja odpowiedzialna za jazdę do celu"""
        e=self.angle_to_go(xg,yg)-self.fi
        korekta=k*e
        self.LM.run_forever(speed_sp=speed-korekta)
        self.PM.run_forever(speed_sp=speed+korekta)
   
    def away_from_obstacle(self,speed=500):
        """Funkcja niewykorzystana do omijania"""
        e=self.angle_to_obstacle() - self.fi
       
        if(self.dist_to_obstacle()[0] <= 40 or self.dist_to_obstacle()[1] <= 40):
            k=250
        elif self.dist_to_obstacle()[2] <= 60:
            self.stop()
            time.sleep(1)
            self.l.run_forever(speed_sp=speed-k*e)
        else:
            k=0
       
        korekta=k*e
        self.LM.run_forever(speed_sp=speed-korekta)
        self.PM.run_forever(speed_sp=speed+korekta)
   
    def if_in_box(self,xg,yg):
        """Funkcja sprawdzająca położenie od pobranych
        punktów z uwzględnieniem dopuszczalnej granicy błędu(drift)
        """
        wynik=True
        if (xg-self.drift < self.x < xg+self.drift and yg-self.drift < self.y < yg + self.drift):
            wynik=False
       
        return wynik

    def alfa(self):
        """Funkcja zwracają wagę ALFA użytej w funkcji blend, która zależy
        od odległości czujników od przeszkody
        """
        if (self.dist_to_obstacle()[0] < 10 or self.dist_to_obstacle()[1] < 10 or self.dist_to_obstacle()[2] < 20):
            a = [0.2,250]
        elif(self.dist_to_obstacle()[0] < 40 or self.dist_to_obstacle()[1] < 40 or self.dist_to_obstacle()[2] < 40):
            a = [0.5,200]
        elif(self.dist_to_obstacle()[0] < 60 or self.dist_to_obstacle()[1] < 60 or self.dist_to_obstacle()[2] < 60):
            a = [0.8,150]
        # elif (self.dist_to_obstacle()[0] <= 20 or self.dist_to_obstacle()[1] <= 20 or self.dist_to_obstacle()[2] <= 30):
        #     return [0.4,200]
        else:
            a = [1, 100]
        return a

    def blender(self,xg,yg):
        """Metoda odpowiedzialna za blending na wyliczonej
        wagi ALFA przez funkcje alfa
        """
        a= self.alfa()[0]
        print(a)
        eg=self.angle_to_go(xg,yg) - self.fi
        ea=self.angle_to_obstacle() - self.fi
        eblend=(a*eg)+((1-a)*ea)
        return eblend



    def move_to_goal(self,xg,yg):
        """Finalna funkcja odpowiedzialna za jazdę do celu z wykorzystaniem blendingu"""
        while (self.if_in_box(xg,yg)):
            k = self.alfa()[1]
            self.calk_position()  #obliczanie pozycji
            speed=300
            e=self.blender(xg,yg)
            korekta=k*e
            self.LM.run_forever(speed_sp=speed-korekta)
            self.PM.run_forever(speed_sp=speed+korekta)


        self.LM.stop()
        self.PM.stop()


if __name__=="__main__":
    robo=robot()
    waypoints=[[400,0]]  #jedna wartość użyta do wyścigu z omijaniem
    for goal in waypoints:
        xg=goal[0]
        yg=goal[1]
        robo.move_to_goal(xg,yg)
    LM.stop()
    PM.stop()