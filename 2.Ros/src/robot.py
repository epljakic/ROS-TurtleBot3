#!/usr/bin/env python3

# Uvoz biblioteka
import rospy
import math
import csv
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int64, Float64

# Uvoz srv fajla
from drugi_domaci.srv import robot, robotResponse

"""""
Funkcija koja se poziva prilikom unosa podataka preko servisa.
Poruka koja se salje preko servisa je oblika:
mod - biranje manuelnog ili automatskog rezima rada (0 ili 1)
lin_x - linearna brzina u manuelnom rezimu (0 ili 1)
ang_z - ugaona brzina u manuelnom rezimu (0 ili 1)
pos_x - zeljena pozicija po x-osi
pos_y - zeljena pozicija po y-osi
ang_theta - zeljena orjentacija robota
"""""
def robot_callback(req):
    """""
    Promenljiva tipa Twist() koja sadrzi polja unutar sebe koja sluze za upravljanje 
    linearnim i ugaonim brzinama rabota. Promenljiva je ovog tipa iz razloga sto topik
    "cmd_vel" prima ovaj tip podataka.
    """""
    vel = Twist()
    
    # Rucno upravljanje
    if req.mod == 0:
        # Konstantna linearna i ugaona brzina prilikom rucnog upravljanja
        const_lin = 0.1
        const_ang = 0.1
        
        if req.lin_x == 1:
            # Provera da li idemo napred ili nazad
            rospy.loginfo('Napred/nazad ?')
            napred = input()
            
            # Zadavanje konstantne linearne brzine
            if int(napred) == 1:
                vel.linear.x = abs(const_lin)
            else:
                vel.linear.x = -abs(const_lin)
        
        # Zadavanje konstantne ugaone brzine
        if req.ang_z == 1:
            vel.angular_z = const_ang
        
        # Slanje poruke topiku "cmd_vel"
        publisher.publish(vel)
    
    # Automatsko upravljanje
    else:
        # Ciljna pozicija
        xg = req.pos_x
        yg = req.pos_y
        # Konverzija iz stepeni u radijane
        tetag = req.ang_theta / 180 * np.pi
        
        # Promenljiva "kraj" govori da li je ciljna pozicija ostvarena
        global kraj 
        
        # Vrsimo upravljanje sve dok ciljnu poziciju i orijentaciju ne ostvarimo
        while(1):
            kraj = False
            
            # Parametri kontrolera
            kro = 0.3
            kalfa = 1.0
            kbeta = -0.5
            
            # Parametri u polarnom koordinantnom sistemu
            """""
            Parametri x, y, i teta su globalne promenljive u funkciji odom_callback.
            Ovi parametri predstavljaju trenutnu poziciju i orijentaciju robota.
            """""
            ro = np.sqrt(np.square(xg-x) + np.square(yg-y))
            atan = np.arctan2(yg-y, xg-x)
            alfa = atan - teta
            beta = tetag - atan
            
            # Ako je cilj iza robota
            if (alfa > np.pi/2) or (alfa < -np.pi/2):
                alfa = atan - teta - np.pi*np.sign(alfa)
                beta = tetag - atan - np.pi*np.sign(beta)
                kro = -kro
            
            # Linearna i ugaona brzina
            v = kro * ro
            w = kalfa * alfa + kbeta * beta
            
            # Zaustavljanje
            """""
            Zaustavljanje se vrsi onda kada smo dosli na ciljnu poziciju i kada je 
            ispunjena zeljena orijentacija robota.
            """""
            if ro < 0.05:
                v = 0
                if abs(alfa+beta) < 0.1:
                    w = 0
                    kraj = True
            
            # Konstantna brzina i ubrzanje
            """""
            Odrzavama konstantnu linearnu brzinu na delovima putanje koji imaju 
            rastojanje vece od 0.2 od cilja. Razlog tome je sto zelimo da u blizini ciljne 
            pozicije udjemo sa manjom brzinom a ne sa prevelikom. Ukoliko se ovo ne uradi
            petlja zakasni za jednu iteraciju i onda dok se izda komanda zaustavljanja bude 
            kasno i robot predje preko svoje ciljne pozicije.
            Parametri se mogu povecati ukoliko zelimo vece brzine.
            """""
            if ro > 0.2:
                odnos = abs(v/w)
                v = 0.3 * np.sign(v)
                w = 0.3 / odnos * np.sign(w)
            
            
            vel.linear.x = v
            vel.angular.z = w
            
            # Slanje poruke topiku "cmd_vel"
            publisher.publish(vel)
            
            # Ukoliko je pronadjena ciljna pozicija zavrsavamo sa upravljanjem
            if kraj == True:
                break
            """""    
            Ispis
            rospy.loginfo("linearna = " + str(v)+ " ugaona=" + str(w))
            rospy.loginfo("CILJ : x=" + str(xg) + " y=" + str(yg) + " teta=" + str(tetag))
            rospy.loginfo("ODOM : x=" + str(x) + "  y=" + str(y) + " teta=" + str(teta))
            rospy.loginfo("POLARNE : ro=" + str(ro) + " alfa=" + str(alfa) + " beta=" + str(beta))
            rospy.loginfo("-----------------------------------------------------------------------------")
            """""
    return robotResponse(True)



"""""
Funkcija koja se poziva prilikom dobijanja informacije sa topika "odom".
Cilj ove funkcije jeste da odredi trenutni polozaj i orjentaciju naseg robota.
Odom vraca kvaternionske parametre za orijentaciju pa je potrebno izvrsiti odgovarajucu
transformaciju kako bi dobili ugao teta koji zelimo.
"""""
def odom_callback(req):
    global x, y, teta
    q_x = req.pose.pose.orientation.x
    q_y = req.pose.pose.orientation.y
    q_z = req.pose.pose.orientation.z
    q_w = req.pose.pose.orientation.w
    # Odgovarajuca transformacija :)
    teta = np.arctan2(2.0*(q_w*q_z + q_x*q_y), 1.0 - 2.0*(q_y**2 + q_z**2))
    x = req.pose.pose.position.x
    y = req.pose.pose.position.y


# Inicijalizacija noda
rospy.init_node('robot_control', anonymous=True)
# Pabliser za topik "cmd_vel"
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
# Sabskrajber za topik "odom"
subscriber = rospy.Subscriber('/odom', Odometry, odom_callback)
s = rospy.Service('robot', robot, robot_callback)
rospy.loginfo('Service is ready !')
rospy.spin()
