#!/usr/bin/env python3

# Uvoz biblioteka
import rospy
import csv

# Uvoz tipova podataka
from std_msgs.msg import String, Int64, Float64, Float64MultiArray, MultiArrayDimension

"""
'Prikaz' nod koji dobija poruku od noda 'Obrada'. Poruka koju dobija je prosecna dnevna
temperatura u toku dana. Potrebno je formirati grafik prosecnih temperatura pomocu 
funkcije rqt_plot. Funkcija rqt_plot radi tako sto je potrebno da prima informacije sa nekog
topika i onda ih iscrta. Da bi omogucili normalan rad funkcije rqt_plot kreiran je pabliser 
koji ce slati primljenu poruku, a to je prosecna temperatura u toku dana.
"""

pub_graf = rospy.Publisher('topik_graf', Float64, queue_size = 1)

def callback(data):
    avg_temp = round(data.data, 1)
    rospy.loginfo('Prosecna dnevna temperatura je: %s', avg_temp)
    # Slanje podataka funkciji rqt_plot zarad iscrtavanja
    pub_graf.publish(avg_temp)

def subscriber():
    # Definisanje noda 'Prikaz'
    rospy.init_node('prikaz_subscriber', anonymous = False)
    sub = rospy.Subscriber('topik_prikaz', Float64, callback)
    rospy.spin()
      
if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
