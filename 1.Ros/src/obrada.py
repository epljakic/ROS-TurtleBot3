#!/usr/bin/env python3

# Uvoz biblioteka
import rospy
import csv
	
# Uvoz tipova podataka
from std_msgs.msg import String, Int64, Float64, Float64MultiArray, MultiArrayDimension

"""
'Obrada' nod koji je zvanicno sabskrajber nodu 'Merenja', ali u okviru tog noda potrebno je
slati odgovarajuce informacije koje ce drugi sabskrajberi koristiti. Zbog toga smo napravili
dva pablisera unutar ovog dela. Prvi ce slati informacije o razlici maksimalne i minimalne dnevne 
temperature nodu 'Akcija', a drugi ce slati prosecnu dnevnu remperaturu nodu 'Prikaz'.
"""

pub_akcija = rospy.Publisher('topik_akcija', Int64, queue_size = 1) 
pub_prikaz = rospy.Publisher('topik_prikaz', Float64, queue_size = 1)


def callback(data):
    # Konverzija temperature iz ℉ u ℃
    max_temp = (data.data[0]-32)*5/9
    min_temp = (data.data[1]-32)*5/9
    avg_temp = (data.data[2]-32)*5/9
    # Razlika maksimalne i minimalne temperature
    result = round(max_temp - min_temp, 1)
    # Slanje poruke nodu 'Prikaz'
    pub_prikaz.publish(avg_temp)
    
    if result >= 15:
        # Slanje poruke nodu 'Akcija'
        pub_akcija.publish(1)
    # Prikaz razlike maksimalne i minimalne temperature    
    rospy.loginfo('Razlika je: %s', result)


def subscriber():
    # Definisanje noda 'Obrada'
    rospy.init_node('obrada_subscriber', anonymous = False)
    sub = rospy.Subscriber('topik_obrada', Float64MultiArray, callback)
    rospy.spin()
      
if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
