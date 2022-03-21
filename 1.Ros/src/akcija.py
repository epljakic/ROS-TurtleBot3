#!/usr/bin/env python3

# Uvoz biblioteka
import rospy
import csv
	
# Uvoz tipova podataka
from std_msgs.msg import String, Int64, Float64, Float64MultiArray, MultiArrayDimension

# Promenljiva action
action = 0

"""
'Akcija' nod koji dobija poruku od noda 'Obrada' i koji je zaduzen da broji dane
u kojima je razlika maksimalne i minimalne temperature bila veca od 15℃. Kreirana je 
pomocna promenljiva action koja ce da broji potrebne dane. Poruka koju saljemo nodu 'Akcija' je
jedinica, zbog toga ovde i postoji ispitivanje da li je poruka koja je pristigla jednaka jedinici.
"""

def callback(data):
    global action
    if data.data == 1:
        action += 1
        rospy.loginfo('Akcija broj: %s', action)

def subscriber():
    # Definisanje noda 'Akcija'
    rospy.init_node('akcija_subscriber', anonymous = False)
    sub = rospy.Subscriber('topik_akcija', Int64, callback)
    rospy.spin()
      
if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
