#!/usr/bin/env python3

# Uvoz biblioteka
import rospy
import csv
	
# Uvoz tipova podataka	
from std_msgs.msg import String, Int64, Float64, Float64MultiArray, MultiArrayDimension

"""
'Merenja' nod koji ima zadatak da ucita podatke koji se nalaze u odgovarajucem csv fajlu
i da ih nakon toga prosledi na topik. Podaci koje ucitavamo su maksimalna, minimalna i 
prosecna dnevna temperatura. Svi podaci su objedinjeni u jedan niz koji je prosledjen 
nodu 'Obrada'.
"""

def publisher():
    # Definisanje noda
    pub = rospy.Publisher('topik_obrada', Float64MultiArray, queue_size = 1)
    rospy.init_node('merenje_publisher', anonymous = False)
    # Odredjivanje brzine kojom ce se slati informacije
    rate = rospy.Rate(10)
    
    # Ucitavanje podataka iz datog csv fajla
    with open('/home/ros/Workspaces/getting_started/src/prvi_domaci/src/podaci.csv', 'r') as csv_file:
        while not rospy.is_shutdown():
            csv_reader = csv.reader(csv_file)
            # Prolazak kroz csv fajl
            for line in csv_reader:
                if line[1] != 'maximum temperature':
                    message = [float(line[1]), float(line[2]), float(line[3])]
                    # Formiranje niza(poruke) koja ce se slati
                    send_message = Float64MultiArray(data = message)
                    # Slanje poruke nodu 'Obrada'
                    pub.publish(send_message)
                    rospy.loginfo(message)
                    rate.sleep()
       
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
