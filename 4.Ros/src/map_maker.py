#!/usr/bin/env python3

# Ucitavanje biblioteka
import rospy
import os
import pandas as pd
from laser_line_extraction.msg import LineSegment,LineSegmentList

# Inicijalizacija noda
rospy.init_node('make_map', anonymous=False)

Map = pd.DataFrame(columns=['rho','alpha'])
map_saved = False

# Funkcija koja ima ulogu da procita informacije dobijene sa topika line_segments i da ih upise u odgovarujci fajl
# Ovde zapravo kreiramo mapu naseg okruzenja
def callback(data):
	global Map, map_saved

	extracted_lines = data.line_segments
	if not map_saved:
		for line in extracted_lines:
			rho = line.radius
			alpha = line.angle
			Map = Map.append({'ro':rho,'theta':alpha}, ignore_index=True)

		output_directory = os.path.join(os.getcwd(),'map.csv')
		Map.to_csv(output_directory)
		map_saved = True
		print('Map saved!') 

def listener():
	rospy.Subscriber('line_segments', LineSegmentList, callback)
		 
if __name__ == '__main__':
	try:
	    listener()
	except rospy.ROSInterruptException:
        pass
