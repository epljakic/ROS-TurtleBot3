#!/usr/bin/env python3

# Uvoz biblioteka
import rospy
import csv

# Uvoz odgovarajuceg srv fajla
from prvi_domaci.srv import server, serverResponse
# Uvoz tipova podataka
from std_msgs.msg import String, Int64, Float64, Float64MultiArray, MultiArrayDimension

"""
Bilo je potrebno kreirati server koji ce moci da primi nove parametre a to su minimalna, 
maksimalna i prosecna dnevna temperatura, da ih konvertuje iz iz ℃ u ℉ i da ih nakon toga
upise u vec postojeci csv fajl. Nakon upisa u dati csv fajl sveze unete podatke je potrebno obraditi
na isti nacin kao i podatke pre njih, dakle potrebno je da prodju ceo proces kroz sve nodove.
U datom server.srv fajlu imamo tri ulazne promenljive koje odgovaraju prethodno pomenutim parametrima
tipa float i jednu izlaznu promenljivu tipa bool koja signalizira da je podatak unet ispravno u csv fajl.
"""

def response_callback(req):
    # Konverzija iz ℃ u ℉ 
    max_temp = req.max*9/5 + 32
    min_temp = req.min*9/5 + 32
    avg_temp = req.avg*9/5 + 32
    # Otvaranje csv fajla za upis
    with open('/home/ros/Workspaces/getting_started/src/prvi_domaci/src/podaci.csv', 'a') as csv_file:
        # Definisanje nacina na koji ce se upisivati podaci
        csv_file = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        # Upisivanje podataka
        csv_file.writerow(['dd-mm-gggg', max_temp, min_temp, avg_temp])
        rospy.loginfo('max: %d, min: %d, avg: %d', max_temp, min_temp, avg_temp) 
        return serverResponse(True)
 
# Definisanje noda    
rospy.init_node('server_srv')
s = rospy.Service('server', server, response_callback)
rospy.loginfo('Service is ready !')
rospy.spin()
