#! /usr/bin/env python3
# Uvoz biblioteka
import rospy
import tf
import math as m
import numpy as np
# Uvoz podataka 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from timeit import default_timer as timer
    
# Pabliser koji salje poruke RVIZ-u
pub_rviz = rospy.Publisher('visualization_marker', Marker, queue_size=0)    
"""""
Funkcija koja se poziva zarad vizuelnog prikaza detektovanih linija u RVIZ-u.
"""""
def print_lines(points):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.LINE_STRIP
    marker.action = marker.MODIFY

    # marker scale
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01

    # marker color
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # Niz u kome pamtimo pocetne i krajnje tacke linija koje ce se iscrtavati
    marker.points = []
    for i in range(points.shape[0]):
        p = Point()
        p.x, p.y, p.z = points[i][0], points[i][1], 0
        # Dodavanje tacaka u nizu za iscrtavanje
        marker.points.append(p)
    return marker
    
"""""
Funkcija koja na osnovu izmerene distance i ugla vrati polozaj u xy koord. sistemu.
"""""
def polar_xy(r, alpha):
    return np.transpose(np.array([np.cos(alpha)*r, np.sin(alpha)*r]))
    
"""""
Funkcija koja sluzi za dobijanje jednoznacnih parametara detektovane linije.
"""""   
def fit_line(X,Y):
    xc = np.mean(X)
    yc = np.mean(Y)   
    Xp = xc - X
    Yp = yc - Y
    sum_1 = np.sum(Xp*Yp)
    sum_2 = np.sum(Yp**2 - Xp**2)
    alpha = 1/2 * m.atan2(-2*sum_1, sum_2)
    r = xc*m.cos(alpha) + yc*m.sin(alpha)
    return [r,alpha]
    
"""""
Funkcija koja sluzi za proveru ukoliko je dobijena negativna distanca
"""""
def check_param(r,alpha):
    if r < 0:
        alpha = alpha + m.pi
        if alpha > m.pi:
            alpha = alpha-2*m.pi
        r = -r
    return [r,alpha]

"""""
Funkcija koja vraca udaljenost tacke P u odnosu na liniju medju tackama Pstart i Pend.
"""""
def distance(P,Ps,Pe):
    if np.all(np.equal(Ps,Pe)):
        return np.linalg.norm(P-Ps)
    return np.divide(np.abs(np.linalg.norm(np.cross(Pe-Ps,Ps-P))),np.linalg.norm(Pe-Ps))

"""""
Funkcija koja primi tacke koje pripadaju odgovarajucoj liniji i vrati indeks i rastojanje najudaljenije tacke od te linije.
"""""
def max_distance(P):
    d_max = 0
    index = -1
    for i in range(1,P.shape[0]):
        d = distance(P[i,:],P[0,:],P[-1,:])
        if (d > d_max):
            index = i
            d_max = d
    return d_max,index
  
"""""
Funkcija koja od niza tacaka kreira novi niz tacaka u kojem ne postoje dve susedne tacke koje su jednake.
"""""  
def create_points(points):
    p = np.array(points)               
    point = []
    point.append(p[0])
    last_p = point[0]
    for i in range(1, p.shape[0]):
        if np.all(p[i] != last_p):
            point.append(p[i])
            last_p = p[i]
    pointt = np.array(point)
    return pointt

"""""
Funkcija koja obavlja rekurzivni split and merge algoritam.
"""""  
def split_and_merge_rec(P,threshold):
    # Pronalazanje tacke koja je najudaljenija. Ukoliko je ona veca od zadatog praga onda se ta linija deli na dve.
    d,ind = max_distance(P)
    if (d>threshold):
        first = split_and_merge_rec(P[:ind+1,:],threshold) 
        second = split_and_merge_rec(P[ind:,:],threshold)
        points = np.vstack((first[:-1,:],second))
    else:
        points = np.vstack((P[0,:],P[-1,:]))
    return points
  
"""""
Funkcija koja obavlja iterativni split and merge algoritam.
"""""      
def split_and_merge_iter(P, threshold):
    points = []
    rest_of_points = []
    current_points = P
    # Dokle god imamo tacke koje pripadaju odg. linijama i koje trebaju da se provere algoritam ce se pokretati.
    while True:
        # Pronalazanje tacke koja je najudaljenija. Ukoliko je ona veca od zadatog praga onda se ta linija deli na dve.
        d,ind = max_distance(current_points)
        if d > threshold:
            rest_of_points.append(current_points[ind:,:])
            current_points = current_points[:ind+1,:]
        else:
            points.append(current_points[0,:])
            points.append(current_points[-1,:])
            if not rest_of_points:
                break
            else:
                current_points = rest_of_points.pop() 
    point = create_points(points)
    return point
    
"""""
Funkcija koja ima zadatak da spoji dve linije koje su kolinerne i da od njih napravi novu.
"""""
def merge(points, param):
    structure = np.zeros((2,4))
    lines = np.zeros((param.shape[0],2,4))
    point = []
    for i in range(param.shape[0]):
        structure[0,0:2] = param[i]
        structure[0,2:4] = points[i]
        structure[1,2:4] = points[i+1]
        lines[i] = structure
    for i in range(lines.shape[0]-1):
        r = lines[i][0][0]
        alfa = lines[i][0][1]
        r1 = lines[i-1][0][0]
        alfa1 =lines[i-1][0][1]
        
        # Poredjenje trenutne linije sa prethodnom i ispitivanje kolinearnosti te dve linije
        if (np.abs(r - r1) < 1) and (np.abs(alfa-alfa1) < 0.05):
            point.append(lines[i-1][0][2:4])
            point.append(lines[i][1][2:4])
            
        else:   
            point.append(lines[i][0][2:4])
            point.append(lines[i][1][2:4])       
    return point

"""""
Funkcija koja prima parametre sa lidara i unutar koje se poziva odgovarajuci algoritam.
"""""
def callback(data):
    global pub_rviz, threshold, alg
    ranges = np.asarray(data.ranges)
    alpha = np.linspace(data.angle_min,data.angle_max,360) 
    ranges[ranges==np.inf] = 3.5
    P = polar_xy(ranges,alpha)
    # SPLIT 
    if alg == 1:
        print('IZABRAN JE REKURZIVNI ALGORITAM')
        start = timer()
        points = split_and_merge_rec(P,threshold)
        end = timer()
        
    elif alg == 0:
        print('IZABRAN JE ITERATIVNI ALGORITAM')
        start = timer()
        points = split_and_merge_iter(P, threshold)
        end = timer()
    
    time = end - start
    print('Vreme izvrsavanja algoritma je = ' + str(time) + ' s')
   
    param = np.zeros((points.shape[0]-1,2))
 
    for i in range(points.shape[0]-1):
        param[i] = fit_line(points[i:i+2,0],points[i:i+2,1])
        param[i] = check_param(param[i,0],param[i,1])
    # MERGE
    merged = merge(points, param)
    points = create_points(merged)
    p = points
    p[0] = p[p.shape[0]-1]
    
    print('Parametri pronadjenih linija:')
    param = np.zeros((p.shape[0]-1,2))
    for i in range(p.shape[0]-1):
        param[i] = fit_line(p[i:i+2,0],p[i:i+2,1])
        param[i] = check_param(param[i,0],param[i,1])
        print('r = ' + str(param[i,0]) + ', alfa = ' + str(param[i,1]))
    print('-------------------------------')
    
    # ISCRTAVANJE LINIJA
    marker = print_lines(p)
    pub_rviz.publish(marker)

"""""
Funkcija koja se poziva iz maina i omogucava da se odabrani algoritam izvrsava dokle god je terminal otvoren.
"""""
def lines():
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('line_detection')
    threshold = 0.25
    
    # Izbor algoritma
    print('Koji algoritam zelite ? (0-iterativni, 1-rekurzivni)')
    choise = input()
    if choise == '0':
        alg = 0
    else:
        alg = 1
     
    try:
        lines()
    except rospy.ROSInterruptException:
        pass
