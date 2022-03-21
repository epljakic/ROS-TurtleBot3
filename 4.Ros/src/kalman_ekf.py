#!/usr/bin/env python3

# Ucitavanje biblioteka
import sys
import time
import math
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from scipy.linalg import block_diag
from sensor_msgs.msg import JointState
from cetvrti_domaci.srv import control, controlResponse
from laser_line_extraction.msg import LineSegment, LineSegmentList
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# =================================== KALMAN =============================================
"""""
Predikcija narednog stanja i izracunavanje Jakobijan matrice Fx i Fu.
"""""
def transitionFunction(x_staro,u,b): 
    k1 = (u[0,0]+u[1,0])/2
    k2 = (u[1,0]-u[0,0])/2/b   ### (dsr-dsl)/(2b)
    teta_staro = x_staro[2,0]
    x = x_staro + np.array([[k1*math.cos(teta_staro+k2)],[k1*math.sin(teta_staro+k2)],[2*k2]])
    teta = x[2,0]
    Fx = np.array([[1, 0, -k1*math.sin(teta + k2)], [0, 1, k1*math.cos(teta + k2)], [0, 0, 1]])
    Fu = np.zeros(shape=(3,2))
    Fu[0,0] = 1/2*math.cos(teta + k2) + 1/(2*b)*math.sin(teta + k2)*k1
    Fu[0,1] = 1/2*math.cos(teta + k2) - 1/(2*b)*math.sin(teta + k2)*k1
    Fu[1,0] = 1/2*math.sin(teta + k2) - 1/(2*b)*math.cos(teta + k2)*k1
    Fu[1,1] = 1/2*math.sin(teta + k2) + 1/(2*b)*math.cos(teta + k2)*k1
    Fu[2,0] = -1/b
    Fu[2,1] = 1/b
    return x,Fx,Fu

"""""
Funkcija koja vraca predikciju stanja i kovarijacione matrice.
"""""
def prediction(x_staro,P_staro,u,b):
    x_pred,Fx,Fu = transitionFunction(x_staro, u, b)
    k = 0.03 #0.03
    Q = np.array([[k*abs(u[0,0]),0],[0,k*abs(u[1,0])]])
    P_pred = np.dot(Fx,np.dot(P_staro,Fx.T))+np.dot(Fu,np.dot(Q,Fu.T))
    return x_pred,P_pred

def measurementFunction(x_pred,m_i):
    alfa,ro = m_i # globalne koordinate
    H = np.array([[0, 0, -1], [-math.cos(alfa), -math.sin(alfa), 0]])
    x,y,teta = x_pred
    x = float(x)
    y = float(y)
    teta = float(teta)
    z_pred = np.array([[korekcijaUgla(alfa-teta)],[ro-x*math.cos(alfa)-y*math.sin(alfa)]])
    return z_pred,H

"""""
Funkcija koja vrsi uparivanje detektovanih prepreka.
"""""
def associateMeasurements(x_pred,P,Z,R,M,g):
    v = []
    H = []
    R_novo = []
    ukupno = 0

    for i in range(M.shape[1]):
        z_pred_i,H_i = measurementFunction(x_pred, M[:,i])
        for j in range(Z.shape[1]):
            z_ops_j = np.reshape(Z[:,j],(2,1))
            v_ij = z_ops_j - z_pred_i
            Sigma_ij = np.dot(H_i, np.dot(P,H_i.T)) + R[j,:,:]
            d_ij = np.dot(v_ij.T,np.dot(np.linalg.inv(Sigma_ij), v_ij))
            if (abs(d_ij) < g**2):
                v.append(v_ij)              
                H.append(H_i)               
                R_novo.append(R[j,:,:])         
                ukupno = ukupno +1

    v = np.reshape(np.array(v), (-1,1)) 

    H = np.reshape(np.array(H), (-1,3)) 
    R_novo = np.reshape(np.array(R_novo), (-1,2,2))
    R_novo = block_diag(*R_novo)                  

    return v,H,R_novo

"""""
Funkcija koja vrsi finalnu estimaciju.
"""""
def filterStep(x_staro,P_staro,u,Z,R,M,g,b):
    x_pred,P_pred = prediction(x_staro, P_staro, u, b)
    v,H,R_upareno = associateMeasurements(x_pred, P_pred, Z, R, M, g)
    S = np.dot(H, np.dot(P_pred, H.T))+R_upareno
    K = np.dot(P_pred,np.dot(H.T,np.linalg.inv(S)))
    P = np.dot(np.eye(3)-np.dot(K,H),P_pred)
    x = x_pred + np.dot(K,v)
    x[2,0] = korekcijaUgla(x[2,0])
    return x,P

def korekcijaUgla(ugao):
    while (ugao > math.pi): 
        ugao -= 2*math.pi
    while (ugao < -math.pi):
        ugao += 2*math.pi
    return ugao

"""""
Funkcija za ucitavanje mape koja je kreirana pomocu skripte map_maker.py.
"""""
def load_map():
    ime = '/home/ros/Workspaces/getting_started/src/cetvrti_domaci/src/mapa.txt'
    mapa = open(ime,"r")
    M = []
    for red in mapa:
        x = red.split()
        ro = float(x[0])
        alfa = float(x[1])
        M.append([alfa, ro])
    mapa.close()
    return np.reshape(np.array(M).T,(2,-1))

#================================== CALL_BACK =========================================

"""""
Funkcija u kojoj dobijamo informaciju o predjenoj putanji levog i desnog tocka.
"""""
def joint_callback(data):
    global u, sr, sl, sr_novo, sl_novo, upravljanje
    if (not upravljanje):
        upravljanje = True
        sr, sl = data.position
    sr_novo, sl_novo = data.position

"""""
Funkcija koja belezi detektovane prepreke u okolini robota i belezi parametre r i teta.
"""""
def line_callback(data):
    global Z,R,merenje
    Z_novo = []
    R_novo = []
    for line in data.line_segments:
        alfa = line.angle
        ro = line.radius
        kovarijansa = np.asarray(line.covariance)
        Z_novo.append([alfa, ro])
        R_novo.append(kovarijansa.reshape((2,2)))
    if (len(Z_novo) == 0):
        sys.exit('Nema merenja sa senzora')
    Z = np.reshape(np.array(Z_novo).T,(2,-1))
    R = np.reshape(np.array(R_novo), (-1,2,2))

    merenje = True

"""""
Funkcija koja dobija podatke sa odoma i cuvamo dobijenu poziciju i kovarijacionu matricu.
"""""
def odom_callback(data):
    global x_odom, P_odom, odometrija,r,vreme_staro
    p = data.pose.pose
    indx = [0,1,5,6,7,11,30,31,35]
    cov_matrica = np.array(data.pose.covariance)
    cov_matrica = cov_matrica[indx]
    x0 = p.position.x
    y0 = p.position.y
    teta = np.arctan2(2.0*(p.orientation.w*p.orientation.z + p.orientation.x*p.orientation.y), 1.0 - 2.0*(p.orientation.y**2 + p.orientation.z**2))
    #(roll,pitch,teta) = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]) 
    # Cuvanje pozicije i kov. matrice
    x_odom = np.array([[x0],[y0],[teta]])
    P_odom = np.reshape(cov_matrica,(3,3))

    odometrija = True

"""""
Funkcia koja omogucava ispis parametara i gasenje programa.
"""""
def response_callback(req):
    global run, is_print
    if req.control == 'e':
        run = False
    elif req.control == 'p':
        is_print = True
    else :
        is_print = False
    return controlResponse(True)
     
"""""
Funkcija koja prima estimiranu poziciju i koja salje tu poziciju na topik drugi_domaci.
"""""
def talker(x_tr):
    pub = rospy.Publisher('drugi_domaci', Odometry, queue_size = 1)
    x,y,teta = x_tr
    x = float(x)
    y = float(y)
    teta = float(teta)
    position = Odometry()
    position.pose.pose.position.x = x
    position.pose.pose.position.y = y
    position.pose.pose.position.z = 0
    (q_x, q_y, q_z, q_w) = quaternion_from_euler(0, 0, teta)
    position.pose.pose.orientation.x = q_x
    position.pose.pose.orientation.y = q_y
    position.pose.pose.orientation.z = q_z
    position.pose.pose.orientation.w = q_w
    pub.publish(position)

# ===================================== MAIN ============================================

if __name__=='__main__':
    run = True
    is_print = False
    # Pozicija i kovarijaciona matrica dobijena sa odom-a
    x_odom = np.zeros((3,1))
    P_odom = np.zeros((3,1))
    # Upravljanje
    u = np.zeros((2,1))
    vreme_staro = 0
    sr_novo = 0
    sl_novo = 0
    sr = 0
    sl = 0
    Z = []
    R = []
    merenje = False
    upravljanje = False
    odometrija = False
    
    print('JAAAAAAAAAAAAAAAAAAAAAAAAAAAA' )
    try:
        rospy.init_node('kalman_ekf', anonymous=False)
        rospy.Subscriber('joint_states', JointState, joint_callback)
        rospy.Subscriber('line_segments', LineSegmentList, line_callback)
        rospy.Subscriber('odom', Odometry, odom_callback)
        s = rospy.Service('control', control, response_callback)
        
        # Informacije o tocku
        b = 0.16 
        r = 0.033 
        g = 1 #0.35
        # Ucitavanje mape
        M = load_map()
        while (not (odometrija and merenje and upravljanje)):
            continue
            
        x = x_odom
        P = P_odom
        while (run):
            ur = r*(sr_novo - sr)
            ul = r*(sl_novo - sl)
            sr = sr_novo
            sl = sl_novo
            u = np.array([[ul],[ur]])

            x,P = filterStep(x, P, u, Z, R, M, g, b)
            if is_print:
                print('==================================================================')
                print('Estimacija pozicije :\n{} \nPozicija sa odom-a :\n{}'.format(x.T, x_odom.T)) 
                print('Estimacija kovarijacione matrice : \n{} \nKovarijaciona matrica sa odom-a :\n{}'.format(P, P_odom))
            
            # Slanje estimirane pozicije drugom domacem zadatku
            talker(x)
            time.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
