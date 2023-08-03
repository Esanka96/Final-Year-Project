import _thread
import serial
ser = serial.Serial('COM11', 250000)
ser.flush()
import socket
import time
import cv2
from socket import *
rwww=0
lwww=0
import matplotlib.pyplot as plt
from DrRobot import DrRobot
DrRobot = DrRobot()
check=True

def move_cmd():
    print("running")
    while(check):
        DrRobot.emergency_stop_release()
        DrRobot.go_forward(20*1,20*-1, 1)
    DrRobot.emergency_stop()
    DrRobot.close_connection()

import numpy as np
import math
import time
dt = 0.01
ts =10
t = np.arange(0, ts+dt, dt)
w = 0.52
r = 0.135
factor = 0.025
x_scale = 10

kp = 0.3
kv = 0.0001

theta_res_dot = np.array([0, 0]).reshape(2, 1)
x_res_n = np.array([[0], [0], [0]])
x_res_o = np.array([0, 0, 0]).reshape(3, 1)
thetam_doubledot = np.array([0, 0]).reshape(2, 1)
thetam_dot = np.array([0, 0]).reshape(2, 1)
theta_res = np.array([0, 0]).reshape(2, 1)
v_pk = 0
psi_o = 0
x0 = 0
y0 = 0
psi0 = math.pi/4
eta = np.array([[], [], []])
x_gps = np.array([[], [], []])
x_kf = np.array([[], [], []])
x_kf_o = 0
x_kf_n = 0
x_d = 0  # derivation of kf

eta_dot = np.array([[], [], []])
# encoder
x_enco = np.array([[], []])

x = np.array([[], [], []])
theta_res_o = np.array([0, 0]).reshape(2, 1)

theta_res_ch = np.array([0, 0]).reshape(2, 1)
theta_res_dot_ch = np.array([0, 0]).reshape(2, 1)

endt = 0.00
#start1 = 0.00
xgps1 = 0.0
ygps1 = 0.0
thetagps1 = 0.0
mse = 0

def calculation():
    x_d_n=0
    x_d_o=0
    global check
    global dt,ts,t,w,r,factor,x_scale,kp,kv
    global eta,eta_dot,psi_n,j_psi,j_psi_inv,x_res_dot,v_pk,x_res_n,x_res_o,x,x_kf_n,x_d,x_kf_o,mse,psi_o,x_doubledot
    global thetam_doubledot,thetam_dot,m,lwww,rwww,line,theta_res,theta_res_o,theta_res_dot,x_kf
    t1=0
    t2=0
    line_o=[0,0]
    c=0
    for i in range(len(t)):
        t2=time.time()
        #dt=t2-t1
        #print(t2-t1)
        t1=t2
        
        eta = np.append(eta, np.array([t[i]/x_scale, math.sin(factor*t[i]), math.atan(
            x_scale*factor*math.cos(factor*t[i]))]).reshape(3, 1), axis=1)
        eta_dot = np.append(eta_dot, np.array([1/x_scale, factor*math.cos(factor*t[i]), -x_scale*factor**2*math.sin(
            t[i])/(1+(x_scale*factor*math.cos(factor*t[i]))**2)]).reshape(3, 1), axis=1)
        psi_n = (theta_res[0, 0]-theta_res[1, 0])*r/w
        j_psi = np.array([[(r/2)*math.cos(psi_n), (r/2)*math.cos(psi_n)],
                         [(r/2)*math.sin(psi_n), (r/2)*math.sin(psi_n)],
                         [r/w, -r/w]])

        j_psi_inv = 1/r*np.array([[math.cos(psi_n), math.sin(psi_n), w/2],
                                 [math.cos(psi_n), math.sin(psi_n), -w/2]])
        x_res_dot = np.dot(j_psi, theta_res_dot)

        v_pk = (x_res_dot[0, 0]**2+x_res_dot[1, 0]**2)**0.5

        x_res_n = np.array([x_res_o[0, 0]+v_pk*math.cos((psi_n+psi_o)/2)*dt,
                           x_res_o[1, 0]+v_pk*math.sin((psi_n+psi_o)/2)*dt, psi_n]).reshape(3, 1)

        x_res_o = x_res_n
        x = np.append(x, x_res_n, axis=1)
        x_kf_n =x_res_n
        x_d = (x_kf_n-x_kf_o)/dt
        g=5
        #lowpass filter
        x_d_n+=g*(x_d-x_d_o)*dt
        x_d_o=x_d_n
        x_d_n=x_d
        
        x_kf_o = x_kf_n
        
        x_kf = np.append(x_kf, x_kf_n, axis=1)
        mse += ((eta[0, i]-x_kf_n[0])**2+(eta[1, i]-x_kf_n[1])**2)##mean square error
        # x_d = np.append(x_d,x_res_dot, axis=1)
        psi_o = psi_n
        x_doubledot = (eta[:, i].reshape(3, 1)-x_kf_n)*kp + \
            (eta_dot[:, i].reshape(3, 1)-x_d_n)*kv

        thetam_doubledot = np.dot(j_psi_inv, x_doubledot)
        thetam_dot = thetam_dot+thetam_doubledot*dt  
        m1=80
        m2=80
        if (thetam_doubledot[0]*m1>100):
            lwww=[100]
        elif (thetam_doubledot[0]*m1<-100):
            lwww=[-100]
        else:
            lwww=thetam_doubledot[0]*m1
        if (thetam_doubledot[1]*m2>100):
            rwww=[100]
        elif (thetam_doubledot[1]*m2<-100):
            rwww=[-100]
        else:
            rwww=thetam_doubledot[1]*m2

        line = ser.readline().decode().rstrip().split()
        if line==line_o:
            c+=1
        else:
            print(c)
            c=0
        line_o=line
        ser.flush()
        en=150
        #time.sleep(0.05)
        theta_res = np.array([2*math.pi*int(line[0])/en,2*math.pi*int(line[1])/en]).reshape(2, 1)
        print(ser.readline())
        theta_res_dot = (theta_res-theta_res_o)/dt
        theta_res_o = theta_res

    ser.flush()
    check=False
    plt.plot(eta[0, :], eta[1, :], "c", label='ground truth')
    plt.scatter(x[0, :], x[1, :], s=3, color='b', label='encoder')
    plt.legend()
    plt.show()
    """"plt.plot(eta[0, :], t, "c", label='x VS Time')
    plt.plot(eta[1, :], t, "b", label=' VS Time')
    plt.show()"""
    
def camera():
    while(check):
        ret, frame = cap.read()
    
        if ret == True:
            t=time.time()
            img_name =  f'{t}.jpg'
            cv2.imwrite(img_name, frame)
            img_counter += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()


_thread.start_new_thread(move_cmd,())
_thread.start_new_thread(calculation,())
#_thread.start_new_thread(camera,())

