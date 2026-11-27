from cgitb import reset
from logging import root
import os
import sys
import time
import math
from tracemalloc import stop
import keyboard
import tkinter as tk  

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

#Figure out how to stop it from asking for IP later
#####################################################################################

if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        # ip = input('Please input the xArm ip address:')
        ip = "192.168.1.200"
        if not ip:
            print('input error, exit')
            sys.exit(1)

#####################################################################################

counter = 0
posCounter = 0
SurgeryPos = 0
speed = 50
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
#Reset Variables:
Rx = 71.8
Ry = -355.8
Rz = 320.8 #290
Rr = -173.8
Rp = -0.2
RYaw = -90.9

#Equation Variables
dx = 0
dy = 0
dz = 0
dR = 0.0
dP = 0.0
dY = 0.0

#Starting Variables
f1 = [101.576, -316.251, 209.116, -175.62754973008145, -13.643901271229808, -85.68566637447609]
f2 = [87.847, -400.419, 208.735, -175.3408989451775, 19.076515197321733, -95.66195020751445]
f3 = [51.469, -297.305, 189.965, -176.05898694981497, -14.620737016148349, -91.83035224835659]
f4 = [6.408, -430.872, 163.482, -162.0227241804796, 41.34429072196313, -111.83929896147423]
f5 = [-42.856, -302.035, 129.169, -158.26962143925462, 52.37699413766372, -168.08003399060263]
t1 = [137.1,-357.5,213.9,170.8,8.9,-70]
t2 = [-20.3,-398.6,160,-165.2,28.5,-127.2]

zoffset = 10 # units of mm


def confirmPosition():
    global dx
    global dy
    global dz
    global dY
    global newPos
    if counter == 1 :
        newPos = arm.position
        dx += newPos[0] - f1[0]
        dy += newPos[1] - f1[1]
        dz += newPos[2] - f1[2]
        f1[0] = newPos[0]
        #dy += newPos[1] - f1[1]
        f1[1] = newPos[1]
        #dz += newPos[2] - f1[2]
        f1[2] = newPos[2]
        f1[3] = newPos[3]
        #dR = newPos[3] - f1[3]
        f1[4] = newPos[4]
        #dP = newPos[4] - f1[4]
        f1[5] = newPos[5]
        #dY += newPos[5] - f1[5]
        print(newPos) 
        #print(dx,dy,dz)
    
    if counter == 2 :
        newPos = arm.position
        dx += newPos[0] - f2[0]
        dy += newPos[1] - f2[1]
        dz += newPos[2] - f2[2]
        f2[0] = newPos[0]
        f2[1] = newPos[1]
        f2[2] = newPos[2]
        f2[3] = newPos[3]
        f2[4] = newPos[4]
        f2[5] = newPos[5]
        
        print(newPos) 
        
    if counter == 3 :
        newPos = arm.position
        dx += newPos[0] - f3[0]
        dy += newPos[1] - f3[1]
        dz += newPos[2] - f3[2]
        f3[0] = newPos[0]
        f3[1] = newPos[1]
        f3[2] = newPos[2]
        f3[3] = newPos[3]
        f3[4] = newPos[4]
        f3[5] = newPos[5]
        print(newPos) 

    if counter == 4 :
        newPos = arm.position
        dx += newPos[0] - f4[0]
        dy += newPos[1] - f4[1]
        dz += newPos[2] - f4[2]
        f4[0] = newPos[0]
        f4[1] = newPos[1]
        f4[2] = newPos[2]
        f4[3] = newPos[3]
        f4[4] = newPos[4]
        f4[5] = newPos[5]
        print(newPos) 

    if counter == 5 :
        newPos = arm.position
        dx += newPos[0] - f5[0]
        dy += newPos[1] - f5[1]
        dz += newPos[2] - f5[2]
        f5[0] = newPos[0]
        f5[1] = newPos[1]
        f5[2] = newPos[2]
        f5[3] = newPos[3]
        f5[4] = newPos[4]
        f5[5] = newPos[5]
        print(newPos) 

    #displacement averages
    dx = dx
    dy = dy
    dz = dz
    print(dx,dy,dz)


def increment_position():
    global counter
    counter += 1 
    print(counter)

    if counter == 1 :
            arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)
            time.sleep(1)
            arm.set_position(f1[0],f1[1],f1[2]+zoffset,f1[3],f1[4],f1[5], speed=80, wait=True)
            find_position = tk.Label(window, text=str("Go to the UFACTORY Studio and go into live control and turn the robot into manual mode and move it to the exact point of the feducial"), font=("Helvetica", 16)) 
            find_position.pack(pady=5)
    elif  counter == 2:
            arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)
            time.sleep(1)
            arm.set_position(f2[0],f2[1],f2[2]+zoffset,f2[3],f2[4],f2[5], speed=80, wait=True)
    elif  counter == 3:   
            arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)
            time.sleep(1)  
            arm.set_position(f3[0],f3[1],f3[2]+zoffset,f3[3],f3[4],f3[5], speed=80, wait=True)          
    elif  counter == 4:
            arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)
            time.sleep(1)
            arm.set_position(f4[0],f4[1],f4[2]+zoffset,f4[3],f4[4],f4[5], speed=80, wait=True)           
    elif  counter == 5:
            arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)
            time.sleep(1)
            arm.set_position(f5[0],f5[1],f5[2]+zoffset,f5[3],f5[4],f5[5], speed=80, wait=True)   
    enable() 

def BeginSurgery():
    arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)
    time.sleep(1)
    arm.set_position(t1[0]+(dx/5),t1[1]+(dy/5),t1[2]+(dz/5),t1[3],t1[4],t1[5], speed=80, wait=True)

def nextSurgeryPos():
     arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)
     time.sleep(1)
     arm.set_position(t2[0]+(dx/5),t2[1]+(dy/5),t2[2]+(dz/5),t2[3],t2[4],t2[5], speed=80, wait=True)

def enable():
    register_button.config(state=tk.NORMAL, text="Register Subject") #Enable the button and set its text

def reset_position():
    global counter
    global zoffset
    counter = 0 
    zoffset = 0
    arm.set_position(Rx, Ry, Rz, Rr, Rp, RYaw, speed=80, wait=True)

def quit():
      exit() 

def getPosition(): 
      #posCounter += 1 
      #print("posCounter: ",posCounter)
      display_position = tk.Label(position_window, text=str(arm.position), font=("Helvetica", 16)) 
      display_position.pack(pady=5)


window = tk.Tk()
window.title("Subject Registration")
register_button = tk.Button(window, text="Register Subject",command = increment_position) 
register_button.pack(pady=20) 

reset_button = tk.Button(window, text="Reset XArm",command=reset_position) 
reset_button.pack(pady=40)

quit_button = tk.Button(window, text="Quit Program",command=quit) 
quit_button.pack(pady=50)

position_window = tk.Toplevel(window)
position_window.title("List of positions")

position_button = tk.Button(window, text="position",command=getPosition) 
position_button.pack(pady=70)

changePos_button = tk.Button(window, text="Confirm fine tune position",command=confirmPosition) 
changePos_button.pack(pady=72)

StartSurgery_button = tk.Button(window, text="Start Surgery",command=BeginSurgery) 
StartSurgery_button.place(x=30, y=20)  

ContinueSurgery_button = tk.Button(window, text="Continue Surgery",command=nextSurgeryPos) 
ContinueSurgery_button.place(x=30, y=100)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

window.mainloop()
