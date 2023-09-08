#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 21:56:20 2021

@author: kaydee
"""

from PIL import Image, ImageTk
import requests
import matplotlib.pyplot as plt
import numpy as np
import cv2

import pickle

def removeNoise(img):
    r,mask = cv2.threshold(img,10,255,cv2.THRESH_BINARY)
    eout = cv2.bitwise_and(mask,img)
    return eout




imloc ="https://ars.els-cdn.com/content/image/1-s2.0-S221471601500010X-gr1b.jpg"
#im = Image.open(requests.get(imloc, stream=True).raw)
img = cv2.imread("s15.png")
img = cv2.resize(img,(252,252))
plt.imshow(img)



#im = im.resize((252,252))



try:
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    inv_img = 255 - img
    #plt.imshow(inv_im)
   
except:
    print("Error, trying other way1")
    plt.imshow(img)
    plt.show()
    
    ret,inv_img = cv2.threshold(img,1,255,cv2.THRESH_BINARY)
    



cells = []
run = False


for r in range(0,252,28):
    for c in range(0,252,28):
        im = inv_img[r:r+28][:,c:c+28]
        pad = np.zeros(im.shape,dtype="uint8")
        pad[2:26][:,2:26] = im[2:26][:,2:26]
        im = pad.copy()
        centroid = pad[5:20][:,5:20]
        if run:
            
            plt.imshow(pad)
            plt.show()
        
        
            
        plt.title(sum(centroid.ravel()))
        contours, hierarchy = cv2.findContours(im,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(im, contours, -1, (0,255,0), 3)
        
        
        t = im.copy()
        t2 = im.copy() * 0
        try:
            maxc = max(contours, key = cv2.contourArea)
        except:
            pass
        
        
        cv2.drawContours(t2, [maxc], -1, 255, 2)

        
        
        cv2.fillPoly(t2, pts =[maxc], color=255)
        im = cv2.bitwise_and(t,t2)
        #plt.imshow(im)
        im = removeNoise(im)
        cells.append(im)
       
        
mark = {x:False for x in range(1,11)}
mark[10] = True
with open('counter.pickle', 'rb') as f:
    counter = pickle.load(f)

for i in cells:
    if all(mark.values()):
        break
    plt.imshow(i)
    plt.show()
    dirpath = input()
    if mark[int(dirpath)]:
        continue
    fpath = "assets/"+dirpath+"/"+dirpath
    cv2.imwrite(fpath+str(counter[int(dirpath)])+'.jpeg',i)
    counter[int(dirpath)]+=1
    mark[int(dirpath)] = True
    

with open("counter.pickle", 'wb') as f:
    pickle.dump(counter, f)


    





