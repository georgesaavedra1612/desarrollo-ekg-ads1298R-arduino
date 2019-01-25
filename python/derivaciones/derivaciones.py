# -*- coding: utf-8 -*-
"""
Created on Sat Jan 12 23:30:54 2019

@author: George
"""

import numpy as np
import matplotlib.pyplot as plt

ch1 = np.genfromtxt('ch1.txt')
ch2 = np.genfromtxt('ch2.txt')
ch3 = np.genfromtxt('ch3.txt')
ch4 = np.genfromtxt('ch4.txt')
ch5 = np.genfromtxt('ch5.txt')
ch6 = np.genfromtxt('ch6.txt')
ch7 = np.genfromtxt('ch7.txt')
ch8 = np.genfromtxt('ch8.txt')

DI=ch2
DII=ch3
DIII=(DII-DI)
aVR=(-1*(DI+DII)/2)
aVL=((DI-DII)/2)
aVF=((DII-DI)/2)
V1=ch8
V2=ch4
V3=ch5
V4=ch6
V5=ch7
V6=ch1

plt.figure(1)
#plt.plot(DI[250:1500])  
plt.title('Grafica lectura ch1')
plt.plot(ch1)   
plt.grid()

plt.figure(2)
#plt.plot(DII[250:1500])   
plt.title('Grafica lectura ch2') 
plt.plot(ch2)   
plt.grid()

plt.figure(3)
#plt.plot(DIII[250:1500])   
plt.title('Grafica lectura ch3') 
plt.plot(ch3)   
plt.grid()

plt.figure(4)
#plt.plot(aVR[250:1500])   
plt.title('Grafica lectura ch4') 
plt.plot(ch4)   
plt.grid()

plt.figure(5)
#plt.plot(aVL[250:1500])    
plt.title('Grafica lectura ch5')
plt.plot(ch5)   
plt.grid()

plt.figure(6)
#plt.plot(aVF[250:1500])    
plt.title('Grafica lectura ch6')
plt.plot(ch6)   
plt.grid()

plt.figure(7)
#plt.plot(V1[250:1500])    
plt.title('Grafica lectura ch7')
plt.plot(ch7)   
plt.grid()

plt.figure(8)
#plt.plot(V2[250:1500])  
plt.title('Grafica lectura ch8')  
plt.plot(ch8)   
plt.grid()

plt.figure(9)
plt.plot(V3[250:1500])    
plt.grid()

plt.figure(10)
plt.plot(V4[250:1500])    
plt.grid()

plt.figure(11)
plt.plot(V5[250:1500])    
plt.grid()

plt.figure(12)
plt.plot(V6[250:1500])    
plt.grid()
plt.show()