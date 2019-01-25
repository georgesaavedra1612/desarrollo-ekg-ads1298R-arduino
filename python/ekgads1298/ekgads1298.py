# -*- coding: utf-8 -*-
"""
Created on Thu Jan 10 20:56:19 2019

@author: George
"""

from scipy.fftpack import fft
import numpy as np
import matplotlib.pyplot as plt

lectura = np.genfromtxt('140BPM.txt')
#lectura = np.genfromtxt('ARRITMIA140BPM.txt')

ondasR=[] 
Instantes=[]
Periodos=[]
N=len(lectura) 
Fs1=500.0
Ts1=1.0/Fs1  
t1 = np.linspace(0.0, N*Ts1, N) 
ekg1=lectura[50:N]


#16

#FILTRO NOTCH:
fc=61.685
fn=(Fs1/2)
wc=(fc*np.pi/fn)
b1=(-2*np.cos(wc))
b2=1
R=0.99

notch=np.zeros(np.shape(ekg1))
notch[0]=ekg1[0]
notch[1]=(ekg1[1]+(b1*ekg1[0])-(R*b1*notch[0]))
notch[2]=(ekg1[2]+(b1*ekg1[1])+(b2*ekg1[0])-(R*b1*notch[1])-(R*R*notch[0]))

for j in range(2,4511):
    notch[j]=ekg1[j]+(b1*ekg1[j-1])+(b2*ekg1[j-2])-(R*b1*notch[j-1])-(R*R*notch[j-2])

#12

#ALGORITMO GEORGE SAAVEDRA MARTINEZ#
comp=np.zeros(np.shape(notch))
for i in range(len(notch)):
    if notch[i]<=10: #1000
        comp[i]=0
    if notch[i]>10:  #1000
        comp[i]=notch[i]
        
signo=np.zeros(np.shape(comp))
for i in range(len(comp)):
    if i==0:
        signo[0]=(-1)
    if i>0:    
        resta=comp[i]-comp[i-1]    
        if resta>0: 
            signo[i]=1
        if resta<=0:
            signo[i]=(-1)  
           
for i in range(0, 4450):
    if i==0:
        print(" ")
    if i>0:
        if signo[i]==signo[i-1]:
            print(" ")
        if signo[i]!=signo[i-1]:
            if signo[i]==1 and signo[i-1]==-1:
                print(" ")
            if signo[i]==-1 and signo[i-1]==1:
                ondasR.append(i-1)
                Instantes.append(((i-1)*0.002))    
                
for i in range(1, len(Instantes)):
    Periodos.append(Instantes[i]-Instantes[i-1])    
                 
#2014
                 
plt.figure(1)
plt.figure(figsize=(12,12))
plt.subplot(211)
plt.title('Resultado del comparador con umbral de 1000')
#plt.plot(notch[250:5000])  
#plt.plot(notch)
#plt.plot(comp[1000:2000])
plt.plot(comp[250:5000])
#plt.plot(ekg1[50:500])
plt.grid()
plt.subplot(212)
plt.title('Resultado del detector de signo de la pendiente')
plt.plot(signo[250:5000])
#plt.plot(ekg1[250:5000])         
#plt.plot(ekg1)
plt.grid()


yf = fft(ekg1) # transformada discreta de fourier de la señal y: DFT(y)
yf[0]=0
xf = np.linspace(0.0, 1.0/(2.0*Ts1), N//2) # intervalo de frecuencia desde 0 hasta 1/(2*ts), N/2 puntos

yt = fft(notch)
yt[0]=0
xt = np.linspace(0.0, 1.0/(2.0*Ts1), N//2)

plt.figure(2)
plt.figure(figsize=(12,12))
plt.subplot(211)
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))  # ve el espectro hasta la frec niquist
plt.subplot(212)
plt.plot(xt, 2.0/N * np.abs(yt[0:N//2]))
plt.grid()
plt.show()

