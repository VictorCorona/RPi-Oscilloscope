#!/usr/bin/python

#    Victor Corona
#     June 2020
# 

import wave
#import pygame
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
import time
import RPi.GPIO as GPIO
import sys
from Adafruit import ADS1x15

class Scope(object):
    def __init__(self, ax, maxt, frequency,  dt=0.02):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = np.array([])
        self.ydata = np.array([])
        self.t0 = time.perf_counter()
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-5.0, 5.0)
        self.ax.set_xlim(0, self.maxt)

    def update(self, data):
        t,y = data
        self.tdata = np.append(self.tdata, t)
        self.ydata = np.append(self.ydata, y)
        self.ydata = self.ydata[self.tdata > (t-self.maxt)]
        self.tdata = self.tdata[self.tdata > (t-self.maxt)]
        self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt - 0.2)
#        self.ax.figure.canvas.draw()
        self.line.set_data(self.tdata, self.ydata)
        return self.line,

    def emitter(self, time_int=0.1):
        adc = ADS1x15()

        while True:
           t = time.perf_counter() - self.t0
           v23 = adc.readADCDifferential23(4096, 3300)*0.001
           yield t, v23

if __name__ == '__main__':
    dt = 0.01
    fig, ax = plt.subplots()
    frequency = input('Enter Desired Frequency in Hz: ')
    frequency = int(frequency)


    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(12, GPIO.OUT)
    pi_pwm = GPIO.PWM(12,2*frequency)
    pi_pwm.start(50)
    maxt = input('Enter the time interval you want displayed in seconds: ')
    maxt = float(maxt)
    adc = ADS1x15()
    trig_value = input('Enter trigger voltage level: ')
    trig_value = float(trig_value)
    wait = time.time()
    while True:
       sample = adc.readADCDifferential23(4096, 3300)*0.001
       if sample >= trig_value:
          scope = Scope(ax, maxt, frequency, dt=dt)
          ani = animation.FuncAnimation(fig, scope.update, scope.emitter, interval=dt*1000., blit=True)
          plt.show()
          GPIO.cleanup()
          break
       elif time.time() - wait >= 3:
          print('Trigger value too low; Try smaller ')
          trig_value = input('Enter trigger voltage level: ')
          trig_value = float(trig_value)
          pass
