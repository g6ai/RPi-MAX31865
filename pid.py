#!/usr/bin/env python3

import time
import board
import busio
import digitalio
import adafruit_max31865
import RPi.GPIO as GPIO
from simple_pid import PID
import matplotlib.pyplot as plt
import mpld3
from mpld3._server import serve
import numpy as np

GPIO.cleanup() # Clean up GPIO assignment

# Sensor initiation
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D22) # Chip select of MAX31865
sensor = adafruit_max31865.MAX31865(spi, cs, wires=4, rtd_nominal=1000.0, ref_resistor=4300.0)

# Heater initiation
GPIO.setup(4, GPIO.OUT)
GPIO.output(4, False)
relay_defined = False
period = 1
relay = GPIO.PWM(4, 1 / period)
relay_defind = True
relay.start(0)

# PID initiation
k_p = 5
k_i = 0.01
k_d = 100
pid = PID(k_p, k_i, k_d)
pid.sample_time = 0.1 # Then no need to use time.sleep()
pid.output_limits = (0, 100)

# Timer initiation
start_time = time.time()

# Data recording initiation
temp_data = np.array([time.time()-start_time, sensor.temperature])

# Control
try:
    while True:
        try:
            target_temp = float(input('Enter target temperature (\N{DEGREE SIGN}C): '))
            pid.setpoint=target_temp
            while True:
                current_temp = sensor.temperature

                output_power = float(pid(current_temp))
                relay.ChangeDutyCycle(output_power)
                
                print('Temperature: current {0:0.1f} \N{DEGREE SIGN}C, target {1:0.0f} \N{DEGREE SIGN}C; Power: {2:0.0f}%'.format(current_temp, target_temp, output_power))

                temp_data = np.vstack((temp_data, [time.time()-start_time, current_temp]))

        except KeyboardInterrupt:
            pid_run_flag = input("\nPID ended, stop now (Ctrl-C) or repeat (press any other key then enter): ")

            # Re-initiate sensor, or it'll get stuck
            spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
            cs = digitalio.DigitalInOut(board.D22) # Chip select of MAX31865
            sensor = adafruit_max31865.MAX31865(spi, cs, wires=4, rtd_nominal=1000.0, ref_resistor=4300.0)

finally:
    if relay_defined == True:
        relay.stop()

    GPIO.cleanup() # Reset GPIO for next run

    filename_str = str(input("\nFilename: "))
    pid_data_dir = "/home/pi/pid_data/"
    pid_data_filename = "p"+str(k_p)+"i"+str(k_i)+"d"+str(k_d)+"_"+filename_str
    np.savetxt(pid_data_dir+pid_data_filename+".txt", temp_data)

    fig = plt.figure()
    plt.plot(temp_data[:, 0], temp_data[:, 1])
    plt.hlines(y=target_temp, xmin=0, xmax=time.time()-start_time, colors='r')
    plt.title(pid_data_filename)
    html = mpld3.fig_to_html(fig)
    serve(html, ip='0.0.0.0')
