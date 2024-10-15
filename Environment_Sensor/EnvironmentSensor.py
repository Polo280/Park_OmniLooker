import os
import sys
sys.path.append("/home/squirreldj/Park_OmniLooker/Environment_Sensor")

import time
from libs.SH1106 import *   #OLED
from libs.MPU9255 import *  #Gyroscope/Acceleration/Magnetometer
from libs.BME280 import *   #Atmospheric Pressure/Temperature and humidity
from libs.LTR390 import *   #UV
from libs.TSL2591 import *  #LIGHT
from libs.ICM20948 import * 
from libs.SGP40 import * 
from PIL import Image,ImageDraw,ImageFont


class Environment_Sensor:
    def __init__(self):
        # Declare the sensor instances 
        self.bme280 = None
        self.light = None
        self.uv = None
        self.sgp = None 
        self.icm20948 = None
        self.oled = None 
        # Start the sensor instances 
        self.startSensors()
        
    
    ''' Starts the environment station on board sensors and display '''
    def startSensors(self):
        self.bme280 = BME280()
        self.bme280.get_calib_param()
        self.light = TSL2591()
        self.uv = LTR390()
        self.sgp = SGP40()
        self.icm20948=ICM20948()
        self.oled = SH1106()
        
        # Data Stores
        self.env_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Store sensor data -> [temp, hum, pressure, lux, uv, gas]
        self.gyro_data = [0.0, 0.0, 0.0]                # Store IMU data -> [roll, pitch, yaw]
        self.accel_data = [0.0, 0.0, 0.0]               # Store accelerations 
        self.magnetometer_data = [0.0, 0.0, 0.0]        # Store magnetometer data 
    
    
    ''' Get general environment data from the on-board sensors'''
    def getEnvironmentData(self):
        bme_data = self.bme280.readData()           # BME280
        self.env_data[0] = round(bme_data[1], 2)    # Temperature
        self.env_data[1] = round(bme_data[2], 2)    # Humidity
        self.env_data[2] = round(bme_data[0], 2)    # Pressure
        
        self.env_data[3] = round(self.light.Lux(), 2)
        self.env_data[4] = self.uv.UVS()
        self.env_data[5] = round(self.sgp.raw(), 2)
        
        
    ''' Get data from the IMU and magnetometer '''
    def getIMU_Data(self):
        imu_data = self.icm20948.getdata()
        
        # Get accelerations
        self.accel_data[0] = round(imu_data[0], 2)
        self.accel_data[1] = round(imu_data[1], 2)
        self.accel_data[2] = round(imu_data[2], 2)
        
        # Get orientation
        self.gyro_data[0] = round(imu_data[3], 2)
        self.gyro_data[1] = round(imu_data[4], 2)
        self.gyro_data[2] = round(imu_data[5], 2)
        
        # Get magnetic field vector
        self.magnetometer_data[0] = round(imu_data[6], 2)
        self.magnetometer_data[1] = round(imu_data[7], 2)
        self.magnetometer_data[2] = round(imu_data[8], 2)
        

    ''' Show the data on the oled display '''
    def showDisplay(self):
        try:
            image = Image.new('1', (self.oled.width, self.oled.height), "BLACK")
            draw = ImageDraw.Draw(image)
            
            x = 0
            font = ImageFont.truetype('./libs/Font.ttc', 10)
            while True:
                x = x + 1
                time.sleep(0.2)
                if(x < 20):
                    bme = []
                    bme = self.bme280.readData()
                    pressure = round(bme[0], 2) 
                    temp = round(bme[1], 2) 
                    hum = round(bme[2], 2)
                    
                    lux = round(self.light.Lux(), 2)
                    
                    # uv = round(uv.UVS(), 2) 
                    uvdata = self.uv.UVS()
                    # ir = round(uv.readdata()[1], 2)
                    
                    gas = round(self.sgp.raw(), 2)
                    
                    draw.rectangle((0, 0, 128, 64), fill = 0)
                    
                    draw.text((0, 0), str(pressure), font = font, fill = 1)
                    draw.text((40, 0), 'hPa', font = font, fill = 1)
                    draw.text((0, 15), str(temp), font = font, fill = 1)
                    draw.text((40, 15), 'C', font = font, fill = 1)
                    draw.text((0, 30), str(hum), font = font, fill = 1)
                    draw.text((40, 30), '%RH', font = font, fill = 1)
                    
                    draw.text((0, 45), str(lux), font = font, fill = 1)
                    draw.text((40, 45), 'Lux', font = font, fill = 1)
                    
                    draw.text((65, 0), str(uvdata), font = font, fill = 1)
                    draw.text((105, 0), 'UV', font = font, fill = 1)
                    
                    draw.text((65, 30), str(gas), font = font, fill = 1)
                    draw.text((105, 30), 'GAS', font = font, fill = 1)
                    
                    self.oled.display(image)
                elif(x<40):
                    icm = []
                    # icm = MPU9255.getdata()
                    icm = icm20948.getdata()

                    #print(icm)
                    roll = round(icm[0], 2)
                    pitch = round(icm[1], 2)
                    yaw = round(icm[2], 2)
                    
                    draw.rectangle((0, 0, 128, 64), fill = 0)
                    font8 = ImageFont.truetype('Font.ttc', 9)
                    
                    draw.text((0, 0), 'RPY', font = font8, fill = 1)
                    draw.text((20, 0), str(roll), font = font8, fill = 1)
                    draw.text((50, 0), str(pitch), font = font8, fill = 1)
                    draw.text((90, 0), str(yaw), font = font8, fill = 1)
                    
                    # draw.text((0, 15), 'Acc', font = font8, fill = 1)
                    # draw.text((20, 15), str(icm[0]), font = font8, fill = 1)
                    # draw.text((50, 15), str(icm[1]), font = font8, fill = 1)
                    # draw.text((90, 15), str(icm[2]), font = font8, fill = 1)

                    draw.text((0, 30), 'Gyr', font = font8, fill = 1)
                    draw.text((20, 30), str(icm[3]), font = font8, fill = 1)
                    draw.text((50, 30), str(icm[4]), font = font8, fill = 1)
                    draw.text((90, 30), str(icm[5]), font = font8, fill = 1)

                    draw.text((0, 45), 'Mag', font = font8, fill = 1)
                    draw.text((20, 45), str(icm[6]), font = font8, fill = 1)
                    draw.text((50, 45), str(icm[7]), font = font8, fill = 1)
                    draw.text((90, 45), str(icm[8]), font = font8, fill = 1)
                    
                    self.oled.display(image)
                elif(x >= 40):
                    x = 0
                    
        except KeyboardInterrupt:
            print("2")
            image2 = Image.new('1', (self.oled.width, self.oled.height), "BLACK")
            self.oled.display(image2)
            return	

        

def main():
    # Scan i2c bus in read mode
    # os.system('i2cdetect -y -r 1')
    sensor = Environment_Sensor()
    sensor.startSensors()
    sensor.showDisplay()
    GPIO.cleanup()


if __name__ == "__main__":
    main()