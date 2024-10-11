#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
import time
import math
import smbus
import Jetson.GPIO as GPIO

ADDR                = (0x68)

# /** Registers */
PWR_M = 0x6B  #
DIV = 0x19  #
CONFIG = 0x1A  #
GYRO_CONFIG = 0x1B  #
ACCEL_CONFIG = 0x1C  #
INT_EN = 0x38  #

ACCEL_X = 0x3B  #
ACCEL_Y = 0x3D  #
ACCEL_Z = 0x3F  #
GYRO_X = 0x43  #
GYRO_Y = 0x45  #
GYRO_Z = 0x47  #
TEMP = 0x41  #

MAG_X = 0x03
MAG_Y = 0x05
MAG_Z = 0x07
ST_1 = 0x02
ST_2 = 0x09
MAG_ADDRESS = 0x0C

MPU9255_REG_ID = 0x75	    # identity of the device, 8 bit
MPU9255_ID = 0x71   # identity of mpu9250 is 0x71
					# identity of mpu9255 is 0x73

# int pin
# INI_PIN = 23

class MPU9255:
	def __init__(self, address=ADDR):
		self.bus = smbus.SMBus(1)
		self.address = address

		# GPIO.setmode(GPIO.BCM)
		# GPIO.setwarnings(False)
		# GPIO.setup(INI_PIN, GPIO.IN)

		self.ID = self.Read_Byte(MPU9255_REG_ID)
		if(self.ID != MPU9255_ID): # ID = 0x73
			print("ID = 0x%x"%self.ID)
			sys.exit()
		
		self.Write_Byte(PWR_M, 0x01)
		self.Write_Byte(DIV, 0x07)
		self.Write_Byte(CONFIG, 0)
		self.Write_Byte(GYRO_CONFIG, 0x18)   # 250dps, 
		self.Write_Byte(INT_EN, 0x01)  # 2g-scale
		#self.Write_Byte(self.ACCEL_CONFIG2, 0b00000000) # low pass

		# self.gyro = Gyro()
		# self.accel = Accel()
		# self.magn = Magn()
		# self.temp = Temp()

		self.Write_Byte(0x37, 0x02)    # enable bus master bypass
		self.Write_Byte(0x36, 0x01)
		self.bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x00)
		time.sleep(0.05)
		self.bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x0F)
		time.sleep(0.05)
		self.bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x00)
		time.sleep(0.05)
		self.bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x06)
		time.sleep(0.1)

	def Read_Byte(self, Addr):
		return self.bus.read_byte_data(self.address, Addr)

	def Write_Byte(self, Addr, val):
		self.bus.write_byte_data(self.address, Addr, val)

	def Read_Word(self, addr):
		high = self.Read_Byte(addr)
		low = self.Read_Byte(addr + 1)
		value = ((high << 8) | low)
		if (value > 32768):
			value = value - 65536
		return value
		
	def accel(self):
		x = self.Read_Word(ACCEL_X)
		y = self.Read_Word(ACCEL_Y)
		z = self.Read_Word(ACCEL_Z)
		Buf = [x, y, z]
		return Buf

	def gyro(self):
		x = self.Read_Word(GYRO_X)
		y = self.Read_Word(GYRO_Y)
		z = self.Read_Word(GYRO_Z)
		Buf = [x, y, z]
		return Buf
		
	def mag(self):
		self.bus.read_byte_data(MAG_ADDRESS, ST_1)
		
		xh = self.bus.read_byte_data(MAG_ADDRESS, MAG_X)
		xl = self.bus.read_byte_data(MAG_ADDRESS, MAG_X + 1)
		x = ((xh << 8) | xl)
		yh = self.bus.read_byte_data(MAG_ADDRESS, MAG_Y)
		yl = self.bus.read_byte_data(MAG_ADDRESS, MAG_Y + 1)
		y = ((yh << 8) | yl)
		zh = self.bus.read_byte_data(MAG_ADDRESS, MAG_Z)
		zl = self.bus.read_byte_data(MAG_ADDRESS, MAG_Z + 1)
		z = ((zh << 8) | zl)

		self.bus.read_byte_data(MAG_ADDRESS, ST_2)
		Buf = [x, y, z]
		return Buf

	def temp(self):
		tempRow = self.Read_Word(TEMP)
		tempC = (tempRow / 340.0) + 36.53
		tempC = "%.2f" % tempC
		return {"TEMP": tempC}
		
	def getdata(self):
		Accel = self.accel()
		Gyro = self.gyro()
		Mag = self.mag()
		
		# gs = [16384.0, 8192.0, 4096.0, 2048.0]
		# AxCal = [-2520, 4395, 1577]
		# Accel[0] = (Accel[0] / gs[1]) - AxCal[0]
		# Accel[1] = (Accel[1] / gs[1]) - AxCal[1]
		# Accel[2] = (Accel[2] / gs[1]) - AxCal[2]
		
		# dps = [131, 65.5, 32.8, 16.4]
		# GxCal = [130, 189, -17]
		# Gyro[0]=Gyro[0]/dps[2] - GxCal[0]
		# Gyro[1]=Gyro[1]/dps[2] - GxCal[1]
		# Gyro[2]=Gyro[2]/dps[2] - GxCal[2]
		
		# ms = [0.15]
		# MxCal = [0, 0, 0]
		# Mag[0]=Mag[0]*ms[0] - MxCal[0]
		# Mag[1]=Mag[1]*ms[0] - MxCal[1]
		# Mag[2]=Mag[2]*ms[0] - MxCal[2]
		
		# print(Accel)
		
		return [Accel[0],Accel[1],Accel[2], Gyro[0],Gyro[1],Gyro[2],Mag[0],Mag[1],Mag[2]]

if __name__ == '__main__':
	sensor = MPU9255()
	icm = []
	# icm = s.temp2()
	# time.sleep(1)
	try:
		while True:
			# print("while")
			# s.mag()
			time.sleep(0.5)
			icm = sensor.getdata()
			print("/-------------------------------------------------------------/")
			# print("Roll = %.2f , Pitch = %.2f , Yaw = %.2f" %(icm[0],icm[1],icm[2]))
			print("Acceleration: X = %d, Y = %d, Z = %d" %(icm[0],icm[1],icm[2]))
			print("Gyroscope:     X = %d , Y = %d , Z = %d" %(icm[3],icm[4],icm[5]))
			print("Magnetic:      X = %d , Y = %d , Z = %d" %(icm[6],icm[7],icm[8]))
	except KeyboardInterrupt:
		print("KeyboardInterrupt")
		# sensor.Disable()
		exit()
