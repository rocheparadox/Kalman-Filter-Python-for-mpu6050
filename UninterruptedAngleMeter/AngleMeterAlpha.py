#Connections
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)


from Kalman import KalmanAngle
import smbus2			#import SMBus module of I2C
import time
import math
import threading


class AngleMeterAlpha:
		#Read the gyro and acceleromater values from MPU6050
		def MPU_Init(self):

				PWR_MGMT_1 = 0x6B
				SMPLRT_DIV = 0x19
				CONFIG = 0x1A
				GYRO_CONFIG = 0x1B
				INT_ENABLE = 0x38
				#write to sample rate register
				self.bus.write_byte_data(self.DeviceAddress, SMPLRT_DIV, 7)

				#Write to power management register
				self.bus.write_byte_data(self.DeviceAddress, PWR_MGMT_1, 1)

				# Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
				self.bus.write_byte_data(self.DeviceAddress, CONFIG, int('0000110', 2))

				#Write to Gyro configuration register
				self.bus.write_byte_data(self.DeviceAddress, GYRO_CONFIG, 24)

				#Write to interrupt enable register
				self.bus.write_byte_data(self.DeviceAddress, INT_ENABLE, 1)


		def read_raw_data(self, addr):
				#Accelero and Gyro value are 16-bit
				high = self.bus.read_byte_data(self.DeviceAddress, addr)
				low = self.bus.read_byte_data(self.DeviceAddress, addr+1)

				#concatenate higher and lower value
				value = ((high << 8) | low)

				#to get signed value from mpu6050
				if(value > 32768):
						value = value - 65536
				return value


		bus = smbus2.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
		DeviceAddress = 0x68   # MPU6050 device address

		def measureAngles(self):
				flag = 0
				kalmanX = KalmanAngle()
				kalmanY = KalmanAngle()

				RestrictPitch = True  # Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
				radToDeg = 57.2957786
				kalAngleX = 0
				kalAngleY = 0
				# some MPU6050 Registers and their Address

				ACCEL_XOUT_H = 0x3B
				ACCEL_YOUT_H = 0x3D
				ACCEL_ZOUT_H = 0x3F
				GYRO_XOUT_H = 0x43
				GYRO_YOUT_H = 0x45
				GYRO_ZOUT_H = 0x47

				time.sleep(1)
				# Read Accelerometer raw value
				accX = self.read_raw_data(ACCEL_XOUT_H)
				accY = self.read_raw_data(ACCEL_YOUT_H)
				accZ = self.read_raw_data(ACCEL_ZOUT_H)

				# print(accX,accY,accZ)
				# print(math.sqrt((accY**2)+(accZ**2)))
				if (RestrictPitch):
					roll = math.atan2(accY, accZ) * radToDeg
					pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
				else:
					roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
					pitch = math.atan2(-accX, accZ) * radToDeg
				#print(roll)
				kalmanX.setAngle(roll)
				kalmanY.setAngle(pitch)
				gyroXAngle = roll;
				gyroYAngle = pitch;
				compAngleX = roll;
				compAngleY = pitch;

				timer = time.time()
				flag = 0

				while True:

						if(flag >100): #Problem with the connection
							print("There is a problem with the connection")
							flag=0
							continue
						try:
							#Read Accelerometer raw value
							accX = self.read_raw_data(ACCEL_XOUT_H)
							accY = self.read_raw_data(ACCEL_YOUT_H)
							accZ = self.read_raw_data(ACCEL_ZOUT_H)

							#Read Gyroscope raw value
							gyroX = self.read_raw_data(GYRO_XOUT_H)
							gyroY = self.read_raw_data(GYRO_YOUT_H)
							gyroZ = self.read_raw_data(GYRO_ZOUT_H)

							dt = time.time() - timer
							timer = time.time()

							if (RestrictPitch):
								roll = math.atan2(accY,accZ) * radToDeg
								pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
							else:
								roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
								pitch = math.atan2(-accX,accZ) * radToDeg

							gyroXRate = gyroX/131
							gyroYRate = gyroY/131

							if (RestrictPitch):

								if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
									kalmanX.setAngle(roll)
									complAngleX = roll
									kalAngleX   = roll
									gyroXAngle  = roll
								else:
									kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

								if(abs(kalAngleY)>90 or True):
									gyroYRate  = -gyroYRate
									kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
							else:

								if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
									kalmanY.setAngle(pitch)
									complAngleY = pitch
									kalAngleY   = pitch
									gyroYAngle  = pitch
								else:
									kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

								if(abs(kalAngleX)>90):
									gyroXRate  = -gyroXRate
									kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

							#angle = (rate of change of angle) * change in time
							gyroXAngle = gyroXRate * dt
							gyroYAngle = gyroYAngle * dt

							#compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
							compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
							compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

							if ((gyroXAngle < -180) or (gyroXAngle > 180)):
								gyroXAngle = kalAngleX
							if ((gyroYAngle < -180) or (gyroYAngle > 180)):
								gyroYAngle = kalAngleY

							#print("Angle X: " + str(complAngleX)+"   " +"Angle Y: " + str(complAngleY))
							self.pitch = compAngleY
							self.roll  = compAngleX

							self.kalman_pitch = kalAngleY
							self.kalman_roll = kalAngleX
							self.compl_pitch = compAngleY
							self.compl_roll = compAngleX
							#print(str(roll)+"  "+str(gyroXAngle)+"  "+str(compAngleX)+"  "+str(kalAngleX)+"  "+str(pitch)+"  "+str(gyroYAngle)+"  "+str(compAngleY)+"  "+str(kalAngleY))
							time.sleep(0.005)

						except Exception as exc:
                                                    if(flag == 100):
                                                        print(exc)
                                                    flag +=1
							

		def __init__(self):
			self.pitch=0
			self.roll = 0
			self.MPU_Init()
			self.bus = smbus2.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
			self.DeviceAddress = 0x68  # MPU6050 device address
			self.compl_pitch = 0
			self.compl_roll = 0
			self.kalman_pitch = 0
			self.kalman_roll = 0

		def measure(self):
			angleThread = threading.Thread(target=self.measureAngles)
			angleThread.start()

		def getRoll(self):
			return self.roll

		def getPitch(self):
			return self.pitch

		def get_int_pitch(self):
			return int(self.pitch)

		def get_int_roll(self):
			return int(self.roll)

		def get_complementary_roll(self):
			return int(self.compl_roll)

		def get_complementary_pitch(self):
			return int(self.compl_pitch)

		def get_kalman_roll(self):
			return int(self.kalman_roll)

		def get_kalman_pitch(self):
			return int(self.kalman_pitch)
