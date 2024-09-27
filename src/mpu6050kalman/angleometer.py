# Author: Roche Christopher

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
	# Read the gyro and accelerometer values from MPU6050
	def init_mpu(self):

		# initialize mpu6050 by writing appropriate values to the registers
		# define register addresses
		PWR_MGMT_1 = 0x6B
		SMPLRT_DIV = 0x19
		CONFIG = 0x1A
		GYRO_CONFIG = 0x1B
		INT_ENABLE = 0x38

		# write to sample rate register
		# FIFO output and sensor register output rate is based on this. ref page 12 of MPU register doc
		self.bus.write_byte_data(self.DeviceAddress, SMPLRT_DIV, 7)
		# Write to power management register
		# set the device clock in "PLL with X axis gyroscope reference" ref page 40 of MPU register doc
		self.bus.write_byte_data(self.DeviceAddress, PWR_MGMT_1, 1)
		# Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.)
		# ref https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
		self.bus.write_byte_data(self.DeviceAddress, CONFIG, int('0000110', 2))
		# Write to Gyro configuration register
		# set the range of the gyroscope scale to ± 2000 °/s. ref page 14 of MPU register doc
		self.bus.write_byte_data(self.DeviceAddress, GYRO_CONFIG, 24)
		# Write to interrupt enable register
		# enable interrupt generation which generates an interrupt everytime data is written to the register.
		# refer page 27 of MPU register doc
		self.bus.write_byte_data(self.DeviceAddress, INT_ENABLE, 1)

	def read_raw_data(self, addr):
		# Accelerometer and gyroscope data are written to two 8 bit registers and hence they have to be read separately
		# and appended together.
		high = self.bus.read_byte_data(self.DeviceAddress, addr)
		low = self.bus.read_byte_data(self.DeviceAddress, addr+1)

		# concatenate higher and lower value
		value = ((high << 8) | low)

		# The data stored in accelerometer and gyroscope registers are signed 16 bit 2's complement values
		if value > 32768:
			value = value - 65536
		return value

	def measure_attitude(self):
		flag = 0
		kalman_roll = KalmanAngle()
		kalman_pitch = KalmanAngle()

		# kindly refer https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
		# Conversion of accelerometer into attitude/angle is based on the calculations given in that doc
		restrict_pitch = True # Comment out to restrict roll to ±90deg instead
		rad2deg = 57.2957786
		kalAngleX = 0
		kalAngleY = 0
		# some MPU6050 Registers and their Address

		accel_xout_h_reg = 0x3B
		accel_yout_h_reg = 0x3D
		accel_zout_h_reg = 0x3F
		GYRO_XOUT_H = 0x43
		GYRO_YOUT_H = 0x45
		GYRO_ZOUT_H = 0x47

		time.sleep(1)
		# Read Accelerometer raw value
		accel_x = self.read_raw_data(accel_xout_h_reg)
		accel_y = self.read_raw_data(accel_yout_h_reg)
		accel_z = self.read_raw_data(accel_zout_h_reg)

		# print(accX,accY,accZ)
		# print(math.sqrt((accY**2)+(accZ**2)))
		# convert accelerometer data into angle.
		# Note that the accelerometer does not provide the angular acceleration but provides
		# linear acceleration due to gravity. Linear acceleration due to gravity can be converted into angle using the
		# calculations given in the nxp documented referred above.
		if restrict_pitch:
			roll = math.atan2(accel_y, accel_z) * rad2deg
			pitch = math.atan(-accel_x / math.sqrt((accel_y ** 2) + (accel_z ** 2))) * rad2deg
		else:
			roll = math.atan(accel_y / math.sqrt((accel_x ** 2) + (accel_z ** 2))) * rad2deg
			pitch = math.atan2(-accel_y, accel_z) * rad2deg
		#print(roll)
		kalman_roll.setAngle(roll) # set roll obtained from accelerometer to the kalman object
		kalman_pitch.setAngle(pitch) # set pitch obtained from accelerometer to the kalman object

		timer = time.time()
		flag = 0

		while True:

			if flag > 100:
				# Problem with the connection
				print("There is a problem with the connection")
				flag = 0
				continue
			try:
				# Read Accelerometer raw value
				accel_x = self.read_raw_data(accel_xout_h_reg)
				accel_y = self.read_raw_data(accel_yout_h_reg)
				accel_z = self.read_raw_data(accel_zout_h_reg)

				# Read Gyroscope raw value
				gyro_x = self.read_raw_data(GYRO_XOUT_H)
				gyro_y = self.read_raw_data(GYRO_YOUT_H)
				gyro_z = self.read_raw_data(GYRO_ZOUT_H)

				dt = time.time() - timer
				timer = time.time()

				if restrict_pitch:
					roll = math.atan2(accel_y, accel_z) * rad2deg
					pitch = math.atan(-accel_x / math.sqrt((accel_y ** 2) + (accel_z ** 2))) * rad2deg
				else:
					roll = math.atan(accel_y / math.sqrt((accel_x ** 2) + (accel_z ** 2))) * rad2deg
					pitch = math.atan2(-accel_y, accel_z) * rad2deg

				gyro_roll_rate = gyro_x/131  # Convert the obtained data to degrees/second i.e rate of change of angle
				gyro_pitch_rate = gyro_y/131  # Convert the obtained data to degrees/second

				if restrict_pitch:
					if(roll < -90 and kalAngleX > 90) or (roll > 90 and kalAngleX < -90):
						kalman_roll.setAngle(roll)
					else:
						kalAngleX = kalman_roll.getAngle(roll,gyroXRate,dt)

					if(abs(kalAngleY)>90 or True):
						gyro_pitch_rate  = -gyro_pitch_rate
						kalAngleY  = kalman_pitch.getAngle(pitch,gyroYRate,dt)
				else:
					if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
						kalman_pitch.setAngle(pitch)
					else:
						kalAngleY = kalman_pitch.getAngle(pitch,gyroYRate,dt)

					if(abs(kalAngleX)>90):
						gyroXRate  = -gyroXRate
						kalAngleX = kalman_roll.getAngle(roll,gyroXRate,dt)

				# angle = (rate of change of angle) * change in time
				gyro_roll = gyro_roll_rate * dt
				gyro_pitch = gyro_pitch_rate * dt


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
				if flag == 100:
					print(exc)
				flag += 1

	def __init__(self):
		self.pitch=0
		self.roll = 0
		self.init_mpu()
		self.bus = smbus2.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
		self.device_address = 0x68  # MPU6050 device address
		self.kalman_pitch = 0
		self.kalman_roll = 0

	def measure(self):
		angleThread = threading.Thread(target=self.measure_attitude)
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
