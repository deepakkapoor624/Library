

### MPU library for running MPU9150

import time
import math
import random
from scipy import pi
import scipy.signal as signal
import smbus
import datetime
from time import sleep
import numpy as np
import numpy.fft as fft
            
bus =0
    #function for 2's complement acc and gyro data
def twos_compliment(data):
    if(data>=0x8000):
        return -((65535-data)+1)
    else:
        return data
          
def Mpu_initialize():
	global address ,bus
	flag =1     
	data=0x7
    bus = smbus.SMBus(1)
    address =0x68   #device address AD0 grounded
  	try:
	    bus.write_byte_data(address,0x6b,0x02)
       	bus.write_byte_data(address,0x6a,0)              ## user control register 
	    bus.write_byte_data(address,0x1A,1)               #setting DLPF Register register
        bus.write_byte_data(address,0x19,data)            #setting sampling_rate_divider 1000/(1+4)=200
        bus.write_byte_data(address,0x38,0x01)            ### interrupt enable register  		                                                     
	except IOError:
		print 'MPU not able Initialize'  
    return 


def  initialize_AK8963():###### initializing magnetometer in MPU_9150
	global calibrated_datax,calibrated_datay,calibrated_dataz,bus
	address_AK8963 =0x0C
	bus.write_byte_data(address,0x37,0x02)            ##enable by_pass_mode
    sleep(.1)   
	bus.write_byte_data(address_AK8963,0x0A,0x00)     ## Power down Magnetometer
    bus.write_byte_data(address_AK8963,0x0A,0x08)
	calibration_data =bus.read_i2c_block_data(address_AK8963,0x10,3)   ## read calibrated data

	#### calculate x,y,z data
	calibrated_datax= (calibration_data[0]-128)/256.0+1.0
	calibrated_datay= (calibration_data[1]-128)/256.0+1.0
	calibrated_dataz= (calibration_data[2]-128)/256.0+1.0
       

def read_data(data):
	
    global address ,bus

    address_AK8963 =0x0C
   
    #### scale parameter   
    acc_scale=16384.0
    gyro_scale=1310.0
    pi = 3.14159

	if data==0:   ###### reading accelerometer data
           
        try:
                                        
            C=bus.read_byte_data(address,0x3A)
        	raw_acc_data=bus.read_i2c_block_data(address,0x3b,6) #reading raw accelerometer data
        
            ##### convert the data  / scale the data 
            acc_scaled_x =twos_compliment((raw_acc_data[0]<<8)+raw_acc_data[1])/acc_scale
        	acc_scaled_y =twos_compliment((raw_acc_data[2]<<8)+raw_acc_data[3])/acc_scale
        	acc_scaled_z =twos_compliment((raw_acc_data[4]<<8)+raw_acc_data[5])/acc_scale
	
        	x = (acc_scaled_x*10)
        	y = (acc_scaled_y*10)
        	z = (acc_scaled_z*10)

        except IOError: 
            print 'Error in reading Accelerometer data'                           
                               
        
	elif data==1:                 # reading raw gyrometer data
             
        try:
            bus.read_byte_data(address,0x3A)
        	raw_gyro_data=bus.read_i2c_block_data(address,0x43,6)   # reading raw gyrometer data
  
  ############scaling of the data
            x =(twos_compliment((raw_gyro_data[0]<<8)+raw_gyro_data[1])/gyro_scale)
        	y =(twos_compliment((raw_gyro_data[2]<<8)+raw_gyro_data[3])/gyro_scale)
        	z =(twos_compliment((raw_gyro_data[4]<<8)+raw_gyro_data[5])/gyro_scale)                                  
        except IOError: 
        	print 'Error in reading Gyrometer Data'
                                       
        	              

	elif data==2:
		  initialize_AK8963()        ### Initialize MPU_9150 to be used as magnetometer
	

	elif data==3:   # reading raw magnetometer data          

        C=bus.read_byte_data(address_AK8963,0x02)
        while(C):
                                
        	raw_magneto_data = bus.read_i2c_block_data(address_AK8963,0x03,6) #reading data 
		    ### data in little endian format and 2's complement
        	magneto_scaled_x =twos_compliment((raw_magneto_data[1]<<8) + (raw_magneto_data[0]))
        	magneto_scaled_y =twos_compliment((raw_magneto_data[3]<<8) + (raw_magneto_data[2]))
        	magneto_scaled_z =twos_compliment((raw_magneto_data[5]<<8) + (raw_magneto_data[4]))
				
			#### scaling of the magnetometer data	
					 
        	mRes = (10.0*1229.0)/4096.0
			magbias_x = -5.0
			magbias_y = -95.0
			magbias_z = -260.0
			magneto_scaled_x = float((magneto_scaled_x*mRes*calibrated_datax) -magbias_x)
			magneto_scaled_y = float((magneto_scaled_y*mRes*calibrated_datay)-magbias_y)
			magneto_scaled_z = float((magneto_scaled_z*mRes*calibrated_dataz) -magbias_z)
			sleep(.5) 
				
			angle = math.degrees(math.atan(magneto_scaled_y/magneto_scaled_x))
	

	        ##### convert the angle to direction			
			if angle <0:
				angle = angle + 360
			if angle >=45 and angle <135:
				print "E"
	        elif angle >=135 and angle <225:
				print "S"
			elif angle >=225 and angle <315:
				print "w"
			else:
				print "N"
				
        	magneto_scaled_x=str(magneto_scaled_x)
        	magneto_scaled_y=str(magneto_scaled_y)
            magneto_scaled_z=str(magneto_scaled_z)
					
            bus.write_byte_data(address_AK8963,0x0A,0x01)
  
        data = np.array([-(float(y)),float(x),float(z)])       ### data are sent in reverse order to align it with the phone                   
        return data
	 

if __name__ == "__main__":
	  
    Mpu_initialize()
    #read_data(bus,0) ## accelerometer
    while(1):
        r = read_data(0)
        print r 
    #read_data(bus,2)    ## configure Magnetometer
    #read_data(bus,3)     ## read data from magnetometer
        time.sleep(.5)






















