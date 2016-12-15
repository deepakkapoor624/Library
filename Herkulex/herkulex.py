#!/usr/bin/env python2.7

import serial
import array
import time


def connect(port,baudrate):                                          ##connecting serial port at baud rate
    global Serial
    try:
        Serial=serial.Serial(port,baudrate,timeout=1)
    except:
        print "Serial Port of servo not connected"
    return serial

def close():
    try:
        Serial.close()
    except:
        print "Failed to close serial port of servo"



##### In Herkulex two checksum is calculated as to verify the data send via communication


#### calculating checksum of data
def checksum1(data):                                                ##calculating checksum
    xor=0x00
    for i in data:
        xor=xor^int(i)
    return xor&0xFE
    
    
def checksum2(data):
    xor=(~data)&0xFE
    return xor




#### only diff between send_data1 and send_data function is the amount of data received back from tbe Servo
def send_data1(data,read_byte):                                                       ##  sending data to herkulex Servo  for Status check
    length=len(data)+5
    data.insert(0,length)
    csm1 = checksum1(data)
    csm2 = checksum2(csm1)

    data.insert(0,0xFF)
    data.insert(1,0xFF)
    data.insert(5,csm1)
    data.insert(6,csm2)

    stringtosend = ""
    for i in range(len(data)):
        byteformat = '%02X' % data[i]
        stringtosend = stringtosend + "\\x" + byteformat

    Serial.write(stringtosend.decode('string-escape'))
    t=9
    rxdata=Serial.read(t)
    return rxdata
    


def send_data(data,read_byte):
    rxdata =0                                                       ##  sending data to herkulex Servo
    length=len(data)+5
    data.insert(0,length)
    csm1 = checksum1(data)
    csm2 = checksum2(csm1)
    data.insert(0,0xFF)

    data.insert(1,0xFF)
    data.insert(5,csm1)
    data.insert(6,csm2)
   
    stringtosend = ""
    for i in range(len(data)):
        byteformat = '%02X' % data[i]
        stringtosend = stringtosend + "\\x" + byteformat
    try:
        Serial.write(stringtosend.decode('string-escape'))
    except:
        print 'NOt able to send Data to herkulex'

    t=11+read_byte

    if(read_byte!=0):                                                                          ##reading data from servo ACK
      rxdata=Serial.read(t)
        
    return rxdata
   

class servo:

   
    def torque_on(self,servoid):     #### enabling the torque of the servo                                                                           ##enabling torque on 
        data=[]
        
        data.append(servoid)
        data.append(0x03)
        data.append(0x34)
        data.append(0x01)
        data.append(0x60)                                             ##command for torque on
        send_data(data,0)

    
    def break_on(self,servoid):   ######## enabling the break on                                                                        
        data=[]
        
        data.append(servoid)
        data.append(0x03)
        data.append(0x34)
        data.append(0x01)
        data.append(0x40)                                             ## 0x40 command for break on
        send_data(data,0)
        return
   
    def set_position(self,servoid,goalposition,goaltime,servo_led):                 ##setting the position of servo
        goalposition_msb = int(goalposition) >> 8
        goalposition_lsb = int(goalposition) & 0xff
        data=[]
        
        data.append(servoid)         
        data.append(0x05)
       
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid)
        data.append(goaltime)
        send_data(data,0)

   
    def set_angle(self,servoid,goalangle,goaltime,servo_led):                           ##seting the angle of servo
        
        goalposition=(goalangle/0.325)+512
        self.set_position(servoid,goalposition,goaltime,servo_led)


    def torque_off(self,servoid):                               ##disabling torque off
       data=[]
      
       data.append(servoid)
       data.append(0x03)
       data.append(0x34)
       data.append(0x01)
       data.append(0x00)
       send_data(data,0)


    def continuous_rotation(self,servoid,clk_anticlk,goalspeed,servo_led):                     ##continuous rotation
       if(clk_anticlk):
           goalspeed_msb = int(goalspeed) >> 8
           goalspeed_lsb = int(goalspeed) & 0xff
       else:
           goalspeed_msb = 64+((int(goalspeed)) >> 8)
           goalspeed_lsb = (goalspeed) & 0xff
 
       data = []
       
       data.append(servoid)
       data.append(0x05)
       data.append(goalspeed_lsb)
       data.append(goalspeed_msb)
       data.append(0x02|servo_led)
       data.append(servoid)
       data.append(0x3C)            ########play time
       send_data(data,0)



    def continuous_rotation_mul_servos(self,servoID1,servoID2,clk_anticlk,goalspeed,servo_led):                     ##continuous rotation
        if(clk_anticlk):
            goalspeed_msb = int(goalspeed) >> 8
            goalspeed_lsb = int(goalspeed) & 0xff
        else:
          
            goalspeed_msb = 64+((int(goalspeed)) >> 8)
            goalspeed_lsb = (goalspeed) & 0xff
        data = []
       
        data.append(0xFE)
        data.append(0x05)

        data.append(goalspeed_lsb)
        data.append(goalspeed_msb)
        data.append(0x02|servo_led)
        data.append(servoID1)
        data.append(0xa)            ########play time

        data.append(goalspeed_lsb)
        data.append(goalspeed_msb)
        data.append(0x02|servo_led)
        data.append(servoID2)
        data.append(0xa)       
        #print data 
        send_data(data,0)
        return



    def Ram_read(self,servoid,address,byte_read):                      ##reading data from Ram register
      
       data=[]
       check_data=[]
       
       data.append(servoid)
       data.append(0x04)
       data.append(address)
       data.append(byte_read)
        
       read_byte=send_data(data,byte_read)
       
       result=[]
       check_data.append(ord(read_byte[2]))
       check_data.append(ord(read_byte[3]))
       check_data.append(ord(read_byte[4]))
       check_data.append(ord(read_byte[7]))
       check_data.append(ord(read_byte[8]))
       check_data.append(ord(read_byte[9]))
       check_data.append(ord(read_byte[10]))
       check_data.append(ord(read_byte[11]))
         
       if(byte_read==2):
           check_data.append(ord(read_byte[12]))

       ### calculating checksum and verifying the data received         
       csum1=checksum1(check_data)
       csum2=checksum2(csum1)
      
       if (csum1==ord(read_byte[5]) and(csum2==ord(read_byte[6]) )):
           if(byte_read==2):
               result.append(ord(read_byte[10]))
               result.append(ord(read_byte[9]))
           else:
               result.append(ord(read_byte[9]))
    
       return result
       
        
    


    def servo_deadzone(self,servoid):    ####### defining the deadZone                                 
       s=self.Ram_read(servoid,0xA,0x01)
       return s


    def get_servo_position(self,servoid):                                                            ##getting servo position
       s=self.Ram_read(servoid,0x3c,0x02)

       if(len(s)):
           s=((s[0]&0x03)<<8 |(s[1]&0xff))
           angle=(s-512)*.325
           return angle
       return 0
   

    def moveall_position(self,servoid,servoid1,servoid2,servoid3,goalposition,goalposition1,goalposition2,goalposition3,goaltime,servo_led):  ##moving 4 servos at a time
        goalposition_msb = int(goalposition) >> 8
        goalposition_lsb = int(goalposition) & 0xff
        data=[]
        data.append(0xFE)
        data.append(0x06)
        data.append(goaltime)
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid)
        
        goalposition_msb = int(goalposition1) >> 8
        goalposition_lsb = int(goalposition1) & 0xff
        
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid1)
        
        goalposition_msb = int(goalposition2) >> 8
        goalposition_lsb = int(goalposition2) & 0xff
        
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid2)

        goalposition_msb = int(goalposition3) >> 8
        goalposition_lsb = int(goalposition3) & 0xff
        
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid3)
        
        send_data(data,0)
        


   
    def moveall_angle(self,servoid,servoid1,servoid2,servoid3,goalangle,goalangle1,goalangle2,goalangle3,goaltime,servo_led):      ##setting angle of 4 servos at a time
        goalposition1=(goalangle1/0.325)+512
        goalposition2=(goalangle2/0.325)+512
        goalposition=(goalangle/0.325)+512
        goalposition3=(goalangle3/0.325)+512
        self.moveall_position(servoid,servoid1,servoid2,servoid3,goalposition,goalposition1,goalposition2,goalposition3,goaltime,servo_led)



    def move2_position(self,servoid,servoid1,goalposition,goalposition1,goaltime,servo_led):  ##moving 2 servos at a time
        goalposition_msb = int(goalposition) >> 8
        goalposition_lsb = int(goalposition) & 0xff
        data=[]
        data.append(0xFE)
        data.append(0x06)
        data.append(goaltime)
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid)
        
        goalposition_msb = int(goalposition1) >> 8
        goalposition_lsb = int(goalposition1) & 0xff
        
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid1)
       
        send_data(data,0)
        

   
    def move2_angle(self,servoid,servoid1,goalangle,goalangle1,goaltime,servo_led):      ##setting angle of 2 servos at a time
        goalposition1=(goalangle1/0.325)+512  
        goalposition=(goalangle/0.325)+512
        self.move2_position(servoid,servoid1,goalposition,goalposition1,goaltime,servo_led)


    
    
    def move_angle(self,servoid,goalangle,goaltime,servo_led):      ##setting angle of 1 servo at a time
       
        goalposition=(goalangle/0.325)+512
       
        self.move_position(servoid,goalposition,goaltime,servo_led)


    def move_position(self,servoid,goalposition,goaltime,servo_led):  ##moving 1 servo at a time
        goalposition_msb = int(goalposition) >> 8
        goalposition_lsb = int(goalposition) & 0xff
        
        data=[]
        data.append(0xFE)
        data.append(0x06)
        data.append(goaltime)
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(servo_led)
        data.append(servoid)
        
        send_data(data,0)
             

    def reboot(self,servoid):                      ##rebooting the servos
        data=[]
        data.append(servoid)
        data.append(0x09)
        send_data(data,0)

   

    def Ram_write(self,servoid,address,byte_data):              ## writing  1 byte data to ram register
        data=[]
        data.append(servoid)
        data.append(0x03)
        data.append(address)
        data.append(01)
        data.append(byte_data)
        send_data(data,0)    


    def write_ticktime(self,servoid,data):
        self.Ram_write(servoid,0x39,data)


    def write_deadzone(self,servoid,data):
        self.Ram_write(servoid,0xA,data)

    

    def get_servo_ticktime(self,servoid):                            #getting servo angle from feedback
       return self.Ram_read(servoid,0x39,0x01)



    def get_servo_kp(self,servoid):                                 ##getting kp of the servo
       s=self.Ram_read(servoid,0x18,0x02)
       if(len(s)):
           s=((s[0]&0x03)<<8 |(s[1]&0xff))
           return s


    def get_servo_kd(self,servoid):                                   ##getting the kd of the servo
       s=self.Ram_read(servoid,0x1A,0x02)
       if(len(s)):
           s=((s[0]&0x03)<<8 |(s[1]&0xff))
           
           return s


    def get_min_voltage(self,servoid):                                   ##getting the Minimum voltage  that can be supplied to the servo
       s=self.Ram_read(servoid,0x6,0x01)
       voltage= int(s[0])*.074    
       return voltage


    def voltage_status(self,servoid):                                  
       try:
           s=self.Ram_read(servoid,0x36,0x01)
           voltage = int(s[0])*.074
           print 'voltage',voltage
       except:
           voltage =-1
      
       return voltage



    def overload_threshold_period(self,servoid):
        s=self.Ram_read(servoid,42,0x01)
        return s

    def overload_pwm_threshold(self,servoid):
        s=self.Ram_read(servoid,18,0x02)
        print hex(s[0]) ,hex(s[1])
        return s


    def acceleration_ratio(self,servoid):
        s=self.Ram_read(servoid,8,0x01)
        return s

    def status_detail(self,servoid):  ####give the status of the servo for calucating the error occurred in servo
        if(1): 
            data=[]
            data.append(servoid)
            data.append(0x07)
            read_byte = send_data1(data,0)
            
            result=[]
            check_data =[]
            check_data.append(ord(read_byte[2]))
            check_data.append(ord(read_byte[3]))
            check_data.append(ord(read_byte[4]))
            check_data.append(ord(read_byte[7]))
            check_data.append(ord(read_byte[8]))


            csum1=checksum1(check_data)
            csum2=checksum2(csum1)
            if (csum1==ord(read_byte[5]) and(csum2==ord(read_byte[6]) )):
                 
                result.append(ord(read_byte[7]))
                result.append(ord(read_byte[8]))
      
        else:
            result =0
        return result[0]


    def ack_policy(self,servoid):
        s=self.Ram_read(servoid,0x01,0x01)      ###   0x01 Ram Address 0x01 byte to read
        return s

  
    def minimum_PWM(self,servoId):
        s = self.Ram_read(servoId,0x0f,0x01)
        return s



    def EEP_read(self,servoid,address,byte_read):                      ##reading data from Ram register
      
        data=[]
        check_data=[]
       
        data.append(servoid)
        data.append(0x02)
        data.append(address)
        data.append(byte_read)
        
        read_byte=send_data(data,byte_read)
       
       
        result=[]
        check_data.append(ord(read_byte[2]))
        check_data.append(ord(read_byte[3]))
        check_data.append(ord(read_byte[4]))
        check_data.append(ord(read_byte[7]))
        check_data.append(ord(read_byte[8]))
        check_data.append(ord(read_byte[9]))
        check_data.append(ord(read_byte[10]))
        check_data.append(ord(read_byte[11]))
         
        if(byte_read==2):
            check_data.append(ord(read_byte[12]))
        
        csum1=checksum1(check_data)
        csum2=checksum2(csum1)
        if (csum1==ord(read_byte[5]) and(csum2==ord(read_byte[6]) )):
            if(byte_read==2):
                result.append(ord(read_byte[10]))
                result.append(ord(read_byte[9]))
            else:
                result.append(ord(read_byte[9]))
    
        return result

    def ack_policy_EEP(self,servoid):
        s=self.EEP_read(servoid,0x07,0x01)      ###   0x01 Ram Address 0x01 byte to read
        return s

  
    def minimum_PWM_EEP(self,servoId):
        s = self.EEP_read(servoId,0x15,0x01)
        return s

    def overload_threshold_period_EEP(self,servoid):
        s=self.EEP_read(servoid,48,0x01)
        return s

    def EEP_write(self,servoid,address,byte_data):              ## writing  1 byte data to ram register
        data=[]
        data.append(servoid)
        data.append(0x01)
        data.append(address)
        data.append(01)
        data.append(byte_data)
        send_data(data,0)
        return

    def EEP_address(self,servoId,data):
        addr = 0x06
        self.EEP_write(servoId,addr,data)        
        return

    def overload_detection_Period_EEP_Write(self,servoId,data):
        addr = 48
        self.EEP_write(servoId,addr,data)
        return


#### get the amount of torque servo gived
    def get_torque_data(self,servoid):                                                            ##getting servo position
        s=self.Ram_read(servoid,0x34,0x01)
        print int(s[0]) 
        return


if __name__ =="__main__":
    connect("/dev/ttyUSB0",115200)
    close()
    time.sleep(2)
    connect("/dev/ttyUSB0",115200)

