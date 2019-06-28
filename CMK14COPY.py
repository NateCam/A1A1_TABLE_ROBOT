##!/usr/bin/python3
#A1-A1 Table Droid
#cd robotFiles/python
#THIS IS A TEST OF push!!!!
#sudo python CMK8.py
#sudo iwconfig wlan0 power off #turns off power management
#iwconfig reports status

#cd Videos then omxplayer --fps 23.976 robotcam.h264 to play clip
    # sudo i2cdetect -y 1

#CMK14 March 14 2019 switch to Python 3 and accelerometer incorporation and attempted new MicroPython Motorhat Librarincorporation
    # didn't work. git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library
    #install python-dev if haven't already
    #sudo apt-get update
    #sudo apt-get install build essential
    #go to cd /home/pi/Adafruit?-Motor_HAT-Python_library
    # sudo python3 setup.py install
    # back to old library but installed LIS3DH then switched out for LSM9DS1 and got that integrated
    
#CMK13 dec 29 incorp sensor slip for high IR environmnets using camera analog gain as a trigger
#CMK12 dec 23 incorp TopGuard code for lip protection fine tune cliff accuracy    
#CMK11 dec 22 incorp cliff sensor in scan and ext media record. fixed bug where clip overwritten by same dayte stamp
    # also, more intelligent route change on cliff sense.  Burn in of file name at head of clip
#CMKX dec 21 2017 "Cheese & Crackers" build party loop external record
#CMK10 dec 19 2017: party loop after clean install
#CMK9 dec 18 2017: party loop
#CMK8 Dec 12 2017: temp removal on hall sensor until replaced, motor testing.
#CMK7 Dec 4 2017: work on iterable movement beyond first loop
#CMK6 Dec 1, 2017:enable camera recording at text append.  lasers used in rough scan
    #refine nav_rules implement crude speed ramping in steppers
       #cd Videos then omxplayer --fps 23.976 robotcam.h264 to play clip
    # sudo i2cdetect -y 1 
#CMKV modularize movement
#CMKIV solved nav problem motor dir calc mistake

#CMKIII evolution of database of 360 degree movement
#chassisMKII 2nd motor and Hall effect Sensor

#These are in reverse order from above
#chassisTest3- stepper pionts sensor pckage to desired travel  body moves in desired direction with accel/decel
#lrscan = calculates coarse and fine scan steps but does not actually activate ultrasonic sensor
#scan2.py = intigrates serial_com.py for actual ultrasonic ranges
#chassisTest4= intigrates scan2.py and makes nested functions of coarse scan and fine scan
    #and added stepper turn off and distance measure etc
#ct4.py further refinements of scans and full multiplex of 3 lasers
#ct5.py fixed sonic buffer mis-order and on motor step error to drifting.
#ct6.py 7/22/17 put dates in further revisions. physical build is roughed in
    #sensor reading to nav changes and doubling ultrasonic for accuracy
#

import board
import digitalio
import busio


i2c =busio.I2C(board.SCL, board.SDA)

## LSM9DS1 Accelerometer gyro, magnetometer, temp
import adafruit_lsm9ds1
DOF9sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
DOF9sensor.mag_gain= adafruit_lsm9ds1.MAGGAIN_4GAUSS
DOF9sensor.gyro_scale= adafruit_lsm9ds1.GYROSCALE_245DPS


#LIS3DH Accelerometer code
#import adafruit_lis3dh
#int1= digitalio.DigitalInOut(board.D26)
#lis3dh= adafruit_lis3dh.LIS3DH_I2C(i2c)



from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
mh = Adafruit_MotorHAT(addr=0x60)

motorOne = mh.getMotor(1)
motorTwo = mh.getMotor(2)
motorThree = mh.getMotor(3)

#New library that doesnt work yet
##from adafruit_motorkit import MotorKit
##kit = MotorKit()
from picamera import PiCamera
from picamera import PiCamera, Color
import time
from time import sleep


import VL53L0X
import atexit
import math
import RPi.GPIO as gpio
import sys
import serial
from fractions import Fraction #for cam anaog gain sensing

stream=False


date_string = time.strftime("%Y-%m-%d-%H-%M")
print((str(date_string)))
    #time.sleep(1)

##########LIVESTREAM AND T ############
#if (stream==True):
    #raspivid -o - -t 0 -md 4 -w 1640 -h 1232 -awb tungsten -fps 30 -b 500000| tee test_video.h264 | ffmpeg -re -ar 44100 -ac 2 -acodec pcm_s16le -f s16le -ac 2 -i /dev/zero -f h264 -i - -vcodec copy -acodec aac -ab 128k -g 50 -strict experimental -f flv rtmp://a.rtmp.youtube.com/live2/1zrq-b67d-vqb7-f7w7
    #raspivid -o - -t 0 -md 4 -drc high -vs -awb tungsten -st ae 32,0xff, 0x808000 -a "TEST FOOTAGE"-fps 23.976 -b 500000| tee test_video.h264 | ffmpeg -re -ar44100 -ac 2 -acodec pcm_s16le -f s16le -ac 2 -i /dev/zero -f h264 -i - -vcodec copy -acodec aac -ab 128k -g 50 -strict experimental -f flv rtmp://a.rtmp.youtube.com/live2/1zrq-b67d-vqb7-f7w7
    #last bit after last / is STREAM KEY you get from YOUTUBE renews 7 days


if (stream==False):
    camera=PiCamera()
    #max res 3240, 2464
    camera.sensor_mode =(4)
    camera.resolution =(1640, 1232)
    #camera.resolution=(2592, 1944)
    camera.image_effect='denoise'
    camera.drc_strength='high'
    camera.video_stabilization ='True'
    camera.framerate= 23.976
    camera.awb_mode = 'tungsten'# 'auto"
    #fluorescent
    camera.color_effects= (128,128) # BLACK AND WHITE
    camera.annotate_text_size=80
    #camera.annotate_text= "HELLO WORLD!"
    #save_path = '/home/pi/Videos'
    #max stills resolution 3280x2464
    camera.start_preview()
    time.sleep(2)
    wb= camera.awb_gains
    camera.awb_mode= 'off'
    camera.awb_gains =wb
    

if stream==False:
    #camera.start_recording('/home/pi/Videos/robotcam %s.h264' % date_string)
    camera.start_recording('/media/pi/MEDIA/robotvid/robotcam %s.h264' % date_string)

    camera.annotate_text= "HELLO WORLD!      record event: Robotcam %s"   % date_string     

    time.sleep(2)
    
    camera.annotate_text= "record event: Robotcam %s"   % date_string     




ser = serial.Serial(port='/dev/ttyS0',baudrate = 9600,parity = serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)




# Create a VL53L0X object for device on TCA9548A bus 1
tofRight1 = VL53L0X.VL53L0X(TCA9548A_Num=1, TCA9548A_Addr=0x70)#RIGHT
# Create a VL53L0X object for device on TCA9548A bus 2
tofLeft2 = VL53L0X.VL53L0X(TCA9548A_Num=2, TCA9548A_Addr=0x70)# LEFT
# Create a VL53L0X object for device on TCA9548A bus 3
tofCliff3 = VL53L0X.VL53L0X(TCA9548A_Num=3, TCA9548A_Addr=0x70)
# Create a VL53L0X object for device on TCA9548A bus 4
tofTopGuard = VL53L0X.VL53L0X(TCA9548A_Num=4, TCA9548A_Addr=0x70)

def act_lasers():
    if stream==False:
        camera.annotate_text= "Activating multiplexed lasers!"
    # Start ranging on TCA9548A bus 1
    tofRight1.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)
    # Start ranging on TCA9548A bus 2
    tofLeft2.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)
    # Start Ranging on TCA9548A bus 3 (cliff sensor)
    tofCliff3.start_ranging(VL53L0X.VL53L0X_BEST_ACCURACY_MODE)
    #Start Ranging on TCA9548A bus 4 (Top sensor)
    tofTopGuard.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)
    
    #CHOICES include:
    #VL53L0X_GOOD_ACCURACY_MODE
        #perhaps default? 1.2m 30ms 4%-12% depending on lighting and surface
        #accuracy same as long range
    
    #VL53L0X_BETTER_ACCURACY_MODE
    
    #VL53L0X_BEST_ACCURACY_MODE
        #200ms 1.2M +-3% accuracy
    
    #VL53L0X_LONG_RANGE_MODE
        #33ms no IR conditions
    
    #VL53L0X_HIGH_SPEED_MODE
        # 20ms
    #range is from 50mm to 1200mm or 2 inches to 47 inches (4 ft) 

    #note in sunny conditions accruracy can be as bad as 2x worse than dark


#LIST OF DEFINITIONS

#emergency_stop=int (1)
    
TopGuard_nominal= (700)

reverse_course= False

party_counter=(0) # increment breaks between recording

move_counter= (0) # counting movement

step_wait= (.01)

angle =(0)

decel_point=(1)

collision_warning= False

best_sonic_inches= round(float(0),2)#initial value for best ultrasonicdistance

sonic_inches = round (float(0),2) #initail value for ultrasonic

step_pos=int(0) #initial stepper position

best_step_goal= (0)

print(("step_pos" + str(step_pos)))

steps_in_motor= int(200*5.40) #steps in motor*gearbox=1080 or 3 steps per degree of 360
print(("\nsteps_in_motor" + str (steps_in_motor)))

coarse_scan_angle=int(15) # degree angle of coarse scan
print(("\ncoarse_scan_angle;" + str(coarse_scan_angle)))

coarse_scan_value= float(coarse_scan_angle*steps_in_motor/360.0)# carry decimal for accumulating errors
print(("\ncoarse_scan_value" +str(coarse_scan_value)))

SqRt3o2 = math.sqrt(3.0/2.0) # do math here so it doesnt calc square every loop

speed=int(5) #this affects for loop in acceleration.
# 0-6 means 5 steps accel/decel


scan_var= int(254) #mm of +/- allowable variation of 2 scans

cliff_nominal=int(710) #square root of(floor height  squared*2)in mm


hall= int(25)   # hall effect sensor pin number


fine_scan_steps= int(6) # ach 3 steps = 1.00 degrees
print(("\nFine Scan STEPS:" + str (fine_scan_steps)))

 ## max min values for DOF sensor##
max_pitch= (0.0)
min_pitch=(0.0)
    
max_roll=(0.0)
min_roll=(0.0)
    


Tokyo_drift_angle = int(0) #angle off main bearing to drift to avoid collision


##########read lasers #######
def read_lasers():
    global distance_L
    global distance_R
    global distance_Cliff
    global distance_TopGuard
    
    distance_R = tofRight1.get_distance()
    if (distance_R > 0):
        print(("1: %d mm, %d cm" % (distance_R, (distance_R/10))))

    # Get distance from VL53L0X  on TCA9548A bus 2
    distance_L = tofLeft2.get_distance()
    if (distance_L > 0):
        print(("2: %d mm, %d cm" % (distance_L, (distance_L/10))))
    # get distance from VL53L0X on TCA9548A bus 3 (Cliff sensor)
    distance_Cliff = tofCliff3.get_distance()
    if (distance_Cliff > 0):
        print(("cliff: %d mm, %d cm" % (distance_Cliff, (distance_Cliff/10))))
    distance_TopGuard= tofTopGuard.get_distance()
    if (distance_TopGuard > 0):
        print(("TopGuard: %d mm, %d cm" % (distance_TopGuard, (distance_TopGuard/10))))
    
def deactivate_lasers():
    tofRight1.stop_ranging()
    tofLeft2.stop_ranging()
    tofCliff3.stop_ranging()
    tofTopGuard.stop_ranging()
    
# 27:1 motot R G Y B  Normal motor B G R B
#pin setup
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

#GPIO for easyDriver stepper driver
gpio.setup(23, gpio.OUT)# direction lower stepper motor
gpio.setup(22, gpio.OUT)# directiion upper stepper motor
gpio.setup(24, gpio.OUT) #step
gpio.setup(5,gpio.OUT) #MS1 blue wire
gpio.setup(6, gpio.OUT) #MS2 purple wire
gpio.setup(13, gpio.OUT) #sleep brown wire

#GPIO hall sensor
#connected to 3V out!
gpio.setup(hall, gpio.IN) # hall effect sensor
#if High then no magnet


#stepper definitions
def stepper_run(): #execute one step
    global step_wait
    
    gpio.output(24,0)
    time.sleep(step_wait)
    gpio.output(24,1)
    time.sleep(0.001)
    
def stepper_run_fast():
    gpio.output(24,0)
    time.sleep(0.002)
    gpio.output(24,1)
    time.sleep(0.002)
    
def stepper_run_slow():
    gpio.output(24,0)
    time.sleep(0.01)
    gpio.output(24,1)
    time.sleep(0.01)
    

def set_cw():
    gpio.output(23,1)
    gpio.output(22,0)
    
def set_ccw():
    gpio.output(23,0)
    gpio.output(22,1)

def ms_steps():
    gpio.output(5,0)
    gpio.output(6,0)
    
def stepper_sleep():
    time.sleep(1)#keep stepper from drifting if coming off move
    gpio.output(13,0)

def stepper_wake():
    gpio.output(13,1)

def ultrasonic_scan(): #With error checking  Note: "angle" determined here
    
    global best_sonic_inches
    global angle
    global best_step_goal
    global sonic_inches
    global distance_L
    global distance_R
    global distance_Cliff
    global cliff_nominal
    qc= (False) #quality control for 2 scans
    read_lasers()
    print(("Right: %d mm, %d cm" % (distance_R, (distance_R/10))))
    print(("Left: %d mm, %d cm" % (distance_L, (distance_L/10))))
    print(("Cliff: %d mm, %d cm" % (distance_Cliff, (distance_Cliff/10))))

    #if the side lasers are not too close to something commence sonic scan
    if (distance_L >150) and (distance_R>150) and (distance_Cliff >(cliff_nominal-110)) and (distance_Cliff< (cliff_nominal +110)) and distance_TopGuard> (TopGuard_nominal):
        
        while qc== (False):
            #delay before first scan
            time.sleep(0.010)

            
            #first scan
            x= ser.read(6) #read 6 bytes  sometimes it doesnt hit R
            print("RAW ASCII VALUE")
            ser.flushInput() #reset to collect more data
            time.sleep(.001)
            #print((x, x[0], ord(x[0])))# "mm")
            print(x)
            print ("decoded ")
            print (x.decode ('ascii'))            
            #THIS GETS INT FROM BYTE
            if (x[0]==82):# is unicode for R the first of the numbers- encoded correctly
                print ("\nit works!\n")
                x= (x[1:5])
                print (x.decode ('ascii'))
                
            else:
                ser.flushInput()
                print("\nRESET BUFFER!! RESET BUFFER!! RESET BUFFER!!\n")
                if (x[1]==82):
                    print ("\n one off")
                    x= (x[2:])
                    print (x.decode ('ascii'))
                                 
                elif (x[2]==82):
                    print ("\n two off")
                    x= (x[3:]+x[0:1])
                    print (x.decode ('ascii'))
                    
                elif (x[3]==82):
                    print ("\n three off")
                    x= (x[4:]+x[:2])
                    print (x.decode ('ascii'))
                    
                elif (x[4]==82):
                    print ("\n four off")
                    x= (x[5:]+x[:3])
                    print (x.decode ('ascii'))
                    
                elif (x[5]==82):
                    print ("\n five off")
                    x= (x[:4])
                    print (x.decode ('ascii'))
                    
            #delay between scans
            time.sleep(0.100)

            #second scan
            y= ser.read(6) #read 6 bytes  sometimes it doesnt hit R 
            print("RAW ASCII VALUE")
            ser.flushInput() #reset to collect more data
            time.sleep(.001)
            print(y)
            print ("decoded ")
            print (y.decode ('ascii'))
                            
            #THIS GETS INT FROM BYTE
            if (y[0]==82):# is R the first of the numbers- encoded correctly
                y= (y[1:5])
                print (y.decode ('ascii'))
                #time.sleep (1)
                #break

            else:
                ser.flushInput()
                print("\nRESET BUFFER!! RESET BUFFER!! RESET BUFFER!!\n")
                if (y[1]==82):
                    y= (y[2:])#from 2 to end
                    print (y.decode ('ascii'))
                            
                elif (y[2]==82):
                    y= (y[3:]+y[0:1])
                    print (y.decode ('ascii'))
                          
                elif (y[3]==82):
                    y= (y[4:]+y[:2])
                    print (y.decode ('ascii'))
                        
                elif (y[4]==82):
                    y= (y[5:]+y[:3])
                    print (y.decode ('ascii'))
                        
                elif (y[5]==82):
                    y= (y[:4])
                    print (y.decode ('ascii'))
            

            #compare 2 scans
            #convert byte string to int
                    
            X= int(x.decode ('utf-8'))
            Y= int(y.decode ('utf-8'))
            print ("X int value")
            print (X)
            print ("Y int value")
            print(Y)
            if (X<(Y+scan_var)) and (X> (Y-scan_var)):
                #is x both less than 1 scan_var lower and less than 1 scan var higher
                qc=(True)
                print ("INSIDE TOLERANCE!")
            else:
                print ("OUTSIDE TOLERANCE! OUTSIDE TOLERANCE! RE-INITIATE!")
            print ("PAIR OF SCANS COMPLETE")
            #print ("distance (in mm) 1:", +str (x), "Distance (in mm) 2:", +str (y))


        sonic_inches = round(((X+Y)/2)*.03937,2) #convert to inches round to 2 decimal places
        print(("in feet:",  round((sonic_inches/12),2)))
        #sonic_inches = round(y*0.3937,2)
        print(("average sonic inches:" , sonic_inches))
        
        ## here if best dist angle gets assigned
        if (sonic_inches>=best_sonic_inches):
            best_sonic_inches=sonic_inches
            angle = round ((step_pos*360.0)/steps_in_motor)
            best_step_goal=step_pos
            #print("Range Distance:", R_dist_inches)
        print(("average sonic inches:" , sonic_inches))
    #if (sonic_inches>=best_sonic_inches) and ((distance_L< 175) or (distance_R <175)):
        #camera.annotate_text= " best range Abort: Range: %s Best Range:%s" %(sonic_inches, best_sonic_inches) 
        
                


    
def ultrasonic_scan_lite():#fast scan w no error checking
    #while True:
    global best_sonic_inches
    global angle
    global best_step_goal
    global sonic_inches
    y= ser.read(6) #read 6 bytes  sometimes it doesnt hit R
    print("RAW ASCII VALUE")
    ser.flushInput() #reset to collect more data
    time.sleep(.001)
    print(y.decode('ascii'))


    if (y[0]==82):# is R the first of the numbers- encoded correctly
        y= (y[1:5])
        print (y.decode ('ascii'))
                
    else:
        if (y[1]==82):
            y= (y[2:])#from 2 to end
            print (y.decode ('ascii'))
                        
        elif (y[2]==82):
            y= (y[3:]+y[0:1])
            print (y.decode ('ascii'))
                      
        elif (y[3]==82):
            y= (y[4:]+y[:2])
            print (y.decode ('ascii'))
                    
        elif (y[4]==82):
            y= (y[5:]+y[:3])
            print (y.decode ('ascii'))
                    
        elif (y[5]==82):
            y= (y[:4])
            print (y.decode ('ascii'))

    
    Y= int(y.decode ('utf-8'))    
    print ("Y int value")
    print(Y)
    sonic_inches = round(Y*.03937,2) #convert to inches round to 2 decimal places
    print(("Distance in inches:", sonic_inches))
    #if (sonic_inches>=best_sonic_inches):
        #angle = round ((step_pos*360.0)/steps_in_motor)
        #best_step_goal=step_pos
        #print("Range Distance:", R_dist_inches)
    #print("Best result:", best_sonic_inches, angle)     
        #time.sleep(30)
    
def command_step_move():
    stepper_wake() #turn on easy driver
    time.sleep(0.01)
    
    global step_move
    global step_pos
    
    step_move=int((step_goal)-step_pos) 
    if (step_move <0):
        set_ccw()
        step =-1
    if (step_move>0):
        set_cw()
        step=1

    if step_move==(0):
        step=0
    
    step_move_mod= abs((step_move)- (step*10)-10)
    # abs value for first 10 step slow step
    
    #step motor to position
    while step_pos != int(step_goal):
        if abs(step_pos-step_goal) < 10 or abs(step_goal-step_pos)> (step_move_mod):
            # slow steps first 10 and last 10 steps
            stepper_run_slow()# do ramping here?
        else:
            stepper_run()
        step_pos+=step
        #print (step_pos)
        time.sleep(0.001)
    print(((step_pos), (steps_in_motor), 360))
    print(("STEPPER IN POSITION ", round((step_pos*360.0)/steps_in_motor,2), "DEGREES"))
    

            

        
def zero_calibration():
    print ("zero calibration hall test")
    camera.annotate_text= " Zero Calibration Routine"                

    stepper_wake() 
    time.sleep(0.1)
    set_ccw()
    z=int(0)#interval counter to make sure it doesnt go over 360 degrees
    while (z<(steps_in_motor) and (gpio.input(hall) ==1)):# total steps in 360
        z=(z+1)
        stepper_run()
    
    if (gpio.input(hall) ==0):
        print ("MAGNET DETECT!")
        print ("This is set for default zero calibration point.")
        global step_pos
        step_pos=int(0)
    else:
        print ("SOMETHING WENT WRONG NO HALL SENSOR FOUND. PLEASE FIX.")
        stepper_sleep()
        gpio.cleanup()
        sys.exit()
        
            

#/STATIC RANGE TESTING/ TROUBLESHOOTING & CALIBRATING
def sensor_Test():
    for sensor_test in range(2000):
        ultrasonic_scan()
        #print("ultrasonic distance" + str (sonic_inches))
       
        #Get distance from VL53L0X  on TCA9548A bus 1
        distance = tofRight1.get_distance()
        #if (distance > 0):
            #print ("1: %d mm, %d cm" % (distance, (distance/10)))

        # Get distance from VL53L0X  on TCA9548A bus 2
        distance = tofLeft2.get_distance()
        #if (distance > 0):
            #print ("2: %d mm, %d cm" % (distance, (distance/10)))
        # Get distance from VL53L0X  on TCA9548A bus 3
        distance = tofCliff3.get_distance()
        print(("cliff: %d mm, %d cm" % (distance, (distance/10))))
        #if (distance > 310):
            #print ("CLIFF!!!!!AGHHHHHH!")
        distance = tofTopGuard.get_distance()
        print(("TopGuard: %d mm, %d cm" % (distance, (distance/10))))
        time.sleep(0.266)

    
def coarse_scan():
    global step_goal
    global best_step_goal
    scan_interval_count=(0)
    accumulated_coarse_value=float(0)
    zero_calibration() #can remove this if you call for zero calibration before 
    global best_sonic_inches
    best_sonic_inches=(0)
    global sonic_inches
    
    
    
    step_goal=round(coarse_scan_value)
    
    #scan at initial zero value
    print(("\nstep goal value" + str(step_goal)))
    
    print ("\nFIRST SCAN COMMENCING")
    print(("\nSCAN ANGLE:" + str ((step_goal))))#*360/steps_in_motor)))
    ultrasonic_scan()
    DOF9sensor_read()
    
    
    while (step_goal <steps_in_motor):#1080 is total steps available plus 100 multiplier (5.40*200)
        print("\nULTRASONIC SCAN")  
    
        #coarse scan intervals past zero begin:

        if (scan_interval_count >0):
            step_goal= (step_goal+(accumulated_coarse_value/scan_interval_count))
        
        scan_interval_count+=1
        
        command_step_move()
           
        accumulated_coarse_value=float(accumulated_coarse_value + coarse_scan_value)
    
        print(("\nstep goal value" + str(step_goal)))
        
        #print ("\nSCANNING COMMENCING")
        print(("\nSCAN ANGLE:" + str ((step_goal*360/steps_in_motor))))
        
        
        ultrasonic_scan()
        if stream==False:
            camera.annotate_text= "Angle:%s Range:%s inches Best Range%s Best Angle:%s " %( (step_goal*360/steps_in_motor),sonic_inches,best_sonic_inches,(best_step_goal*360/steps_in_motor)) 
        
                
        #time.sleep(0.1)
        
    
def fine_scan():
    #begin fine scan
    global best_sonic_inches
    global step_goal
    global best_step_goal
    global step_move
    if stream==False:
        camera.annotate_text="Fine Scan begin"
    print("FINE SCAN BEGIN")
    print(("best Step Goal: " , best_step_goal, angle))
    print(("current step pos:", step_pos))
    print(("current angle:" , (step_pos*360/steps_in_motor)))
    #time.sleep(20)
    #first move 1 additional coarse scan arc, and set end point
    #to backwards two coarse scan arcs.
    #scan_interval_count=(0)
    #step_goal=angle #angle from coarse scan 
    step_move=int((best_step_goal+coarse_scan_value)-step_pos)
    print(("best step goal vs step move", best_step_goal, step_move))
    #move back to best step move plus 1 coarse scan arc.
    print(("last good angle:", angle, "step move:", step_move))
    print(("best step goal& coarse scan val; ", best_step_goal, coarse_scan_value))

    fine_scan_stop= int(best_step_goal-(coarse_scan_value)) #pass through 2 coarse arc
    fine_scan_start=int(best_step_goal+coarse_scan_value)
    print(("fine_scn_stop" + str (fine_scan_stop)))
    print("stepping to start of fine step arc")
    print(("begin/middle/end", int(fine_scan_start), best_step_goal, int(fine_scan_stop)))
    step_goal=fine_scan_start
    #time.sleep(0.01)
    command_step_move()
    


    print("Fine scan begins")
    #camera.annotate_text= "FINE SCAN BEGIN"
    while (step_goal>fine_scan_stop):
        step_goal= int((step_pos)-fine_scan_steps)
        print(("\nstep goal: " +str(step_goal)))  
        print(("current step position " +str(step_pos)))
        time.sleep(0.01)
        command_step_move()
        
        print("step to fine step scan")
            

        print(("STEPPER IN POSITION "+ str(step_pos*360/steps_in_motor)))
        
        print ("\nFine Mode ultrasonic scan now!")
        ultrasonic_scan()
        if stream==False:
            camera.annotate_text= "Angle:%s Range:%s inches Best Range%s Best Angle:%s " %( (step_goal*360/steps_in_motor),sonic_inches,best_sonic_inches,(best_step_goal*360/steps_in_motor)) 
        
        time.sleep(0.01)
        

    #STEP TO FINAL BEST ANGLE BEFORE MOVING
    print(" Step to final best angle")
    time.sleep(.1)
    step_move=((best_step_goal)-step_pos)
    #NOTE BEST STEP GOAL NOT STEP GOAL
    print(("\nstepper step move " + str(step_move)))
    print(("\nstep goal value" + str(best_step_goal)))
    step_goal=best_step_goal
    print("step to final target angle")
    command_step_move()
    
            

    print(("STEPPER IN POSITION "+ str(step_pos*360/steps_in_motor)))
    print ("clearing best sonic inches:")
    best_sonic_inches=(0)
    #stepper_sleep()#disable stepper so it doesnt consume power 

def DOF9sensor_read():
    ##because 9DOF placement,9DOFy=x,left/Right 9DOFz =y,forward/backward 9DOFx= Z,up down
    #global accel_x, accel_y, accel_z
    global max_pitch
    global min_pitch
    global max_roll
    global min_roll
    global collision_warning
    global pitch
    global roll
    
    #definitions for orientation of sensor
    accel_x, accel_y, accel_z = DOF9sensor.acceleration# 9DOFX=z axis 9DOFY= x axis 9DOFZ= y axis
    mag_x, mag_y, mag_z =DOF9sensor.magnetic
    gyro_x, gyro_y, gyro_z = DOF9sensor.gyro

    ## sensor as marked
##    pitch= math.atan2(accel_y, accel_z)*57.3
##    roll= math.atan2 ((-accel_x), math.sqrt (accel_y*accel_y +accel_z*accel_z))*57.3

    
    #definitions for orientation of robot
##    accel_sideways, accel_down, accel_forward = DOF9sensor.acceleration
##    gyro_pitch, gyro_yaw, gyro_roll = DOF9sensor.magnetic
##    mag_eastwest, mag_updown, mag_northsouth, =DOF9sensor.magnetic
    accel_sidewaysX, accel_downZ, accel_forwardY = DOF9sensor.acceleration

    temp =DOF9sensor.temperature
    
     #pitch negative means nose down
   
    pitch= math.atan2(accel_forwardY, -(accel_downZ))*57.3# (-) on accel_downZ is beacuse of orientation of sensor

    #roll negative means roll right
    roll= math.atan2 ((-accel_sidewaysX), math.sqrt (accel_forwardY*accel_forwardY +(-(accel_downZ)*-(accel_downZ))))*57.3
    #print ("PITCH:", pitch)
    #print ("ROLL:", roll)
    print ("PITCH:", pitch)
    print ("ROLL:", roll)
##    heading= 180* math.atan2(mag_eastwest, mag_northsouth)/SqRt3o2
##    if (heading<0):
##        heading+=360
##    print("heading", heading)
##   
##    print("acceleration forward, sideways, down")
##    print('Acceleration (m/s sq): ({0:0.3f},{1:0.3f}, {2:0.3f})'.format(accel_forward, accel_sideways, accel_down))
##
##    print("gyro roll, gyro pitch, gyro yaw:")
##    print('Gyro (degrees/sec): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(gyro_roll, gyro_pitch, gyro_yaw))
##
##    print ("magnetometer eastwest, updown, northsouth")
##    print('Magnetometer (gauss): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_eastwest, mag_updown,mag_northsouth))
    print('temperature: {0:0.3f}C'.format(temp))


   

    if max_pitch< pitch:
        max_pitch=pitch
        
    if min_pitch> pitch:
        min_pitch=pitch

    if max_roll<roll:
        max_roll=roll
        
        
    if min_roll>roll:
        min_roll=roll

    print( "MAX PITCH",max_pitch)
    print ("MIN PITCH:", min_pitch)
    print ("MAX ROLL RIGHT:", min_roll)
    print ("MAX ROLL LEFT:", max_roll)
    
    ## maybe move this to nav_rules() because collision_warning gets zeroed at head of nav rules its nested in nav rules
    if roll > 14 or roll <-14:
        print ("roll TRIGGER!")
        collision_warning= True
    if pitch > 14 or pitch < -14:
        print ("PITCH TRIGGER!")
        collision_warning=True
    
    


        
    

def working_angle_calc(): #calculates angle after nav rules adjustments
    
    global working_angle
    #working_angle to fix past 360 or 0 calculations
    #working_angle only a local variable
    working_angle=int(angle+Tokyo_drift_angle+30)
    #30 is error correct for mk2 wheel position
    if working_angle >360:
        working_angle=(working_angle-360)
    if working_angle <1:
        working_angle = working_angle+360
    print(("working angle is:" + str (working_angle)))


##For old Adafruit MOTOR HAT LIBRARY)
def motor_dir():# look into release maybe not because then wheel locked?
    global working_angle
    # Motor Clockwise/countrer clockwise based on wheel value
    if ((MotorOneAltSpeeds[working_angle-1])>0):
        (motorOne.run(Adafruit_MotorHAT.FORWARD))
        print ("\nWheel_1 FORWARD")
    elif((MotorOneAltSpeeds[working_angle-1])<0):
        (motorOne.run(Adafruit_MotorHAT.BACKWARD))
        print ("\nWheel_1 BACKWARD")
    elif((MotorOneAltSpeeds[working_angle-1])==0):
        (motorOne.run(Adafruit_MotorHAT.RELEASE))
        print ("\nWheel_1 RELEASE")
         
    if((MotorTwoAltSpeeds[working_angle-1])>0):
        (motorTwo.run(Adafruit_MotorHAT.FORWARD))
        print ("Wheel_2 FORWARD")
    elif ((MotorTwoAltSpeeds[working_angle-1])<0):
        (motorTwo.run(Adafruit_MotorHAT.BACKWARD))
        print ("Wheel_2 BACKWARD")
    elif ((MotorTwoAltSpeeds[working_angle-1])==0):
        (motorTwo.run(Adafruit_MotorHAT.RELEASE))
        print ("Wheel_2 RELEASE")
     
    if ((MotorThreeAltSpeeds[working_angle-1])>0):
        (motorThree.run(Adafruit_MotorHAT.FORWARD))
        print ("Motor_3 FORWARD")
    elif ((MotorThreeAltSpeeds[working_angle-1])<0):
        (motorThree.run(Adafruit_MotorHAT.BACKWARD))
        print ("Motor_3 BACKWARD")
    elif ((MotorThreeAltSpeeds[working_angle-1])==0):
        (motorThree.run(Adafruit_MotorHAT.RELEASE))
        print("Motor_3 RELEASE")


def nav_rules():
    global cliff_nominal
    global TopGuard_nominal
    global distance_R
    global distance_L
    global distance_Cliff
    global distance_TopGuard
    global Tokyo_drift_angle
    global collision_warning
    global sonic_inches
    global decel_point
    global reverse_course
    global accel_x, accel_y, accel_z
    
    collision_warning=False
    
    distance_Cliff_old=distance_Cliff
    read_lasers()
    ultrasonic_scan_lite()
    
####camera analog gain as sensor tweaker##
    cam_analog_value= (float(Fraction(camera.analog_gain)))
    print(("Cam_analog_value:" + str (cam_analog_value)))
    if cam_analog_value <= 1.0:
        print ("analag gain at UNITY: sensor slip activated")
        sensor_slip= (40) #fudge factor for ir lasers in high IR environmnent
    else:
        sensor_slip= (0)
        print ("sensor_slip=0")
####
##       Accelerometer reading
    DOF9sensor_read()
    
    
##### clif variance for carpet

    if distance_Cliff> (distance_Cliff_old+50):# bigger than old
        print ("CLIFF DIFFERENTIAL BIGGER")
        collision_warning=True
        

    elif distance_Cliff< (distance_Cliff_old-50):#smaller than old
        print ("CLIFF DIFFERENTIAL SMALLER")
        collision_warning=True
    if sonic_inches < ((decel_point*2)+14): # from 16' to 31"? collision 
        collision_warning=True
        print(("Sonic Distance to Collision " + str(int(decel_point*2)+14)))
        if stream==False:
            camera.annotate_text="SONIC COLLISION WARNING"
    ####RIGHT TURN TOKYO DRIFT CALC#######
    if distance_R < distance_L :#and (abs(Tokyo_drift_angle)<100):
    #distance_R = tofRight1.get_distance()
    #print ("right: %d mm, %d cm" % (distance_R, (distance_R/10)))
        if (distance_R < 700 and distance_R >600):# 700mm about 28" 600mm 24" range 50-1200
            #print ("right: %d mm, %d cm" % (distance_R, (distance_R/10)))
            print ("TOKYO DRIFT LEFT15!")
            Tokyo_drift_angle= int(-15)
        elif (distance_R <=600 and distance_R >500):# 700mm about 24" 600mm to 20" 24" range 50-1200
            #print ("right: %d mm, %d cm" % (distance_R, (distance_R/10)))
            print ("TOKYO DRIFT LEFT30!")
            Tokyo_drift_angle= int(-30)
        elif (distance_R <=500):# 700mm about 28" 600mm 24" range 50-1200
            #print ("right: %d mm, %d cm" % (distance_R, (distance_R/10)))
            print ("TOKYO DRIFT LEFT45!")
            Tokyo_drift_angle= int(-45)
        elif (distance_L <=300):
            print ("TOKYO DRIFT RIGHT 60")
            Tokyo_drift_angle= int(60)
        
    
        
    #elif(abs(Tokyo_drift_angle)<100):
    else:
       
        if (distance_L < 700 and distance_L > 600):# 15 degree 
            #print ("left: %d mm, %d cm" % (distance_L, (distance_L/10)))
            print ("TOKYO DRIFT RIGHT 15!")
            Tokyo_drift_angle = int(15)
        
        elif (distance_L <=600 and distance_L > 500):# 15 degree 
            #print ("left: %d mm, %d cm" % (distance_L, (distance_L/10)))
            print ("TOKYO DRIFT RIGHT 30!")
            Tokyo_drift_angle = int(30)
        
        elif (distance_L <=500):# 45 degree 
            #print ("left: %d mm, %d cm" % (distance_L, (distance_L/10)))
            print ("TOKYO DRIFT RIGHT 45!")
            Tokyo_drift_angle = int(45)
        elif (distance_L <=300):
            print ("TOKYO DRIFT RIGHT 60")
            Tokyo_drift_angle= int(60)

##        #CLIFF SENSING###    
##        if (distance_Cliff< (cliff_nominal-100)):
##            print ("CLIFF/CARPET DANGER CLOSE +100")
##            #if (decel_point>0):
##                #decel_point= (decel_point-1)
##            
##            if (decel_point<3):# so not full speed ahead and reverse
##                Tokyo_drift_angle= int (-150)
##            else:
##                Tokyo_drift_angle= int (-60)
##            
##            collision_warning=True
##            
##        elif (distance_Cliff> (cliff_nominal+150)): #need 8190 cutoff?
##            print ("CLIFF AHEAD! ")
##            Tokyo_drift_angle = int (180)
##            collision_warning=True

        
   
    #collision warning side sensors turn on
    if (distance_R <140) or (distance_L< 140):
        collision_warning=True
    #GO STRAIGHT 
        
    if ((distance_L)>750) and ((distance_R)> 750) and sonic_inches> (20) and (Tokyo_drift_angle != -180):
        Tokyo_drift_angle=int(0)
    #CLIFF SENSORS
    if (distance_Cliff< (cliff_nominal-100)):
        print ("CLIFF/CARPET DANGER CLOSE-100")
        collion_warning=True
        #if (decel_point>0):#speed up the decel
            #decel_point= (decel_point-1)
    if (distance_TopGuard < (TopGuard_nominal)):
        print ("TOPGUARD TRIGGER")
        collision_warning=True

        if (decel_point<3) :# so not full speed ahead and reverse
            Tokyo_drift_angle= int (-180)
        #else:
            #Tokyo_drift_angle= int (-60)
            
    elif (distance_Cliff> (cliff_nominal+150)):#need 8180 cutoff?
        print ("CLIFF AHEAD! ")
        Tokyo_drift_angle = int (-180)
        collision_warning=True
        reverse_course=True

    if stream==False:
        camera.annotate_text= "Range: %s inches Left:%s mm Right:%s mm Cliff:%s mm course correct: %s" %(sonic_inches, distance_L, distance_R, distance_Cliff, Tokyo_drift_angle) 
        
def turnOffMotors(): #all motors inluding stepper
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
    stepper_sleep()

def accelerate():
    global move_counter
    global decel_point
    global cliff_nominal
    global collision_warning
    global emergency_stop
    global decel_point
    #global working_angle #only need this global for printing
    ultrasonic_scan_lite()
    if (sonic_inches>0):
        print ("ACCELRATE!")
        for i in range(speed):
            ii=float(i)
            print ("accel mode")
            print("Working Angle:", (working_angle))
            decel_point= i
            move_counter= (move_counter+ decel_point)
            ultrasonic_scan_lite()
            nav_rules() #collision avoidance/steering and direction of wheels
            working_angle_calc()# direction of wheels and working angle
            
## OLD LIBRARY COMMANDS
            motorOne.setSpeed (int (round(abs(MotorOneAltSpeeds[working_angle-1])*(ii/speed))))
            motorTwo.setSpeed (int(round(abs(MotorTwoAltSpeeds[working_angle-1])*(ii/speed))))
            motorThree.setSpeed (int(round(abs(MotorThreeAltSpeeds[working_angle-1])*(ii/speed))))
            motor_dir()    
            print (i)
            print(("MOVE COUNTER;" + str (move_counter)))
            if collision_warning== True:
                break


def decelerate():
     #Decelerate!
    global decel_point
    global emergency_stop
    global working_angle #only global for printing
    ultrasonic_scan_lite()
    if (sonic_inches>0):
        
        print ("DECELERATE triggered!") #to stop
        for i in reversed(list(range(decel_point))):
            print ("decel mode")
            print("Working Angle:", (working_angle))
            ii=float(i)
            print ("decel i number:")
            print (i)

            ultrasonic_scan_lite()
            nav_rules()
            working_angle_calc()
           
## OLD LIBRARY COMMANDS
            motorOne.setSpeed (int(round(abs(MotorOneAltSpeeds[working_angle-1])*(ii/speed))))
            motorTwo.setSpeed (int(round(abs(MotorTwoAltSpeeds[working_angle-1])*(ii/speed))))
            motorThree.setSpeed (int(round(abs(MotorThreeAltSpeeds[working_angle-1])*(ii/speed))))

            motor_dir()
            time.sleep(0.001)
              
        print("Release")
        turnOffMotors()
        
#######BEGIN MAIN LOOP#######
act_lasers()
atexit.register(turnOffMotors)
atexit.register(stepper_sleep)


##########BEGIN RECORDING######

#camera.start_recording('/home/pi/Videos/robotcam.h264')#, resize=(960, 720))
#camera.start_recording('/home/pi/Videos/robotcam.h264', resize= (1920, 1080)
#time.sleep(2)

######## make motor speed database for all degrees 
MotorOneSpeeds=[]
MotorOneAltSpeeds=[]
MotorTwoSpeeds=[]
MotorTwoAltSpeeds=[]
MotorThreeSpeeds=[]
MotorThreeAltSpeeds=[]
global wheel_1
global wheel_2
global wheel_3
global speed_factor
    #working_angle to fix past 360 or 0 calculations
    #working_angle only a local variable
#working_angle=(angle+Tokyo_drift_angle+30)
    #30 is error correct for mk2 wheel position
#if working_angle >360:
    #working_angle=(working_angle-360)
#if working_angle <0:
    #working_angle = working_angle+360
for degreeDB in range (1,361):
    
    print (degreeDB)
    if stream==False:
        camera.annotate_text= "Kiwi 360 drive angle/motor speed table being populated:%s" %degreeDB 
        
                

    #angleRadian = math.radians(degreeDB) #may delete this line
    #print(("\nAngle:" + str (degreeDB)))
    #print(("Radian: " + str (angleRadian)))

    vectorX= math.cos(math.radians(degreeDB))
    vectorY= math.sin(math.radians(degreeDB))

    #vectorX= math.cos(math.radians(degreeDB))
    #vectorY= math.sin(math.radians(degreeDB))
        
    # code to zero out very low numbers on X&Y axis
    if vectorX < 0.001 and vectorX > -0.001:
        vectorX=0.0
    if vectorY< 0.001 and vectorY> -0.001:
        vectorY=0.0

    #wheel_1=( -(vectorX))
    #wheel_2=(0.5*(vectorX)-SqRt3o2*vectorY)
    #wheel_3=(0.5*(vectorX)+SqRt3o2*vectorY)

    wheel_1=( -(vectorX))
    wheel_2=( 0.5*(vectorX)-SqRt3o2*vectorY)
    wheel_3= ( 0.5*(vectorX)+SqRt3o2*vectorY)
    print("\nVector-X-DB: ", vectorX)
    print("Vector-Y-DB: ", vectorY)

    
    print(("\nVector-X-DB: ", vectorX))
    print(("Vector-Y-DB: ", vectorY))

    #normalize values/
    #keep max value for drive wheels no higher than 1 or less than-1 
    wheel_list=(abs(wheel_1),abs(wheel_2),abs(wheel_3))
    #print(("wheel list:" + str (wheel_list)))
    print ("max wheel list")
    print(max(wheel_list))
    speed_factor= 255/(max(wheel_list))# keep values no higher than 1 or lower-1
    print ("speed factor")
    print(speed_factor)
    #record normalized values in database
    MotorOneSpeeds.append(wheel_1)
    MotorTwoSpeeds.append(wheel_2)
    MotorThreeSpeeds.append(wheel_3)
    MotorOneAltSpeeds.append(wheel_1*speed_factor)
    MotorTwoAltSpeeds.append(wheel_2*speed_factor)
    MotorThreeAltSpeeds.append(wheel_3*speed_factor)
    
    
print("\nWheel One Speeds")
print (MotorOneSpeeds)
print("\nWheel Two Speeds")
print(MotorTwoSpeeds)
print("\nWheel Three Speeds")
print(MotorThreeSpeeds)
print("\nALT SPEEDS")
print (MotorOneAltSpeeds)
print ("ALT SPEED 2")
print (MotorTwoAltSpeeds)
print ("ALT SPEED 3")
print (MotorThreeAltSpeeds)







while True:
    camera.annotate_text= " ACTIVATE LASERS!"
    read_lasers()
    while True:
        sensor_Test()
    

    
    print("ZERO CALIBRATION")
    zero_calibration()
    print(step_pos)
    time.sleep(.5)
    #step_goal=(steps_in_motor)#just to test full 360 capability of motors
    #print("step to 360 degrees")
    #command_step_move()
    #print(step_pos)
    #time.sleep(1)
    
    
            
    
    #COARSE SCAN
    print(("best_sonic_inches", best_sonic_inches))
    print ("BEGIN COURSE SCAN")
    coarse_scan()
    #time.sleep(1)

    #FINE SCAN
    fine_scan()
    time.sleep(0.01)
    
    #CALCULATE WHEEL SPEED AND DIRECTION
    working_angle_calc()
    print ("working angle CALC HAPPENING")
        

        
        
    #BEGIN MOVEMENT

        
    print("\nout of scanning loop:")
    #stepper_sleep()
    time.sleep(1)
    move_loop=(1) #how many times accel/decel loop
    
    accelerate()

    ## reset max/min pitch and roll after accel
    max_pitch=0.0
    min_pitch=0.0
    
    max_roll=0.0
    min_roll=0.0

    #FULL SPEED
        #full speed is 16"/s and full accel decel is 39"
    #print ("FULL SPEED")
    #ultrasonic_scan()
    while(collision_warning ==False):
        print ("CRUISE SPEED")
        move_counter= (move_counter+decel_point)
        print(("MOVE COUNTER:" +str (move_counter)))
        ultrasonic_scan_lite()
        nav_rules()
        working_angle_calc()
       

#OLD LIBRARY COMMANDS
        motorOne.setSpeed (int(round(abs(MotorOneAltSpeeds[working_angle-1]))))
        motorTwo.setSpeed (int(round(abs(MotorTwoAltSpeeds[working_angle-1]))))
        motorThree.setSpeed (int(round(abs(MotorThreeAltSpeeds[working_angle-1]))))
        motor_dir()
        time.sleep(0.001)
        
    print ("DECLERATE IN MAIN LOOP:")

    ## reset max/min pitch and roll after accel
    max_pitch= 0
    min_pitch=0
    
    max_roll=0
    min_roll=0
    
    decelerate()
    time.sleep(.5)

    ####second iteration#####
    while move_counter <300:
        time.sleep(.5)
        move_loop= move_loop+1
        if reverse_course== True:# pick new course 180 from cliff 
            reverse_course= False
            if best_step_goal> 540:
                best_step_goal= best_step_goal-540 # 180 no tangle of cables
            else:
                best_step_goal= best_step_goal+540
        
            #STEP TO FINAL BEST ANGLE BEFORE MOVING
            print(" Step to final best angle")
            time.sleep(.1)
            step_move=((best_step_goal)-step_pos)
            #NOTE BEST STEP GOAL NOT STEP GOAL
            print(("\nstepper step move " + str(step_move)))
            print(("\nstep goal value" + str(best_step_goal)))
            step_goal=best_step_goal
            print("step to final target angle")
            command_step_move()
            best_sonic_inches=(0)
            ultrasonic_scan_lite()
            angle = round ((step_pos*360.0)/steps_in_motor)
            nav_rules()
            working_angle_calc() #calc bearing for motor

    
        ultrasonic_scan_lite()
        print(("LOOP ITERATION")+ str(move_loop))
        if sonic_inches > (25):# original route is clear
            #time.sleep(1)
            accelerate()
            
            while (collision_warning ==False):
                #Cruise/Full speed
                
                print ("CRUISE LOOP SPEED")
                print("Working Angle:", (working_angle))
               
                ultrasonic_scan_lite()
                nav_rules()
                working_angle_calc()
                move_counter= (move_counter+ decel_point)
                print(("MOVE COUNTER" + str (move_counter)))
                #motorOne.setSpeed (int(round(abs(MotorOneAltSpeeds[working_angle-1]))))
                #motorTwo.setSpeed (int(round(abs(MotorTwoAltSpeeds[working_angle-1]))))
                #motorThree.setSpeed (int(round(abs(MotorThreeAltSpeeds[working_angle-1]))))
                #motor_dir()
       
                time.sleep(0.001)
            decelerate()
        else:# plot new route and move that way
            print("PLOT NEW ROUTE!")
            coarse_scan()
            fine_scan()
            working_angle_calc()
            accelerate()
            while (collision_warning ==False):
                #Cruise/Full speed
                
                print ("CRUISE LOOP SPEED") 
               
                ultrasonic_scan_lite()
                nav_rules()
                working_angle_calc()
                move_counter= (move_counter+ decel_point)
                print(("MOVE COUNTER" + str (move_counter)))
                #motorOne.setSpeed (int(round(abs(MotorOneAltSpeeds[working_angle-1]))))
                #motorTwo.setSpeed (int(round(abs(MotorTwoAltSpeeds[working_angle-1]))))
                #motorThree.setSpeed (int(round(abs(MotorThreeAltSpeeds[working_angle-1]))))
                #motor_dir()

               #G detector
                #Xaxis, Yaxis, Zaxis = [value /adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]
                #print("X axis = %0.3f G, Y axis = %0.3f G,z axis = %0.3f G," %(Xaxis, Yaxis, Zaxis))
    
                time.sleep(0.001)
            decelerate()

    
    #go into standby mode
        turnOffMotors()
##        motorOne.run(Adafruit_MotorHAT.RELEASE)
##        motorTwo.run(Adafruit_MotorHAT.RELEASE)
##        motorThree.run(Adafruit_MotorHAT.RELEASE)
##    kit.motor1.throttle = None
##    kit.motor2.throttle = None
##    kit.motor3.throttle = None
    
    stepper_sleep()
    deactivate_lasers()
    if stream== False:
        camera.annotate_text= "ALL HUMANS DIE!"
        time.sleep(4)
        camera.annotate_text= "PLEASE ENJOY SNACKS!"
        time.sleep(4)
        camera.annotate_text= "GOOD BYE. KLAATU BARADA NIKTO."
        time.sleep(3)
        camera.annotate_text= "record sequence shutting down"
        time.sleep(3)
        camera.stop_recording()
        print ("STOP RECORDING")
        
    # sleep for 60 seconds
    stepper_sleep()
    time.sleep(40)#4 minutes
    move_counter=(0)
    party_counter= (party_counter +1)
    if party_counter == (25):# about two hours?
        stepper_sleep()
        time.sleep(1)
        gpio.cleanup()
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        gpio.setup(13, gpio.OUT) #sleep brown wire
        stepper_sleep()
        print(("Party_COUNTER:" +str(party_counter)))
        break
        
    act_lasers()
    stepper_wake()
    
    #sleep for 60 seconds
    if stream==False:
        date_string = time.strftime("%Y-%m-%d-%H-%M")
        print((str(date_string)))
        #camera.start_recording('/home/pi/Videos/robotcam %s.h264' % date_string)
        #camera.start_recording('/home/MEDIA/robotvid/robotcam %s.h264' % date_string)
        camera.start_recording('/media/pi/MEDIA/robotvid/robotcam %s.h264' % date_string)
        camera.annotate_text= "HELLO WORLD"
        time.sleep(2)
        camera.annotate_text= "record:  Unit: %s ! record robotvid %s" %(party_counter, date_string)
        print ("NEW RECORDING")
        
        time.sleep(3)
    
    #go_again= raw_input("\nWould you like to go again? ")
    #if go_again=="n":
         
##        stepper_sleep()
##        time.sleep(1)
##        gpio.cleanup()
##        gpio.setmode(gpio.BCM)
##        gpio.setwarnings(False)
##        gpio.setup(13, gpio.OUT) #sleep brown wire
##        stepper_sleep()
##        print ("MOVE_COUNTER:" +str(move_counter))
##        
##        if stream==False:
##            camera.stop_recording()
##        
##        print("GOOD BYE. KLAATU BARADA NIKTO.")
##        break

