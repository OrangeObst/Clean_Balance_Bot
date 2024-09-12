['''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com


****************************************************************
mpu6050  rpi3
scl to RPiJ8-5
sda to RPiI8-3
GND to GND
VCC to 3.3 volts

check if I2C user port enabled
enter ls /dev/*i2c* in terminal desktop
response should be /dev/i2c-1
is not run raspi configuration
sudo raspi-config in a terminal window
select interfacing options I2C <yes> <ok>
might need reboot
check if I2C user port enabled
enter ls /dev/*i2c* in terminal desktop
response should be /dev/i2c-1
try sudo i2cdetect -y 1
(might need to get i2c tools.
sudo apt-get install -y i2c-tools)
mpu6050 should show up as being device 68
wait at least 30ms, after waking unit, to acquire gyro info




calibrate mpu6050
firmly physically hold mpu6050 from moving
I put mpu6050 on breadboard, I place weights on each side of breadboard. I have breadboard on soild surface
Iike to play music with a heavy beat. during calibration turn music off. sit quietly, avoid low frequency sounds and movements.
run MPU_Cali()
results will print out. enter offset results for each offset.
I found calibration measurement of less than 51 measurements to be innaccurate offsets and
measurements above 300 data points of little gain in accuracy.


idahowalker 16-Apr-2018
************************************************************************************
'''
import smbus			#import SMBus module of I2C
import time
import math
import datetime

#some MPU6050 Registers and their Address
'''
PWR_MGMT_1 address Hex=0x6Bb, Decimal=107
bit7         bit6   bit5  bit4 bit3     bit2,1,0
DEVICE_RESET SLEEP  CYCLE   x  TEMP_DIS CLKSEL (2-0)

bit6 to 1 = low power sleep mode
bit5 = 1 with SLEEP 0 unit cycles sleep n wake to take single sample readings
       at a rate determined by register 108
bit3 = 1 to disable temperature sensor
bit2,1,0 sets clock source default 0 = 8Mhz
'''
PWR_MGMT_1   = 0x6B

'''
bit7, bit6        bit5    bit4    bit3    bit2    bit1    bit0
LP_WAKE_CTRL(1:0) STBY_XA STBY_YA STBY_ZA STBY_XG STBY_YG STBY_ZG

LP WAKE UP

0 1.25HZ
1 5HZ
2 20HZ
4 40HZ

SET OTHER BITS TO PUT PARTS OF UNIT INTO STSNDBY MODE

'''
##PWR_MGMT_2   = 0x6C


'''
SMPLRT_DIV
Sample Rate Divider
8 bit value determined by
SampleRate=Gryoscope Output Rate/(1+SMPLRT_DIV)
Gyroscope Output Rate=8Khz default
see DLPF_CFG register (26) to change)
SMPLRT_DIV = 8 bit value
'''
SMPLRT_DIV   = 0x19
'''
CONFIG (26)
Configures Frame Synchronization (FSYNC) pin and
Digital Low Pass Filter (DLPF) for gyros and accelerometers
bit2,1,0 DLPF_CONFIG

         Accel         Gyro
DLPF_CFG Hz  Delay(ms) Hz   Delay(ms)   Fs(kHz)
0        260 0         256  0.98        8
1        184 2.0       188  1.9         1
2        94  3.0       98   2.8         1
3        44  4.9       42   4.8         1
4        21  8.5       20   8.3         1
5        10  13.8      10   13.4        1
6        5   19.0      5    18.6        1
7 RESERVED
'''
CONFIG       = 0x1A
'''
GYRO_CONFIG
Used to trigger gyro self test and full scale range
FS_SEL bits4,3
FS_SEL  Full Scale Range
0       +/- 250 deg/sec
1       +/- 500 deg/sec
2       +/- 1000 deg/sec
3       +/- 2000 deg/sec
decimal 24=+/- 2000 deg/sec
'''
GYRO_CONFIG  = 0x1B
'''
Configure Interrupt (int)
11011100
b7 to b0
b7 = 1, int active low
b6 = 1, open drain set Pi to pull up
b5 = 0, mpu int out 50us pulse
b4 = 1, int status cleared on data read
b3 = 1, FSYNC pin active low
b2 = 1, enable int
b1 = 0, bypass i2c bus
b0 = 0, n/a
'''
CONFIG_INT = 0X37
'''
INT_ENABLE
enables interrupt generation by interrupt sources
bit4 FIFO_OFLOW_EN when 1 enables FIFO buffer to generate interrupy
bit3 I2C_MST_INT_EN when 1 enables I2C Master interrupt sources to generate interrupt
bit0 DATA_RDY_EN when 1 this enables a Data Ready Interrupt, which occurs each time
a write operation to all the sensor registets has been completed

'''
INT_ENABLE   = 0x38
SIGNAL_PATH_RESET = 0x68
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


Ax = 0.0
Ay = 0.0
Az = 0.0
Gx = 0.0
Gy = 0.0
Gz = 0.0

AxRaw = 0.0
AyRaw = 0.0
AzRaw = 0.0

GxRaw = 0.0
GyRaw = 0.0
GzRaw = 0.0

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x69   # MPU6050 device address


GX_OFFSET = 0.9448346055979611
GY_OFFSET = 0.42473282442748
GZ_OFFSET = -0.007175572519084001

AZ_OFFSET = -0.20509161536206405
AY_OFFSET = -0.0027156625622984485 
AX_OFFSET = -0.00187335092348285 

X_Rotation_Offset = 0.5184471935554903 
Y_Rotation_Offset = -0.7581521549108168

GX_Drift = 0.0
GY_Drift = 0.0
GZ_Drift = 0.000103

ACCEL_SCALE = 18192.0
GYRO_SCALE = 65.5

iRound = 8
iCaliPoints = 300 #do at least 100 calibration measurements
iCaliSleepTime = 1

K = 0.98
K1 = 1 - K

lastX = 0.0
lastY = 0.0

tDiff = 1.0/400.0 


def MPU_Init():
        #Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1) ## x gyro as clock source
	## time.sleep( .1 ) ## wait
	## bus.write_byte_data(Device_Address, SIGNAL_PATH_RESET, 7) #reset device signal paths
	## time.sleep( .1 )
        #write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt configure & enable register
	## bus.write_byte_data(Device_Address, CONFIG_INT , 220)
	## bus.write_byte_data(Device_Address, INT_ENABLE, 1)
	return;

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
        value = 0
        value = twos_complement( ((high << 8) | low) )
        return value;



def MPU_Cali():

    global GX_OFFSET
    global GY_OFFSET
    global GZ_OFFSET

    global AZ_OFFSET
    global AY_OFFSET
    global AX_OFFSET

    global X_Rotation_Offset
    global Y_Rotation_Offset
    
    X_Rotation_Offset = 0.0
    Y_Rotation_Offset = 0.0

    GX_OFFSET = 0.0
    GY_OFFSET = 0.0
    GZ_OFFSET = 0.0

    AZ_OFFSET = 0.0
    AY_OFFSET = 0.0
    AX_OFFSET = 0.0

    iCalibrateCount = 0

    MPU_Init()
    
    while (iCalibrateCount != iCaliPoints) :
        
        MPU_ReadRawData()
        iCalibrateCount = iCalibrateCount + 1

        ##gather info to be averaged
        GX_OFFSET += GxRaw
        GY_OFFSET += GyRaw
        GZ_OFFSET += GzRaw
        AZ_OFFSET += AzRaw
        AY_OFFSET += AyRaw
        AX_OFFSET += AxRaw

        X_Rotation_Offset += fGet_x_rotation(AxRaw, AyRaw, AzRaw)
        Y_Rotation_Offset += fGet_y_rotation(AxRaw, AyRaw, AzRaw)

        print ("Pass number: " + str(iCalibrateCount))
        time.sleep(iCaliSleepTime)


        
    #calculate averages n change sign
    GX_OFFSET = -(GX_OFFSET/iCaliPoints)
    GY_OFFSET = -(GY_OFFSET/iCaliPoints)
    GZ_OFFSET = -(GZ_OFFSET/iCaliPoints)
    AZ_OFFSET = -(AZ_OFFSET/iCaliPoints)
    AY_OFFSET = -(AY_OFFSET/iCaliPoints)
    AX_OFFSET = -(AX_OFFSET/iCaliPoints)

    X_Rotation_Offset = -(X_Rotation_Offset/iCaliPoints)
    Y_Rotation_Offset = -(Y_Rotation_Offset/iCaliPoints)
    
    print(str(GX_OFFSET) + " GX OFFSET")
    print(str(GY_OFFSET) + " GY OFFSET")
    print(str(GZ_OFFSET) + " GZ OFFSET")
    print(str(AZ_OFFSET) + " AZ OFFSET")
    print(str(AY_OFFSET) + " AY OFFSET")
    print(str(AX_OFFSET) + " Ax OFFSET")
    print(str(X_Rotation_Offset) + " X_Rotation_Offset")
    print(str(Y_Rotation_Offset) + " Y_Rotation_Offset")

    return;
##################################################################
#
##################################################################
def twos_complement( val ):
    if ( val >= 0x8000 ):
        val = -( ( 65535 - val ) +1 )
    return val;

def millisTime():
      dateDateTimeNow = datetime.datetime.now()
      return ((dateDateTimeNow.microsecond/1000) + (dateDateTimeNow.second*1000));
'''
Inspiration for the gyro drift rate code
from M.Hefny, mohammad.hefny@gmail.com, Date: 13 June 2015
'''
#Uses calibrate averages, and try to detect drift 
def fCal_Gyro_Drift():
    iCount = 0
    timeOld = 0.0
    timeNew  = 0.0
    timeDiff = 0.0
    fltGyro_x_int = 0.0
    fltGyro_y_int = 0.0
    fltGyro_z_int = 0.0
    fltGyro_x = 0.0
    fltGyro_y = 0.0
    fltGyro_z = 0.0
    fltGyro_AvgDriftRate_X = 0.0
    fltGyro_AvgDriftRate_Y = 0.0
    fltGyro_AvgDriftRate_Z = 0.0
        
    print ("calibrating gyro (min average error selection) - getting drift")
    print ("Start time " + str(datetime.datetime.now()))
    MPU_Init()
    while iCount < iCaliPoints:
        iCount = iCount + 1

        timeNew = millisTime()        
        timeDiff = timeNew - timeOld
        MPU_ReadRawData()
        timeOld = timeNew
        #apply offset 
        fltGyro_x = GxRaw + GX_OFFSET
        fltGyro_y = GyRaw + GY_OFFSET
        fltGyro_z = GzRaw + GZ_OFFSET

        fltGyro_x_int += (fltGyro_x / timeDiff)
        fltGyro_y_int += (fltGyro_y / timeDiff) 
        fltGyro_z_int += (fltGyro_z / timeDiff)
        print ("Pass number: " + str(iCount))
        time.sleep(iCaliSleepTime)
    fltGyro_AvgDriftRate_X = -(fltGyro_x_int / iCount)
    fltGyro_AvgDriftRate_Y = -(fltGyro_y_int / iCount)
    fltGyro_AvgDriftRate_Z = -(fltGyro_z_int / iCount)
    print ("Calibrating loops (rad/sec) %d  x:%f y:%f z:%f"
        %(iCount, fltGyro_AvgDriftRate_X, fltGyro_AvgDriftRate_Y, fltGyro_AvgDriftRate_Z))
    print ("End time " + str(datetime.datetime.now()))

    return;


def MPU_ReadRawData():

    global Ax
    global Ay
    global Az
    global Gx
    global Gy
    global Gz
    global AxRaw
    global AyRaw
    global AzRaw
    global GxRaw
    global GyRaw
    global GzRaw
    

    AxRaw = 0.0
    AxRaw = read_raw_data(ACCEL_XOUT_H)
    AyRaw = 0.0
    AyRaw = read_raw_data(ACCEL_YOUT_H)
    AzRaw = 0.0
    AzRaw = read_raw_data(ACCEL_ZOUT_H)
####
    GxRaw = 0.0
    GxRaw = read_raw_data(GYRO_XOUT_H) 
    GyRaw = 0.0
    GyRaw = read_raw_data(GYRO_YOUT_H)
    GzRaw = 0.0
    GzRaw = read_raw_data(GYRO_ZOUT_H)
####
#### adjust with scale factors
#### acclerometer offset adjustments made here
    preScaled = AxRaw
    AxRaw = AxRaw / ACCEL_SCALE
    Ax = ( preScaled + AX_OFFSET ) / ACCEL_SCALE
            
    preScaled = AyRaw
    AyRaw = AyRaw / ACCEL_SCALE
    Ay = ( preScaled + AY_OFFSET ) / ACCEL_SCALE

    preScaled = AzRaw
    AzRaw = AzRaw / ACCEL_SCALE
    Az = ( preScaled + AZ_OFFSET ) / ACCEL_SCALE
#### do not adjist gyro offsets here
    GxRaw = GxRaw / GYRO_SCALE
    Gx = GxRaw
    GyRaw = GyRaw / GYRO_SCALE
    Gy = GyRaw
    GzRaw = GzRaw / GYRO_SCALE
    Gz = GzRaw

    return;

##############################
# used for accelerometer angles
##############################
def fDist(a,b):
    return math.sqrt((a*a)+(b*b));

def fGet_y_rotation(x,y,z):
    radians = math.atan2(y, fDist(x,z))
    return math.degrees(radians);

def fGet_x_rotation(x,y,z):
    radians = math.atan2(x, fDist(y,z))
    return -math.degrees(radians);
###########################################################
'''

'''
###########################################################
def fRunLoop():
    global Ax
    global Ay
    global Az
    global Gx
    global Gy
    global Gz
    global lastX
    global lastY
    
    gyroTotalX = 0.0
    gyroTotalY = 0.0
    
    now = time.time()
    time_diff = 0.01
    ## K = 0.98
    ## K1 = 1 - K
    
    print (" Reading Data of Gyroscope and Accelerometer")
    MPU_Init()
    
    MPU_ReadRawData()

    ## lastX = fGet_x_rotation( AxRaw, AyRaw, AzRaw )
    ## gyroOffSetX = Gx
    ## gyroTotalX = (lastX) - gyroOffSetX
    
    ## lastY = fGet_y_rotation( AxRaw, AyRaw, AzRaw )
    ## gyroOffSetY = Gy
    ## gyroTotalY = (lastY) - gyroOffSetY
    
    while True:
        
        
        
        MPU_ReadRawData()
        
        AxRotRaw = fGet_x_rotation( AxRaw, AyRaw, AzRaw )
        AyRotRaw = fGet_y_rotation( AxRaw,  AyRaw,  AzRaw )
        AxRot =  fGet_x_rotation(  Ax,  Ay,  Az )
        AyRot =  fGet_y_rotation(  Ax,  Ay,  Az )
#### if no last rotation / time past get one.
        if (  lastX == 0.0 ):
            lastX = AxRot
            lastY = AyRot
## apply gyro offsets here       
        Gx += ( GX_OFFSET - X_Rotation_Offset )
        Gy += ( GY_OFFSET - Y_Rotation_Offset )
## obtain gyro rotation since last reading Gx * tDiff
## get rotation accelerometer use scaled and adjusted accelerometer data fGet_x_rotation
##  complementary filter to combine data
        lastX = K * ( lastX + ( Gx * tDiff ) ) + ( K1 * AxRot )
        lastY = K * ( lastY + ( Gy * tDiff ) ) + ( K1 * AyRot )

        print( "Time:%.4f ArotX %.2f gX %.2f lastX %.2f ArotY %.2f gY %.2f lastY %.2f" %( time.time(), AxRot, Gx, lastX, AyRot, Gy, lastY ) )
        time.sleep( .5 )
      
    return;

'''
must be mannualy entered into
##GX_OFFSET = 0.0
##GY_OFFSET = 0.0
##GZ_OFFSET = 0.0
##
##AZ_OFFSET = 0.0
##AY_OFFSET = 0.0
##AX_OFFSET = 0.0
overwrite existing values with calibration values

run MPU_Cali() first to get averages
'''
## MPU_Cali()
'''
calibrate gyro drft uses average drift rate
to calculate gryo drift over time
in rad/s^2
mamually enter results into the variables
GX_Drift = 0.0
GY_Drift = 0.0
GZ_Drift = 0.0

run after MPU_Cali() to get drift rates
'''
## fCal_Gyro_Drift()
'''
uncomment fRunLoop() to see mpu6050 data
compare before and after calibration.
remember the loop displays mpu data +/- .0000
tiny numbers
'''

## fRunLoop()
