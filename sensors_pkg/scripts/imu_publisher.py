#!/usr/bin/python
import smbus
import math
import rospy
import numpy as np
from sensor_msgs.msg import Imu

# Declare variable of Imu type 
my_imu_data = Imu()
freq = 50

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect

# Activate to be able to address the module
bus.write_byte_data(address, power_mgmt_1, 0)

def imu_data():
    pub = rospy.Publisher('imu_topic', Imu, queue_size=10) # imu topic, message type string, 10 - amount of queued messages 
    rospy.init_node('my_imu_node', anonymous=True) # my_imu_node - node, anonymous - adds random numbers to the node name
    rate = rospy.Rate(freq) # 50hz
    while not rospy.is_shutdown():
        my_imu_data.linear_acceleration.x, my_imu_data.linear_acceleration.y, my_imu_data.linear_acceleration.z  = accData()
        my_imu_data.angular_velocity.x, my_imu_data.angular_velocity.y, my_imu_data.angular_velocity.z = gyroData()
        #my_imu_data.orientation.w, my_imu_data.orientation.x, my_imu_data.orientation.y, my_imu_data.orientation.z = quatData()
        
        my_imu_data.orientation_covariance = [0, 0, 0,
                                              0, 0, 0,
                                              0, 0 ,0]
        my_imu_data.angular_velocity_covariance = [0, 0, 0,
                                                   0, 0, 0,
                                                   0, 0, 0]
        my_imu_data.linear_acceleration_covariance = [0, 0, 0,
                                                      0, 0, 0,
                                                      0, 0, 0]
        
        my_imu_data.header.frame_id = 'imu_frame' # Define transform from imu_frame to base_link
        time = rospy.get_rostime()
        my_imu_data.header.stamp.secs = time.secs
        my_imu_data.header.stamp.nsecs = time.nsecs
        
        #rospy.loginfo(my_imu_data)
        pub.publish(my_imu_data)
        rate.sleep()
 
def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_z_rotation(x,y,z):
    radians = math.atan2(z, dist(x,y))
    return radians # CHECK sign

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -radians
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return radians

 
#print("Gyroskop")
#print ("--------")
def gyroData(): 
    gyroscope_xout = read_word_2c(0x43)/131.0 * np.pi/180 # Divide by 131 for scaled value, convert degrees to radians
    gyroscope_yout = read_word_2c(0x45)/131.0 * np.pi/180 # Divide by 131 for scaled value, convert degrees to radians
    gyroscope_zout = read_word_2c(0x47)/131.0 * np.pi/180 # Divide by 131 for scaled value, convert degrees to radians
    
    return gyroscope_xout, gyroscope_yout, gyroscope_zout
 
#print("gyroscope_xout: ", ("%5d" % gyroscope_xout), " scaled: ", (gyroscope_xout / 131))
#print("gyroscope_yout: ", ("%5d" % gyroscope_yout), " scaled: ", (gyroscope_yout / 131))
#print("gyroscope_zout: ", ("%5d" % gyroscope_zout), " scaled: ", (gyroscope_zout / 131))
# 
#print()
#print("Accelerometer")
#print("---------------------")
def accData(): 
    accelerometer_xout = read_word_2c(0x3b)
    accelerometer_yout = read_word_2c(0x3d)
    accelerometer_zout = read_word_2c(0x3f)
     
    accelerometer_xout_scaled = accelerometer_xout / 16384.0 
    accelerometer_yout_scaled = accelerometer_yout / 16384.0 
    accelerometer_zout_scaled = accelerometer_zout / 16384.0
    
    return accelerometer_xout_scaled, accelerometer_yout_scaled, accelerometer_zout_scaled

def rotData(x_acc, y_acc, z_acc):
    x_rot = get_x_rotation(x_acc, y_acc, z_acc)
    y_rot = get_y_rotation(x_acc, y_acc, z_acc)
    z_rot = get_z_rotation(x_acc, y_acc, z_acc)
    
    return x_rot, y_rot, z_rot
def quatData():
    x_acc, y_acc, z_acc = accData()
    x_rot, y_rot, z_rot = rotData(x_acc, y_acc, z_acc)
    
    cy = math.cos(z_rot * 0.5)
    sy = math.sin(z_rot * 0.5)
    cp = math.cos(y_rot * 0.5)
    sp = math.sin(y_rot * 0.5)
    cr = math.cos(x_rot * 0.5)
    sr = math.sin(x_rot * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return w, x, y, z
#print("accelerometer_xout: ", ("%6d" % accelerometer_xout), " scaled: ", accelerometer_xout_scaled)
#print("accelerometer_yout: ", ("%6d" % accelerometer_yout), " scaled: ", accelerometer_yout_scaled)
#print("accelerometer_zout: ", ("%6d" % accelerometer_zout), " scaled: ", accelerometer_zout_scaled)
# 
#print("X Rotation: " , get_x_rotation(accelerometer_xout_scaled, accelerometer_yout_scaled, accelerometer_zout_scaled))
#print("Y Rotation: " , get_y_rotation(accelerometer_xout_scaled, accelerometer_yout_scaled, accelerometer_xout_scaled))

if __name__ == '__main__':
   try:
       imu_data()
   except rospy.ROSInterruptException:
       pass
