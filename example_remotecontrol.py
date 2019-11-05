#!/usr/bin/env python
# -*- coding: utf-8-*
import rospy

from sailboat_message.msg import Sensor_msg
from sailboat_message.msg import Mach_msg

from spare_function.cfg import spare_function_Config
from spare_function.msg import spare_function_out
from spare_function.msg import spare_function_para
from collections import deque
from collections import Counter

from dynamic_reconfigure.server import Server
import math
import numpy as np

# from numpy import sin,cos,pi,arctan2,sqrt,rad2deg


sensor_submsg = [0,0,0,0,0,0,0,0,0,0]
para_cfg = [0,0,0,0,0,0,0,0]


def getOutMachPut(msg): #sailboat_message::Mach_msg
    mach_pub = Mach_msg()
    mach_pub.header.stamp = rospy.Time.now()
    mach_pub.header.frame_id = 'AHRS'
    #mach_pub.timestamp = rospy.Time.now()
    mach_pub.motor = 0
    mach_pub.rudder = msg[0]
    mach_pub.sail   = msg[1]
    mach_pub.PCCtrl = msg[2]
    return mach_pub

# rudder, sail, u, v, r, dheel, x, y, heading, heel,aws,awa
def getOutput(msg): #spare_function::spare_function_out
    out_pub = spare_function_out()
    out_pub.rudder = msg[0]
    out_pub.sail = msg[1]
    out_pub.u = msg[2]
    out_pub.v = msg[3]
    out_pub.r = msg[4]
    out_pub.dheel = msg[5]
    out_pub.x = msg[6]
    out_pub.y = msg[7]
    out_pub.heading = msg[8]
    out_pub.heel = msg[9]
    out_pub.aws = msg[10]
    out_pub.awa = msg[11]   
    return out_pub


def getOutParaPut(msg):#spare_function::spare_function_para
    para_pubmsg = spare_function_para()
    para_pubmsg.oyaw   = msg[1]
    para_pubmsg.rudderP= msg[2]
    para_pubmsg.rudderI= msg[3]
    para_pubmsg.rudderD= msg[4]
    para_pubmsg.sailP  = msg[5]
    para_pubmsg.sailI  = msg[6]
    para_pubmsg.sailD  = msg[7]
    return para_pubmsg

def sensorCallback(msg): #sailboat_message::Sensor_msg
    global sensor_submsg 
    sensor_submsg[0] = msg.ux
    sensor_submsg[1] = msg.vy
    sensor_submsg[2] = msg.gz
    sensor_submsg[3] = msg.gx
    sensor_submsg[4] = msg.Posx
    sensor_submsg[5] = msg.Posy
    sensor_submsg[6] = msg.Yaw
    sensor_submsg[7] = msg.Roll
    sensor_submsg[8] = msg.AWS
    sensor_submsg[9] = msg.AWA


def getConfigCallback(config, level): # spare_function::spare_function_Config
    global para_cfg
    if (config.PC_Ctrl == True):
        para_cfg[0] = 1
    else:
        para_cfg[0] = 0
    para_cfg[1] = config.oyaw
    para_cfg[2] = config.rudderP
    para_cfg[3] = config.rudderI
    para_cfg[4] = config.rudderD
    para_cfg[5] = config.sailP
    para_cfg[6] = config.sailI
    para_cfg[7] = config.sailD
    return config

if __name__ == "__main__":
    rospy.init_node("example", anonymous = True)

    mach_pub = rospy.Publisher('mach', Mach_msg, queue_size=5)
    spare_function_pub = rospy.Publisher('spare_function_out', spare_function_out, queue_size=5)
    spare_function_para_pub = rospy.Publisher('spare_function_para', spare_function_para, queue_size=5)
    
    rospy.Subscriber("sensor_kalman_msg", Sensor_msg, sensorCallback)
    config_srv = Server(spare_function_Config, getConfigCallback)

    rate = rospy.Rate(10)
   
    try:
        while not rospy.is_shutdown():

            
            out_np = [para_cfg[5], para_cfg[6], sensor_submsg[0], sensor_submsg[1], sensor_submsg[2], sensor_submsg[3], sensor_submsg[4], sensor_submsg[5], sensor_submsg[6], sensor_submsg[7], sensor_submsg[8], sensor_submsg[9]]

            ra = para_cfg[5]/180*np.pi
            sa = para_cfg[6]/180*np.pi
            mach_np = [ra, sa, 1]

            print('position ',sensor_submsg[4],sensor_submsg[5])
            print('heading_deg',sensor_submsg[6]*180/np.pi)

            # input : sensor_submsg 
            # cfg: para_cfg
            # output : mach_np out_np para_np

            mach_pubmsg = getOutMachPut(mach_np)
            out_pubmsg = getOutput(out_np)
            para_pubmsg = getOutParaPut(para_cfg)

            mach_pub.publish(mach_pubmsg)
            spare_function_pub.publish(out_pubmsg)
            spare_function_para_pub.publish(para_pubmsg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    #finally:
        #close()
    rospy.spin()
