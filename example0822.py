# -- coding: utf-8 --
import rospy

from sailboat_message.msg import Sensor_msg
from sailboat_message.msg import Mach_msg

from spare_function.cfg import spare_function_Config
from spare_function.msg import spare_function_out
from spare_function.msg import spare_function_para

from dynamic_reconfigure.server import Server
import numpy as np
from collections import Counter
from collections import deque
import math

sensor_submsg = [0,0,0,0,0,0,0,0,0,0]
para_cfg = [0,0,0,0,0,0,0,0]


sa_out = deque(maxlen=10)
sa_out.append(0)#输出帆角        
heading_error = deque(maxlen=100)
heading_error.append(0)
twa_list=deque(maxlen=50)
tws_list=deque(maxlen=50)
# tws_list=[0]
# twa_list=[0]
LABEL=0
# print(LRX.shape)
# print()

# 根据角度判断状态代码
def angle2num(angle):
    if angle > -np.pi*7/8 and angle <= -np.pi*5/8:
        num = 6
    elif angle > -np.pi*5/8 and angle <= -np.pi*3/8:
        num = 7
    elif angle > -np.pi*3/8 and angle <= -np.pi/8:
        num = 8
    elif angle > -np.pi/8 and angle <= np.pi/8:
        num = 1
    elif angle > np.pi/8 and angle <= np.pi*3/8:
        num = 2
    elif angle > np.pi*3/8 and angle <= np.pi*5/8:
        num = 3
    elif angle > np.pi*5/8 and angle <= np.pi*7/8:
        num = 4
    else:
        num = 5
    return num





def aw2tw(aws, awa, u, v, heading):
    vx = u*np.cos(heading)-v*np.sin(heading)
    vy = u*np.sin(heading)+v*np.cos(heading)
    twx = aws*math.cos(awa+heading+np.pi)+vx
    twy = aws*math.sin(awa+heading+np.pi)+vy
    tws = np.sqrt(twx*twx+twy*twy)
    if awa>0:
        twa = angle_limit(math.atan2(twy, twx)+np.pi)
    else:
        twa = angle_limit(math.atan2(twy, twx)-np.pi)
    # wind_num = angle2num(angle_limit(twa-heading))
    
    return tws, twa

def goal_angle_func(ship_x, ship_y, heading, goal_x, goal_y,twa):
    goal_angle = math.atan2(goal_y-ship_y, goal_x-ship_x)
    goal_wind = angle_limit(goal_angle - twa)

    return goal_angle,goal_wind

# def aw2tw(aws, awa, u, v, heading):
#   if awa>0:
#       awa=awa-np.pi
#   else:
#       awa=awa+np.pi
#   vx = u*np.cos(heading)-v*np.sin(heading)
#   vy = u*np.sin(heading)+v*np.cos(heading)
#   twx = aws*math.cos(awa+heading+np.pi)+vx
#   twy = aws*math.sin(awa+heading+np.pi)+vy
#   tws = np.sqrt(twx*twx+twy*twy)
#   if awa>0:
#       twa = angle_limit(math.atan2(twy, twx)+np.pi)
#   else:
#       twa = angle_limit(math.atan2(twy, twx)-np.pi)
#   wind_num = angle2num(twa-heading)
#   return tws, twa, wind_num
def sa_num_func(twa,heading):
    wha =angle_limit(twa-heading)
    if abs(wha)>np.pi/8 and abs(wha)<=np.pi*3/8:
        sa_num=1
    elif abs(wha)>np.pi*3/8 and abs(wha)<=np.pi*7/8:
        sa_num=2
    elif abs(wha)>np.pi*7/8 and abs(wha)<=np.pi:
        sa_num=3
    else:
        sa_num=4
    return sa_num

def sa_goal_func(twa, sa_num, heading):
    wha =angle_limit(twa-heading)
    if sa_num == 1:
        if wha > 0:
            sa_goal = wha-5/8*np.pi
        else: 
            sa_goal = wha+5/8*np.pi
    elif sa_num == 2:
        if wha > 0:
            sa_goal = 0.5*wha-np.pi*7/16
        else:
            sa_goal = 0.5*wha+np.pi*7/16
    elif sa_num == 3:
        sa_goal = 0
    else:
        if wha > 0:
            sa_goal = wha-5/8*np.pi
        else: 
            sa_goal = wha+5/8*np.pi
    return sa_goal

# def sa_tacking(twa,heading,awa):
#     wha = angle_limit(twa-heading)
#     if np.abs(wha) > np.pi/4:
#         if wha > 0:
#             sa_goal = awa-np.pi*5/8
#         else: 
#             sa_goal = awa+np.pi*5/8
#     else:
#         if awa > 0:
#             sa_goal = awa-np.pi/2
#         else: 
#             sa_goal = awa+np.pi/2
#     return sa_goal


def sa_func(heading_error, sa_goal):
    # sa_step = np.pi/60 # 每步转帆角度
    # if np.abs(sa_goal) < np.abs(sa_old): # 当前帆角比目标帆角靠外，直接到达目标帆角
    #     sa_output = sa_goal
    # elif np.abs(sa_goal-sa_old) > np.pi/4: # 当前帆角与目标帆角相差较大，直接到达目标帆角
    #     sa_output = sa_goal
    # elif u_judgement == 0: # 当前航速比目标航速小，帆角往内一步
    #     if sa_goal > 0:
    #         sa_output = sa_old-sa_step
    #     else:
    #         sa_output = sa_old+sa_step
    # elif np.abs(sa_goal-sa_old) < sa_step:
    #     sa_output = sa_goal
    # else: # 否则帆角向外增加一步
    #     if sa_goal > 0:
    #         sa_output = sa_old+sa_step
    #     else:
    #         sa_output = sa_old-sa_step
    skp = para_cfg[5]
    ski = para_cfg[6]
    skd = para_cfg[7]

    sa_output = sa_goal+skp*heading_error[-1]+skd*(heading_error[-1]-heading_error[-2])+ski*(sum(heading_error))

    return sa_output # 返回控制帆角值

# heading_goal_func 正常 jibing1右直行 jibing2左直行
def heading_goal_func(goal_angle, heading,twa):
    heading_twa=angle_limit(heading-twa-np.pi)
    goal_twa=angle_limit(goal_angle-twa-np.pi)
    heading_goal=goal_twa-heading_twa
    return heading_goal

# def heading_jibing1(goal_angle, heading,twa):
#     heading_twa=angle_limit(heading-twa-np.pi)
#     goal_twa=-np.pi*3/4
#     heading_goal=goal_twa-heading_twa
#     return heading_goal

# def heading_jibing2(goal_angle, heading,twa):
#     heading_twa=angle_limit(heading-twa-np.pi)
#     goal_twa=np.pi*3/4
#     heading_goal=goal_twa-heading_twa
#     return heading_goal


def ra_func(heading_error):
    ra_limit=0.15
    rkp = para_cfg[2]
    rki = para_cfg[3]
    rkd = para_cfg[4]

    ra_output = rkp*heading_error[-1]+rkd*(heading_error[-1]-heading_error[-2])+rki*(sum(heading_error))
    if ra_output>ra_limit:
        ra_output=ra_limit
    elif ra_output<-ra_limit:
        ra_output=-ra_limit
    return ra_output


# 角度限制（限制在(-pi, pi]范围内）
def angle_limit(angle):
    if angle > np.pi:
        angle -= 2*np.pi
    if angle <= -np.pi:
        angle += 2*np.pi
    return angle

# 根据传感器输入规划帆船的帆角sa和舵角ra

def module(sensor_submsg,goal):
    global LABEL, point10_x, point10_y
    u, v, r, dheel, x, y, heading, heel,aws,awa=sensor_submsg[0], sensor_submsg[1], sensor_submsg[2], sensor_submsg[3], sensor_submsg[4], sensor_submsg[5], sensor_submsg[6], sensor_submsg[7],sensor_submsg[8],sensor_submsg[9]
    tws_temp, twa_temp = aw2tw(aws, awa, u, v, heading)
    twa_list.append(twa_temp)
    tws_list.append(tws_temp)
    # if len(twa_list)==50:  #5s计算一次风速
    #     twa_list=[0]
    #     tws_list=[0]
    #     twa=np.mean(twa_list)
    #     tws=np.mean(tws_list)
    twa=np.mean(twa_list)  #前50个点平均计算风速
    tws=np.mean(tws_list)    
    print('heading',heading,'twa',twa,'awa',awa)
    sa_num=sa_num_func(twa,heading)
    goal_angle,goal_wind = goal_angle_func(x,y,heading, goal[0],goal[1],twa)
    twa_limit=np.pi*1/3#禁航区范围
    goal_twa_limit=np.pi*3/4#逆风跑的弧度,twa坐标系

    if LABEL == 1: #右前直行
        if abs(goal_wind)>twa_limit:
            LABEL=11
            point10_x=x
            point10_y=y
        heading_twa=angle_limit(heading-twa-np.pi)
        goal_twa=-goal_twa_limit
        heading_goal=goal_twa-heading_twa
        if abs(heading_goal)<np.pi/6:
            sa_goal=sa_goal_func(twa, sa_num, heading)
        else:
            sa_goal=sa_goal_func(twa, sa_num, heading)

    elif LABEL == 11:#右前执行延长10m
        if math.sqrt((point10_x-x)*(point10_x-x)+(point10_y-y)*(point10_y-y))>10:
            LABEL=0
        if abs(goal_wind)>2*twa_limit:
            LABEL=0
        heading_twa=angle_limit(heading-twa-np.pi)
        goal_twa=-goal_twa_limit
        heading_goal=goal_twa-heading_twa
        if abs(heading_goal)<np.pi/6:
            sa_goal=sa_goal_func(twa, sa_num, heading)
        else:
            sa_goal=sa_goal_func(twa, sa_num, heading)


    elif LABEL == 2:#左前直行
        if abs(goal_wind)>twa_limit:
            LABEL=22
            point10_x=x
            point10_y=y
            print(point10_x)
        heading_twa=angle_limit(heading-twa-np.pi)
        goal_twa=goal_twa_limit
        heading_goal=goal_twa-heading_twa
        if abs(heading_goal)<np.pi/6:
            sa_goal=sa_goal_func(twa, sa_num, heading)
        else:
            sa_goal=sa_goal_func(twa, sa_num, heading)

    elif LABEL == 22:#左前执行延长10m
        if math.sqrt((point10_x-x)*(point10_x-x)+(point10_y-y)*(point10_y-y))>10:
            LABEL=0
        if abs(goal_wind)>2*twa_limit:
            LABEL=0
        heading_twa=angle_limit(heading-twa-np.pi)
        goal_twa=goal_twa_limit
        heading_goal=goal_twa-heading_twa
        if abs(heading_goal)<np.pi/6:
            sa_goal=sa_goal_func(twa, sa_num, heading)
        else:
            sa_goal=sa_goal_func(twa, sa_num, heading)


    else:#正常航行
        if abs(goal_wind)<twa_limit:
            #if goal_wind>0:
            if angle_limit(heading-twa-np.pi)<0:
                LABEL=1
            else:
                LABEL=2
        heading_goal=heading_goal_func(goal_angle, heading,twa)

        if abs(heading_goal)<np.pi/6:
            sa_goal=sa_goal_func(twa, sa_num, heading)
        else:
            sa_goal=sa_goal_func(twa, sa_num, heading)


    print('goal_angle',goal_angle,'heading_goal',heading_goal,'LABEL',LABEL)
    heading_error.append(heading_goal)
    sa_output=sa_func(heading_error, sa_goal)
    sa_out.append(sa_output)
    ra_output=ra_func(heading_error)
    return sa_output,ra_output

def getOutMachPut(msg): #sailboat_message::Mach_msg
    mach_pub = Mach_msg()
    mach_pub.header.stamp = rospy.Time.now()
    mach_pub.header.frame_id = 'AHRS'
    #mach_pub.timestamp = rospy.Time.now()
    mach_pub.motor = 55
    mach_pub.rudder = msg[0]
    mach_pub.sail   = msg[1]
    mach_pub.PCCtrl = msg[2]
    return mach_pub


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
    
    rospy.Subscriber("sensor", Sensor_msg, sensorCallback)
    config_srv = Server(spare_function_Config, getConfigCallback)

    rate = rospy.Rate(10)
    goal_list = [(20,0),(0,0),(10,-10)]
    k=0
    try:
        while not rospy.is_shutdown():

            sa, ra = module(sensor_submsg, goal_list[k])
            distance=math.sqrt((goal_list[k][0]-sensor_submsg[4])*(goal_list[k][0]-sensor_submsg[4])+(goal_list[k][1]-sensor_submsg[5])*(goal_list[k][1]-sensor_submsg[5]))
            if distance<3:
                k=k+1

            if k>len(goal_list)-1:
                k=0




            mach_np = [ra, sa, 1]
            out_np = [ra*180/np.pi, sa*180/np.pi, sensor_submsg[0], sensor_submsg[1], sensor_submsg[2], sensor_submsg[3], sensor_submsg[4], sensor_submsg[5], sensor_submsg[6], sensor_submsg[7], sensor_submsg[8], sensor_submsg[9]]

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
