#!/usr/bin python3

import argparse
import logging
import sys
sys.path.append('..')
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time
import rospy
from std_msgs.msg import Float32MultiArray

# node
rospy.init_node("RTDE_NODE")
pub=rospy.Publisher('/rtde_data', Float32MultiArray,queue_size=10)
rate=rospy.Rate(10)


# parameters
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='192.168.0.80',help='name of host to connect to (localhost)')
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
parser.add_argument('--samples', type=int, default=0,help='number of samples to record')
parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
parser.add_argument('--config', default='record_configuration.xml', help='data configuration file to use (record_configuration.xml)')
parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
parser.add_argument("--binary", help="save the data in binary format", action="store_true")
args = parser.parse_args()

if args.verbose:
    logging.basicConfig(level=logging.INFO)

conf = rtde_config.ConfigFile(args.config)
output_names, output_types = conf.get_recipe('out')

con = rtde.RTDE(args.host, args.port)
con.connect()

# settings
con.get_controller_version()
con.send_output_setup(output_names, output_types, frequency=args.frequency)
con.send_start()

# initialize variables
X = 0
Y = 0
Z = 0
RX = 0
RY = 0
RZ = 0
# main loop
i = 1

f = open("fuerzas.txt", "w")
f.write(" \n ")
f.close

while not rospy.is_shutdown():
    if args.samples > 0 or rospy.is_shutdown():
        keep_running = False
    time.sleep(0.5)
    try:
        if args.buffered:
            state = con.receive_buffered(args.binary)
        else:
            state = con.receive(args.binary)
        if state is not None:
            X,Y,Z,RX,RY,RZ = state.actual_TCP_force
            date_and_time = state.timestamp
            payload = state.payload
            cog_pay = state.payload_cog
            vel = state.actual_TCP_speed
            tcp_force = state.tcp_force_scalar
            # print(str(date_and_time)+" TCP: force ["+str(X)+", "+str(Y)+", "+str(Z)+"] N, torque ["+str(RX)+", "+str(RY)+", "+str(RZ)+"] Nm")     
            print(" TCP: force ["+str(X)+", "+str(Y)+", "+str(Z)+"] N, torque ["+str(RX)+", "+str(RY)+", "+str(RZ)+"] Nm")
            #print("Payload: " + str(payload))
            #print("Center of Gravity: " + str(cog_pay))
            #print("Velocity: " + str(vel))
            #print("TCP scalar: " + str(tcp_force))
            f = open("fuerzas.txt", "a")
            f.write(str(vel) + " \n ")
            f.close
        if not rospy.is_shutdown():
            pub.publish(data= [X,Y,Z,RX,RY,RZ])
            rate.sleep()   
    except rtde.RTDEException:
        break
    except KeyboardInterrupt:
        break
    
con.send_pause()
con.disconnect()
