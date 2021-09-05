# -*- coding: UTF-8 -*-
import math
import os
import time
import airsim
from airsim.types import DistanceSensorData
from xml.dom.minidom import parse
from mpl_toolkits.mplot3d import axes3d
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


client = airsim.MultirotorClient()
client.confirmConnection()


dom = parse('/home/rui/AirSim/ros/src/airsim_ros_pkgs/scripts/testing.xml')
data = dom.documentElement
original_odo = pd.read_csv('/home/rui/Desktop/data/transformation7/original/cmd_1.txt')


# os.system('. ~/Desktop/run_with_sim.sh')
    
# os.system('gnome-terminal -t "start_planner" -- bash -c "rosbag record -O position_cmd1.bag /planning/pos_cmd"')

def run_test(comment2,index):
    os.system('. ~/Desktop/run_all.sh')
    while not client.simGetCollisionInfo().has_collided:
        if os.path.exists('/home/rui/Desktop/data/reached.tmp'):
            current_x = client.simGetGroundTruthKinematics().position.x_val
            current_y = client.simGetGroundTruthKinematics().position.y_val
            if math.sqrt(current_x**2+current_y**2) > 0.1:
                print ("Reached the Goal!")
                break
            else:
                os.system('rm -rf /home/rui/Desktop/data/reached.tmp')
    os.system(comment2)
    client.reset()
    if client.simGetCollisionInfo().has_collided:
        print('Get Collision in testcase:')
    os.system('. ~/Desktop/kill_ros.sh')
    os.system('rm -rf /home/rui/Desktop/data/reached.tmp')

def add_drone(drone_name,pose):
    # add new vehicle
    client.simAddVehicle(drone_name, "simpleflight", pose)
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)
    client.takeoffAsync(vehicle_name = drone_name).join()
    # client.moveToPositionAsync(x,y,-3,2,vehicle_name=drone_name).join()

tests = data.getElementsByTagName('test')
for i in range(1,11):
    client.enableApiControl(True)
    client.armDisarm(True)
    transformation = 'transformation7'
    testcase = 'testcase' + str(i)
    drone_name = "Drone"+ str(i)
    row = math.floor((i-0.1) *0.1* original_odo.shape[0])
    position_x = original_odo.iloc[row].at['x']
    position_y = original_odo.iloc[row].at['y']
    position_z = - original_odo.iloc[row].at['z']
    pose = airsim.Pose(airsim.Vector3r(position_x, - position_y, position_z), airsim.to_quaternion(0, 0, 0))
    if not os.path.exists('/home/rui/Desktop/data/' + transformation):
        os.system('mkdir /home/rui/Desktop/data/' + transformation)
    if not os.path.exists('/home/rui/Desktop/' + transformation + '/' + testcase):
        os.system('mkdir /home/rui/Desktop/data/' + transformation + '/' + testcase)

    
    save_data = 'mv /home/rui/Desktop/data/*.txt /home/rui/Desktop/data/' + transformation + '/' + testcase
    print("******************Start a testing******************")
    print('test_id:', i,  ', test_name:',testcase)
    print ('drone_name:',drone_name)
    add_drone(drone_name,pose)
    # print(pose)
    run_test(save_data,i)
