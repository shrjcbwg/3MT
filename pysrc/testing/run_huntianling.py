# -*- coding: UTF-8 -*-
import math
import os
import time
import airsim
from airsim.types import DistanceSensorData
from xml.dom.minidom import parse
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

dom = parse('~/AirSim/ros/src/airsim_ros_pkgs/scripts/testing.xml')
data = dom.documentElement

# os.system('. ~/Desktop/run_with_sim.sh')
    
# os.system('gnome-terminal -t "start_planner" -- bash -c "rosbag record -O position_cmd1.bag /planning/pos_cmd"')

def run_test(comment2):
    os.system('. ~/Desktop/run_all.sh')
    while not client.simGetCollisionInfo().has_collided:
        if os.path.exists('~/Desktop/data/reached.tmp'):
            current_x = client.simGetGroundTruthKinematics().position.x_val
            current_y = client.simGetGroundTruthKinematics().position.y_val
            if math.sqrt(current_x**2+current_y**2) > 0.1:
                print ("Reached the Goal!")
                break
            else:
                os.system('rm -rf ~/Desktop/data/reached.tmp')
    os.system(comment2)
    client.reset()
    print('Get Collision')
    os.system('. ~/Desktop/kill_ros.sh')
    os.system('rm -rf ~/Desktop/data/reached.tmp')


tests = data.getElementsByTagName('test')
for test in tests:
    # 获取标签属性值
    test_id = test.getAttribute('id')
    test_name = test.getAttribute('name')
    # 获取标签中内容
    transformation = test.getElementsByTagName('transformation')[0].childNodes[0].nodeValue
    testcase = test.getElementsByTagName('testcase')[0].childNodes[0].nodeValue
    if not os.path.exists('~/Desktop/data/' + transformation):
        # creat_folder = 'mkdir ~/Desktop/data/' + transformation
        os.system('mkdir ~/Desktop/data/' + transformation)
    if not os.path.exists('~/Desktop/' + transformation + '/' + testcase):
        os.system('mkdir ~/Desktop/data/' + transformation + '/' + testcase)
    save_data = 'mv ~/Desktop/data/*.txt ~/Desktop/data/' + transformation + '/' + testcase
    print("******************Start a testing******************")
    print('test_id:', test_id,  ', test_name:',test_name)
    run_test(save_data)

