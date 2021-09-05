import math
from posixpath import abspath
from airsim.utils import to_dict
from mpl_toolkits.mplot3d import axes3d
from numpy.core.fromnumeric import _mean_dispatcher
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
pointclouds = pd.read_csv('/home/rui/Desktop/point_cloud/MAD.txt')
fuel_ = pd.read_csv('/home/rui/Desktop/point_cloud/FUEL.txt')
weather = pd.read_csv('/home/rui/Desktop/point_cloud/Weather.txt')

mad_mean = pointclouds['sim'].mean()
fuel_data = fuel_['sim']
weather_data = weather['sim']
transformation = 'transformation7_'
mad_path = 6.281847344778836

def validate_map (pd,testname):
    print("*****Start",testname, "Testing*****")
    for i in range (1,6):
        count = 0
        test_lambda = i*0.5
        dataset = pd['sim']
        for i in dataset:
            error = 1 - i
            if error > (test_lambda*(1-mad_mean)):
            # print ("Testcase",i," is an abnormal behavior !! while lambda is:",test_lambda)
                count = count +1
        print("There are totally ",count,"/",pd.shape[0] ,"number of abnormal behavior in",testname ,"while lambda is:",test_lambda )

def eucliDist(a,b):
    return math.sqrt(sum([(a-b)**2 for (a,b) in zip (a,b)]))

def calcu_traj_len(dataset):
    path_distance = 0
    for j in range(0,dataset.shape[0]-1):
        position_x = dataset.iloc[j].at['x']
        position_y = dataset.iloc[j].at['y']
        position_z = - dataset.iloc[j].at['z']

        position_x1 = dataset.iloc[j+1].at['x']
        position_y1 = dataset.iloc[j+1].at['y']
        position_z1 = - dataset.iloc[j+1].at['z']
        dis = eucliDist([position_x,position_y,position_z],[position_x1,position_y1,position_z1])
        path_distance = path_distance + dis
    return path_distance

def validate_traj(T,index):
    # total_distance = 0
    # total_error = 0
    original_data = pd.read_csv('/home/rui/Desktop/data/' + T + '/original/cmd_1.txt')
    orig_length = calcu_traj_len(original_data)
    for i in range(1,index):
        # path_distance = 0
        testcase = 'testcase' + str(i)
        path = '/home/rui/Desktop/data/' + T + '/' + testcase + '/cmd_1.txt'
        dataset = pd.read_csv(path)
        length = calcu_traj_len(dataset)
        if os.path.exists('/home/rui/Desktop/data/' + T + '/' + testcase + '/Collision.txt'):
            print ("Testcase",i," have collision !!")
        else:
            if abs(length - orig_length)>mad_path:
                print ("Testcase",i," is an abnormal behavior !!")
   
        # print ("The flight distance of ", testcase,"is: ",path_distance)
        # total_distance = total_distance + path_distance
    # return total_error


# mean_path = validate_traj(transformation, 31)/30
# mean_path = 94.80156347671524
# print (mean_path)

validate_map(fuel_,"FUEL")
validate_map(weather,"Weather")
validate_traj(transformation, 11)



# fig = plt.figure()
# ax = fig.gca(projection='3d')
# print (position_x)
# print (position_z)
# print (position_y)
# # # set figure information
# ax.set_title("3D_Curve")
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")

# # draw the figure, the color is r = read
# figure1 = ax.plot(x_data, y_data, z_data, c='r')
# figure2 = ax.plot(x_odo, y_odo, z_odo, c='b')
# plt.show()
