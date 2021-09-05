
import airsim
from airsim.types import Pose, Vector3r, Quaternionr
import time
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True,"Drone1")
# client.armDisarm(True)
client.simDestroyObject("Drone1")
# pose = airsim.Pose(airsim.Vector3r(10, 0, -1.5), airsim.to_quaternion(0, 0, 0))
# client.takeoffAsync(vehicle_name="Drone2").join()
# pose = client.simGetVehiclePose()
# obj = client.simSpawnObject("obj1", "CheckeredGate16x16", pose, 0.75, True)


# pose = client.simGetObjectPose("Car_21")
# print (pose)

# client.simSetObjectScale("DeerBothBP2_19", Vector3r(2, 2, 2))
# client.simSetObjectPose("DeerBothBP2_19", pose,teleport=True)

















# client.moveToZAsync(10, 1).join()
# import setup_path
# import airsim
# import sys
# import time

# # For high speed ascent and descent on PX4 you may need to set these properties:
# # param set MPC_Z_VEL_MAX_UP 5
# # param set MPC_Z_VEL_MAX_DN 5

# z = 5
# if len(sys.argv) > 1:
#     z = float(sys.argv[1])

# client = airsim.MultirotorClient()
# client.confirmConnection()
# client.enableApiControl(True)

# client.armDisarm(True)

# landed = client.getMultirotorState().landed_state
# if landed == airsim.LandedState.Landed:
#     print("taking off...")
#     client.takeoffAsync().join()
# else:
#     print("already flying...")
#     client.hoverAsync().join()

# print("make sure we are hovering at {} meters...".format(z))

# if z > 5:
#     # AirSim uses NED coordinates so negative axis is up.
#     # z of -50 is 50 meters above the original launch point.
#     client.moveToZAsync(-z, 5).join()
#     client.hoverAsync().join()
#     time.sleep(5)

# if z > 10:
#     print("come down quickly to 10 meters...")
#     z = 10
#     client.moveToZAsync(-z, 3).join()
#     client.hoverAsync().join()

# print("landing...")
# client.landAsync().join()
# print("disarming...")
# client.armDisarm(False)
# client.enableApiControl(False)
# print("done.")