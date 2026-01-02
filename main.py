import numpy as np
import pybullet as p
import pybullet_data
from numpy import sin, cos
from psa import PSA
import time
if p.isConnected():
    p.disconnect()

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)


arm = p.loadURDF(
    "ur_description/urdf/ur10_robot.urdf",
    basePosition=[0, 0, 0],
    useFixedBase=True
)
num_joints = p.getNumJoints(arm)





import ikpy.chain
my_chain = ikpy.chain.Chain.from_urdf_file(
"ur_description/urdf/ur10_robot.urdf"
)

def FP(*angles):

    

    ikpy_angles = [0] + list(angles) + [0]  
    

    ikpy_transform = my_chain.forward_kinematics(ikpy_angles)
    ikpy_pos = ikpy_transform[:3, 3]
    

    return np.round(ikpy_pos, 2)

PSA_solver=PSA(dim=6,fk=FP)
def solve(target=[0.18,0.25,0.011]):
    best_parameters=PSA_solver.optimize(iterations=1200,N=900,a=0.99,b=1.3,c=1.8,target=target)
    return best_parameters

def move(joint_angles):
    for i in range(1,num_joints-1):
        # print(i,joint_angles[i-1])
        p.resetJointState(arm, i, joint_angles[i-1])
    ee_link_index = num_joints - 1
    ee_state = p.getLinkState(arm, ee_link_index, computeForwardKinematics=True)
    ee_pos = ee_state[4]  
    print(ee_pos)

cur_pose=[1.1842999458312988, 0.2561410069465637, 0.011600001715123653]
while True:
    p.stepSimulation()
    keys=p.getKeyboardEvents()
  
    if not keys:
        continue
    print(keys)
    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        cur_pose[1] += 0.5
    if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        cur_pose[1] -= 0.5
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        cur_pose[0] -= 0.5
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        cur_pose[0] += 0.5
    if 65297 in keys and keys[65297]==3:
        cur_pose[2]+=0.5
    if 65298 in keys and keys[65298]==3:
        cur_pose[2]-=0.5
    bp=solve(cur_pose)
    move(bp)
    time.sleep(1/240)
    print("target",cur_pose)
    ee_link_index = num_joints - 1
    ee_state = p.getLinkState(arm, ee_link_index, computeForwardKinematics=True)
    ee_pos = ee_state[4]  
    print("current pose",ee_pos)

