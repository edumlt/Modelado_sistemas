import pybullet as p 
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.setTimeStep(1./240.)
p.setRealTimeSimulation(True)

planetId = p.loadURDF("plane.urdf")

euler_angles = [0,0,0]
starOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0,0,0.1]

robotId = p.loadURDF("opcional1.urdf", startPosition, starOrientation)
p.changeDynamics(robotId, 1, localInertiaDiagonal=[0.0,0.0,(1/3)*1*0.4])

numJoints = p.getNumJoints(robotId)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
     print("{} - {}".format(p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

frictionId = p.addUserDebugParameter("joint Friction", 0, 100, 10)
torqueId = p.addUserDebugParameter("joint Torque", -20, 20, -9)


while (True):

    frictionForce = p.readUserDebugParameter(frictionId)
    jointTorque = p.readUserDebugParameter(torqueId)

    p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
    p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=jointTorque)



p.disconnect()