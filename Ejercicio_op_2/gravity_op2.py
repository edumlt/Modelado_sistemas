import pybullet as p 
import pybullet_data
import time
import math as m

#diametro de la esfera
DIAMETRO = 0.5

# Tiempo del step
STEP = 1./240.

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planetId = p.loadURDF("plane.urdf")

euler_angles = [0,0,0]
starOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0,0,3]

ballId = p.loadURDF("sphere2.urdf", startPosition, starOrientation)

# datos iniciales
gravity = -9.81
v_init = 0.0

while (True):

    # posición en cada instante
    pos, orn = p.getBasePositionAndOrientation(ballId)
    z_init = pos[2]

    if  z_init > DIAMETRO:

        z_pos = z_init + v_init*STEP + 1/2*gravity*m.pow(STEP, 2) # Posición siguiente en un intervalo step
        v_init = v_init + gravity*STEP # velocidad en caída libre 
        
        position = [pos[0],pos[1],z_pos]

        p.resetBasePositionAndOrientation(ballId, position, orn)

    else :
        v_init = 0.0

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()