from Kinematics import Kinematics
import numpy as np
import math

class Three_Wheel_Omni_Kinematics(Kinematics):
    def __init__(self,L,W,R):
        super().__init__(L,W,R)
        #wheel 1 at 90 deg,wheel 2 at 210 deg,wheel 3 at 330 deg
        #R=radius
        #L=distance from center of the robot to the center of the wheel
        self.M_forward=np.array([
            [(-2*R)/3, R/3, R/3],
            [0, -R/(math.sqrt(3)), R/math.sqrt(3)],
            [R/(3*L), R/(3*L), R/(3*L)]
        ])

        self.M_inverse=np.array([
            [-1/R, 0, L/R],
            [0.5/R, (-(math.sqrt(3))/2)/R, L/R],
            [0.5/R, (math.sqrt(3)/2)/R, L/R]
        ])

    #input wheel speeds:
    def forward(self,w):
        omega_array=np.array([w[0], w[1], w[2]])
        forward_result=self.M_forward @ omega_array
        return forward_result
 
    def inverse(self,vx,vy,wz):
        velocity_array=np.array([vx, vy, wz])
        inverse_result=self.M_inverse @ velocity_array
        return inverse_result


        