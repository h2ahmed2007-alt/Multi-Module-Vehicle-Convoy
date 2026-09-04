from Kinematics import Kinematics
import numpy as np
import math

class Four_Wheel_Omni_kinematics(Kinematics):
    def __init__(self,L,W,R):
        super().__init__(L,W,R)
        #wheel 1 at 45 deg,wheel 2 at 135 deg,wheel 3 at 225 deg,wheel 4 at 315 deg
        #R=radius
        #L=distance from center of the robot to the center of the wheel
        self.M_forward = np.array([
            [-(math.sqrt(2)*R)/4, -(math.sqrt(2)*R)/4, (math.sqrt(2)*R)/4, (math.sqrt(2)*R)/4],
            [(math.sqrt(2)*R)/4, -(math.sqrt(2)*R)/4, -(math.sqrt(2)*R)/4, (math.sqrt(2)*R)/4],
            [R/(4*L), R/(4*L), R/(4*L), R/(4*L)]
            ])

        self.M_inverse = np.array([
            [-(math.sqrt(2)/2)/R, (math.sqrt(2)/2)/R, L/R],
            [-(math.sqrt(2)/2)/R, -(math.sqrt(2)/2)/R, L/R],
            [(math.sqrt(2)/2)/R, -(math.sqrt(2)/2)/R, L/R],
            [(math.sqrt(2)/2)/R, (math.sqrt(2)/2)/R, L/R]
            ])


    # Input wheel speeds:
    def forward(self, w):
        omega_array = np.array([w[0], w[1], w[2], w[3]])
        forward_result = self.M_forward @ omega_array
        return forward_result
 
    def inverse(self, vx, vy, wz):
        velocity_array = np.array([vx, vy, wz])
        inverse_result = self.M_inverse @ velocity_array
        return inverse_result
