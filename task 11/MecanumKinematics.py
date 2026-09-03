from Kinematics import Kinematics
import numpy as np

class MecanumKinematics(Kinematics):
    
    def __init__(self, L, W ,R):
        super().__init__(L, W, R)



      # ---->forward matrix coefficients  
        self.M_forward = np.array([
            [R/4, R/4, R/4, R/4],
            [R/4, -R/4, -R/4, R/4],
            [R/(4*(L+W)), -R/(4*(L+W)), R/(4*(L+W)), -R/(4*(L+W))]
        ])

     # ---->  inverse matrix coefficients  
        self.M_inverse = np.array([
           [1/R, 1/R, (L+W)/R],
           [1/R, -1/R, -(L+W)/R],
           [1/R, -1/R, (L+W)/R],
           [1/R, 1/R, -(L+W)/R]
        ])

    # Input wheel speeds: 
    # [omega_FL, omega_FR, omega_RL, omega_RR]
    def forward(self, w):
           
        omega_array = np.array([w[0],w[1],w[2],w[3]])
        forward_result = self.M_forward @ omega_array 
        return forward_result
        
        
        
    def inverse(self, vx, vy, wz): 
        velocity_array = np.array([vx,vy,wz])
        inverse_result = self.M_inverse @ velocity_array 
        return inverse_result

