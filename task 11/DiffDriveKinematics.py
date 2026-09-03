from Kinematics import Kinematics
import numpy as np

class DiffDriveKinematics(Kinematics):
    
    def __init__(self, L, W ,R):
        super().__init__(L, W, R)



     ----> # forward matrix coefficients  
        self.M_forward = np.array([
            [R/2 ,R/2 ],  
            [0, 0],
            [R/L, -R/L]
        ])

     ----> # inverse matrix coefficients  
        self.M_inverse = np.array([
            [1/R ,0 , L/(2*R)],
            [1/R ,0 , -L/(2*R)]

        ])


    def forward(self, w):
        omeage_array = np.array([w[0],w[1]])
        result = omeage_array @ self.M_forward
        return result
        
        
        
    def inverse(self, vx, vy, wz): 







