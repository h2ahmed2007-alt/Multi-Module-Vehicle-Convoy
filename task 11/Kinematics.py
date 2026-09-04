import numpy as np
import math

class Kinematics :
    def __init__(self, L, W, R):
        self.L = L
        self.W = W
        self.R = R

class DiffDriveKinematics(Kinematics):
    
    def __init__(self, L, W ,R):
        super().__init__(L, W, R)



      # ---->forward matrix coefficients  
        self.M_forward = np.array([
            [R/2 ,R/2 ],  
            [0, 0],
            [R/L, -R/L]
        ])

     # ---->  inverse matrix coefficients  
        self.M_inverse = np.array([
            [1/R ,0 , L/(2*R)],
            [1/R ,0 , -L/(2*R)]

        ])


    def forward(self, w):
        omega_array = np.array([w[0],w[1]])
        forward_result = self.M_forward @ omega_array 
        return forward_result
        
        
        
    def inverse(self, vx, vy, wz): 
        velocity_array = np.array([vx,vy,wz])
        inverse_result = self.M_inverse @ velocity_array 
        return inverse_result

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

        
