# Test vorwärts Kinematic mit DH
# sollte ergeben [-0.4565, 0, 0.6655]
###### FUNKTIONIERT NICHT wie es soll

#### TCP ermitteln ##############################################################
import numpy as np

# DH-Parameter für UR3e (in Metern und Radians)
dh_params = [
    {'a': 0,      'd': 0.1519, 'alpha': np.pi/2},
    {'a': 0.2435, 'd': 0,      'alpha': 0},
    {'a': 0.213,  'd': 0,      'alpha': 0},
    {'a': 0,      'd': 0.11235, 'alpha': np.pi/2},
    {'a': 0,      'd': 0.08535, 'alpha': -np.pi/2},
    {'a': 0,      'd': 0.0921, 'alpha': 0}
]

def compute_forward_kinematics(joint_angles):
    T = np.eye(4)    
    # DH-Transformationen
    for i in range(6):
        theta = joint_angles[i]
        a = dh_params[i]['a']
        d = dh_params[i]['d']
        alpha = dh_params[i]['alpha']
        
        Ti = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d               ],
            [0,              0,                            0,                           1               ]
        ])
        T = np.dot(T, Ti) # Matrixmultiplikation
    
    # Korrekturmatrix **nach** den DH-Transformationen
    R_correct = np.array([
        [0, 1, 0, 0],    # X_Webots = Y_DH
        [-1, 0, 0, 0],   # Y_Webots = -X_DH
        [0, 0, 1, 0],    # Z bleibt gleich
        [0, 0, 0, 1]
    ])
    #T = np.dot(T, R_correct)

    # Nach der Koordinatensystem-Korrektur:
    T_offset = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -0.174],  # 174 mm in Z-Richtung
        [0, 0, 0, 1]
    ])  
    #T = np.dot(T, T_offset)
    
    return T
# sollte ergeben [-0.4565, 0, 0.6655]
#ergibt TCP-Position: [-0.4569  -0.02025  0.06655]
#### TCP ermitteln ##############################################################

current_joint_angles = [0, 0, 0, 0, 0, 0] # => sollte ergeben [-0.4565, 0, 0.6655]
#                                                     ergibt  [0.4565, -0.19425, 0.06655]
T = compute_forward_kinematics(current_joint_angles)
print("TCP-Position:", T[:3, 3])  # Erwartet: [-0.4565, 0, 0.6655]