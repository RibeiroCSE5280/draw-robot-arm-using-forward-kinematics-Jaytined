#!/usr/bin/env python
# coding: utf-8


from vedo import *
import numpy as np

def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
	
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij


def forward_kinematics(Phi, L1, L2, L3, L4):
    arm_location = np.array([[3], [2], [0.0]])
    radius = 0.4

    # First joint angle
    phi1 = Phi[0]

    # Matrix of Frame 1
    R_01 = RotationMatrix(phi1, axis_name='z')
    t_01 = arm_location
    T_01 = getLocalFrameMatrix(R_01, t_01)

    # Second joint angle
    phi2 = Phi[1]

    # Matrix of Frame 2
    R_12 = RotationMatrix(phi2, axis_name='z')
    # Origin of the Frame
    t_12 = np.array([[L1+2*radius], [0.0], [0.0]]) 
    T_12 = getLocalFrameMatrix(R_12, t_12)

    T_02 = T_01 @ T_12

    # Third joint angle
    phi3 = Phi[2]

    # Matrix of Frame 3
    R_23 = RotationMatrix(phi3, axis_name='z')
    t_23 = np.array([[L2+2*radius], [0.0], [0.0]]) # Frame's origin
    T_23 = getLocalFrameMatrix(R_23, t_23)

    T_03 = T_01 @ T_12 @ T_23

    # Fourth joint angle
    phi4 = Phi[3]

    # Matrix of Frame 4
    R_34 = RotationMatrix(phi4, axis_name='z')
    t_34 = np.array([[L3+radius], [0.0], [0.0]]) 

    T_34 = getLocalFrameMatrix(R_34, t_34)

    T_04 = T_01 @ T_12 @ T_23 @ T_34

    e = T_04[0:3,-1]

    return T_01, T_02, T_03, T_04, e

def main():
    # Set the limits of the graph x, y, and z ranges 
    axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))

    # Joint angles and lengths of arm parts
    L1, L2, L3, L4 = [5, 8, 3, 0]
    Phi = np.array([-30, 50, 30, 0])

    # Compute forward kinematics
    T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
    print(e)

    # Testing
    expected = np.array([18.47772028,  4.71432837,  0. ])
    print(expected)

    # Visualize
    show([axes], viewup="z").close()
	

if __name__ == '__main__':
    main()

