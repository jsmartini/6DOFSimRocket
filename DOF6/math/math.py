import numpy as np
from DOF6.math.constants import *

def Qvector(theta, a_vec) -> np.array:
    """
    calculates the quarternion vector [s, v^]
    Equation (1) in paper
    :param theta: rotation in radians around axis of rotation (a)
    :param a_vec: axis of rotation vector
    :return: [s, v^] -> Q^
    """
    s = np.cos(theta/2)
    nu_x = np.sin(theta/2) * a_vec[0]
    nu_y = np.sin(theta/2) * a_vec[1]
    nu_z = np.sin(theta/2) * a_vec[2]
    return np.array([s, nu_x, nu_y, nu_z])

def RotationMatrix(theta, a_vec) -> np.array:
    """
    equation (2) in paper
    :param theta:
    :param a_vec:
    :return: rotation matrix
    """
    Q = Qvector(theta, a_vec)
    return np.array([
        [1 - 2*Q[2]**2 - 2*Q[3]**2, 2*Q[1]*Q[2] - 2*Q[0]*Q[3], 2*Q[1]*Q[3]+2*Q[0]*Q[2]],
        [2*Q[1]*Q[2] + 2*Q[0]*Q[3], 1 - 2*Q[1]**2 - 2*Q[3]**2, 2*Q[2]*Q[3] - 2*Q[0]*Q[1]],
        [2*Q[1]*Q[3] - 2*Q[0]*Q[2], 2*Q[2]*Q[3] + 2*Q[0]*Q[1], 1 - 2*Q[1]**2 - 2*Q[2]**2]
    ])

"""
computes the yaw, pitch, and roll based on reference vectors in DOF6.math.constants.
"""
#eq3
def YawA(R: np.array) -> np.array:
    return np.multiply(R, YA.T)
#eq3
def PitchA(R: np.array) -> np.array:
    return np.multiply(R, PA.T)
#eq3
def RollA(R: np.array) -> np.array:
    return np.multiply(R, RA.T)

"""
Computes Rocket Properties
"""
def LinearVelocityVector(P:np.array)->np.array:
    #P = velocity vector * mass at current time
    #calculates X dot
    return P/Me

def BuildInertiaTensor(Ixx, Iyy, Izz):
    """
    Generates reference inertial matrix Equation (6)
    :param Ixx:
    :param Iyy:
    :param Izz:
    :return: I_not
    """
    return np.array([
        [Ixx, 0, 0],
        [0, Iyy, 0],
        [0, 0, Izz]
    ])

def AngularVelocity(R, I, L):
    """
    Calculates angular velocity of the rocket Eq. (5)
    :param R: Rotation Matrix (np.array)
    :param I: Inertial Matrix (np.array)
    :param L: Angular Momentum vector (np.array)
    :return: omega: angular velocity vector
    """
    return R * np.linalg.inv(I) * R.T * L.T

def AngleOfAttack(V: np.array, Ra: np.array) -> float:
    """
    Calculates alpha, angle of attack
    :param V: Current Velocity vector
    :param Ra: Current roll vector (RollA calculation)
    :return: alpha, angle of attack
    """
    return np.arccos(np.dot(np.linalg.norm(V), Ra))




