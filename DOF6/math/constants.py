import numpy as np

"""
Table 1: Constants used in Dynamic Models
"""
global gamma
global R
global Xi_not
global mu_not
global C
global Me
global rE
global Y
global P
global R
global Ixx
global Iyy
global Izz


gamma = 1.4 #ratio of specific heats for air
R = 287 #Gas Constant for air
Xi_not = 291.15 #<k>  reference temperature
mu_not = 1.827*10**-7 #<Ps * s> Reference Dynamic Viscosity
C = 120 #<k> Sutherland's constant
Me = 5.974*10**24 #<kg> Mass of Earth
rE = 6.673*10**-11 #<m^3/kg*s> ugc
YA = np.array([1,0,0]) #reference yaw axis
PA = np.array([0,1,0]) #reference pitch axis
RA = np.array([0,0,1]) #reference roll axis


