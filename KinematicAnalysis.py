#!/usr/bin/env python
import json
import numpy as np

# Constant parameters
THETA = 0
D = 1
A = 2
ALFA = 3

"""
In file:
- dh_params: Matrices of the elements in the robot
- vel: Linear or angular velocity of each degree of freedom
- degType: 1=revolution, 0=prismatic
"""
def readDataFromFile(file):
    with open(file, 'r') as dataFile:
        data = json.load(dataFile)

    return data

"""
Create array of matrices from DH
- return: Array of matrices (3D matrix)
"""
def DH_Matrix(DH_Params):
    numberDegree = int(len(DH_Params)/4)
    try:
        dh = np.array(DH_Params).reshape(numberDegree, 4)
    except:
        print("Error while converting to matrix")
        exit(-1)

    # Define matrix as single point precision
    dh = dh.astype(np.single)

    # Print input DH matrix
    #print("DH:\n{}".format(dh))

    # Convert from deg to rad
    dh = degToRad(dh)

    MT_n = np.zeros([numberDegree, 4, 4]).astype(np.single)
    for vec in range(numberDegree):
        MT_n[vec] = dh_single(dh[vec])

    MT = np.linalg.multi_dot(MT_n)

    V = linearVelocity(MT_n, MT)

"""
Create a single matrix to concatenate to the principal array
"""
def dh_single(dh_params):
    theta = dh_params[THETA]
    d = dh_params[D]
    a = dh_params[A]
    alfa = dh_params[ALFA]

    return np.around(np.array([
        [np.cos(theta), -np.cos(alfa)*np.sin(theta),  np.sin(alfa)*np.sin(theta), a*np.cos(theta) ],
        [np.sin(theta),  np.cos(alfa)*np.cos(theta), -np.sin(alfa)*np.cos(theta), a*np.sin(theta) ],
        [            0,                np.sin(alfa),                np.cos(alfa),               d ],
        [            0,                           0,                           0,               1 ]
    ]), decimals=3)

"""
Convert the angles from degrees to radians of the data matrix
"""
def degToRad(mat):
    for x in range(mat.shape[0]):
        mat[x][0] = np.pi*mat[x][0]/180
        mat[x][3] = np.pi*mat[x][3]/180

    return mat

"""
"""
def linearVelocity(matrices, MT):
    pass

"""
"""
def angularVelocity(matrices, MT, degType):
    pass

"""
Main function
"""
def Analysis(filename="AnalisisVelocidad/data.json"):
    data = readDataFromFile(filename)

    dh_params = data["dh_params"]
    vel = data["velocity"]
    degType = data["degType"]

    DH_Matrix(dh_params)

if __name__ == "__main__":
    Analysis()