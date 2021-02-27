#!/usr/bin/env python
import json
import numpy as np
import fire

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
    try:
        with open(file, 'r') as dataFile:
            data = json.load(dataFile)
    except json.decoder.JSONDecodeError as err:
        print("Error: Invalid JSON file format")
        exit(-1)
    except FileNotFoundError as fnf:
        print("Error: {}".format(fnf.strerror))
        exit(-1)

    return data

"""
Create array of matrices from DH
- return: MT_n = Array of matrices (3D matrix)
"""
def DH_Matrix(DH_Params, numDeg):
    try:
        dh = np.array(DH_Params, dtype=np.single).reshape(numDeg, 4)
    except:
        print("Error while converting to matrix")
        exit(-1)

    # Convert from deg to rad
    dh = degToRad(dh)

    MT_n = np.zeros([numDeg, 4, 4]).astype(np.single)
    for vec in range(numDeg):
        MT_n[vec] = dh_single(dh[vec].transpose())

    return MT_n

"""
Return the matrix of Zn
"""
def getZn(numDeg, MT_n):
    Zn = np.zeros((numDeg, 3), np.single)

    Zn[0][2] = 1
    for col in range(numDeg - 1):
        Zn[col+1] = MT_n[col, 0:3, 2]

    return np.around(Zn, decimals=3)

"""
Return the matrix On
"""
def getOn(numDeg, MT, MT_n):
    On = np.zeros((numDeg+1, 3), np.single)

    for col in range(numDeg - 1):
        On[col+1] = MT_n[col, 0:3, 3]

    On[numDeg] = MT[0:3, 3]

    return np.around(On, decimals=3)

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
Compute the Jacobian matrix for linear velocity
"""
def linearJacobian(numDeg, Zn, On):
    Jv = np.zeros([numDeg, 3], np.single)

    for vec in range(numDeg):
        Jv[vec] = np.cross(Zn[vec], On[numDeg] - On[vec])

    return Jv

"""
Compute the Jacobian matrix for angular velocity
"""
def angularJacobian(degType, numDeg, Zn):
    Jw = np.zeros([numDeg, 3], np.single)

    for vec in range(numDeg):
        Jw[vec] = degType[vec]*Zn[vec]

    return Jw

"""
Compute the linear velocity of the robot
"""
def linearVelocity(Jv, q_dot):
    v = np.matmul(Jv.transpose(), q_dot)
    return np.around(v, decimals=3)

"""
Compute the angular velocity of the robot
"""
def angularVelocity(Jw, q_dot):
    w = np.matmul(Jw.transpose(), q_dot)
    return np.around(w, decimals=3)

"""
Print results of linear and angular velocity
"""
def printResults(v, w):
    print("Linear velocity:\n{}\n".format(v))
    print("Angular velocity:\n{}\n".format(w))

"""
Main function
"""
def Analysis(filename="AnalisisVelocidad/seiko.json"):
    data = readDataFromFile(filename)

    dh_params = data["dh_params"]
    q_dot = data["q_dot"]
    degType = data["degType"]

    numberDegree = int(len(dh_params)/4)

    MT_n = DH_Matrix(dh_params, numberDegree)
    MT = np.linalg.multi_dot(MT_n)

    Zn = getZn(numberDegree, MT_n)
    On = getOn(numberDegree, MT, MT_n)

    Jv = linearJacobian(numberDegree, Zn, On)
    Jw = angularJacobian(degType, numberDegree, Zn)

    v = linearVelocity(Jv, q_dot)
    w = angularVelocity(Jw, q_dot)

    printResults(v, w)

if __name__ == "__main__":
    fire.Fire(Analysis)