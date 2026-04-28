import numpy as np
import random

class MatrixMath:
    #pointsA and pointsB are lists of 4 points in the form [[x,y,z],[x,y,z],[x,y,z],[x,y,z]]
    def __init__(self, pointsA, pointsB):
        self.A = self.createMatrix(pointsA[0],pointsA[1],pointsA[2],pointsA[3])
        self.B = self.createMatrix(pointsB[0],pointsB[1],pointsB[2],pointsB[3])
        self.T = self.solveConversionMatrix(self.A,self.B)  

    def convertToHom(self,vec):
        v = vec[:]
        return np.append(v,1)

    def convertToCart(self,hom):
        x = hom[0]/hom[3]
        y = hom[1]/hom[3]
        z = hom[2]/hom[3]
        vec = np.array([x,y,z])
        return vec

    def createMatrix(self,vec1,vec2,vec3,vec4):
        return np.array([self.convertToHom(vec1),
                        self.convertToHom(vec2),
                        self.convertToHom(vec3),
                        self.convertToHom(vec4)])

    def solveConversionMatrix(self,A, B):
        invA = np.linalg.inv(A)
        return np.matmul(invA,B)

    def translateA_B(self,point):
        hom = self.convertToHom(point)
        result = np.matmul(hom,self.T)
        return self.convertToCart(result)

    def translateB_A(self,point):
        hom = self.convertToHom(point)
        result = np.matmul(hom,np.linalg.inv(self.T))
        return self.convertToCart(result)

    #Helper Math Functions for moving robots 

    #Combine xyz vec to a rotation vec to feed to robot
    def addRot(self,point, rot):
        return np.append(point,rot)

    #Direction 0 = x, 1 = y, 2 = z
    def scoot(self,a, scoot, direction = 2):
        copy = a[:]
        copy[direction] = copy[direction] + scoot
        return copy

