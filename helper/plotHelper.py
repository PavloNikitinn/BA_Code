import numpy as np
import matplotlib.pyplot as plt
import vpython as vp
#Helper class to init graphs in main file 
class PlotHelper:
    def __init_(self):
        return self
    

    def initGraph(self, title, number, *args):
        self.graph = vp.graph(title=title,align ="left" ,width=800, xmax=20, xmin=0, scroll=True)
        
        if(number == 3):
            self.curve1 = vp.gcurve(color=vp.color.red,graph=self.graph,label="X", legend=True)
            self.curve2 = vp.gcurve(color=vp.color.green,graph=self.graph,label="Y", legend=True )
            self.curve3 = vp.gcurve(color=vp.color.blue,graph=self.graph,label="Z", legend=True)#
        if(number == 4):
            self.curve1 = vp.gcurve(color=vp.color.red,graph=self.graph,label="W", legend=True )
            self.curve2 = vp.gcurve(color=vp.color.green,graph=self.graph,label="X", legend=True )
            self.curve3 = vp.gcurve(color=vp.color.blue,graph=self.graph,label="Y", legend=True )
            self.curve4 = vp.gcurve(color=vp.color.blue,graph=self.graph,label="Z", legend=True )
        return self  

    def plotGraph(self, dt, x,y = None, z =None , w=None):
        self.curve1.plot(dt,x)
        if(y != None):
            self.curve2.plot(dt,y)
        if(w != None):
            self.curve2.plot(dt,w)
        if(z != None):
            self.curve3.plot(dt,z)

        return self

def plot3(dt, x,y,z):
    return

def initArrays():
    kfarrayX = np.array([])
    kfarrayY = np.array([])
    kfarrayZ = np.array([])
    kfarrayFrame = np.array([])


    arrayX = np.array([])
    arrayY = np.array([])
    arrayZ = np.array([])
    arrayFrame = np.array([])

    arrayAverageX = np.array([])
    arrayAverageY = np.array([])
    arrayAverageZ = np.array([])
    arrayAverageFrame = np.array([])

    arraySecond = np.array([])
    arraySecondY = np.array([])
    arraySecondZ = np.array([])
    arraySecondFrame = np.array([])


    arrayAccelRightX = np.array([])
    arrayAccelRightY = np.array([])
    arrayAccelRightZ = np.array([])

    arrayAccelLeftX = np.array([])
    arrayAccelLeftY = np.array([])
    arrayAccelLeftZ = np.array([])
    return kfarrayX,kfarrayY,kfarrayZ,kfarrayFrame,arrayX,arrayY,arrayZ,arrayFrame,arrayAverageX,arrayAverageY,arrayAverageZ,arraySecond,arraySecondY,arraySecondZ,arraySecondFrame,arrayAccelRightX,arrayAccelRightY,arrayAccelRightZ,arrayAccelLeftX,arrayAccelLeftY,arrayAccelLeftZ
