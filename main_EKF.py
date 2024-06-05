import time
import pyrealsense2 as rs
import numpy as np
import sys
from numpy import dot
import matplotlib.pyplot as plt
sys.path.insert(0,"helper")
from filterpy.kalman import ExtendedKalmanFilter
from helper.plotHelper import initArrays, PlotHelper
from helper.Quaternion import ObjectOrientation
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import vpython as vp
from helper.virtualIMU import virtualIMU
import csv

#vIMU = virtualIMU()
mg = Madgwick()
mg_links = Madgwick()
Q = np.array([1.0  ,0.0, 0.0, 0.0])
Q2 = np.array([1.0, 0.0, 0.0, 0.0])


#------------------------------------------------------Arrays Init--------------------------------------------------


accel_rechts = np.empty((0, 3))
accel_links = np.empty((0, 3))
gyro_links = np.empty((0, 3))
gyro_rechts = np.empty((0, 3))
average_gyro  = np.empty((0, 3))
average_accel = np.empty((0, 3))
ekf_accel_links = np.empty((0, 3))
trans_Accel_links = np.empty((0, 3))
orientation_rechts = np.empty((0, 4))
average_accel_EKF = np.array([])

PositionX = np.array([])
Positiony = np.array([])
PositionZ = np.array([])

kf_rightrrayXaccel = np.array([])
kf_rightrrayYaccel = np.array([])
kf_rightrrayZaccel = np.array([])
kf_rightrrayFrame = np.array([])


velocityArray = np.empty((0, 3))
velocityArray_Links = np.empty((0, 3))
positionXYZL = np.empty((0, 3))
positionXYZA = np.empty((0, 3))

initial_orientierung = np.array([0,0,0])
kf_rightrrayX, kf_rightrrayY, kf_rightrrayZ, timeInMiliSec, arrayX, arrayY, arrayZ, arrayFrame, arrayAverageX, arrayAverageY, arrayAverageZ, arraySecond, arraySecondY, arraySecondZ, arraySecondFrame, arrayAccelRightX, arrayAccelRightY, arrayAccelRightZ, arrayAccelLeftX, arrayAccelLeftY, arrayAccelLeftZ = initArrays()


#------------- Some Helper functions------------------------------
def get_gyro(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])

def get_accel(accel):
    return np.asarray([accel.x, accel.y, accel.z])


def appendArray(x,y,z,input):
    x = np.append(x,input[0])
    y = np.append(y,input[1])
    z = np.append(z,input[2])
    return x,y,z

def plotStuff(ax, x,y,z,frame,label):
    ax.plot(frame,x, color="red", label="X") 
    ax.plot(frame,y, color="green", label="Y")  
    ax.plot(frame,z, color="blue",label="Z") 
    ax.set_ylabel(label)
    ax.legend()


def getCSVData(row):
    d = row.__next__()
    gyro = np.array([float(d[0]),float(d[1]),float(d[2])])
    gyro2 = np.array([float(d[3]),float(d[4]),float(d[5])])
    accel = np.array([float(d[6]),float(d[7]),float(d[8])])
    accel2 = np.array([float(d[9]),float(d[10]),float(d[11])])
    timestamp = float(d[12])
    timestamp2= float(d[13])
    return accel,accel2,gyro,gyro2,timestamp,timestamp2

def getCSVData2(row):
    d = row.__next__()
    data = np.array([float(d[0]),float(d[1]),float(d[2])])
    timestamp = float(d[3])
    return data,timestamp


file_accel1 = open('Daten/rig_in_trolley/accel_stream_1.csv', "r",newline='')
file_gyro1 = open('Daten/rig_in_trolley/gyro_stream_1.csv', "r",newline='')
file_accel2 = open('Daten/rig_in_trolley/accel_stream_2.csv', "r",newline='')
file_gyro2 = open('Daten/rig_in_trolley/gyro_stream_2.csv', "r",newline='')
#Known guess for IMU Orientation
Q = np.array([ 0.7071068, 0., -0.7071068, 0. ])
Q2 = np.array([ 0.7071068, 0., -0.7071068, 0. ]) 

dR_accel1 = csv.reader(file_accel1,delimiter=";",quotechar="|")
dR_gyro1 = csv.reader(file_gyro1,delimiter=";",quotechar="|")
dR_accel2 = csv.reader(file_accel2,delimiter=";",quotechar="|")
dR_gyro2 = csv.reader(file_gyro2,delimiter=";",quotechar="|")

#Used to get data from pipeline instead of csv
def getData(pipeline,pipeline2):

    frames = pipeline.wait_for_frames()
    gyro = get_gyro(frames[2].as_motion_frame().get_motion_data())
    accel = get_accel(frames[1].as_motion_frame().get_motion_data()) 
    
        
    frames2 = pipeline2.wait_for_frames()
    gyro2 = get_gyro(frames2[2].as_motion_frame().get_motion_data())
    accel2 = get_accel(frames2[1].as_motion_frame().get_motion_data())
    return gyro,accel,frames,gyro2,accel2,frames2

# Integration functions
def integrate_acceleration(velocity, acceleration, delta_t):
    return velocity + acceleration * delta_t

def integrate_velocity(position, velocity, delta_t):
    return position + velocity * delta_t 

#Fusion Function
def fusion(ekf1, ekf2, master):
    P1_inv = np.linalg.inv(ekf1.P)
    P2_inv = np.linalg.inv(ekf2.P)
    PM_inv = np.linalg.inv(master.P)

    P_fused_inv = P1_inv + PM_inv + P2_inv
    Pf = np.linalg.inv(P_fused_inv)
    #https://www.researchgate.net/publication/277929452_Modified_federated_Kalman_filter_for_INSGNSSCNS_integration
    Xf = np.dot(Pf, np.dot(PM_inv, master.x) + np.dot(P1_inv, ekf1.x) + np.dot(P2_inv, ekf2.x))    

    #compute the fusion weights 
    master.x = Xf
    master.P = Pf
    

timestampold = 0
timestampold2 = 0
dt = 0
timestampold = 0
sampleTime = 0
sampleTime_links= 0
timestampold_links = 0
#Postition = initial_position

pos = np.array([[0,0,0]])

#-------------------------------------Vpython for Real time Plots---------------------------

gyroscale = 0.05
accelscale = 0.1
Graph1 = PlotHelper().initGraph("Accel FLipped",3,accelscale)
Graph2 = PlotHelper().initGraph("Raw Accel",3,accelscale)

Graph3 = PlotHelper().initGraph("Raw Velocity",3,accelscale)
Graph4 = PlotHelper().initGraph("Gyro Links Raw",3,accelscale)

Graph6 = PlotHelper().initGraph("X EKF",3,accelscale)
Graph7 = PlotHelper().initGraph("Y EKF",3,gyroscale)
Graph8 = PlotHelper().initGraph("X Velocity EKF",3,20)
Graph5 = PlotHelper().initGraph("X Velocity EKF ",3,20)

Graph9 = PlotHelper().initGraph("Position",3,10)
Graph10 = PlotHelper().initGraph("Orientierung",3,10)
Graph11 = PlotHelper().initGraph("Accel EKF Data",3,gyroscale)

PosGraph = vp.graph(title='Pos',align ="right")
xcurve = vp.gdots(color=vp.color.red,graph=PosGraph,label="X" )


QuaternionGraph = vp.graph(title='Orientation Quaternion',align ="right",width=800, xmax=60, xmin=1, scroll=True)
orientationQ1Curve = vp.gcurve(color=vp.color.red,graph=QuaternionGraph,label="X", legend=True )
orientationQXCurve = vp.gcurve(color=vp.color.red,graph=QuaternionGraph,label="Q2", legend=True )
orientationQYCurve = vp.gcurve(color=vp.color.green,graph=QuaternionGraph,label="Q3",legend=True)
orientationQZCurve = vp.gcurve(color=vp.color.blue,graph=QuaternionGraph,label="Q4", legend=True)

QuaternionGraph2 = vp.graph(title='Orientation Comp Filter Quaternion',align ="right",width=800, xmax=60, xmin=1, scroll=True)
orientationQ1Curve2 = vp.gcurve(color=vp.color.orange,graph=QuaternionGraph2,label="X", legend=True )
orientationQXCurve2 = vp.gcurve(color=vp.color.red,graph=QuaternionGraph2,label="Q2", legend=True )
orientationQYCurve2 = vp.gcurve(color=vp.color.green,graph=QuaternionGraph2,label="Q3",legend=True)
orientationQZCurve2 = vp.gcurve(color=vp.color.blue,graph=QuaternionGraph2,label="Q4", legend=True)

imuOrientierung = ObjectOrientation()
qOld = np.zeros(3)
orientationOld =  np.zeros(3)





    

    
#------------------------------------------------------------Master  FILTER ---------------------------------------------------------------------------------------------------------
def h_m(x):
    return x

def H_jacobian_m(x):     
    return np.array([
                  [0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,1]
                 ])
# Measurement function
def Hx_m(x):
      return np.array([x[6],x[7],x[8]])

dim_x = 9  # Dimension of the state vector 
dim_z = 3  # Dimension of the measurement vector
kf_master = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z)
theta = np.array([0,0,0])
# Set initial state
initial_state = np.array([0,0,0,0,0,0,0,0,0])  # Initial 
kf_master.x = initial_state
# Set state transition and measurement functions
   # Identity matrix 
kf_master.P = np.array([[1,0,0,0,0,0,0,0,0],
                  [0,1,0,0,0,0,0,0,0],
                  [0,0,1,0,0,0,0,0,0],
                  [0,0,0,1,0,0,0,0,0],
                  [0,0,0,0,1,0,0,0,0],
                  [0,0,0,0,0,1,0,0,0],
                  [0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,1],
                 ]) 
# Measurement noise covariance matrix, noise at the sensor itself
kf_master.R = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]
                 ])
kf_master.Q  = np.array([ [1,0,0,0,0,0,0,0,0],
                            [0,1,0,0,0,0,0,0,0],
                            [0,0,1,0,0,0,0,0,0],
                            [0,0,0,1,0,0,0,0,0],
                            [0,0,0,0,1,0,0,0,0],
                            [0,0,0,0,0,1,0,0,0],
                            [0,0,0,0,0,0,1,0,0],
                            [0,0,0,0,0,0,0,1,0],
                            [0,0,0,0,0,0,0,0,1],
                            ])
kf_master.HJacobian = H_jacobian_m
kf_master.Hx = Hx_m

#------------------------------------------------------------KALMAN FILTER ---------------------------------------------------------------------------------------------------------
# Function to map the state space to measurement space
def h(x):
    return x

def H_jacobian_left(x):     
    return np.array([
                  [0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,1]
                 ])
# Measurement function
def Hx(x):
      return np.array([x[6],x[7],x[8]])

dim_x = 9  # Dimension of the state vector 
dim_z = 3  # Dimension of the measurement vector
kf_left = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z)
theta = np.array([0,0,0])
# Set initial state
initial_state = np.array([0,0,0,0,0,0,0,0,0])  # Initial 
kf_left.x = initial_state
# Set state transition and measurement functions
   # Identity matrix 
kf_left.P = np.array([[1,0,0,0,0,0,0,0,0],
                  [0,1,0,0,0,0,0,0,0],
                  [0,0,1,0,0,0,0,0,0],
                  [0,0,0,1,0,0,0,0,0],
                  [0,0,0,0,1,0,0,0,0],
                  [0,0,0,0,0,1,0,0,0],
                  [0,0,0,0,0,0,0.01,0,0],
                  [0,0,0,0,0,0,0,0.01,0],
                  [0,0,0,0,0,0,0,0,0.01],
                 ]) 
kf_left.P = np.array([[1,0,0,0,0,0,0,0,0],
                  [0,1,0,0,0,0,0,0,0],
                  [0,0,1,0,0,0,0,0,0],
                  [0,0,0,1,0,0,0,0,0],
                  [0,0,0,0,1,0,0,0,0],
                  [0,0,0,0,0,1,0,0,0],
                  [0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,1],
                 ]) 
# Measurement noise covariance matrix, noise at the sensor itself
kf_left.R = np.array([[0.1,0,0],
                  [0,0.1,0],
                  [0,0,0.1]
                 ]) 


kf_left.Q = np.array([ [0.001,0,0,0,0,0,0,0,0],
                            [0,0.001,0,0,0,0,0,0,0],
                            [0,0,0.001,0,0,0,0,0,0],
                            [0,0,0,0.10,0,0,0,0,0],
                            [0,0,0,0,0.00010,0,0,0,0],
                            [0,0,0,0,0,0.10,0,0,0],
                            [0,0,0,0,0,0,0.00010,0,0],
                            [0,0,0,0,0,0,0,0.00010,0],
                            [0,0,0,0,0,0,0,0,0.00010],
                            ])   

kf_left.HJacobian = H_jacobian_left
kf_left.Hx = Hx


#------------------------------------------------------------KALMAN FILTER ---------------------------------------------------------------------------------------------------------
# Function to map the state space to measurement space
def h(x):
    return x

def H_jacobian_right(x):     
        return np.array([
                  [0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,1]
                 ])
# Measurement function
def Hxa_right(x):
        return np.array([x[6],x[7],x[8]])

dim_x = 9  # Dimension of the state vector 
dim_z = 3  # Dimension of the measurement vector
kf_right = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z)
# Set initial state
initial_state = np.array([0,0,0,0,0,0,0,0,0])  
kf_right.x = initial_state
# Set state transition and measurement functions
kf_right.F = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]
                 ])   
# state covariance matrix
kf_right.P = np.array([[0.1,0,0,0,0,0,0,0,0],
                  [0,0.1,0,0,0,0,0,0,0],
                  [0,0,0.1,0,0,0,0,0,0],
                  [0,0,0,0.1,0,0,0,0,0],
                  [0,0,0,0,0.1,0,0,0,0],
                  [0,0,0,0,0,0.1,0,0,0],
                  [0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,1],
                 ]) 

kf_right.Q  = np.array([ [0.001,0,0,0,0,0,0,0,0],
                            [0,0.001,0,0,0,0,0,0,0],
                            [0,0,0.001,0,0,0,0,0,0],
                            [0,0,0,0.10,0,0,0,0,0],
                            [0,0,0,0,0.00010,0,0,0,0],
                            [0,0,0,0,0,0.10,0,0,0],
                            [0,0,0,0,0,0,0.00010,0,0],
                            [0,0,0,0,0,0,0,0.00010,0],
                            [0,0,0,0,0,0,0,0,0.00010],
                            ]) 
# Measurement noise covariance matrix, noise at the sensor itself

kf_right.R = np.array([[0.1,0,0],
                  [0,0.1,0],
                  [0,0,0.1]
                 ]) 

kf_right.HJacobian = H_jacobian_right
kf_right.Hx = Hxa_right

try:

    while True:

        accel,tsa1 = getCSVData2(dR_accel1)
        accel2,tsa2 = getCSVData2(dR_accel2)
        gyro,tsg1 = getCSVData2(dR_gyro1)
        gyro2,tsg2 = getCSVData2(dR_gyro2)
        #Store data in Arrays for post process plots
        accel_rechts = np.append(accel_rechts,accel)
        accel_links = np.append(accel_links,accel2)
        gyro_rechts = np.append(gyro_rechts,gyro)
        gyro_links = np.append(gyro_links,gyro2) 
        ts = tsa1
        ts2 = tsa2

        #Calculate Mean beforehand and subtract the bias
        accel -= np.array([0.03352462056443676,-0.1594900996253734,-0.21846269177])
        accel2 -= np.array([0.09098343345380965,-0.1740056225586505,0.101])

        #calculate time between frames
        if(sampleTime == 0):
            sampleTime = ts

        timestamp = ts
        sampleTime = (timestamp - sampleTime) /1000
        if(timestampold == 0):
            timestampold = timestamp

        if(sampleTime_links == 0):
            sampleTime_links = ts2

        timestamp_links = ts2
        sampleTime_links = (timestamp_links - sampleTime_links) /1000
        if(timestampold_links == 0):
            timestampold_links = timestamp_links

        dt2 = timestampold_links - timestampold_links
        dt = timestamp - timestampold
        ac2 = accel
        dt = dt/1000
        #--------------------------------------------CompFilter für Angle Estimation----------------------------
        accelcf = np.array([accel[2],-accel[0],-accel[1]])
        gyrocf = np.array([gyro[2],-gyro[0],-gyro[1]])
        accel2cf = np.array([accel2[2],-accel2[0],-accel2[1]])
        gyro2cf = np.array([gyro2[2],-gyro2[0],-gyro2[1]])

        mg.setDt(sampleTime)
        mg_links.setDt(sampleTime_links)
        Q = mg.updateIMU(Q,gyr=gyrocf,acc=accelcf)
        Q2 = mg_links.updateIMU(Q2,gyr=gyro2cf,acc=accel2cf)

        #rotate accel and get rid of gravity
        rotated_accel = imuOrientierung.rotate_acceleration(accelcf,np.array([Q[0],Q[1],Q[2],Q[3]]))
        rotated_accel2 = imuOrientierung.rotate_acceleration(accel2cf,np.array([Q2[0],Q2[1],Q2[2],Q2[3]]))
        rotated_accel -= np.array([0,0,9.81])
        rotated_accel2 -= np.array([0,0,9.81])
        
        #------------------------------------------------------------KF FILTER  ---------------------------------------------------------------------------------------------------------

        kf_left.F = np.array([  [1,0,0,sampleTime_links,0,0,0,0,0],
                            [0,1,0,0,sampleTime_links,0,0,0,0],
                            [0,0,1,0,0,sampleTime_links,0,0,0],
                            [0,0,0,1,0,0,sampleTime_links,0,0],
                            [0,0,0,0,1,0,0,sampleTime_links,0],
                            [0,0,0,0,0,1,0,0,sampleTime_links],
                            [0,0,0,0,0,0,1,0,0],
                            [0,0,0,0,0,0,0,1,0],
                            [0,0,0,0,0,0,0,0,1]
                 ])
    
        filtered_data = kf_left.x

        kf_rightrrayX, kf_rightrrayY, kf_rightrrayZ = appendArray(kf_rightrrayX, kf_rightrrayY, kf_rightrrayZ,np.array([filtered_data[0],filtered_data[1],filtered_data[2]]))
        kf_rightrrayFrame = np.append(kf_rightrrayFrame, dt)
        kf_right.F = np.array([  [1,0,0,sampleTime,0,0,0,0,0],
                            [0,1,0,0,sampleTime,0,0,0,0],
                            [0,0,1,0,0,sampleTime,0,0,0],
                            [0,0,0,1,0,0,sampleTime,0,0],
                            [0,0,0,0,1,0,0,sampleTime,0],
                            [0,0,0,0,0,1,0,0,sampleTime],
                            [0,0,0,0,0,0,1,0,0],
                            [0,0,0,0,0,0,0,1,0],
                            [0,0,0,0,0,0,0,0,1]
                 ])

        kf_master.F = np.array([  [1,0,0,sampleTime_links,0,0,0,0,0],
                            [0,1,0,0,sampleTime_links,0,0,0,0],
                            [0,0,1,0,0,sampleTime_links,0,0,0],
                            [0,0,0,1,0,0,sampleTime_links,0,0],
                            [0,0,0,0,1,0,0,sampleTime_links,0],
                            [0,0,0,0,0,1,0,0,sampleTime_links],
                            [0,0,0,0,0,0,1,0,0],
                            [0,0,0,0,0,0,0,1,0],
                            [0,0,0,0,0,0,0,0,1]
                 ])

        if(dt > 2): #Wait for Madgwick Filter get right orientation
            kf_left.predict(u=0)
            kf_left.update(z=rotated_accel2, HJacobian=H_jacobian_left, Hx=Hx)
            kf_master.predict(u=0)
            kf_right.predict(u=0)
            kf_right.update(z=rotated_accel, HJacobian=H_jacobian_right, Hx=Hxa_right)
        filtered_data_master = kf_master.x
        filtered_data_right = kf_right.x

        #-------Append Arrays for data collection------------------>>
        ekf_accel_links = np.append(ekf_accel_links,np.array([filtered_data_master[6],filtered_data_master[7],filtered_data_master[8]]))
        kf_rightrrayXaccel, kf_rightrrayYaccel, kf_rightrrayZaccel = appendArray(kf_rightrrayXaccel, kf_rightrrayYaccel, kf_rightrrayZaccel,np.array([filtered_data_right[6],filtered_data_right[7],filtered_data_right[8]]))
        
        arrayX,arrayY,arrayZ = appendArray(arrayX,arrayY,arrayZ, gyro) 
        timeInMiliSec = np.append(timeInMiliSec,dt)
        velocityKF = np.array([filtered_data_right[0],filtered_data_right[1],filtered_data_right[2]])
        arrayAccelRightX, arrayAccelRightY, arrayAccelRightZ = appendArray(arrayAccelRightX, arrayAccelRightY, arrayAccelRightZ,rotated_accel)   
        trans_Accel_links = np.append(trans_Accel_links,rotated_accel2)
        
        velocityR = np.array([filtered_data_right[3],filtered_data_right[4],filtered_data_right[5]])
        velocityL = np.array([filtered_data[3],filtered_data[4],filtered_data[5]])
        velocityA = np.array([filtered_data_master[3],filtered_data_master[4],filtered_data_master[5]])
        velocityArray_Links = np.append(velocityArray_Links,velocityA)
        velocityArray = np.append(velocityArray,velocityR)

        position = np.array([filtered_data_right[0],filtered_data_right[1],filtered_data_right[2]])
        positionL = np.array([filtered_data[0],filtered_data[1],filtered_data[2]])
        positionM = np.array([filtered_data_master[0],filtered_data_master[1],filtered_data_master[2]])
        positionXYZL = np.append(positionXYZL,positionL)
        positionXYZA = np.append(positionXYZA,positionM)
        PositionX , Positiony, PositionZ = appendArray(PositionX, Positiony, PositionZ, position)

        sampleTime = timestamp
        sampleTime_links = timestamp_links

        orientation_rechts = np.append(orientation_rechts,Q2)

#------------------------------------------------------------Virtual IMU and realtime plotting-----------------------
        #Graph2.plotGraph(dt,magnitude,0,0)
        # Graph1.plotGraph(dt,accelcf[0],accelcf[1],accelcf[2])
        
        #Graph3.plotGraph(dt,velocity[0],y=velocity[1],z=velocity[2])
        #Graph4.plotGraph(dt,gyro2[0],gyro2[1],gyro2[2])
        # Graph6.plotGraph(dt,filtered_data[6],y=filtered_data_right[6],z=filtered_data_master[6])
        # Graph7.plotGraph(dt,filtered_data[7],y=filtered_data_right[7],z=filtered_data_master[7])

        # Graph5.plotGraph(dt,filtered_data[3],y=filtered_data_right[3],z=filtered_data_master[3])
        # Graph8.plotGraph(dt,filtered_data[4],y=filtered_data_right[4],z=filtered_data_master[4])
        # Graph9.plotGraph(dt,filtered_data[5],y=filtered_data_right[5],z=filtered_data_master[5])
        #euler = R.from_quat(Q).as_euler("xyz",degrees=True)

        
        xcurve.plot(filtered_data_right[0],filtered_data_right[1])
        xcurve.plot(filtered_data[0],filtered_data[1])
        xcurve.plot(filtered_data_master[0],filtered_data_master[1])
        #xcurve.plot(filtered_data_master[0],filtered_data_master[1])
        #xcurve.plot(filtered_data[0],filtered_data[1])
        # orientationQ1Curve2.plot(dt,Q[0])
        # orientationQXCurve2.plot(dt,Q[1])
        # orientationQYCurve2.plot(dt,Q[2])
        # orientationQZCurve2.plot(dt,Q[3])
        # orientationQ1Curve.plot(dt,Q2[0])
        # orientationQXCurve.plot(dt,Q2[1])
        # orientationQYCurve.plot(dt,Q2[2])
        # orientationQZCurve.plot(dt,Q2[3])

        rotation = R.from_quat(np.array([Q2[1],Q2[2],Q2[3],Q2[0]]))
        stuff = R.as_quat(rotation)
        rotation = rotation.as_euler('xyz',degrees=True)
        rotation_change = rotation - initial_orientierung
        initial_orientierung = rotation
        
        fusion(kf_right,kf_left,kf_master)
        
        #Graph10.plotGraph(dt,rotation[0],y=rotation[1],z=rotation[2])
        #print(rotation)
        #vIMU.set_orientation(rotation_change)
        #vIMU.set_position(position)

        #time.sleep(0.005)
        


         
finally:  #Reshape arrays and plot the needed data to visualize

    
    accel_links = np.reshape(accel_links,(-1,3))
    accel_rechts = np.reshape(accel_rechts,(-1,3))
    gyro_rechts = np.reshape(gyro_rechts,(-1,3))
    average_accel = np.reshape(average_accel,(-1,3))
    trans_Accel_links = np.reshape(trans_Accel_links,(-1,3))
    ekf_accel_links = np.reshape(ekf_accel_links,(-1,3))
    positionXYZL = np.reshape(positionXYZL,(-1,3))
    positionXYZA = np.reshape(positionXYZA,(-1,3))
    velocityArray = np.reshape(velocityArray,(-1,3))
    velocityArray_Links = np.reshape(velocityArray_Links,(-1,3))
    orientation_rechts = np.reshape(orientation_rechts,(-1,4))
    zeros = np.zeros(len(timeInMiliSec),dtype=float)
    
    trajectory2D, ax2D = plt.subplots()
    #ax2D.scatter(positionXYZA[:,0:1], positionXYZA[:,1:2],label="Trajectory")
    ax2D.scatter(PositionX, Positiony,label="Trajectory Right")
    ax2D.scatter(positionXYZL[:,0:1],positionXYZL[:,1:2],label="Trajectory Left")
    ax2D.scatter(positionXYZA[:,0:1],positionXYZA[:,1:2],label="Trajectory Fused")
    #ax2D.scatter(PositionX, Positiony,label="Trajectory")

    ax2D.set_xlabel("X")
    ax2D.set_ylabel("Y")
    ax2D.grid(True,"both","both")
    ax2D.legend()
    #plt.savefig("Pictures/2DEKF_data_geradeaus_fahren.png.png")  

    figBA, (aEKF,aVel,aOrient) = plt.subplots(3,1)

    aOrient.plot(timeInMiliSec,ekf_accel_links[:,0:1], label="X")
    aOrient.plot(timeInMiliSec,ekf_accel_links[:,1:2], label="Y")
    aOrient.plot(timeInMiliSec,ekf_accel_links[:,2:3], label="Z")
 
    aOrient.set_ylabel("accel (m/s^2)")
    aOrient.set_xlabel("Time in seconds") 
    aOrient.grid(True,"both","both")
    aOrient.legend() 
   
    aVel.plot(timeInMiliSec,velocityArray_Links[:,0:1],label="X")
    aVel.plot(timeInMiliSec,velocityArray_Links[:,1:2],label="Y")
    aVel.plot(timeInMiliSec,velocityArray_Links[:,2:3],label="Z")
    aVel.set_ylabel("Velocity (m/s)")
    aVel.set_xlabel("Time in seconds")
    aVel.grid(True,"both","both")
    aVel.legend()   

    
    aEKF.plot(timeInMiliSec,positionXYZA[:,0:1], label="Y")
    aEKF.plot(timeInMiliSec,positionXYZA[:,1:2], label="X")
    aEKF.plot(timeInMiliSec,positionXYZA[:,2:3], label="Z")
    
    aEKF.set_ylabel("Position (m)")
    aEKF.set_xlabel("Time in seconds")
    aEKF.legend()   
    aEKF.grid(True,"both","both")
    #print(positionM[:,0:1],)
    #plt.savefig("Pictures/EKF_data_geradeaus_fahren.png")

    fig3d, ax3d = plt.subplots(subplot_kw={"projection": "3d"})
    ax3d.scatter(PositionX, Positiony, PositionZ,label="Trajectory Right")
    ax3d.scatter(positionXYZL[:,0:1], positionXYZL[:,1:2],positionXYZL[:,2:3],label="Trajectory Left")
    ax3d.scatter(positionXYZA[:,0:1], positionXYZA[:,1:2],positionXYZA[:,2:3],label="Trajectory Fused ")
    ax3d.legend()

    #plt.savefig("Pictures/3DEKF_data_geradeaus_fahren.png")  
    plt.show()
    

