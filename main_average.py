import time
import pyrealsense2 as rs
import numpy as np
import sys
from numpy import dot
import matplotlib.pyplot as plt
sys.path.insert(0,"helper")
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
mg_av = Madgwick()
Q = np.array([1.0  ,0.0, 0.0, 0.0])
Q2 = np.array([1.0, 0.0, 0.0, 0.0])
Q3 = np.array([1.0, 0.0, 0.0, 0.0])



initial_orientierung = np.array([0,0,0])

#------------------------------------------------------Arrays Init--------------------------------------------------


accel_rechts = np.empty((0, 3))
accel_links = np.empty((0, 3))
gyro_links = np.empty((0, 3))
gyro_rechts = np.empty((0, 3))
average_gyro  = np.empty((0, 3))
average_accel = np.empty((0, 3))
trans_Accel_links = np.empty((0, 3))

PositionX = np.array([])
Positiony = np.array([])
PositionZ = np.array([])
timeInMiliSec = np.array([])
positionXYZL = np.empty((0, 3))
positionXYZA = np.empty((0, 3))

initial_velocity = np.array([0, 0, 0])  
initial_position = np.array([0, 0, 0]) 
initial_velocityL = np.array([0, 0, 0])  
initial_positionL = np.array([0, 0, 0])
initial_velocityA = np.array([0, 0, 0])  
initial_positionA = np.array([0, 0, 0])
initial_velocity_links = np.array([0, 0, 0])  
initial_position_links = np.array([0, 0, 0])  
velocityArray = np.empty((0, 3))
velocityArray_Links = np.empty((0, 3))
velocityX = np.array([])
velocityZ = np.array([])


timestampold = 0
timestampold_links = 0
dt = 0
timestampold_links = 0
sampleTime = 0
sampleTime_links= 0

kfarrayX, kfarrayY, kfarrayZ, timeInMiliSec, arrayX, arrayY, arrayZ, arrayFrame, arrayAverageX, arrayAverageY, arrayAverageZ, arraySecond, arraySecondY, arraySecondZ, arraySecondFrame, arrayAccelRightX, arrayAccelRightY, arrayAccelRightZ, arrayAccelLeftX, arrayAccelLeftY, arrayAccelLeftZ = initArrays()

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

file_accel1 = open('Daten/rig_in_trolley/accel_stream_1.csv', "r",newline='')
file_gyro1 = open('Daten/rig_in_trolley/gyro_stream_1.csv', "r",newline='')
file_accel2 = open('Daten/rig_in_trolley/accel_stream_2.csv', "r",newline='')
file_gyro2 = open('Daten/rig_in_trolley/gyro_stream_2.csv', "r",newline='')
#Known guess for IMU Orientation
Q = np.array([ 0.7071068, 0., -0.7071068, 0. ])
Q2 = np.array([ 0.7071068, 0., -0.7071068, 0. ]) 
Q3 = np.array([ 0.7071068, 0., -0.7071068, 0. ]) 

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

def getCSVData2(row):
    d = row.__next__()
    data = np.array([float(d[0]),float(d[1]),float(d[2])])
    timestamp = float(d[3])
    return data,timestamp


Postition = initial_position

pos = np.array([[0,0,0]])
#-------------------------------------Vpython for Real time Plots---------------------------

gyroscale = 0.05
accelscale = 0.1
Graph1 = PlotHelper().initGraph("Raw Accel Data Rechts",3,accelscale)
Graph2 = PlotHelper().initGraph("Raw Accel Data Links",3,accelscale)

Graph3 = PlotHelper().initGraph("Gyro Rechts Raw",3,accelscale)
Graph4 = PlotHelper().initGraph("Gyro Links Raw",3,accelscale)

Graph5 = PlotHelper().initGraph("Transformed Accel Rechts ",3,20)
Graph6 = PlotHelper().initGraph("Transformed Accel Links",3,accelscale)
Graph7 = PlotHelper().initGraph("EKF Accel Data",3,gyroscale)
Graph8 = PlotHelper().initGraph("Velocity",3,20)
Graph9 = PlotHelper().initGraph("Position",3,10)
Graph10 = PlotHelper().initGraph("Orientierung",3,10)
Graph11 = PlotHelper().initGraph("Accel EKF Data",3,gyroscale)

PosGraph = vp.graph(title='Pos',align ="right")
xcurve = vp.gdots(color=vp.color.red,graph=PosGraph,label="X" )
ycurve = vp.gcurve(color=vp.color.green,graph=PosGraph,label="Y" )
zcurve = vp.gcurve(color=vp.color.blue,graph=PosGraph,label="Z" )

x2curve = vp.gcurve(color=vp.color.orange,graph=PosGraph,label="X2" )
y2curve = vp.gcurve(color=vp.color.yellow,graph=PosGraph,label="Y2" )
z2curve = vp.gcurve(color=vp.color.black,graph=PosGraph,label="Z2" )



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



try:

    while True:
        #gyro,accel,frames,gyro2,accel2,frames2 = getData(pipeline,pipeline2)
        accel,tsa1 = getCSVData2(dR_accel1)
        accel2,tsa2 = getCSVData2(dR_accel2)
        gyro,tsg1 = getCSVData2(dR_gyro1)
        gyro2,tsg2 = getCSVData2(dR_gyro2)
        ts = tsa1
        ts2 = tsa2

        #Calculate Mean beforehand and subtract the bias
        accel -= np.array([0.03352462056443676,-0.1594900996253734,-0.21846269177])
        accel2 -= np.array([0.09098343345380965,-0.1740056225586505,0.101])
        accel_rechts = np.append(accel_rechts,accel)
        accel_links = np.append(accel_links,accel2)
        gyro_rechts = np.append(gyro_rechts,gyro)
        gyro_links = np.append(gyro_links,gyro2) 
        average_accel_data = (accel + accel2)/2
        average_gyro_data = (gyro + gyro2) / 2
        average_gyro = np.append(average_gyro,average_gyro_data)
        
        #calculate time between frames
        if(sampleTime == 0):
            sampleTime = ts

        timestamp = ts
        sampleTime = (timestamp - sampleTime) /1000
        if(timestampold == 0):
            timestampold = timestamp

        dt = timestamp - timestampold
        
        #print(sampleTime)
        if(sampleTime_links == 0):
            sampleTime_links = ts2

        timestamp_links = ts2
        sampleTime_links = (timestamp_links - sampleTime_links) /1000
        if(timestampold_links == 0):
            timestampold_links = timestamp_links

        dt2 = timestampold_links - timestampold_links
        dt = dt/1000
        #--------------------------------------------CompFilter für Angle Estimation----------------------------
        
        accelcf = np.array([accel[2],-accel[0],-accel[1]])
        gyrocf = np.array([gyro[2],-gyro[0],-gyro[1]])
        accel2cf = np.array([accel2[2],-accel2[0],-accel2[1]])
        gyro2cf = np.array([gyro2[2],-gyro2[0],-gyro2[1]])

        accelAcf = (accelcf+accel2cf)/2
        gyroAcf = (gyrocf+gyro2cf)/2 
        mg.setDt(sampleTime)
        mg_links.setDt(sampleTime_links)

        Q = mg.updateIMU(Q,gyr=gyrocf,acc=accelcf)
        Q2 = mg.updateIMU(Q2,gyr=gyro2cf,acc=accel2cf)
        Q3 = mg_av.updateIMU(Q3,gyr=gyroAcf,acc=accelAcf)
        #rotate accel and get rid of gravity
        rotated_accel = imuOrientierung.rotate_acceleration(accelcf,np.array([Q[0],Q[1],Q[2],Q[3]]))
        rotated_accelL = imuOrientierung.rotate_acceleration(accel2cf,np.array([Q2[0],Q2[1],Q2[2],Q2[3]]))
        rotated_accelA = imuOrientierung.rotate_acceleration(accelAcf,np.array([Q3[0],Q3[1],Q3[2],Q3[3]]))
        
        rotated_accel -= np.array([0,0,9.81])
        rotated_accelA -= np.array([0,0,9.81])
        rotated_accelL -= np.array([0,0,9.81])
        #-------Append Data Arrays------------------>>
        arrayX,arrayY,arrayZ = appendArray(arrayX,arrayY,arrayZ, gyro) 
        timeInMiliSec = np.append(timeInMiliSec,dt)
        arrayAccelRightX, arrayAccelRightY, arrayAccelRightZ = appendArray(arrayAccelRightX, arrayAccelRightY, arrayAccelRightZ,rotated_accel)   
  
        if(dt> 2):  #Wait for Madgwick Filter get right orientation
            velocity = integrate_acceleration(initial_velocity,rotated_accel,sampleTime)
            velocityL = integrate_acceleration(initial_velocityL,rotated_accelL,sampleTime_links)
            velocityA = integrate_acceleration(initial_velocityA,rotated_accelA,sampleTime)
            initial_velocity = velocity
            initial_velocityL = velocityL
            initial_velocityA = velocityA
        else:
            velocity = np.array([0,0,0])
            velocityL = np.array([0,0,0])
            velocityA = np.array([0,0,0])
            initial_velocity = velocity
            initial_velocityL = velocityL
            initial_velocityA = velocityA

        position = integrate_velocity(initial_position,velocity,sampleTime)
        positionL = integrate_velocity(initial_positionL,velocityL,sampleTime_links)
        positionA = integrate_velocity(initial_positionA,velocityA,sampleTime)
        initial_position = position
        initial_positionL = positionL
        initial_positionA = positionA

#-------Append Arrays for data collection------------------>>
        #velocityArray = np.append(velocityArray, np.array([[velocity[0],velocity[1],velocity[2]]]), axis=0)
        PositionX , Positiony, PositionZ = appendArray(PositionX, Positiony, PositionZ, position)
        positionXYZL = np.append(positionXYZL,positionL)
        positionXYZA = np.append(positionXYZA,positionA)
        average_accel = np.append(average_accel,rotated_accelA)
        velocityX = np.append(velocityX, velocity[0])
        velocityZ = np.append(velocityZ, velocity[1])
        velocityArray_Links = np.append(velocityArray_Links,initial_velocityA)
        Postition = np.append(Postition, position)
        sampleTime = timestamp
        sampleTime_links = timestamp_links

#------------------------------------------------------------Virtual IMU and realtime plotting-----------------------
        # Graph1.plotGraph(dt,accelcf[0],accelcf[1],accelcf[2])
        # #Graph2.plotGraph(dt,accel2cf[0],accel2cf[1],accel2cf[2])
        # Graph3.plotGraph(dt,gyrocf[0],gyrocf[1],gyrocf[2])
        #Graph4.plotGraph(dt,gyrocf[2],gyro2cf[2],z=None)

        Graph1.plotGraph(dt,velocity[0],y=velocity[1],z=velocity[2])
        #Graph3.plotGraph(dt,gyrocf[0],gyrocf[1],gyrocf[2])

        
        Graph6.plotGraph(dt,rotated_accel[0],rotated_accel[1],z=rotated_accel[2])

        #Graph5.plotGraph(dt,velocity[0],velocity[1],velocity[2])
        #Graph8.plotGraph(dt,position[0],y=position[1],z=position[2])
        euler = R.from_quat(Q).as_euler("xyz",degrees=True)

        #Graph10.plotGraph(dt,mag,y=None,z=mag)
        xcurve.plot(position[0],position[1])
        xcurve.plot(positionL[0],positionL[1])
        # xcurve.plot(dt,velocity[0])
        # ycurve.plot(dt,velocity[1])
        # zcurve.plot(dt,velocity[2])
        # x2curve.plot(dt,velocityL[0])
        # y2curve.plot(dt,velocityL[1])
        # z2curve.plot(dt,velocityL[2])
        #xcurve.plot(positionL[0],positionL[1])
        orientationQ1Curve2.plot(dt,Q[0])
        orientationQXCurve2.plot(dt,Q[1])
        orientationQYCurve2.plot(dt,Q[2])
        orientationQZCurve2.plot(dt,Q[3])
        orientationQ1Curve.plot(dt,Q2[0])
        orientationQXCurve.plot(dt,Q2[1])
        orientationQYCurve.plot(dt,Q2[2])
        orientationQZCurve.plot(dt,Q2[3])
        rotation = R.from_quat(np.array([Q3[1],Q3[2],Q3[3],Q3[0]]))

        stuff = R.as_quat(rotation)
        rotation = rotation.as_euler('xyz',degrees=True)
        rotation_change = rotation - initial_orientierung
        initial_orientierung = rotation
        
        length2 = np.linalg.norm(accel)
        unit_vec2 = accel / length2
        #Graph10.plotGraph(dt,rotation[0],y=rotation[1],z=rotation[2])
        #print(rotation)
        #vIMU.set_orientation(rotation_change)
        #vIMU.set_position(position)

        #time.sleep(0.001)



         
finally: #Reshape arrays and plot the needed data to visualize

    accel_links = np.reshape(accel_links,(-1,3))
    accel_rechts = np.reshape(accel_rechts,(-1,3))
    gyro_rechts = np.reshape(gyro_rechts,(-1,3))
    average_accel = np.reshape(average_accel,(-1,3))
    trans_Accel_links = np.reshape(trans_Accel_links,(-1,3))
    positionXYZL = np.reshape(positionXYZL,(-1,3))
    positionXYZA = np.reshape(positionXYZA,(-1,3))
    velocityArray_Links = np.reshape(velocityArray_Links,(-1,3))

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
    #plt.savefig("Pictures/2DRAW_data_geradeaus_fahren.png.png")  

    figBA, (aEKF,aVel,aOrient) = plt.subplots(3,1)
    aOrient.plot(timeInMiliSec,average_accel[:,0:1], label="X", zorder=3)
    aOrient.plot(timeInMiliSec,average_accel[:,1:2], label="Y",zorder=2.5)
    aOrient.plot(timeInMiliSec,average_accel[:,2:3], label="Z",zorder=2)
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
    #plt.savefig("Pictures/RAW_data_geradeaus_fahren.png")

    fig3d, ax3d = plt.subplots(subplot_kw={"projection": "3d"})
    ax3d.scatter(PositionX, Positiony, PositionZ,label="Trajectory Right")
    ax3d.scatter(positionXYZL[:,0:1], positionXYZL[:,1:2],positionXYZL[:,2:3],label="Trajectory Left")
    ax3d.scatter(positionXYZA[:,0:1], positionXYZA[:,1:2],positionXYZA[:,2:3],label="Trajectory Fused ")
    ax3d.legend()
    #plt.savefig("Pictures/3DRAW_data_geradeaus_fahren.png")  
    plt.show()
    

