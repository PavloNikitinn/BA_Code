import numpy as np
import csv
import matplotlib.pyplot as plt

#helper functions
def getCSVData(row):
    d = row
    accel = np.array([float(d[0]),float(d[1]),float(d[2])])
    accel2 = np.array([float(d[3]),float(d[4]),float(d[5])])
    gyro = np.array([float(d[6]),float(d[7]),float(d[8])])
    gyro2 = np.array([float(d[9]),float(d[10]),float(d[11])])
    timestamp = float(d[12])
    timestamp2= float(d[13])
    return accel,accel2,gyro,gyro2,timestamp,timestamp2

def reshapeArrays(accel,accel2,gyro,gyro2):
            
    accel_R = np.reshape(accel,(-1,3))
    accel_L = np.reshape(accel2,(-1,3))
    gyro_R = np.reshape(gyro,(-1,3))
    gyro_L = np.reshape(gyro2,(-1,3))
    return accel_R, accel_L, gyro_R, gyro_L

def sliceArrays(array):
    return array[:,0:1],array[:,1:2],array[:,2:3]

def plot_slope_line(taus, ads, slope,slope2):
    # convert to log
    log10_tau = np.log10(taus)
    log10_ad = np.log10(ads)
    
    # diff log values
    dlogtau = np.diff(log10_tau)
    dlogad = np.diff(log10_ad)
    slopes = dlogad / dlogtau

    # find value where slope fits
    idx = np.argmin(np.abs(slopes - slope))
    idxRRW = np.argmin(np.abs(slopes - slope2))
    
    # find y value 
    b = log10_ad[idx] - slope * log10_tau[idx]
    bRRW = log10_ad[idxRRW] - slope2 * log10_tau[idxRRW]
    
    #calculate N at tau 1
    logN = slope*np.log(1)+b
    N = 10**logN
    lineN = N / np.sqrt(taus)

    #calculate K at tau 3
    logK = slope2 * np.log10(3) + bRRW
    K = 10**logK
    lineK = K * np.sqrt(taus/3)

    return N, lineN, K,lineK


def plot(x,y,z):
    return x,y,z

def AllanDeviation(data, fs, maxNumM):
    ts= 1.0 / fs
    #ts= 0.03
    N = len(data)
    maxM = 2**np.floor(np.log2(N / 2))
    m = np.logspace(np.log10(1), np.log10(maxM), maxNumM)
    m = np.ceil(m).astype(int)  # m must be an integer
    m = np.unique(m)  # Remove duplicates  
    #compute tau 
    tau = m * ts
    avar = np.zeros(len(m))
    # Compute Allan variance
    for i in range(len(m)):
        mi = m[i]
        avar[i] = np.sum((data[2*mi:N] - 2*data[mi:N-mi] + data[0:N-2*mi])**2)
    # Normalize Allan variance
    avar = avar / (2 * tau**2 * (N - 2 * m))
    return (tau, np.sqrt(avar)) 

#prepare arrays for data
accel_R = np.empty((1604165,3))
accel_L = np.empty((1604165,3))
gyro_R = np.empty((1604165,3))
gyro_L =np.empty((1604165,3))
ts_R = np.empty((1604165,3))
ts_L = np.empty((1604165,3))
k = 0

#open csv and fill arrays
with open('CSVS/accel.csv',newline='') as f:
    dataReader = csv.reader(f,delimiter=";",quotechar="|")
    for row in dataReader:
        accel,accel2,gyro,gyro2,ts,ts2 = getCSVData(row)
        accel_R[k] = accel
        accel_L[k] = accel2
        gyro_R[k] = gyro
        gyro_L[k] = gyro2
        ts_R[k] = ts
        ts_L[k] = ts2
        #print(ts)
        k +=1


#left sensor
# x = gyro_L[:,0:1] * (180.0 / np.pi)  
# y = gyro_L[:,1:2] * (180.0 / np.pi)
# z = gyro_L[:,2:3] * (180.0 / np.pi)
        
#right sensor
# x = gyro_R[:,0:1] * (180.0 / np.pi)  
# y = gyro_R[:,1:2] * (180.0 / np.pi)
# z = gyro_R[:,2:3] * (180.0 / np.pi)

#average sensor
# x = (gyro_L[:,0:1]+gyro_R[:,0:1])/2 * (180.0 / np.pi) 
# y = (gyro_L[:,1:2]+gyro_R[:,1:2])/2 * (180.0 / np.pi)
# z = (gyro_L[:,2:3]+gyro_R[:,2:3])/2 * (180.0 / np.pi)



#Left sensor
x = accel_L[:,0:1]
y = accel_L[:,1:2] 
z = accel_L[:,2:3]

#right sensor
# x = accel_R[:,0:1]  
# y = accel_R[:,1:2] 
# z = accel_R[:,2:3] 

#average sensor
# x = (accel_L[:,0:1]+accel_R[:,0:1])/2
# y = (accel_L[:,1:2]+accel_R[:,1:2])/2 
# z = (accel_L[:,2:3]+accel_R[:,2:3])/2 

slope = -0.5
slope2 = 0.5
fs = 200
ts = 1.0 / fs
#ts = 0.03
thetax = np.cumsum(x) * ts 
thetay = np.cumsum(y) * ts
thetaz = np.cumsum(z) * ts

def compute(AllanDeviation, fs, thetax, thetay, thetaz):
    (taux, adx) = AllanDeviation(thetax, fs, maxNumM=200)
    (tauy, ady) = AllanDeviation(thetay, fs, maxNumM=200)
    (tauz, adz) = AllanDeviation(thetaz, fs, maxNumM=200)
    Nx, lineX, Kx, KlineX = plot_slope_line(taux, adx, slope,slope2)
    Ny, lineY, Ky, KlineY = plot_slope_line(tauy, ady, slope,slope2)
    Nz, lineZ, Kz, KlineZ = plot_slope_line(tauz, adz, slope,slope2)
    plt.plot(taux, adx, label='X',color="red")
    plt.plot(tauy, ady, label='Z',color="blue")
    plt.plot(tauz, adz, label='Y',color="green")
    plt.plot(taux, lineX, '--', label=f'Velocity Random Walk ',color ="violet")
    plt.plot(tauy, lineY, '--',color ="violet")
    plt.plot(tauz, lineZ, '--',color ="violet")
    plt.plot(taux, KlineX, 'y--', label=f'Rate Random Walk ')
    plt.plot(tauy, KlineY, 'y--')
    plt.plot(tauy, KlineZ, 'y--')

    print("Angle Random Walk:", Nx, Ny,Nz)
    print("Rate Random Walk; ", Kx, Ky, Kz)
    #plt.plot(-0.5)
    plt.xlabel(r'$\tau$')
    plt.ylabel(r'$\sigma(\tau)$')
    plt.grid(True, which="both", ls="-",lw=0.5)
    plt.legend()
    plt.xscale('log')
    plt.yscale('log')
    plt.show()


compute(AllanDeviation, fs, thetax, thetay, thetaz)
