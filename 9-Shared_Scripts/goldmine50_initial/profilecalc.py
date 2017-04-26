import math, numpy
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

global dm, m0
dm = -(66.93+403.5350) #rate of fuel consumption kg/s
m01 = 107.671*(10.0**3.0) #mass at 0m/s
m0 = m01
mwet = 85.98*(10.0**3.0) #mass of first stage fuel tank with fuel
mdry = 5.342*(10.0**3.0) #mass of first stage fuel tank without fuel

fw = (mwet-mdry) #total mass of fuel used after 100m/s
tf = -fw/dm #time of burn

def pressure(y):#atmospheric pressure
    #Fitted from EarthAtmoDataRSS.mat in 1-Simulation/MATLAB and Simulink Models
    p1 = 1*(10**(-39))
    p2 = -6.750*(10**(-34))
    p3 = 1.9328*(10**(-28))
    p4 = -3.0598*(10**(-23))
    p5 = 2.922*(10**(-18))
    p6 = -1.7208*(10**(-13))
    p7 = 6.1036*(10**(-9))
    p8 = -0.00011966
    p9 = 1.0031

    pres = p1*(y**8) + p2*(y**7) + p3*(y**6) + p4*(y**5)+ p5*(y**4) + p6*(y**3) + p7*(y**2) + p8*y + p9

    if pres<0:
        pres = 0
    
    return pres

def rho(y): #atmospheric density
    #Fitted from EarthAtmoDataRSS.mat in 1-Simulation/MATLAB and Simulink Models
    p1 = -6.955*(10**(-35))
    p2 = 4.2501*(10**(-29))
    p3 = -1.0784*(10**(-23))
    p4 = 1.468*(10**(-18))
    p5 = -1.1537*(10**(-13))
    p6 = 5.2193*(10**(-9))
    p7 = -1.2567*(10**(-4))
    p8 = 1.2456

    density = p1*(y**7) + p2*(y**6) + p3*(y**5) + p4*(y**4)+ p5*(y**3) + p6*(y**2) + p7*y + p8

    if density<0:
        density = 0
    
    return density

def ISP(y):#ISP calculation
    vacuum_isp = 363
    sealevel_isp = 453
    calc_isp = vacuum_isp+(sealevel_isp - vacuum_isp)*pressure(y)

    return calc_isp

def Thrust(dm,y):#Thrust calculation
    thrott_limit = 0.755#change as necessary, can even be a function
    return -dm*ISP(y)*thrott_limit*9.81

def Fric(y,v): #aerodynamic drag
    Cd = 0.65#drag coefficient
    Ref_Area = 10.9114#cross-sectional area
    return 0.5*rho(y)*v*v*Cd*Ref_Area

r = 6371000 #Earth radius

def g(y): #gravity
    return 3.986*(10.0**14.0)/((y+r)*(y+r))

def dv(y,psi,v,t): #acceleration
    steering_loss = 0
    if v>100:
        steering_loss = 0#numpy.deg2rad(0.25)
    return Thrust(dm,y)*numpy.cos(steering_loss)/(m0+dm*t) - g(y)*math.cos(psi) - Fric(y,v)/(m0+dm*t)

def dpsi(y,psi,v): #change in pitch angle
    steering_loss = 0
    if v>100:
        steering_loss = 0#numpy.deg2rad(0.25)
    return g(y)*math.sin(psi)/v + Thrust(dm,y)*numpy.sin(steering_loss)/v

def model(v, a, b): #fitting model
    return a*numpy.arctan(b*(v))

y0 = 41#launch pad height
v0 = 0.001#non-zero initial velocity for algorithm
psi0 = 0.0#initial pitch angle


incr = 0.01#increment for while loop

#storage arrays
H = [] #Height
V = [] #Velocity
Psi = [] #Pitch angle(actually 90-pitch angle)
T = [] #Time

t1=0.0
while v0<=100: #Simple Euler method for solving
    y1 = y0 + v0*math.cos(psi0)*incr #new value for height
    psi1 = psi0 + dpsi(y0,psi0,v0)*incr #new value for pseudo-pitch
    v1 = v0 + dv(y0,psi0,v0,t1)*incr #new value for velocity

    #setting up for next loop
    y0 = y1
    psi0 = psi1
    v0 = v1

    #storing
    H.append(y0)
    V.append(v0)
    T.append(t1)
    t1+=incr
    
print("Initial Height: %f \t" % y0)
print("Initial velocity: %f \t" % v0)
print("Initial pitch: %f \t" % (90-math.degrees(psi0)))

psi0 = math.radians(2.0) #varied quantity, pitch-over angle

#storage arrays
H = [] #Height
V = [] #Velocity
Psi = [] #Pitch angle(actually 90-pitch angle)
T = [] #Time

while t1<tf: #Simple Euler method for solving
    y1 = y0 + v0*math.cos(psi0)*incr #new value for height
    psi1 = psi0 + dpsi(y0,psi0,v0)*incr #new value for pseudo-pitch
    v1 = v0 + dv(y0,psi0,v0,t1)*incr #new value for velocity

    #setting up for next loop
    y0 = y1
    psi0 = psi1
    v0 = v1

    #storing
    H.append(y0)
    V.append(v0)
    Psi.append(math.degrees(psi0))
    T.append(t1)
    t1+=incr
    


popt, pcov = curve_fit(model, V, Psi, bounds = ([0,0],[100,1]))
c = 0

while numpy.abs(model(100+c, *popt)-math.radians(1.5))>0.001 and c>=-100:
    c += -0.001

params = popt

params[0] = math.radians(params[0])

print("Parameters for model: a*arctan(b*(v+c)) \n")
print("a: %f" % params[0])
print("b: %f" % params[1])
print("c: %f" % c)
print("Final Height: %f \t" % y0)
print("Final velocity: %f \t" % v0)
print("Final pitch: %f \t" % (90-math.degrees(psi0)))

tAp = v0*math.cos(psi0)/g(y0)
print("Time to Apoapsis: %f mins\t" % (tAp/60))
h = v0*math.cos(psi0)*tAp - 0.5*g(y0)*tAp*tAp
print("Predicted Apoapsis: %f m\t" % (y0+h))
