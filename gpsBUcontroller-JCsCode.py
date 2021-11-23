# -*- coding: utf-8 -*-
"""231_project_JC5a.ipynb
"""

# # IN COLAB install required dependencies
import sys
IN_COLAB = 'google.colab' in sys.modules
if IN_COLAB:
  !pip install polytope
  !pip install -q pyomo
  !apt-get install -y -qq glpk-utils
  !apt-get install -y -qq coinor-cbc
  !wget -N -q "https://ampl.com/dl/open/ipopt/ipopt-linux64.zip"
  !unzip -o -q ipopt-linux64
  !pip install ipympl
  !pip install isochrones

"""<center><img src='kinematic_bicycle_model.png' width="300"/> </center>

\begin{align}
\dot{x} &= v \cos(\psi+\beta)\\ 
\dot{y} &= v \sin(\psi+\beta)\\
\dot{v} &= a \\
\dot{\psi} &= \frac{v}{l_r} \sin(\beta)\\
\beta &= \tan^{-1} \left( \frac{l_r}{l_f+l_r}  \tan(\delta_f)\right)
\end{align}

\begin{align}
x &= \text{ global x coordinate} \nonumber \\
y &= \text{ global y coordinate} \nonumber \\
v &= \text{ speed of the vehicle} \nonumber \\
\psi &= \text{ global heading angle} \nonumber \\
\beta &= \text{ angle of the current velocity with respect to the longitudinal axis of the car} \nonumber \\
a &= \text{ acceleration of the center of mass into this direction} \nonumber \\
l_r &= \text{ distance from the center of mass of the vehicle to the rear axle} \nonumber \\
l_f &= \text{ distance from the center of mass of the vehicle to the front axle} \nonumber\\
\delta_f &= \text{ steering angle of the front wheels with respect to the longitudinal axis of the car} \nonumber
\end{align}

# <font color=blue> MPC design </font>


Consider a simplified kinematic model:

\begin{align}
x(k+1) &= x(k) + v \cos(\psi+\beta)\\ 
y(k+1) &= y(k) + v \sin(\psi+\beta)\\
v(k+1) &= v(k) + a(k) \\
\psi(k+1) &= \psi(k) + \frac{v(k)}{l_r} \sin(\beta)\\
\end{align}

The state and input constraints are
\begin{equation}\begin{aligned}
\mathcal{U}: &-1\leq a(k)\leq 1 \\ 
&-\pi/3\leq \psi(k)\leq \pi/3 \\
\mathcal{Z}: &\begin{bmatrix} -300\\ -300\\ -10\\ -\pi/2 \end{bmatrix} \leq z(k) \leq \begin{bmatrix} 300\\ 300\\ 80\\ \pi/2 \end{bmatrix}
\end{aligned} 
\label{eq:con1}\end{equation}

In this problem, we will design several MPC controllers of the form: 


\begin{equation*}
   \begin{array}{lll}
       J_0^*(z_0)=  &\displaystyle{\min_{{U_0}}}& \displaystyle{ {(z_N-z_D)'P(z_N-z_D)} + \sum_{k=0}^{{N}-1}
  （z_k-z_D)'Q（z_k-z_D)+（u_k-u_{k+1})'R(u_k-u_{k+1}))} \\
    \end{array}
\end{equation*}

In this problem, $Q=\begin{bmatrix} 10 & 0 & 0 & 0\\ 0 & 10 & 0 & 0\\ 0 & 0 & 5 & 0\\ 0 & 0 & 0 & 20 \end{bmatrix}$, $R=\begin{bmatrix} 1 & 0\\ 0 & 1 \end{bmatrix}$, and $P = \begin{bmatrix} 5 & 0 & 0 & 0\\ 0 & 5 & 0 & 0\\ 0 & 0 & 5 & 0\\ 0 & 0 & 0 & 20 \end{bmatrix}$

where
\begin{align}
x &= \text{ global x coordinate} \nonumber \\
y &= \text{ global y coordinate} \nonumber \\
v &= \text{ speed of the vehicle} \nonumber \\
\psi &= \text{ global heading angle} \nonumber \\
\beta &= \text{ angle of the current velocity with respect to the longitudinal axis of the car} \nonumber \\
a &= \text{ acceleration of the center of mass into this direction} \nonumber \\
l_r &= \text{ distance from the center of mass of the vehicle to the rear axle} \nonumber \\
l_f &= \text{ distance from the center of mass of the vehicle to the front axle} \nonumber\\
\ \nonumber
\end{align}
"""

# import IDMPY5a
# myplanner = IDMPY5a.IDMPlanner(4.0)
# egoState = [[0,0]]  #[s,v]
# current_acc = 0
# [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc)
# print("t:",t_sub)
# print("v:",spl_v_plan_val)
# print("s:",spl_s_plan_val)

#***************** model dynamics *****************

import matplotlib.pyplot as plt
import numpy as np
from numpy import sin, cos, tan, arcsin, arctan, random

#************* for project use *********************

def carModel(a1, x1, y1, v1, psi1, df1, dt):
    
    lf = lr = 1.738 #*********lf = |cg-front axle|, lr = |cg-rear axle|
    beta = arctan((lr/(lf+lr))*tan(df1))
    xdt = v1*cos(psi1 + beta)
    ydt = v1*sin(psi1 + beta)
    psidt = (v1/lr)*sin(beta)
    vdt = a1
    
    x2 = x1 + xdt*dt 
    y2 = y1 + ydt*dt
    psi2 = psi1 + psidt*dt
    v2 = v1 + vdt*dt
    
    return x2, y2, v2, psi2


def carModelNoise(a1, x1, y1, v1, psi1, df1, dt):
    
    lf = lr = 1.738 #*********lf = |cg-front axle|, lr = |cg-rear axle|
    beta = arctan((lr/(lf+lr))*tan(df1))
    xdt = v1*cos(psi1 + beta)
    ydt = v1*sin(psi1 + beta)
    psidt = (v1/lr)*sin(beta)
    vdt = a1
    
    x2 = x1 + xdt*dt + random.rand() *.1
    y2 = y1 + ydt*dt + random.rand() *.1
    psi2 = psi1 + psidt*dt + random.rand() *.1
    v2 = v1 + vdt*dt + random.rand() *.1
    
    return x2n, y2n, v2n, psi2n

#**************************** simulate model over time *************************

def sim(N, a, df, dt, x0):
    
    
    x = x0[0]
    y = x0[1]
    v = x0[2]
    psi = x0[3]
   
    
    #initialize trends
    
    xtrend = []
    ytrend = []
    vtrend = []
    psitrend = []
    
    #update trends
    xtrend.append(x)
    ytrend.append(y)
    vtrend.append(v)
    psitrend.append(psi)
  
    for i in range (0, N - 1):
        
        x, y, v, psi = carModel(a, x, y, v, psi, df, dt)
        #x, y, v, psi = carModel(a[i], x, y, v, psi, df[i], dt) #*********** for changing a, df make matrix length match time length appropriately ***************


            #update trends
        xtrend.append(x)
        ytrend.append(y)
        vtrend.append(v)
        psitrend.append(psi)
        
    return np.asarray(xtrend, dtype=object), np.asarray(ytrend, dtype=object), np.asarray(vtrend, dtype=object), np.asarray(psitrend, dtype=object)

# #******** set sim parameters (per project) ****************
N = 100

dt = 0.1
time = np.arange(N)*dt

#****************** model ego car *******************

# *** independent vars (inputs per object) ***

a1=0
df1 = 0

#*** dependendent vars (per object) ***

x1 = 0.0
y1 = 0.0
v1 = 20.0
psi1 = np.pi/2

x01 = np.array([x1, y1, v1, psi1])

#*** call function ***

xtrend1, ytrend1,vtrend1, psitrend1  = sim(N, a1, df1, dt, x01)
# print(xtrend)
# print(ytrend)

#****************** model ego car avoiding *******************

#*** create path for car avoiding ***
a1b=0
df1b = 0

#*** dependendent vars (per object) ***

x1b = 0.0
y1b = 0.0
v1b = 20.0
psi1b = np.pi/2

x01b = np.array([x1b, y1b, v1b, psi1b])

#*** call function ***

xtrend1b, ytrend1b,vtrend1b, psitrend1b  = sim(70, a1b, df1b, dt, x01b)
xDesi1b = np.vstack((xtrend1b, ytrend1b, vtrend1b, psitrend1b))
# print(xtrend)
# print(ytrend)

#****************** model obstacle *****************

# *** independent vars (inputs per object) ***

a2=0
df2=0

#*** dependendent vars (per object) ***

x2 = -37
y2 = 150
v2 = 5.0
psi2 = 0

x02 = np.array([x2, y2, v2, psi2])
obsx0=np.array([a2,df2])
#***call function ***

xtrend2, ytrend2,vtrend2, psitrend2  = sim(N, a2, df2, dt, x02)
obs_path = np.vstack((xtrend2, ytrend2, vtrend2, psitrend2))
# print(xtrend)
# print(ytrend)

#***********************************************

# ************* plot model *********************

import matplotlib.pyplot as plt
# Plot animation
from matplotlib import animation, rc
from IPython.display import HTML



def plot_traj (timespan, xtrend, ytrend, vtrend, psitrend, obj):
    
    print(obj)
    
    fig = plt.figure(figsize=(9,6))
    
    plt.subplot(5,1,1)
    plt.plot(time, xtrend, color='blue')
    plt.xlabel('time (s)')
    plt.ylabel('x')
    plt.title('X path')
#     plt.autoscale_view()

    fig = plt.figure(figsize=(9,6))    
    plt.subplot(5,1,2)
    plt.plot(time, ytrend, color='mediumpurple')
    plt.xlabel('time (s)')
    plt.ylabel('y')
    plt.title('Y Path')
#     plt.autoscale_view()

    fig = plt.figure(figsize=(9,6))
    plt.subplot(5,1,3)
    plt.plot(time, vtrend, color='darkmagenta')
    plt.xlabel('time (s)')
    plt.ylabel('speed')
    plt.title('Speed')
#     plt.autoscale_view()

    fig = plt.figure(figsize=(9,6))
    plt.subplot(5,1,4)
    plt.plot(time, psitrend*180/np.pi, color='seagreen')
    plt.xlabel('time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Heading')
#     plt.autoscale_view()
    
    fig = plt.figure(figsize=(9,6))    
    plt.subplot(5,1,5)
    plt.plot(xtrend, ytrend, color='darkorange')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Path')
#     plt.autoscale_view()
    plt.axis('equal')
    
    plt.show()


#plot results

#*** ego car ***

plot_traj (time, xtrend1, ytrend1, vtrend1, psitrend1, "ego car")

#*** moving obstacle ***

plot_traj (time, xtrend2, ytrend2, vtrend2, psitrend2, 'skaters')

#********* animate *************** rewrite - don't fully understand animation.  Insert boxes for buildings.  change size of object.  make box for car shape

from matplotlib import animation, rc
from IPython.display import HTML
import polytope as pt


fig = plt.figure(figsize=(5,5))
#ax = fig.add_subplot(111, autoscale_on=False, xlim=(np.min(xtrend)-5, np.max(xtrend)+5), ylim=(np.min(ytrend)-5, np.max(ytrend)+5))
ax1 = fig.add_subplot(111, autoscale_on=False, xlim=(-120, 120), ylim=(-10, 230))
ax1.set_aspect('equal')
#ax1.grid()

line1, = ax1.plot([], [], 'o', lw=2)
time_template1 = 'time = %.1fs'
time_text1 = ax1.text(0.05, 0.9, '', transform=ax1.transAxes)

def init():
    line1.set_data([],[])
    time_text1.set_text('')
    return line1,time_text1

def animate(i):
    
    x = np.array([xtrend1[i], xtrend2[i]])
    y = np.array([ytrend1[i], ytrend2[i]])

    psi = np.array([psitrend1[i], psitrend2[i]])
    objx = np.array([[x[0]+np.cos(psi1), x[0], x[0]-np.cos(psi1)],
                    [x[1]+np.cos(psi2), x[1], x[1]-np.cos(psi2)]])
    objy = np.array([[y[0]+np.sin(psi1), y[0], y[0]-np.sin(psi1)],
                    [y[1]+np.sin(psi2), y[1], y[1]-np.sin(psi2)]])

    
#     line1.set_data(.1*objx, objy)  #******* ped representation ********
    line1.set_data(objx, objy)  #******* group of skaters representation ********
    time_text1.set_text(time_template1 % (i*dt))
#     line1.set_data(x,y)  # ***** dot representation ******

    return line1,time_text1

ani = animation.FuncAnimation(fig, animate, range(1, N), interval=0.1*1000, blit=True, init_func=init)
rc('animation', html='jshtml')
ani

# *** get desired path ***
#     xdesired =                               # *** path planner x and y values
    
# ************* test sim *****************
# ****************************************
# #******** set sim parameters (per project) ****************

N = 100
# print(time)
dt1 = 0.1

#****************** model ego car *******************

# *** independent vars (inputs per object) ***

a1=0
df1 = 0



#*** dependendent vars (per object) ***

x1 = 0.0
y1 = 0.0
v1 = 20.0
psi1 = np.pi/2

x01 = np.array([x1, y1, v1, psi1])

#*** call function ***

xtrend1, ytrend1,vtrend1, psitrend1  = sim(N, a1, df1, dt1, x01)
xDesi1 = np.vstack((xtrend1, ytrend1, vtrend1, psitrend1))
#print ('xDesired', xDesired)

#****************** model ego car 2a *******************
#*******************************************************

N2a = 100

# *** independent vars (inputs per object) ***

a2a = 0.1
df2a = - 0.1
dt2a = 0.1


#*** dependendent vars (per object) ***

x2a = 1
y2a = 2
v2a = 5
psi2a = 2*np.pi

x02a = np.array([x2a, y2a, v2a, psi2a])

#*** call function ***

xtrend2a, ytrend2a,vtrend2a, psitrend2a  = sim(N2a, a2a, df2a, dt2a, x02a)
# print(xtrend2a, ytrend2a,vtrend2a, psitrend2a)
xDesi2a = np.vstack((xtrend2a, ytrend2a, vtrend2a, psitrend2a))

# *** choose result and set as xDesired ***
xDesired = xDesi1
curveTrackDDesired =xDesi2a
# print('desired states: ',xDesired)

import numpy as np
import pyomo.environ as pyo


xDesired = xDesired  # enter desired path (can be from above path simulator, or from planner)

# *** simulation parameters ***

Ts = 0.1
N = 90 # should be fed from MPC
TFinal = Ts*N

# z is state vector, u is input (a, steering angle)  
z0Bar = np.array([0,0,20,np.pi/2])                                          #*** initial states x,y,v,heading
zNBar = np.array([xDesired[0,N],xDesired[1,N],xDesired[2,N],xDesired[3,N]]) #*** terminal states ***
zMax = np.array([300,300,80,2*np.pi])                                       #*** state limits ***
zMin = np.array([-300,-300,-10,-2*np.pi])
uMax = np.array([3, np.pi/6])                                              #*** input limits ***
uMin = np.array([-4, -np.pi/6])
udotMax = np.array([2,np.pi/3])                                            #*** input rate limits ***
udotMin = np.array([-2,-np.pi/3])

nz = 4 # number of states
nu = 2 # number of inputs

l_r = 1.738 # *** .5 wheelbase

#*********************************

#************************ optimization function with nonlinear constraints ****************
#******************************************************************************************

def min_dx(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDesired,ulast):
    
    
  model = pyo.ConcreteModel()
  model.tidx = pyo.Set(initialize=range(0, N+1)) # length of finite optimization problem
  model.zidx = pyo.Set(initialize=range(0, nz))
  model.uidx = pyo.Set(initialize=range(0, nu))

  # Create state and input variables trajectory:
  model.z = pyo.Var(model.zidx, model.tidx)
  model.u = pyo.Var(model.uidx, model.tidx)

  # Objective:
  # *** define cost weights ***
  #*** state costs ***
  Q = np.eye(4) 
  Q[0,0]=10
  Q[1,1]=10
  Q[2,2]=5
  Q[3,3]=20
  # print('Q = ',Q)
      
  #*** input costs ***
  R = 1*np.eye(2)
      
  #*** terminal costs ***
  P = np.eye(4)
  P[0,0]=5
  P[1,1]=5
  P[2,2]=5
  P[3,3]=20

  model.Q = Q
  model.R = R
  model.P = P
  model.N = N

  def objective_rule(model):
    costX = 0.0
    costU = 0.0
    costTerminal = 0.0
    for t in model.tidx:
      for i in model.zidx:
        for j in model.zidx:
          if t < model.N:
            costX += (model.z[i, t] - xDesired[i, t]) * model.Q[i, j] * (model.z[j, t] - xDesired[j, t])
    for t in model.tidx:
      for i in model.uidx:
        for j in model.uidx:
          if t < model.N:
            costU += (model.u[i, t]-model.u[i,t+1])**2 * model.R[i, j] * (model.u[j, t] - model.u[j, t+1])**2
            #costU += (model.u[i, t])**2 * model.R[i, j] * (model.u[j, t])**2


    for i in model.zidx:
      for j in model.zidx:               
        costTerminal += (model.z[i, N-1] - xDesired[i, N-1]) * model.P[i, j] * (model.z[j, N-1] - xDesired[j, N-1]) # ????????? should be N or N-1 or model.N?
    return costX + costU + costTerminal
      
  model.cost = pyo.Objective(rule = objective_rule, sense = pyo.minimize)

  # Constraints:
  model.init_constraint = pyo.Constraint(model.zidx, rule=lambda model, i: model.z[i, 0] == z0Bar[i])

  # define z nexts
  model.equality_constraint1 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[0, t+1] == model.z[0, t] + Ts*(model.z[2, t]*pyo.cos(model.z[3, t] + model.u[1,t])) if t < N else pyo.Constraint.Skip)
  model.equality_constraint2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[1, t+1] == model.z[1, t] + Ts*(model.z[2, t]*pyo.sin(model.z[3, t] + model.u[1,t])) if t < N else pyo.Constraint.Skip)
  model.equality_constraint3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[2, t+1] == model.z[2, t] + Ts*model.u[0, t] if t < N else pyo.Constraint.Skip)
  model.equality_constraint4 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[3, t+1] == model.z[3, t] + Ts*((model.z[2, t]/l_r)*pyo.sin(model.u[1, t])) if t < N else pyo.Constraint.Skip)

  # define u nexts and rates of change (acceleration dot, psi dot)
  model.input_constraint1 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1] - model.u[0, t] <= Ts*udotMax[0] # *** acceleration rate of change ***
                                          if t < N-1 else pyo.Constraint.Skip)
  model.input_constraint2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1] - model.u[0, t] >= Ts*udotMin[0]
                                          if t < N-1 else pyo.Constraint.Skip)
  model.input_constraint3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1] - model.u[1, t] <= Ts*udotMax[1] # *** steering angle rate of change ***
                                        if t < N-1 else pyo.Constraint.Skip)
  model.input_constraint4 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1] - model.u[1, t] >= Ts*udotMin[1]
                                        if t < N-1 else pyo.Constraint.Skip)

  model.input_constraint5 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] <= 4 if t < N else pyo.Constraint.Skip)  
  model.input_constraint6 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] >= -3 if t < N else pyo.Constraint.Skip)

  model.input_constraint7 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1] - model.u[1, t] <= .6 if t < N-1 else pyo.Constraint.Skip)
  model.input_constraint8 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1] - model.u[1, t] >= -.6 if t < N-1 else pyo.Constraint.Skip)

  model.input_constraint9 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] >= -1 if t < N else pyo.Constraint.Skip)  #******** these are radians . . too big!!!*******
  model.input_constraint10= pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] <= 1 if t < N else pyo.Constraint.Skip)

  model.acc_smooth1 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] <= ulast[0]+Ts*udotMax[0] if t == 0 else pyo.Constraint.Skip)
  model.acc_smooth2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] >= ulast[0]+Ts*udotMin[0] if t == 0 else pyo.Constraint.Skip)

  # define final state constraint
#   model.final_constraint1 = pyo.Constraint(model.zidx, rule=lambda model, i:model.z[0, N] <= zNBar[0]+10)  # zNBar inputted from MPC as xDes[:,-1] (end of stage)
#   model.final_constraint2 = pyo.Constraint(model.zidx, rule=lambda model, i:model.z[0, N] >= zNBar[0]-10)
#   model.final_constraint3 = pyo.Constraint(model.zidx, rule=lambda model, i:model.z[1, N] <= zNBar[1]+10)
#   model.final_constraint4 = pyo.Constraint(model.zidx, rule=lambda model, i:model.z[1, N] >= zNBar[1]-10)

  # define state constraint
  model.state_constraint1 = pyo.Constraint(model.zidx, model.tidx, rule=lambda model, i, t: model.z[2, t] <= zMax[2] if t < N  else pyo.Constraint.Skip)
  model.state_constraint2 = pyo.Constraint(model.zidx, model.tidx, rule=lambda model, i, t: model.z[2, t] >= zMin[2] if t < N else pyo.Constraint.Skip)

  # Now we can solve:
  #results = pyo.SolverFactory('ipopt').solve(model).write()
  solver = pyo.SolverFactory('ipopt')
  results = solver.solve(model)

  if str(results.solver.termination_condition) == "optimal":
    feas = True
  else:
    feas = False
    xOpt = []
    uOpt = []
    JOpt = np.inf
    
  xOpt = np.asarray([[model.z[i,t]() for i in model.zidx] for t in model.tidx]).T
  uOpt = np.asarray([model.u[:,t]() for t in model.tidx]).T
  JOpt = model.cost()

  return([model, feas, xOpt, uOpt, JOpt])

# Tr = 8 #end of test range
# test_zNBar = np.array([xDesired[0,Tr],xDesired[1,Tr],xDesired[2,Tr],xDesired[3,Tr]]) # 1d array
# test_zNBar2 = np.array([xDesired[:,Tr]])                                             # 2d array

# model, feas, xOpt, uOpt, JOpt = min_dx(N, z0Bar, test_zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDesired[:,4:8]) # run example
ulast = np.array([0,0])
model, feas, xOpt, uOpt, JOpt = min_dx(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDesired,ulast) # run example



# print([model, feas, xOpt, uOpt, JOpt])

# model.display()
x = [xOpt[0,0]]
y = [xOpt[1,0]]
v = [xOpt[2,0]]
psi = [xOpt[3,0]]
a = [uOpt[0,0]]
beta = [uOpt[1,0]]

for t in model.tidx:
    if t < N:
        x.append(xOpt[0,t+1])
        y.append(xOpt[1,t+1])
        v.append(xOpt[2,t+1])
        psi.append(xOpt[3,t+1])
    if t < N-1:
        a.append(uOpt[0,t+1])
        beta.append(uOpt[1,t+1])

plt.figure()
plt.subplot(2,1,1)
plt.plot(a)
plt.ylabel('a')
plt.subplot(2,1,2)
plt.plot(beta)
plt.ylabel('beta')

plt.figure()
plt.subplot(2,1,1)
plt.plot(x)
plt.ylabel('x')
plt.subplot(2,1,2)
plt.plot(y)
plt.ylabel('y')
plt.figure()
plt.subplot(2,1,1)
plt.plot(v)
plt.ylabel('v')
plt.subplot(2,1,2)
plt.plot(psi)
plt.ylabel('psi')

fig = plt.figure(figsize=(9,6))    
plt.subplot(2,1,1)
plt.plot(x, y, color='darkorange')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Path')
#     plt.autoscale_view()
plt.axis('equal')

plt.subplot(2,1,2)
plt.plot(xDesired[0], xDesired[1], color='seagreen')
plt.xlabel('desired x')
plt.ylabel('desired y')
plt.title('Desired Path')
#     plt.autoscale_view()
plt.axis('equal')

plt.show()



# # plot results
# x1 = [pyo.value(model.x[0,0])]
# x2 = [pyo.value(model.x[1,0])]
# u1 = [pyo.value(model.u[0,0])]
# u2 = [pyo.value(model.u[1,0])]

# for t in model.tidx:
#     if t < N:
#         x1.append(pyo.value(model.x[0,t+1]))
#         x2.append(pyo.value(model.x[1,t+1]))
#     if t < N-1:
#         u1.append(pyo.value(model.u[0,t+1]))
# plt.figure()
# plt.plot(tGrid, x1,'b')
# plt.plot(tGrid, x2,'g')
# plt.plot(tGrid[0:-1], u1,'r')
# plt.plot(tGrid[0:-1], u2,'c')


# plt.show()



#************************ MPC eyes *********************
#*********************** Close loop simu MPC *********************

# from scipy.interpolate import UnivariateSpline, splev, splrep, CubicSpline
# M = 100  # Simulation time
# N = 5

# fig = plt.figure(figsize=(9, 6))

# def RefernceGenerater(x_refer,y_refer):
#     s_index = np.arange(len(x_refer))
#     spl_x_index = CubicSpline(s_index,x_refer)
#     spl_y_index = CubicSpline(s_index,y_refer)
#     s_refer = [0]
#     x_rel_refer = [x_refer[0]]
#     y_rel_refer = [y_refer[0]]
#     for delta_index in range(1000*len(x_refer)):
#         x_inter = spl_x_index(0.001*(delta_index+1))
#         y_inter = spl_y_index(0.001*(delta_index+1))
#         s_new_point = s_refer[-1] +  np.sqrt((x_inter-x_rel_refer[-1])**2+(y_inter-y_rel_refer[-1])**2)
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
#     S_END = s_refer[-1]
#     print('S_END: ',S_END)
#     last_delta_x = x_rel_refer[-1]-x_rel_refer[-2]
#     last_delta_y = y_rel_refer[-1]-y_rel_refer[-2]
#     last_delta_s = s_refer[-1]-s_refer[-2]
#     for delta_s in range(100):
#         s_new_point = s_refer[-1] + 0.5
#         x_inter = x_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_x
#         y_inter = y_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_y
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
        
#     spl_x_s = CubicSpline(np.array(s_refer),np.array(x_rel_refer)) #x_new = spl_x_s(s_now+delta_s)
#     spl_y_s = CubicSpline(np.array(s_refer),np.array(y_rel_refer)) #y_new = spl_y_s(s_now+delta_s)
#     spl_dx_s =spl_x_s.derivative()
#     spl_dy_s =spl_y_s.derivative()
#     return S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s

# def CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s):
#     dismin = 1000;
#     S_interaction_ego = 0
#     S_interaction_ob = 0
#     for i in range(int(S_ob_END)):
#         for j in range(int(S_END)):
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
#     print('Rough S_interaction_ego: ',S_interaction_ego,' Rough S_interaction_ob: ',S_interaction_ob)
#     s_ego_range = np.arange(S_interaction_ego-1.1,S_interaction_ego+1.1,0.01)
#     s_ob_range = np.arange(S_interaction_ob-1.1,S_interaction_ob+1.1,0.01)
    
#     for i in s_ob_range:
#         for j in s_ego_range:
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
    
#     print('S_interaction_ego: ',S_interaction_ego,' S_interaction_ob: ',S_interaction_ob)
    
#     return S_interaction_ego,S_interaction_ob

def mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path):

    #*************Initialize arrays************
        # *** initialize states and inputs ***
    nz = nx = 4         # number of states: x1 = x value of cars position (x), x2 = y value of cars position (y), x3 = v = sqrt(x**2 + y**2), x4 = psi
    nu = 2              # number of inputs: u1 = acceleration (a), u2 = steering input (df)

    xOpt = np.zeros((nx, M+1))
    uOpt = np.zeros((nu, M))
    xOpt[:, 0] = z0Bar.reshape(nx, )
    xOptpred = np.zeros((nx, M+1))
    xPred = np.zeros((nx, N+1, M))

    feas = np.zeros((M, ), dtype=bool)
    
    # # process ob reference 
    # x_ob_refer = obs_path[0,:]
    # y_ob_refer = obs_path[1,:]
    # S_ob_END,s_ob_refer,x_ob_rel_refer,y_ob_rel_refer, spl_ob_x_s, spl_ob_y_s, spl_ob_dx_s,spl_ob_dy_s = RefernceGenerater(x_ob_refer,y_ob_refer)
    
    
    # # process ego reference 
    # x_refer = 0*np.arange(0,300,1)
    # y_refer = np.arange(0,300,1)
    # S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s = RefernceGenerater(x_refer,y_refer)
    
    
    # # init ego Planner
    # myplanner = IDMPY5a.IDMPlanner(4.0)
    # current_acc = np.array([0])
    # ulast = np.array([0,0])
    # # find the interation point
    # S_interaction_ego,S_interaction_ob = CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s)
    
    # # Simulation Loop
    # last_s_index = 0
    # last_s_ob_index = 0
    # have_add=False
    # for t in range(M):
        
    #     # ob_s_now
    #     dismin = 1000;
    #     s_ob_now = 0
    #     for i in range(last_s_ob_index,len(x_ob_rel_refer),1):
    #         dis = np.sqrt((obs_path[0,t] - x_ob_rel_refer[i])**2+(obs_path[1,t] - y_ob_rel_refer[i])**2)
    #         if dis < dismin:
    #             s_ob_now  = s_ob_refer[i]
    #             dismin = dis
    #             last_s_ob_index = i
    #     print("s_ob_now",s_ob_now)
        
        
    #     # ego_s_now
    #     dismin = 1000;
    #     s_now = 0
    #     for i in range(last_s_index,len(x_rel_refer),1):
    #         dis = np.sqrt((xOpt[0,t] - x_rel_refer[i])**2+(xOpt[1,t] - y_rel_refer[i])**2)
    #         if dis < dismin:
    #             s_now  = s_refer[i]
    #             dismin = dis
    #             last_s_index = i
    #     print("s_now",s_now)
    #     if s_now >= S_END:
    #         break
            
    #     # eye sight
    #     left_bound_eye = (-6-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3-6
    #     right_bound_eye = (2-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3+2
            
    #     # Planner
    #     egoState = [[0,xOpt[2,t]]]  #[s,v]
    #     if (obs_path[0,t]+0.5 >= left_bound_eye or have_add) and S_interaction_ob-s_ob_now>-2.5:
    #         have_add = True
    #         OBState = [[(S_interaction_ego-s_now)-(S_interaction_ob-s_ob_now),obs_path[2,t]]]
    #         print('OBState: ',OBState)
    #         [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc,OBState,True,5,5)
    #     else:
    #         [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc)
    #     x_planed = spl_x_s(s_now+spl_s_plan_val)
    #     y_planed = spl_y_s(s_now+spl_s_plan_val)
    #     dx_planed = spl_dx_s(s_now+spl_s_plan_val)
    #     dy_planed = spl_dy_s(s_now+spl_s_plan_val)
    #     heading_planed = [np.arctan2(dy_planed[i],dx_planed[i]) for i in range(len(dx_planed))]
    #     v_planed = spl_v_plan_val
    #     xDes = np.array([x_planed,y_planed,v_planed,heading_planed])
    #     if t==-1:
    #         print(egoState)
    #         print(current_acc)
    #         print("s: ",s_now+spl_s_plan_val)
    #         print("x: ",x_planed)
    #         print("y: ",y_planed)
    #         print("v: ",v_planed)
    #         print(heading_planed)

            
        # MPC control
        [model, feas[t], x, u, J] = min_dx(N, xOpt[:,t], xDes[:,-1], zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDes,ulast) # *** zNbar per stage is last xDes of stage
        if not feas[t]:
            xOpt = []
            uOpt = []
            break

        # Save open loop predictions
        xPred[:, :, t] = x

        # Save closed loop trajectory, collect optimal states and optimal inputs
        # Note that the second column of x represents the optimal closed loop state
        xOpt[:, t+1] = x[:, 1]
        uOpt[:, t] = u[:, 0].reshape(nu, )
        current_acc = uOpt[0,t]
        ulast = uOpt[:,t]

    return [model, feas, xOpt, uOpt]
    #*****************attempt at iterating 1*******************
    
    
z0Bar = np.array([0.8,82,4,xDesired[3,0]])
print(z0Bar)
model, feas, xOpt, uOpt = mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path)

print('Feasibility =', feas) 
fig = plt.figure(figsize=(10, 10))
line1 = plt.plot(xDesired[0, 41:], xDesired[1, 41:], 'r-')
line2 = plt.plot(xOpt[0, :], xOpt[1, :], 'b-')
plt.xlabel('x')
plt.ylabel('y')
plt.axis('equal')
plt.xlim((-5,5))

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[0,:])
plt.xlabel('t(s)')
plt.ylabel('x(m)')
plt.ylim((-2,2))

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[1,:])
plt.xlabel('t(s)')
plt.ylabel('y(m)')

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[2,:])
plt.xlabel('t(s)')
plt.ylabel('v(m/s)')

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[3,:])
plt.xlabel('t(s)')
plt.ylabel('angle(rad)')
plt.ylim((0,np.pi))

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.0,0.1),uOpt.T)
plt.xlabel('t(s)')
plt.legend(['a','Beta'])
plt.ylabel('u')
plt.show()

fig = plt.figure(figsize=(10,10))
#ax = fig.add_subplot(111, autoscale_on=False, xlim=(np.min(xtrend)-5, np.max(xtrend)+5), ylim=(np.min(ytrend)-5, np.max(ytrend)+5))
ax1 = fig.add_subplot(111, autoscale_on=False, xlim=(-25, 25), ylim=(80, 160))
ax1.set_aspect('equal')
#ax1.grid()

line1, = ax1.plot([], [], 'b-', lw=2)
line2, = ax1.plot([], [], 'ro-', lw=2)
line3, = ax1.plot(np.arange(-25,-5,1), 147+0*np.arange(-25,-5,1),'g-', lw=2)
line4, = ax1.plot(np.arange(2,26,1), 147+0*np.arange(2,26,1), 'g-', lw=2)
line5, = ax1.plot(-6+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)
line6, = ax1.plot(2+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)

line7, = ax1.plot([], [], 'm-', lw=1)
line8, = ax1.plot([], [], 'm-', lw=1)
time_template1 = 'time = %.1fs'
time_text1 = ax1.text(0.05, 0.9, '', transform=ax1.transAxes)

# def init():
#     line1.set_data([],[])
#     line2.set_data([],[])
#     line7.set_data([],[])
#     line8.set_data([],[])
#     time_text1.set_text('')
#     return line1,time_text1

# def animate(i):
    
#     x = np.array([xOpt[0,i], obs_path[0,i]])
#     y = np.array([xOpt[1,i], obs_path[1,i]])

#     psi = np.array([xOpt[3,i], obs_path[3,i]])
#     egox = np.array([x[0]+2*np.cos(psi[0])-1*np.sin(psi[0]),x[0]-2*np.cos(psi[0])-1*np.sin(psi[0]), x[0]-2*np.cos(psi[0])+1*np.sin(psi[0]), x[0]+2*np.cos(psi[0])+1*np.sin(psi[0]),x[0]+2*np.cos(psi[0])-1*np.sin(psi[0])]),
#     obx = np.array([x[1]+np.cos(psi[1]), x[1], x[1]-np.cos(psi[1])])
#     egoy = np.array([y[0]+2*np.sin(psi[0])+1*np.cos(psi[0]),y[0]-2*np.sin(psi[0])+1*np.cos(psi[0]), y[0]-2*np.sin(psi[0])-1*np.cos(psi[0]), y[0]+2*np.sin(psi[0])-1*np.cos(psi[0]),y[0]+2*np.sin(psi[0])+1*np.cos(psi[0])])
#     oby = np.array([y[1]+np.sin(psi[1]), y[1], y[1]-np.sin(psi[1])])
    
#     if y[0]+2*np.sin(psi[0])<147:
#         eye_line1x = np.array([x[0]+2*np.cos(psi[0]),-6,(-6-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8-6])
#         eye_line1y = np.array([y[0]+2*np.sin(psi[0]),147,155])

#         eye_line2x = np.array([x[0]+2*np.cos(psi[0]),2,(2-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8+2])
#         eye_line2y = np.array([y[0]+2*np.sin(psi[0]),147,155])
#     else:
#         eye_line1x = np.array([])
#         eye_line1y = np.array([])

#         eye_line2x = np.array([])
#         eye_line2y = np.array([])

    
# #     line1.set_data(.1*objx, objy)  #******* ped representation ********
#     line1.set_data(egox, egoy)  #******* group of skaters representation ********
#     line2.set_data(obx, oby)
#     line7.set_data(eye_line1x, eye_line1y)
#     line8.set_data(eye_line2x, eye_line2y)
#     time_text1.set_text(time_template1 % (i*dt))
# #     line1.set_data(x,y)  # ***** dot representation ******

#     return line1,time_text1

# ani = animation.FuncAnimation(fig, animate, range(0, M), interval=0.1*1000, blit=True, init_func=init)
# rc('animation', html='jshtml')
# ani



#************************ MPC eyes *********************
#*********************** Close loop simu MPC *********************

# from scipy.interpolate import UnivariateSpline, splev, splrep, CubicSpline
# M = 100  # Simulation time
# N = 5

# fig = plt.figure(figsize=(9, 6))

# def RefernceGenerater(x_refer,y_refer):
#     s_index = np.arange(len(x_refer))
#     spl_x_index = CubicSpline(s_index,x_refer)
#     spl_y_index = CubicSpline(s_index,y_refer)
#     s_refer = [0]
#     x_rel_refer = [x_refer[0]]
#     y_rel_refer = [y_refer[0]]
#     for delta_index in range(1000*len(x_refer)):
#         x_inter = spl_x_index(0.001*(delta_index+1))
#         y_inter = spl_y_index(0.001*(delta_index+1))
#         s_new_point = s_refer[-1] +  np.sqrt((x_inter-x_rel_refer[-1])**2+(y_inter-y_rel_refer[-1])**2)
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
#     S_END = s_refer[-1]
#     print('S_END: ',S_END)
#     last_delta_x = x_rel_refer[-1]-x_rel_refer[-2]
#     last_delta_y = y_rel_refer[-1]-y_rel_refer[-2]
#     last_delta_s = s_refer[-1]-s_refer[-2]
#     for delta_s in range(100):
#         s_new_point = s_refer[-1] + 0.5
#         x_inter = x_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_x
#         y_inter = y_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_y
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
        
#     spl_x_s = CubicSpline(np.array(s_refer),np.array(x_rel_refer)) #x_new = spl_x_s(s_now+delta_s)
#     spl_y_s = CubicSpline(np.array(s_refer),np.array(y_rel_refer)) #y_new = spl_y_s(s_now+delta_s)
#     spl_dx_s =spl_x_s.derivative()
#     spl_dy_s =spl_y_s.derivative()
#     return S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s

# def CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s):
#     dismin = 1000;
#     S_interaction_ego = 0
#     S_interaction_ob = 0
#     for i in range(int(S_ob_END)):
#         for j in range(int(S_END)):
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
#     print('Rough S_interaction_ego: ',S_interaction_ego,' Rough S_interaction_ob: ',S_interaction_ob)
#     s_ego_range = np.arange(S_interaction_ego-1.1,S_interaction_ego+1.1,0.01)
#     s_ob_range = np.arange(S_interaction_ob-1.1,S_interaction_ob+1.1,0.01)
    
#     for i in s_ob_range:
#         for j in s_ego_range:
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
    
#     print('S_interaction_ego: ',S_interaction_ego,' S_interaction_ob: ',S_interaction_ob)
    
#     return S_interaction_ego,S_interaction_ob

def mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path):

    #*************Initialize arrays************
        # *** initialize states and inputs ***
    nz = nx = 4         # number of states: x1 = x value of cars position (x), x2 = y value of cars position (y), x3 = v = sqrt(x**2 + y**2), x4 = psi
    nu = 2              # number of inputs: u1 = acceleration (a), u2 = steering input (df)

    xOpt = np.zeros((nx, M+1))
    uOpt = np.zeros((nu, M))
    xOpt[:, 0] = z0Bar.reshape(nx, )
    xOptpred = np.zeros((nx, M+1))
    xPred = np.zeros((nx, N+1, M))

    feas = np.zeros((M, ), dtype=bool)
    
    # # process ob reference 
    # x_ob_refer = obs_path[0,:]
    # y_ob_refer = obs_path[1,:]
    # S_ob_END,s_ob_refer,x_ob_rel_refer,y_ob_rel_refer, spl_ob_x_s, spl_ob_y_s, spl_ob_dx_s,spl_ob_dy_s = RefernceGenerater(x_ob_refer,y_ob_refer)
    
    
    # # process ego reference 
    # x_refer = 0*np.arange(0,300,1)
    # y_refer = np.arange(0,300,1)
    # S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s = RefernceGenerater(x_refer,y_refer)
    
    
    # # init ego Planner
    # myplanner = IDMPY5a.IDMPlanner(4.0)
    # current_acc = np.array([0])
    # ulast = np.array([0,0])
    # # find the interation point
    # S_interaction_ego,S_interaction_ob = CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s)
    
    # # Simulation Loop
    # last_s_index = 0
    # last_s_ob_index = 0
    # have_add = False
    # for t in range(M):
        
    #     # ob_s_now
    #     dismin = 1000;
    #     s_ob_now = 0
    #     for i in range(last_s_ob_index,len(x_ob_rel_refer),1):
    #         dis = np.sqrt((obs_path[0,t] - x_ob_rel_refer[i])**2+(obs_path[1,t] - y_ob_rel_refer[i])**2)
    #         if dis < dismin:
    #             s_ob_now  = s_ob_refer[i]
    #             dismin = dis
    #             last_s_ob_index = i
    #     print("s_ob_now",s_ob_now)
        
        
    #     # ego_s_now
    #     dismin = 1000;
    #     s_now = 0
    #     for i in range(last_s_index,len(x_rel_refer),1):
    #         dis = np.sqrt((xOpt[0,t] - x_rel_refer[i])**2+(xOpt[1,t] - y_rel_refer[i])**2)
    #         if dis < dismin:
    #             s_now  = s_refer[i]
    #             dismin = dis
    #             last_s_index = i
    #     print("s_now",s_now)
    #     if s_now >= S_END:
    #         break
            
    #     # eye sight
    #     left_bound_eye = (-6-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3-6
    #     right_bound_eye = (2-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3+2
            
    #     # Planner
    #     egoState = [[0,xOpt[2,t]]]  #[s,v]
    #     OBState = [[(S_interaction_ego-s_now)-(S_interaction_ob-s_ob_now),obs_path[2,t]]]
    #     print('OBState: ',OBState)
    #     if (obs_path[0,t]+0.5 >= left_bound_eye or have_add) and S_interaction_ob-s_ob_now>-2.5:
    #         have_add = True
    #         [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc,OBState,True,2,1)
    #     elif S_interaction_ob-s_ob_now>-2.5:
    #         [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc,OBState,True,5,5)
    #     else:
    #         [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc)
    #     x_planed = spl_x_s(s_now+spl_s_plan_val)
    #     y_planed = spl_y_s(s_now+spl_s_plan_val)
    #     dx_planed = spl_dx_s(s_now+spl_s_plan_val)
    #     dy_planed = spl_dy_s(s_now+spl_s_plan_val)
    #     heading_planed = [np.arctan2(dy_planed[i],dx_planed[i]) for i in range(len(dx_planed))]
    #     v_planed = spl_v_plan_val
    #     xDes = np.array([x_planed,y_planed,v_planed,heading_planed])
    #     if t==-1:
    #         print(egoState)
    #         print(current_acc)
    #         print("s: ",s_now+spl_s_plan_val)
    #         print("x: ",x_planed)
    #         print("y: ",y_planed)
    #         print("v: ",v_planed)
    #         print(heading_planed)

            
        # MPC control
        [model, feas[t], x, u, J] = min_dx(N, xOpt[:,t], xDes[:,-1], zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDes,ulast) # *** zNbar per stage is last xDes of stage
        if not feas[t]:
            xOpt = []
            uOpt = []
            break

        # Save open loop predictions
        xPred[:, :, t] = x

        # Save closed loop trajectory, collect optimal states and optimal inputs
        # Note that the second column of x represents the optimal closed loop state
        xOpt[:, t+1] = x[:, 1]
        uOpt[:, t] = u[:, 0].reshape(nu, )
        current_acc = uOpt[0,t]
        ulast = uOpt[:,t]

    return [model, feas, xOpt, uOpt]
    #*****************attempt at iterating 1*******************
    
    
z0Bar = np.array([0.8,82,4,xDesired[3,0]])
print(z0Bar)
model, feas, xOpt, uOpt = mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path)

print('Feasibility =', feas) 
fig = plt.figure(figsize=(10, 10))
line1 = plt.plot(xDesired[0, 41:], xDesired[1, 41:], 'r-')
line2 = plt.plot(xOpt[0, :], xOpt[1, :], 'b-')
plt.xlabel('x')
plt.ylabel('y')
plt.axis('equal')
plt.xlim((-5,5))

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[0,:])
plt.xlabel('t(s)')
plt.ylabel('x(m)')
plt.ylim((-2,2))

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[1,:])
plt.xlabel('t(s)')
plt.ylabel('y(m)')

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[2,:])
plt.xlabel('t(s)')
plt.ylabel('v(m/s)')

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.1,0.1),xOpt[3,:])
plt.xlabel('t(s)')
plt.ylabel('angle(rad)')
plt.ylim((0,np.pi))

fig = plt.figure(figsize=(8, 5))
plt.plot(np.arange(0,10.0,0.1),uOpt.T)
plt.xlabel('t(s)')
plt.legend(['a','Beta'])
plt.ylabel('u')
plt.show()

# fig = plt.figure(figsize=(10,10))
# #ax = fig.add_subplot(111, autoscale_on=False, xlim=(np.min(xtrend)-5, np.max(xtrend)+5), ylim=(np.min(ytrend)-5, np.max(ytrend)+5))
# ax1 = fig.add_subplot(111, autoscale_on=False, xlim=(-25, 25), ylim=(80, 160))
# ax1.set_aspect('equal')
# #ax1.grid()

# line1, = ax1.plot([], [], 'b-', lw=2)

# line2, = ax1.plot([], [], 'ro-', lw=2)
# GPS_noisy = plt.Circle((0, 0), 2.5, color='y', alpha=0.5)

# line3, = ax1.plot(np.arange(-25,-5,1), 147+0*np.arange(-25,-5,1),'g-', lw=2)
# line4, = ax1.plot(np.arange(2,26,1), 147+0*np.arange(2,26,1), 'g-', lw=2)
# line5, = ax1.plot(-6+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)
# line6, = ax1.plot(2+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)

# line7, = ax1.plot([], [], 'm-', lw=1)
# line8, = ax1.plot([], [], 'm-', lw=1)
# time_template1 = 'time = %.1fs'
# time_text1 = ax1.text(0.05, 0.9, '', transform=ax1.transAxes)

# def init():
#     line1.set_data([],[])
#     line2.set_data([],[])
#     line7.set_data([],[])
#     line8.set_data([],[])
#     ax1.add_patch(GPS_noisy)
#     time_text1.set_text('')
#     return line1,time_text1

# def animate(i):
    
#     x = np.array([xOpt[0,i], obs_path[0,i]])
#     y = np.array([xOpt[1,i], obs_path[1,i]])

#     psi = np.array([xOpt[3,i], obs_path[3,i]])
#     egox = np.array([x[0]+2*np.cos(psi[0])-1*np.sin(psi[0]),x[0]-2*np.cos(psi[0])-1*np.sin(psi[0]), x[0]-2*np.cos(psi[0])+1*np.sin(psi[0]), x[0]+2*np.cos(psi[0])+1*np.sin(psi[0]),x[0]+2*np.cos(psi[0])-1*np.sin(psi[0])]),
#     obx = np.array([x[1]+np.cos(psi[1]), x[1], x[1]-np.cos(psi[1])])
#     egoy = np.array([y[0]+2*np.sin(psi[0])+1*np.cos(psi[0]),y[0]-2*np.sin(psi[0])+1*np.cos(psi[0]), y[0]-2*np.sin(psi[0])-1*np.cos(psi[0]), y[0]+2*np.sin(psi[0])-1*np.cos(psi[0]),y[0]+2*np.sin(psi[0])+1*np.cos(psi[0])])
#     oby = np.array([y[1]+np.sin(psi[1]), y[1], y[1]-np.sin(psi[1])])

#     if x[1]+1 < (-6-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*3-6 and x[1]<0:
#         GPS_noisy.center = (x[1], y[1])
#     else:
#         GPS_noisy.center = (0, 0)
    
    
#     if y[0]+2*np.sin(psi[0])<147:
#         eye_line1x = np.array([x[0]+2*np.cos(psi[0]),-6,(-6-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8-6])
#         eye_line1y = np.array([y[0]+2*np.sin(psi[0]),147,155])

#         eye_line2x = np.array([x[0]+2*np.cos(psi[0]),2,(2-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8+2])
#         eye_line2y = np.array([y[0]+2*np.sin(psi[0]),147,155])
#     else:
#         eye_line1x = np.array([])
#         eye_line1y = np.array([])

#         eye_line2x = np.array([])
#         eye_line2y = np.array([])

    
# #     line1.set_data(.1*objx, objy)  #******* ped representation ********
#     line1.set_data(egox, egoy)  #******* group of skaters representation ********
#     line2.set_data(obx, oby)
#     line7.set_data(eye_line1x, eye_line1y)
#     line8.set_data(eye_line2x, eye_line2y)
#     time_text1.set_text(time_template1 % (i*dt))
# #     line1.set_data(x,y)  # ***** dot representation ******

#     return line1,time_text1

# ani = animation.FuncAnimation(fig, animate, range(0, M), interval=0.1*1000, blit=True, init_func=init)
# rc('animation', html='jshtml')
# ani

# from scipy.interpolate import UnivariateSpline, splev, splrep, CubicSpline
# M = 70  # Simulation time
# N = 5

# fig = plt.figure(figsize=(9, 6))

# def RefernceGenerater(x_refer,y_refer):
#     s_index = np.arange(len(x_refer))
#     spl_x_index = CubicSpline(s_index,x_refer)
#     spl_y_index = CubicSpline(s_index,y_refer)
#     s_refer = [0]
#     x_rel_refer = [x_refer[0]]
#     y_rel_refer = [y_refer[0]]
#     for delta_index in range(1000*len(x_refer)):
#         x_inter = spl_x_index(0.001*(delta_index+1))
#         y_inter = spl_y_index(0.001*(delta_index+1))
#         s_new_point = s_refer[-1] +  np.sqrt((x_inter-x_rel_refer[-1])**2+(y_inter-y_rel_refer[-1])**2)
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
#     S_END = s_refer[-1]
#     print('S_END: ',S_END)
#     last_delta_x = x_rel_refer[-1]-x_rel_refer[-2]
#     last_delta_y = y_rel_refer[-1]-y_rel_refer[-2]
#     last_delta_s = s_refer[-1]-s_refer[-2]
#     for delta_s in range(100):
#         s_new_point = s_refer[-1] + 0.5
#         x_inter = x_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_x
#         y_inter = y_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_y
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
        
#     spl_x_s = CubicSpline(np.array(s_refer),np.array(x_rel_refer)) #x_new = spl_x_s(s_now+delta_s)
#     spl_y_s = CubicSpline(np.array(s_refer),np.array(y_rel_refer)) #y_new = spl_y_s(s_now+delta_s)
#     spl_dx_s =spl_x_s.derivative()
#     spl_dy_s =spl_y_s.derivative()
#     return S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s

# def CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s):
#     dismin = 1000;
#     S_interaction_ego = 0
#     S_interaction_ob = 0
#     for i in range(int(S_ob_END)):
#         for j in range(int(S_END)):
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
#     print('Rough S_interaction_ego: ',S_interaction_ego,' Rough S_interaction_ob: ',S_interaction_ob)
#     s_ego_range = np.arange(S_interaction_ego-1.1,S_interaction_ego+1.1,0.01)
#     s_ob_range = np.arange(S_interaction_ob-1.1,S_interaction_ob+1.1,0.01)
    
#     for i in s_ob_range:
#         for j in s_ego_range:
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
    
#     print('S_interaction_ego: ',S_interaction_ego,' S_interaction_ob: ',S_interaction_ob)
    
#     return S_interaction_ego,S_interaction_ob

# def mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path):

#     #*************Initialize arrays************
#         # *** initialize states and inputs ***
#     nz = nx = 4         # number of states: x1 = x value of cars position (x), x2 = y value of cars position (y), x3 = v = sqrt(x**2 + y**2), x4 = psi
#     nu = 2              # number of inputs: u1 = acceleration (a), u2 = steering input (df)

#     xOpt = np.zeros((nx, M+1))
#     uOpt = np.zeros((nu, M))
#     xOpt[:, 0] = z0Bar.reshape(nx, )
#     xOptpred = np.zeros((nx, M+1))
#     xPred = np.zeros((nx, N+1, M))

#     feas = np.zeros((M, ), dtype=bool)
#     # process ego reference 
#     x_refer = xDesired[0,:]
#     y_refer = xDesired[1,:]
#     S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s = RefernceGenerater(x_refer,y_refer)
    
    
#     # init ego Planner
#     myplanner = IDMPY5a.IDMPlanner(4.0)
#     current_acc = np.array([0])
#     ulast = np.array([0,0])
#     # Simulation Loop
#     last_s_index = 0
#     last_s_ob_index = 0
#     for t in range(M):

#         # ego_s_now
#         dismin = 1000;
#         s_now = 0
#         for i in range(last_s_index,len(x_rel_refer),1):
#             dis = np.sqrt((xOpt[0,t] - x_rel_refer[i])**2+(xOpt[1,t] - y_rel_refer[i])**2)
#             if dis < dismin:
#                 s_now  = s_refer[i]
#                 dismin = dis
#                 last_s_index = i
#         print("s_now",s_now)

            
#         # Planner
#         egoState = [[0,xOpt[2,t]]]  #[s,v]
#         [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc)
#         x_planed = spl_x_s(s_now+spl_s_plan_val)
#         y_planed = spl_y_s(s_now+spl_s_plan_val)
#         dx_planed = spl_dx_s(s_now+spl_s_plan_val)
#         dy_planed = spl_dy_s(s_now+spl_s_plan_val)
#         heading_planed = [np.arctan2(dy_planed[i],dx_planed[i]) for i in range(len(dx_planed))]
#         v_planed = spl_v_plan_val
#         xDes = np.array([x_planed,y_planed,v_planed,heading_planed])
#         if t==-1:
#             print(egoState)
#             print(current_acc)
#             print("s: ",s_now+spl_s_plan_val)
#             print("x: ",x_planed)
#             print("y: ",y_planed)
#             print("v: ",v_planed)
#             print(heading_planed)

            
#         # MPC control
#         [model, feas[t], x, u, J] = min_dx(N, xOpt[:,t], xDes[:,-1], zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDes,ulast) # *** zNbar per stage is last xDes of stage
#         if not feas[t]:
#             xOpt = []
#             uOpt = []
#             break

#         # Save open loop predictions
#         xPred[:, :, t] = x

#         # Save closed loop trajectory, collect optimal states and optimal inputs
#         # Note that the second column of x represents the optimal closed loop state
#         xOpt[:, t+1] = x[:, 1]
#         uOpt[:, t] = u[:, 0].reshape(nu, )
#         current_acc = uOpt[0,t]
#         ulast = uOpt[:,t]

#     return [model, feas, xOpt, uOpt]
#     #*****************attempt at iterating 1*******************
    
    
# z0Bar = np.array([curveTrackDDesired[0,0],4,2,0])
# print(z0Bar)
# model, feas, xOpt, uOpt = mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, curveTrackDDesired, M, obs_path)

# print('Feasibility =', feas) 
# fig = plt.figure(figsize=(10, 10))
# line1 = plt.plot(curveTrackDDesired[0, :], curveTrackDDesired[1, :], 'r-')
# line2 = plt.plot(xOpt[0, :], xOpt[1, :], 'b-')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.axis('equal')
# plt.legend(['Desire Path','Real Path'])

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[0,:])
# plt.xlabel('t(s)')
# plt.ylabel('x(m)')


# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[1,:])
# plt.xlabel('t(s)')
# plt.ylabel('y(m)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[2,:])
# plt.xlabel('t(s)')
# plt.ylabel('v(m/s)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[3,:])
# plt.xlabel('t(s)')
# plt.ylabel('angle(rad)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.0,0.1),uOpt.T)
# plt.xlabel('t(s)')
# plt.legend(['a','Beta'])
# plt.ylabel('u')
# plt.show()

# fig = plt.figure(figsize=(10,10))
# #ax = fig.add_subplot(111, autoscale_on=False, xlim=(np.min(xtrend)-5, np.max(xtrend)+5), ylim=(np.min(ytrend)-5, np.max(ytrend)+5))
# ax1 = fig.add_subplot(111, autoscale_on=False, xlim=(-5, 40), ylim=(-35, 10))
# ax1.set_aspect('equal')
# #ax1.grid()

# line1, = ax1.plot([], [], 'b-', lw=2)
# line2 = ax1.plot(curveTrackDDesired[0, :], curveTrackDDesired[1, :], 'r-')

# time_template1 = 'time = %.1fs'
# time_text1 = ax1.text(0.05, 0.9, '', transform=ax1.transAxes)

# def init():
#     line1.set_data([],[])
#     time_text1.set_text('')
#     return line1,time_text1

# def animate(i):
    
#     x = np.array([xOpt[0,i]])
#     y = np.array([xOpt[1,i]])

#     psi = np.array([xOpt[3,i]])
#     egox = np.array([x[0]+2*np.cos(psi[0])-1*np.sin(psi[0]),x[0]-2*np.cos(psi[0])-1*np.sin(psi[0]), x[0]-2*np.cos(psi[0])+1*np.sin(psi[0]), x[0]+2*np.cos(psi[0])+1*np.sin(psi[0]),x[0]+2*np.cos(psi[0])-1*np.sin(psi[0])]),
#     egoy = np.array([y[0]+2*np.sin(psi[0])+1*np.cos(psi[0]),y[0]-2*np.sin(psi[0])+1*np.cos(psi[0]), y[0]-2*np.sin(psi[0])-1*np.cos(psi[0]), y[0]+2*np.sin(psi[0])-1*np.cos(psi[0]),y[0]+2*np.sin(psi[0])+1*np.cos(psi[0])])

    
# #     line1.set_data(.1*objx, objy)  #******* ped representation ********
#     line1.set_data(egox, egoy)  #******* group of skaters representation ********
#     time_text1.set_text(time_template1 % (i*dt))
# #     line1.set_data(x,y)  # ***** dot representation ******

#     return line1,time_text1

# ani = animation.FuncAnimation(fig, animate, range(0, M), interval=0.1*1000, blit=True, init_func=init)
# rc('animation', html='jshtml')
# ani

# ############################# ADD NOISY IN SIMULATION MODEL ####################################

# from scipy.interpolate import UnivariateSpline, splev, splrep, CubicSpline
# M = 70  # Simulation time
# N = 5

# fig = plt.figure(figsize=(9, 6))

# def carModel(a1, x1, y1, v1, psi1, beta, dt=0.01):
    
#     lf = lr = 1.738 #*********lf = |cg-front axle|, lr = |cg-rear axle|
#     beta += random.rand() *.05
#     a1 += random.rand() *.1
#     xdt = v1*cos(psi1 + beta)
#     ydt = v1*sin(psi1 + beta)
#     psidt = (v1/lr)*sin(beta)
#     vdt = a1
    
#     x2 = x1 + xdt*dt 
#     y2 = y1 + ydt*dt
#     psi2 = psi1 + psidt*dt
#     v2 = v1 + vdt*dt
#     return x2, y2, v2, psi2

# def RefernceGenerater(x_refer,y_refer):
#     s_index = np.arange(len(x_refer))
#     spl_x_index = CubicSpline(s_index,x_refer)
#     spl_y_index = CubicSpline(s_index,y_refer)
#     s_refer = [0]
#     x_rel_refer = [x_refer[0]]
#     y_rel_refer = [y_refer[0]]
#     for delta_index in range(1000*len(x_refer)):
#         x_inter = spl_x_index(0.001*(delta_index+1))
#         y_inter = spl_y_index(0.001*(delta_index+1))
#         s_new_point = s_refer[-1] +  np.sqrt((x_inter-x_rel_refer[-1])**2+(y_inter-y_rel_refer[-1])**2)
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
#     S_END = s_refer[-1]
#     print('S_END: ',S_END)
#     last_delta_x = x_rel_refer[-1]-x_rel_refer[-2]
#     last_delta_y = y_rel_refer[-1]-y_rel_refer[-2]
#     last_delta_s = s_refer[-1]-s_refer[-2]
#     for delta_s in range(100):
#         s_new_point = s_refer[-1] + 0.5
#         x_inter = x_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_x
#         y_inter = y_rel_refer[-1]+ (0.5/last_delta_s)*last_delta_y
#         s_refer.append(s_new_point)
#         x_rel_refer.append(x_inter)
#         y_rel_refer.append(y_inter)
        
#     spl_x_s = CubicSpline(np.array(s_refer),np.array(x_rel_refer)) #x_new = spl_x_s(s_now+delta_s)
#     spl_y_s = CubicSpline(np.array(s_refer),np.array(y_rel_refer)) #y_new = spl_y_s(s_now+delta_s)
#     spl_dx_s =spl_x_s.derivative()
#     spl_dy_s =spl_y_s.derivative()
#     return S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s

# def CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s):
#     dismin = 1000;
#     S_interaction_ego = 0
#     S_interaction_ob = 0
#     for i in range(int(S_ob_END)):
#         for j in range(int(S_END)):
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
#     print('Rough S_interaction_ego: ',S_interaction_ego,' Rough S_interaction_ob: ',S_interaction_ob)
#     s_ego_range = np.arange(S_interaction_ego-1.1,S_interaction_ego+1.1,0.01)
#     s_ob_range = np.arange(S_interaction_ob-1.1,S_interaction_ob+1.1,0.01)
    
#     for i in s_ob_range:
#         for j in s_ego_range:
#             x_ob = spl_ob_x_s(i)
#             y_ob = spl_ob_y_s(i)
#             x_ego = spl_x_s(j)
#             y_ego = spl_y_s(j)
#             dis = np.sqrt((x_ob - x_ego)**2+(y_ob - y_ego)**2)
#             if dis < dismin:
#                 S_interaction_ego  = j
#                 S_interaction_ob = i
#                 dismin = dis
    
#     print('S_interaction_ego: ',S_interaction_ego,' S_interaction_ob: ',S_interaction_ob)
    
#     return S_interaction_ego,S_interaction_ob

# def mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path):

#     #*************Initialize arrays************
#         # *** initialize states and inputs ***
#     nz = nx = 4         # number of states: x1 = x value of cars position (x), x2 = y value of cars position (y), x3 = v = sqrt(x**2 + y**2), x4 = psi
#     nu = 2              # number of inputs: u1 = acceleration (a), u2 = steering input (df)

#     xOpt = np.zeros((nx, M+1))
#     uOpt = np.zeros((nu, M))
#     xOpt[:, 0] = z0Bar.reshape(nx, )
#     xOptpred = np.zeros((nx, M+1))
#     xPred = np.zeros((nx, N+1, M))

#     feas = np.zeros((M, ), dtype=bool)
#     # process ego reference 
#     x_refer = xDesired[0,:]
#     y_refer = xDesired[1,:]
#     S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s = RefernceGenerater(x_refer,y_refer)
    
    
#     # init ego Planner
#     myplanner = IDMPY5a.IDMPlanner(4.0)
#     current_acc = np.array([0])
#     ulast = np.array([0,0])
#     # Simulation Loop
#     last_s_index = 0
#     last_s_ob_index = 0
#     for t in range(M):
#         # add noisy of measurement
#         x_now = xOpt[:,t]
#         for i in range(3):
#             x_now[i] += random.rand() *.03
#         x_now[3] += random.rand() *.002
#         # ego_s_now
#         dismin = 1000;
#         s_now = 0
#         for i in range(last_s_index,len(x_rel_refer),1):
#             dis = np.sqrt((x_now[0] - x_rel_refer[i])**2+(x_now[1] - y_rel_refer[i])**2)
#             if dis < dismin:
#                 s_now  = s_refer[i]
#                 dismin = dis
#                 last_s_index = i
#         print("s_now",s_now)

            
#         # Planner
#         egoState = [[0,x_now[2]]]  #[s,v]
#         [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc)
#         x_planed = spl_x_s(s_now+spl_s_plan_val)
#         y_planed = spl_y_s(s_now+spl_s_plan_val)
#         dx_planed = spl_dx_s(s_now+spl_s_plan_val)
#         dy_planed = spl_dy_s(s_now+spl_s_plan_val)
#         heading_planed = [np.arctan2(dy_planed[i],dx_planed[i]) for i in range(len(dx_planed))]
#         v_planed = spl_v_plan_val
#         xDes = np.array([x_planed,y_planed,v_planed,heading_planed])
#         if t==-1:
#             print(egoState)
#             print(current_acc)
#             print("s: ",s_now+spl_s_plan_val)
#             print("x: ",x_planed)
#             print("y: ",y_planed)
#             print("v: ",v_planed)
#             print(heading_planed)

            
#         # MPC control
#         [model, feas[t], x, u, J] = min_dx(N, x_now, xDes[:,-1], zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDes,ulast) # *** zNbar per stage is last xDes of stage
#         if not feas[t]:
#             xOpt = []
#             uOpt = []
#             break

#         # Save open loop predictions
#         xPred[:, :, t] = x
        
#         uOpt[:, t] = u[:, 0].reshape(nu, )
#         # Save closed loop trajectory, collect optimal states and optimal inputs
#         # Note that the second column of x represents the optimal closed loop state
#         x2, y2, v2, psi2 = carModel(uOpt[0,t], xOpt[0, t], xOpt[1, t], xOpt[2, t], xOpt[3, t], uOpt[1,t])
#         for i in range(9):
#             x2, y2, v2, psi2 = carModel(uOpt[0,t], x2, y2, v2, psi2, uOpt[1,t])
#         xt = np.array([x2, y2, v2, psi2])
#         xOpt[:, t+1] = xt
#         current_acc = uOpt[0,t]
#         ulast = uOpt[:,t]

#     return [model, feas, xOpt, uOpt]
#     #*****************attempt at iterating 1*******************
    
    
# z0Bar = np.array([curveTrackDDesired[0,0],4,1,0])
# print(z0Bar)
# model, feas, xOpt, uOpt = mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, curveTrackDDesired, M, obs_path)

# print('Feasibility =', feas) 
# fig = plt.figure(figsize=(10, 10))
# line1 = plt.plot(curveTrackDDesired[0, :], curveTrackDDesired[1, :], 'r-')
# line2 = plt.plot(xOpt[0, :], xOpt[1, :], 'b-')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.axis('equal')
# plt.legend(['Desire Path','Real Path'])

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[0,:])
# plt.xlabel('t(s)')
# plt.ylabel('x(m)')


# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[1,:])
# plt.xlabel('t(s)')
# plt.ylabel('y(m)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[2,:])
# plt.xlabel('t(s)')
# plt.ylabel('v(m/s)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.1,0.1),xOpt[3,:])
# plt.xlabel('t(s)')
# plt.ylabel('angle(rad)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,7.0,0.1),uOpt.T)
# plt.xlabel('t(s)')
# plt.legend(['a','Beta'])
# plt.ylabel('u')
# plt.show()

# fig = plt.figure(figsize=(10,10))
# #ax = fig.add_subplot(111, autoscale_on=False, xlim=(np.min(xtrend)-5, np.max(xtrend)+5), ylim=(np.min(ytrend)-5, np.max(ytrend)+5))
# ax1 = fig.add_subplot(111, autoscale_on=False, xlim=(-5, 40), ylim=(-35, 10))
# ax1.set_aspect('equal')
# #ax1.grid()

# line1, = ax1.plot([], [], 'b-', lw=2)
# line2 = ax1.plot(curveTrackDDesired[0, :], curveTrackDDesired[1, :], 'r-')

# time_template1 = 'time = %.1fs'
# time_text1 = ax1.text(0.05, 0.9, '', transform=ax1.transAxes)

# def init():
#     line1.set_data([],[])
#     time_text1.set_text('')
#     return line1,time_text1

# def animate(i):
    
#     x = np.array([xOpt[0,i]])
#     y = np.array([xOpt[1,i]])

#     psi = np.array([xOpt[3,i]])
#     egox = np.array([x[0]+2*np.cos(psi[0])-1*np.sin(psi[0]),x[0]-2*np.cos(psi[0])-1*np.sin(psi[0]), x[0]-2*np.cos(psi[0])+1*np.sin(psi[0]), x[0]+2*np.cos(psi[0])+1*np.sin(psi[0]),x[0]+2*np.cos(psi[0])-1*np.sin(psi[0])]),
#     egoy = np.array([y[0]+2*np.sin(psi[0])+1*np.cos(psi[0]),y[0]-2*np.sin(psi[0])+1*np.cos(psi[0]), y[0]-2*np.sin(psi[0])-1*np.cos(psi[0]), y[0]+2*np.sin(psi[0])-1*np.cos(psi[0]),y[0]+2*np.sin(psi[0])+1*np.cos(psi[0])])

    
# #     line1.set_data(.1*objx, objy)  #******* ped representation ********
#     line1.set_data(egox, egoy)  #******* group of skaters representation ********
#     time_text1.set_text(time_template1 % (i*dt))
# #     line1.set_data(x,y)  # ***** dot representation ******

#     return line1,time_text1

# ani = animation.FuncAnimation(fig, animate, range(0, M), interval=0.1*1000, blit=True, init_func=init)
# rc('animation', html='jshtml')
# ani

# from scipy.interpolate import UnivariateSpline, splev, splrep, CubicSpline
# M = 100  # Simulation time
# N = 5


# def mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path):

#     #*************Initialize arrays************
#         # *** initialize states and inputs ***
#     nz = nx = 4         # number of states: x1 = x value of cars position (x), x2 = y value of cars position (y), x3 = v = sqrt(x**2 + y**2), x4 = psi
#     nu = 2              # number of inputs: u1 = acceleration (a), u2 = steering input (df)

#     xOpt = np.zeros((nx, M+1))
#     uOpt = np.zeros((nu, M))
#     xOpt[:, 0] = z0Bar.reshape(nx, )
#     xOptpred = np.zeros((nx, M+1))
#     xPred = np.zeros((nx, N+1, M))

#     feas = np.zeros((M, ), dtype=bool)
    
#     # process ob reference 
#     x_ob_refer = obs_path[0,:]
#     y_ob_refer = obs_path[1,:]
#     S_ob_END,s_ob_refer,x_ob_rel_refer,y_ob_rel_refer, spl_ob_x_s, spl_ob_y_s, spl_ob_dx_s,spl_ob_dy_s = RefernceGenerater(x_ob_refer,y_ob_refer)
    
    
#     # process ego reference 
#     x_refer = 0*np.arange(0,300,1)
#     y_refer = np.arange(0,300,1)
#     S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s = RefernceGenerater(x_refer,y_refer)
    
    
#     # init ego Planner
#     myplanner = IDMPY5a.IDMPlanner(4.0)
#     current_acc = np.array([0])
#     ulast = np.array([0,0])
#     # find the interation point
#     S_interaction_ego,S_interaction_ob = CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s)
    
#     # Simulation Loop
#     last_s_index = 0
#     last_s_ob_index = 0
#     have_add=False
#     for t in range(M):
        
#         # ob_s_now
#         dismin = 1000;
#         s_ob_now = 0
#         for i in range(last_s_ob_index,len(x_ob_rel_refer),1):
#             dis = np.sqrt((obs_path[0,t] - x_ob_rel_refer[i])**2+(obs_path[1,t] - y_ob_rel_refer[i])**2)
#             if dis < dismin:
#                 s_ob_now  = s_ob_refer[i]
#                 dismin = dis
#                 last_s_ob_index = i
#         print("s_ob_now",s_ob_now)
        
#         # add noisy of measurement
#         x_now = xOpt[:,t]
#         for i in range(3):
#             x_now[i] += random.rand() *.03
#         x_now[3] += random.rand() *.002
#         # ego_s_now
#         dismin = 1000;
#         s_now = 0
#         for i in range(last_s_index,len(x_rel_refer),1):
#             dis = np.sqrt((x_now[0] - x_rel_refer[i])**2+(x_now[1] - y_rel_refer[i])**2)
#             if dis < dismin:
#                 s_now  = s_refer[i]
#                 dismin = dis
#                 last_s_index = i
#         print("s_now",s_now)
#         if s_now >= S_END:
#             break
            
#         # eye sight
#         left_bound_eye = (-6-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3-6
#         right_bound_eye = (2-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3+2
            
#         # Planner
#         egoState = [[0,x_now[2]]]  #[s,v]
#         if (obs_path[0,t]+0.5 >= left_bound_eye or have_add) and S_interaction_ob-s_ob_now>-2.5:
#             have_add = True
#             OBState = [[(S_interaction_ego-s_now)-(S_interaction_ob-s_ob_now),obs_path[2,t]]]
#             print('OBState: ',OBState)
#             [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc,OBState,True,5,5)
#         else:
#             [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc)
#         x_planed = spl_x_s(s_now+spl_s_plan_val)
#         y_planed = spl_y_s(s_now+spl_s_plan_val)
#         dx_planed = spl_dx_s(s_now+spl_s_plan_val)
#         dy_planed = spl_dy_s(s_now+spl_s_plan_val)
#         heading_planed = [np.arctan2(dy_planed[i],dx_planed[i]) for i in range(len(dx_planed))]
#         v_planed = spl_v_plan_val
#         xDes = np.array([x_planed,y_planed,v_planed,heading_planed])
#         if t==-1:
#             print(egoState)
#             print(current_acc)
#             print("s: ",s_now+spl_s_plan_val)
#             print("x: ",x_planed)
#             print("y: ",y_planed)
#             print("v: ",v_planed)
#             print(heading_planed)

            
#         # MPC control
#         [model, feas[t], x, u, J] = min_dx(N, x_now, xDes[:,-1], zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDes,ulast) # *** zNbar per stage is last xDes of stage
#         if not feas[t]:
#             xOpt = []
#             uOpt = []
#             break

#         # Save open loop predictions
#         xPred[:, :, t] = x

#         uOpt[:, t] = u[:, 0].reshape(nu, )
#         # Save closed loop trajectory, collect optimal states and optimal inputs
#         # Note that the second column of x represents the optimal closed loop state
#         x2, y2, v2, psi2 = carModel(uOpt[0,t], xOpt[0, t], xOpt[1, t], xOpt[2, t], xOpt[3, t], uOpt[1,t])
#         for i in range(9):
#             x2, y2, v2, psi2 = carModel(uOpt[0,t], x2, y2, v2, psi2, uOpt[1,t])
#         xt = np.array([x2, y2, v2, psi2])
#         xOpt[:, t+1] = xt
#         current_acc = uOpt[0,t]
#         ulast = uOpt[:,t]

#     return [model, feas, xOpt, uOpt]
#     #*****************attempt at iterating 1*******************
    
    
# z0Bar = np.array([0.8,82,4,xDesired[3,0]])
# print(z0Bar)
# model, feas, xOpt, uOpt = mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path)

# print('Feasibility =', feas) 
# fig = plt.figure(figsize=(10, 10))
# line1 = plt.plot(xDesired[0, 41:], xDesired[1, 41:], 'r-')
# line2 = plt.plot(xOpt[0, :], xOpt[1, :], 'b-')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.axis('equal')
# plt.xlim((-5,5))

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[0,:])
# plt.xlabel('t(s)')
# plt.ylabel('x(m)')
# plt.ylim((-2,2))

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[1,:])
# plt.xlabel('t(s)')
# plt.ylabel('y(m)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[2,:])
# plt.xlabel('t(s)')
# plt.ylabel('v(m/s)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[3,:])
# plt.xlabel('t(s)')
# plt.ylabel('angle(rad)')
# plt.ylim((0,np.pi))

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.0,0.1),uOpt.T)
# plt.xlabel('t(s)')
# plt.legend(['a','Beta'])
# plt.ylabel('u')
# plt.show()

# fig = plt.figure(figsize=(10,10))
# #ax = fig.add_subplot(111, autoscale_on=False, xlim=(np.min(xtrend)-5, np.max(xtrend)+5), ylim=(np.min(ytrend)-5, np.max(ytrend)+5))
# ax1 = fig.add_subplot(111, autoscale_on=False, xlim=(-25, 25), ylim=(80, 160))
# ax1.set_aspect('equal')
# #ax1.grid()

# line1, = ax1.plot([], [], 'b-', lw=2)
# line2, = ax1.plot([], [], 'ro-', lw=2)
# line3, = ax1.plot(np.arange(-25,-5,1), 147+0*np.arange(-25,-5,1),'g-', lw=2)
# line4, = ax1.plot(np.arange(2,26,1), 147+0*np.arange(2,26,1), 'g-', lw=2)
# line5, = ax1.plot(-6+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)
# line6, = ax1.plot(2+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)

# line7, = ax1.plot([], [], 'm-', lw=1)
# line8, = ax1.plot([], [], 'm-', lw=1)
# time_template1 = 'time = %.1fs'
# time_text1 = ax1.text(0.05, 0.9, '', transform=ax1.transAxes)

# def init():
#     line1.set_data([],[])
#     line2.set_data([],[])
#     line7.set_data([],[])
#     line8.set_data([],[])
#     time_text1.set_text('')
#     return line1,time_text1

# def animate(i):
    
#     x = np.array([xOpt[0,i], obs_path[0,i]])
#     y = np.array([xOpt[1,i], obs_path[1,i]])

#     psi = np.array([xOpt[3,i], obs_path[3,i]])
#     egox = np.array([x[0]+2*np.cos(psi[0])-1*np.sin(psi[0]),x[0]-2*np.cos(psi[0])-1*np.sin(psi[0]), x[0]-2*np.cos(psi[0])+1*np.sin(psi[0]), x[0]+2*np.cos(psi[0])+1*np.sin(psi[0]),x[0]+2*np.cos(psi[0])-1*np.sin(psi[0])]),
#     obx = np.array([x[1]+np.cos(psi[1]), x[1], x[1]-np.cos(psi[1])])
#     egoy = np.array([y[0]+2*np.sin(psi[0])+1*np.cos(psi[0]),y[0]-2*np.sin(psi[0])+1*np.cos(psi[0]), y[0]-2*np.sin(psi[0])-1*np.cos(psi[0]), y[0]+2*np.sin(psi[0])-1*np.cos(psi[0]),y[0]+2*np.sin(psi[0])+1*np.cos(psi[0])])
#     oby = np.array([y[1]+np.sin(psi[1]), y[1], y[1]-np.sin(psi[1])])
    
#     if y[0]+2*np.sin(psi[0])<147:
#         eye_line1x = np.array([x[0]+2*np.cos(psi[0]),-6,(-6-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8-6])
#         eye_line1y = np.array([y[0]+2*np.sin(psi[0]),147,155])

#         eye_line2x = np.array([x[0]+2*np.cos(psi[0]),2,(2-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8+2])
#         eye_line2y = np.array([y[0]+2*np.sin(psi[0]),147,155])
#     else:
#         eye_line1x = np.array([])
#         eye_line1y = np.array([])

#         eye_line2x = np.array([])
#         eye_line2y = np.array([])

    
# #     line1.set_data(.1*objx, objy)  #******* ped representation ********
#     line1.set_data(egox, egoy)  #******* group of skaters representation ********
#     line2.set_data(obx, oby)
#     line7.set_data(eye_line1x, eye_line1y)
#     line8.set_data(eye_line2x, eye_line2y)
#     time_text1.set_text(time_template1 % (i*dt))
# #     line1.set_data(x,y)  # ***** dot representation ******

#     return line1,time_text1

# ani = animation.FuncAnimation(fig, animate, range(0, M), interval=0.1*1000, blit=True, init_func=init)
# rc('animation', html='jshtml')
# ani

# from scipy.interpolate import UnivariateSpline, splev, splrep, CubicSpline
# M = 100  # Simulation time
# N = 5


# def mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path):

#     #*************Initialize arrays************
#         # *** initialize states and inputs ***
#     nz = nx = 4         # number of states: x1 = x value of cars position (x), x2 = y value of cars position (y), x3 = v = sqrt(x**2 + y**2), x4 = psi
#     nu = 2              # number of inputs: u1 = acceleration (a), u2 = steering input (df)

#     xOpt = np.zeros((nx, M+1))
#     uOpt = np.zeros((nu, M))
#     xOpt[:, 0] = z0Bar.reshape(nx, )
#     xOptpred = np.zeros((nx, M+1))
#     xPred = np.zeros((nx, N+1, M))

#     feas = np.zeros((M, ), dtype=bool)
    
#     # process ob reference 
#     x_ob_refer = obs_path[0,:]
#     y_ob_refer = obs_path[1,:]
#     S_ob_END,s_ob_refer,x_ob_rel_refer,y_ob_rel_refer, spl_ob_x_s, spl_ob_y_s, spl_ob_dx_s,spl_ob_dy_s = RefernceGenerater(x_ob_refer,y_ob_refer)
    
    
#     # process ego reference 
#     x_refer = 0*np.arange(0,300,1)
#     y_refer = np.arange(0,300,1)
#     S_END,s_refer,x_rel_refer,y_rel_refer,spl_x_s,spl_y_s,spl_dx_s,spl_dy_s = RefernceGenerater(x_refer,y_refer)
    
    
#     # init ego Planner
#     myplanner = IDMPY5a.IDMPlanner(4.0)
#     current_acc = np.array([0])
#     ulast = np.array([0,0])
#     # find the interation point
#     S_interaction_ego,S_interaction_ob = CheckInteractionPoint(S_ob_END,spl_ob_x_s,spl_ob_y_s,S_END,spl_x_s,spl_y_s)
    
#     # Simulation Loop
#     last_s_index = 0
#     last_s_ob_index = 0
#     have_add=False
#     for t in range(M):
        
#         # ob_s_now
#         dismin = 1000;
#         s_ob_now = 0
#         for i in range(last_s_ob_index,len(x_ob_rel_refer),1):
#             dis = np.sqrt((obs_path[0,t] - x_ob_rel_refer[i])**2+(obs_path[1,t] - y_ob_rel_refer[i])**2)
#             if dis < dismin:
#                 s_ob_now  = s_ob_refer[i]
#                 dismin = dis
#                 last_s_ob_index = i
#         print("s_ob_now",s_ob_now)
        
#         # add noisy of measurement
#         x_now = xOpt[:,t]
#         for i in range(3):
#             x_now[i] += random.rand() *.03
#         x_now[3] += random.rand() *.002
#         # ego_s_now
#         dismin = 1000;
#         s_now = 0
#         for i in range(last_s_index,len(x_rel_refer),1):
#             dis = np.sqrt((x_now[0] - x_rel_refer[i])**2+(x_now[1] - y_rel_refer[i])**2)
#             if dis < dismin:
#                 s_now  = s_refer[i]
#                 dismin = dis
#                 last_s_index = i
#         print("s_now",s_now)
#         if s_now >= S_END:
#             break
            
#         # eye sight
#         left_bound_eye = (-6-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3-6
#         right_bound_eye = (2-(xOpt[0,t]+2*np.cos(psi[0])))/(147-(xOpt[1,t]+2*np.sin(psi[0])))*3+2
            
#         # Planner
#         egoState = [[0,x_now[2]]]  #[s,v]
#         OBState = [[(S_interaction_ego-s_now)-(S_interaction_ob-s_ob_now),obs_path[2,t]]]
#         print('OBState: ',OBState)
#         if (obs_path[0,t]+0.5 >= left_bound_eye or have_add) and S_interaction_ob-s_ob_now>-2.5:
#             have_add = True
#             [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc,OBState,True,2,1)
#         elif S_interaction_ob-s_ob_now>-2.5:
#             [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc,OBState,True,5,5)
#         else:
#             [t_sub,spl_s_plan_val,spl_v_plan_val] = myplanner.Planning(egoState,current_acc)
#         x_planed = spl_x_s(s_now+spl_s_plan_val)
#         y_planed = spl_y_s(s_now+spl_s_plan_val)
#         dx_planed = spl_dx_s(s_now+spl_s_plan_val)
#         dy_planed = spl_dy_s(s_now+spl_s_plan_val)
#         heading_planed = [np.arctan2(dy_planed[i],dx_planed[i]) for i in range(len(dx_planed))]
#         v_planed = spl_v_plan_val
#         xDes = np.array([x_planed,y_planed,v_planed,heading_planed])
#         if t==-1:
#             print(egoState)
#             print(current_acc)
#             print("s: ",s_now+spl_s_plan_val)
#             print("x: ",x_planed)
#             print("y: ",y_planed)
#             print("v: ",v_planed)
#             print(heading_planed)

            
#         # MPC control
#         [model, feas[t], x, u, J] = min_dx(N, x_now, xDes[:,-1], zMax, zMin, uMax, uMin, udotMax, udotMin, Ts, nz, nu, xDes,ulast) # *** zNbar per stage is last xDes of stage
#         if not feas[t]:
#             xOpt = []
#             uOpt = []
#             break

#         # Save open loop predictions
#         xPred[:, :, t] = x

#         uOpt[:, t] = u[:, 0].reshape(nu, )
#         # Save closed loop trajectory, collect optimal states and optimal inputs
#         # Note that the second column of x represents the optimal closed loop state
#         x2, y2, v2, psi2 = carModel(uOpt[0,t], xOpt[0, t], xOpt[1, t], xOpt[2, t], xOpt[3, t], uOpt[1,t])
#         for i in range(9):
#             x2, y2, v2, psi2 = carModel(uOpt[0,t], x2, y2, v2, psi2, uOpt[1,t])
#         xt = np.array([x2, y2, v2, psi2])
#         xOpt[:, t+1] = xt
#         current_acc = uOpt[0,t]
#         ulast = uOpt[:,t]

#     return [model, feas, xOpt, uOpt]
#     #*****************attempt at iterating 1*******************
    
    
# z0Bar = np.array([0.8,82,4,xDesired[3,0]])
# print(z0Bar)
# model, feas, xOpt, uOpt = mpcsim(N, z0Bar, zNBar, zMax, zMin, uMax, uMin, udotMax, udotMin, nz, nu, xDesired, M, obs_path)

# print('Feasibility =', feas) 
# fig = plt.figure(figsize=(10, 10))
# line1 = plt.plot(xDesired[0, 41:], xDesired[1, 41:], 'r-')
# line2 = plt.plot(xOpt[0, :], xOpt[1, :], 'b-')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.axis('equal')
# plt.xlim((-5,5))

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[0,:])
# plt.xlabel('t(s)')
# plt.ylabel('x(m)')
# plt.ylim((-2,2))

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[1,:])
# plt.xlabel('t(s)')
# plt.ylabel('y(m)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[2,:])
# plt.xlabel('t(s)')
# plt.ylabel('v(m/s)')

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.1,0.1),xOpt[3,:])
# plt.xlabel('t(s)')
# plt.ylabel('angle(rad)')
# plt.ylim((0,np.pi))

# fig = plt.figure(figsize=(8, 5))
# plt.plot(np.arange(0,10.0,0.1),uOpt.T)
# plt.xlabel('t(s)')
# plt.legend(['a','Beta'])
# plt.ylabel('u')
# plt.show()

# fig = plt.figure(figsize=(10,10))
# #ax = fig.add_subplot(111, autoscale_on=False, xlim=(np.min(xtrend)-5, np.max(xtrend)+5), ylim=(np.min(ytrend)-5, np.max(ytrend)+5))
# ax1 = fig.add_subplot(111, autoscale_on=False, xlim=(-25, 25), ylim=(80, 160))
# ax1.set_aspect('equal')
# #ax1.grid()

# line1, = ax1.plot([], [], 'b-', lw=2)

# line2, = ax1.plot([], [], 'ro-', lw=2)
# GPS_noisy = plt.Circle((0, 0), 2.5, color='y', alpha=0.5)

# line3, = ax1.plot(np.arange(-25,-5,1), 147+0*np.arange(-25,-5,1),'g-', lw=2)
# line4, = ax1.plot(np.arange(2,26,1), 147+0*np.arange(2,26,1), 'g-', lw=2)
# line5, = ax1.plot(-6+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)
# line6, = ax1.plot(2+0*np.arange(80,148,1), np.arange(80,148,1), 'g-', lw=2)

# line7, = ax1.plot([], [], 'm-', lw=1)
# line8, = ax1.plot([], [], 'm-', lw=1)
# time_template1 = 'time = %.1fs'
# time_text1 = ax1.text(0.05, 0.9, '', transform=ax1.transAxes)

# def init():
#     line1.set_data([],[])
#     line2.set_data([],[])
#     line7.set_data([],[])
#     line8.set_data([],[])
#     ax1.add_patch(GPS_noisy)
#     time_text1.set_text('')
#     return line1,time_text1

# def animate(i):
    
#     x = np.array([xOpt[0,i], obs_path[0,i]])
#     y = np.array([xOpt[1,i], obs_path[1,i]])

#     psi = np.array([xOpt[3,i], obs_path[3,i]])
#     egox = np.array([x[0]+2*np.cos(psi[0])-1*np.sin(psi[0]),x[0]-2*np.cos(psi[0])-1*np.sin(psi[0]), x[0]-2*np.cos(psi[0])+1*np.sin(psi[0]), x[0]+2*np.cos(psi[0])+1*np.sin(psi[0]),x[0]+2*np.cos(psi[0])-1*np.sin(psi[0])]),
#     obx = np.array([x[1]+np.cos(psi[1]), x[1], x[1]-np.cos(psi[1])])
#     egoy = np.array([y[0]+2*np.sin(psi[0])+1*np.cos(psi[0]),y[0]-2*np.sin(psi[0])+1*np.cos(psi[0]), y[0]-2*np.sin(psi[0])-1*np.cos(psi[0]), y[0]+2*np.sin(psi[0])-1*np.cos(psi[0]),y[0]+2*np.sin(psi[0])+1*np.cos(psi[0])])
#     oby = np.array([y[1]+np.sin(psi[1]), y[1], y[1]-np.sin(psi[1])])

#     if x[1]+1 < (-6-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*3-6 and x[1]<0:
#         GPS_noisy.center = (x[1], y[1])
#     else:
#         GPS_noisy.center = (0, 0)
    
    
#     if y[0]+2*np.sin(psi[0])<147:
#         eye_line1x = np.array([x[0]+2*np.cos(psi[0]),-6,(-6-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8-6])
#         eye_line1y = np.array([y[0]+2*np.sin(psi[0]),147,155])

#         eye_line2x = np.array([x[0]+2*np.cos(psi[0]),2,(2-(x[0]+2*np.cos(psi[0])))/(147-(y[0]+2*np.sin(psi[0])))*8+2])
#         eye_line2y = np.array([y[0]+2*np.sin(psi[0]),147,155])
#     else:
#         eye_line1x = np.array([])
#         eye_line1y = np.array([])

#         eye_line2x = np.array([])
#         eye_line2y = np.array([])

    
# #     line1.set_data(.1*objx, objy)  #******* ped representation ********
#     line1.set_data(egox, egoy)  #******* group of skaters representation ********
#     line2.set_data(obx, oby)
#     line7.set_data(eye_line1x, eye_line1y)
#     line8.set_data(eye_line2x, eye_line2y)
#     time_text1.set_text(time_template1 % (i*dt))
# #     line1.set_data(x,y)  # ***** dot representation ******

#     return line1,time_text1

# ani = animation.FuncAnimation(fig, animate, range(0, M), interval=0.1*1000, blit=True, init_func=init)
# rc('animation', html='jshtml')
# ani

