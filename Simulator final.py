#!/usr/bin/env python
# coding: utf-8

# In[2]:


get_ipython().run_line_magic('pylab', 'inline')
import numpy as np


# In[3]:


# Perfect sensor
def sense(x):
    return x


# In[4]:


def simulate(Δt, x, u):
    x += Δt * u
    return x


# In[5]:


# ellipse


# In[6]:


def control(t, y):
    ux = -1.732*sin(t)-.5*cos(t)
    uy = -1*sin(t)+cos(t)*.886
    return array([ux, uy])


# In[47]:


tf = 13.
Δt = 0.1    # Time step
time = linspace(0.,tf, int(tf / Δt) + 1)  # Time interval


# Initial conditions
x = array([6., 4.])
x_log = [copy(x)]

for t in time:
    y = sense(x)
    u = control(t, y)    
    x = simulate(Δt, x, u)
    x_log.append(copy(x))
    
x_log = array(x_log)


# In[48]:


grid()
plot(x_log[:,0], x_log[:,1])


# In[49]:


import matplotlib.pyplot as plt
from matplotlib import animation
from JSAnimation import IPython_display    
from IPython.display import HTML
    


fig, ax = plt.subplots()

def animate(t):
    ax.clear()
    
    # Path
    plot(x_log[:,0], x_log[:,1], 'r-')
    
    # Initial conditions
    plot(x_log[t,0], x_log[t,1], 'bo')
    
    

anim = animation.FuncAnimation(fig, animate, frames=len(time), interval=60)

HTML(anim.to_jshtml())


# In[ ]:


#figure 8


# In[10]:


def control8(t, y):
    ux = cos(t)*.707-(sin(t)**2-cos(t)**2)*.707
    uy = (sin(t)**2-cos(t)**2)*.707+cos(t)*.707
    return array([ux, uy])


# In[44]:


tf = 13.
Δt = 0.1    # Time step
time = linspace(0.,tf, int(tf / Δt) + 1)  # Time interval


# Initial conditions
x = array([0., 0.])
x_log = [copy(x)]

for t in time:
    y = sense(x)
    u = control8(t, y)    
    x = simulate(Δt, x, u)
    x_log.append(copy(x))
    
x_log = array(x_log)


# In[45]:


grid()
plot(x_log[:,0], x_log[:,1])


# In[46]:


import matplotlib.pyplot as plt
from matplotlib import animation
from JSAnimation import IPython_display    
from IPython.display import HTML
    


fig, ax = plt.subplots()

def animate(t):
    ax.clear()
    
    # Path
    plot(x_log[:,0], x_log[:,1], 'r-')
    
    # Initial conditions
    plot(x_log[t,0], x_log[t,1], 'bo')
    
    

anim = animation.FuncAnimation(fig, animate, frames=len(time), interval=60)

HTML(anim.to_jshtml())


# In[ ]:


#helix


# In[14]:


def controlheli(t, y):
    ux=-sin(t)
    uy=cos(t)
    uh=2/(5*3.14)
    return array([ux,uy,uh])


# In[52]:


tf = 10.
Δt = 0.1    # Time step
time = linspace(0.,tf, int(tf / Δt) + 1)  # Time interval


# Initial conditions
x = array([0., 0.,0])
x_log = [copy(x)]

for t in time:
    y = sense(x)
    u = controlheli(t, y)    
    x = simulate(Δt, x, u)
    x_log.append(copy(x))
    
x_log = array(x_log)


# In[53]:


ax = plt.axes(projection='3d')
ax.plot3D(x_log[:,0], x_log[:,1],x_log[:,2])


# In[59]:


import matplotlib.pyplot as plt
from matplotlib import animation
from JSAnimation import IPython_display    
from IPython.display import HTML
    

fig = plt.figure()
ax = plt.axes(projection='3d')

def animate(t):
    ax.clear()
    
    # Path
    ax.plot3D(x_log[:,0], x_log[:,1],x_log[:,2], 'r-')
    
    # Initial conditions
    ax.plot3D(x_log[t,0], x_log[t,1],x_log[t,2], 'bo')
    
    

anim = animation.FuncAnimation(fig, animate, frames=len(time), interval=60)

HTML(anim.to_jshtml())


# In[ ]:




