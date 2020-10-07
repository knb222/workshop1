#!/usr/bin/env python
# coding: utf-8

# In[50]:


import sim
import numpy
get_ipython().run_line_magic('pylab', 'inline')


# In[51]:


sim.simxFinish(-1)  # Close opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print('Connected')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)

    print('Simulation time in milliseconds: ', sim.simxGetLastCmdTime(clientID))
    
    # Get Object position
    name = 'Omnirob'
    err_code, cuboid = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking)
    res, position = sim.simxGetObjectPosition(clientID, cuboid, -1, sim.simx_opmode_blocking)        
    print('Omnirob is at [x,y,z]=', position)
    
    # Now close the connection to CoppeliaSim:
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
    print('Disconnected')
else:
    print('Failed connecting to remote API server')


# In[52]:


class robot():
    
    def __init__(self, frame_name, motor_names=[], client_id=0):  
        # If there is an existing connection
        if client_id:
                self.client_id = client_id
        else:
            self.client_id = self.open_connection()
            
        self.motors = self._get_handlers(motor_names) 
        
        # Robot frame
        self.frame =  self._get_handler(frame_name)
            
        
    def open_connection(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim 
        
        if clientID != -1:
            print('Robot connected')
        else:
            print('Connection failed')
        return clientID
        
    def close_connection(self):    
        sim.simxGetPingTime(self.client_id)  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        sim.simxFinish(self.client_id)  # Now close the connection to CoppeliaSim:
        print('Connection closed')
    
    def isConnected(self):
        c,result = sim.simxGetPingTime(self.client_id)
        # Return true if the robot is connected
        return result > 0         
        
    def _get_handler(self, name):
        err_code, handler = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_blocking)
        return handler
    
    def _get_handlers(self, names):
        handlers = []
        for name in names:
            handler = self._get_handler(name)
            handlers.append(handler)
        
        return handlers

    def send_motor_velocities(self, vels):
        for motor, vel in zip(self.motors, vels):
            err_code = sim.simxSetJointTargetVelocity(self.client_id, 
                                                      motor, vel, sim.simx_opmode_streaming)      
            
    def set_position(self, position, relative_object=-1):
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)        
        sim.simxSetObjectPosition(clientID, self.frame, relative_object, position, sim.simx_opmode_oneshot)
        
    def simtime(self):
        return sim.simxGetLastCmdTime(self.client_id)
    
    def get_position(self, relative_object=-1):
        # Get position relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, position = sim.simxGetObjectPosition(self.client_id, self.frame, relative_object, sim.simx_opmode_blocking)        
        return array(position)
    
    def get_object_position(self, object_name):
        # Get Object position in the world frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, -1, sim.simx_opmode_blocking)
        return array(position)
    
    def get_object_relative_position(self, object_name):        
        # Get Object position in the robot frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, self.frame, sim.simx_opmode_blocking)
        return array(position)


# In[53]:


r = robot('Quadricopter_target')


# In[54]:


def traject(rp, dp):
    tf = 50.
    tstep = 0.1
    a0 = numpy.array(rp)
    a1 = 0
    a2 = (3*numpy.array(dp)-3*a0)/tf**2
    a3 = (2*a0-2*numpy.array(dp))/tf**3

    # Time interval
    time_steps = linspace(0, tf, int(tf/tstep))

    for t in time_steps:
        # Compute the trajectory
        traj_point = double(a0+a1*t+a2*(t**2)+a3*(t**3))
        vel_traj = double(3*a3*(t**2)+2*a2*t+a1)

        # Location sesing
        robot_position = r.get_position()
    
        r.set_position(traj_point)
    
        time.sleep(tstep)


# In[55]:


#inbetween
desired_position = [2.5, 2.675, 1.475]
robot_position = r.get_position()
traject(robot_position, desired_position)

#sphere 7
desired_position = r.get_object_position('Sphere7')
robot_position = r.get_position()
traject(robot_position, desired_position)

#sphere 10
desired_position = r.get_object_position('Sphere10')
robot_position = r.get_position()
traject(robot_position, desired_position)

#inbetween
desired_position = [6.975, 1.2, 5]
robot_position = r.get_position()
traject(robot_position, desired_position)

#sphere 8
desired_position = r.get_object_position('Sphere8')
robot_position = r.get_position()
traject(robot_position, desired_position)

#inbetween
desired_position=[4.625,-6.85,5]
robot_position = r.get_position()
traject(robot_position, desired_position)

#sphere 9
desired_position = r.get_object_position('Sphere9')
robot_position = r.get_position()
traject(robot_position, desired_position)

#inbetween
desired_position=[-9.125,-8.475,5.625]
robot_position = r.get_position()
traject(robot_position, desired_position)


#sphere 5
desired_position = r.get_object_position('Sphere5')
robot_position = r.get_position()
traject(robot_position, desired_position)

#inbetween
desired_position=[-3.775,8,5.625]
robot_position = r.get_position()
traject(robot_position, desired_position)

#sphere 6
desired_position = r.get_object_position('Sphere6')
robot_position = r.get_position()
traject(robot_position, desired_position)


# In[ ]:




