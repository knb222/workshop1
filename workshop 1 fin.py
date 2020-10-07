#!/usr/bin/env python
# coding: utf-8

# In[6]:


import sim
import numpy
get_ipython().run_line_magic('pylab', 'inline')


# In[7]:


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


# In[8]:


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


# In[242]:





# In[9]:


def traject(rp, dp):
    tf = 50.
    tstep = 0.1
    a0 = rp
    a1 = 0
    a2 = (3*dp-3*a0)/tf**2
    a3 = (2*a0-2*dp)/tf**3

    # Time interval
    time_steps = linspace(0, tf, int(tf/tstep))

    for t in time_steps:
        # Compute the trajectory
        traj_point = double(a0+a1*t+a2*(t**2)+a3*(t**3))
        vel_traj = double(3*a3*(t**2)+2*a2*t+a1)

        # Location sesing
        robot_position = r.get_position()
    
        # drive the robot using the trajectory tracker
        u = 5*(traj_point - robot_position) + vel_traj
    
    
        vx, vy, vz = u
        r.send_motor_velocities([-vy - vx, vy - vx, vy + vx, -vy + vx])
        time.sleep(tstep)


    
    r.send_motor_velocities([0, 0, 0, 0])


# In[10]:


####wheel robot
motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot

#sphere 1
robot_position = r.get_position()
desired_position = r.get_object_position('Sphere1')
traject(robot_position, desired_position)

#inbetween 1
robot_position = r.get_position()
desired_position = numpy.array([7, 8.5, 0])
traject(robot_position, desired_position)

# sphere 2
robot_position = r.get_position()
desired_position = r.get_object_position('Sphere')
traject(robot_position, desired_position)

#inbetween 2
robot_position = r.get_position()
desired_position = numpy.array([8, 2.225, 0])
traject(robot_position, desired_position)

#sphere 3
robot_position = r.get_position()
desired_position = r.get_object_position('Sphere2')
traject(robot_position, desired_position)

#inbetween 3.1
robot_position = r.get_position()
desired_position = numpy.array([8.75, -2.5, 0])
traject(robot_position, desired_position)

#inbetween 3.2
robot_position = r.get_position()
desired_position = numpy.array([8.75, -6.15, 0])
traject(robot_position, desired_position)

#sphere 4
robot_position = r.get_position()
desired_position = r.get_object_position('Sphere3')
traject(robot_position, desired_position)

#inbetween 4
robot_position = r.get_position()
desired_position = numpy.array([1, -6.15, 0])
traject(robot_position, desired_position)

#inbetween 4
robot_position = r.get_position()
desired_position = numpy.array([-3.5, 2.75, 0])
traject(robot_position, desired_position)

#sphere 5
robot_position = r.get_position()
desired_position = r.get_object_position('Sphere0')
traject(robot_position, desired_position)

#inbetween 5
robot_position = r.get_position()
desired_position = numpy.array([0, -1, 0])
traject(robot_position, desired_position)

#inbetween 5.2
robot_position = r.get_position()
desired_position = numpy.array([-4, -8.5, 0])
traject(robot_position, desired_position)

#sphere 6
robot_position = r.get_position()
print('Robot position: (%.2f, %.2f) '%(robot_position[0], robot_position[1]))
desired_position = r.get_object_position('Sphere4')
traject(robot_position, desired_position)

