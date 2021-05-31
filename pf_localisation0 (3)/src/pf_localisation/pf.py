from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.05 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.2 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.2 # Odometry model y axis (side-to-side) noise

 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        print(initialpose)
        particle = PoseArray()
        particle.poses = [Odometry().pose.pose] * 50
        print('Odometry',Odometry())
        print("mypose", particle)
        
        # print('Pose',pose)
        mu = 0
        sigma = 1.0
        
        

        for i in range(0,len(particle.poses)):
            pose= Odometry().pose.pose
            pose.position.x = initialpose.pose.pose.position.x + random.gauss(0,self.ODOM_TRANSLATION_NOISE)
            pose.position.y = initialpose.pose.pose.position.y + random.gauss(0,self.ODOM_DRIFT_NOISE)
            
            pose.orientation = rotateQuaternion(initialpose.pose.pose.orientation, random.gauss(0,self.ODOM_ROTATION_NOISE))

            # pose.orientation = head
            # print(pose)
            # print("heading", head)
            particle.poses[i] = pose
        print('Particle',particle)
        
        return particle

    def update_particle_cloud(self, scan):
        print('int2')
        print('Particle cloud', self.particlecloud.poses)
        # Update particlecloud, given map and laser scan


    def estimate_pose(self):
        print('in3')
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
        return(self.particlecloud.poses[0])