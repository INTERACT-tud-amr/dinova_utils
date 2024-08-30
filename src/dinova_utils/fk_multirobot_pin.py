#!/usr/bin/env python
import rospkg
import rospy
#from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
#import pinocchio for fk
from dinova_utils.fk_pin import MobileManipulatorKinematics #add dinova_utils to avoid module import error

import numpy as np
from sensor_msgs.msg import JointState
from typing import Union, Dict, List
import copy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from derived_object_msgs.msg import Object, ObjectArray


class FKMultiRobot():
    def __init__(self, robot_name):
        # ---- variables from yaml file ---- #
        self.robot_name = robot_name
        self.other_agents = rospy.get_param("all_agents")
        
        del self.other_agents[self.robot_name]
        #get the configuration states of other robots
        self._q_other_agents = [None] * len(self.other_agents)
        #get the configuration velocity of other robots
        self._v_other_agents = [None] * len(self.other_agents)

        
        self.forward_robot_kinematics = MobileManipulatorKinematics()#initialize before the subscriber, othersie no arribute
        self._init_subscribers()
        
    def _init_subscribers(self):
        # --- currently only subscribing to 1 other robot ---- #
        other_agent_name = list(self.other_agents.keys())[0]
        if rospy.get_param("real_robot"):
            self._joint_states_sub = rospy.Subscriber("/"+other_agent_name+'/dinova/omni_states_vicon', JointState, self._joint_states_cb)
            self._joint_velocity_sub = rospy.Subscriber("/"+other_agent_name+'/filtered_velocities', JointState, self._joint_velocities_cb)
        else:
            self._joint_states_sub = rospy.Subscriber("/"+other_agent_name+'/dinova/omni_states', JointState, self._joint_states_simulation_cb) 
            #the velocity can be got directly

    def _joint_states_cb(self, msg: JointState):
        self._q_other_agents[0] = np.array(msg.position)[0:9]
    
    def _joint_velocities_cb(self, msg: JointState):
        self._v_other_agents[0] = np.array(msg.velocity)[0:9]
        
    def _joint_states_simulation_cb(self, msg: JointState):
        self._q_other_agents[0] = np.array(msg.position)[0:9]
        self._v_other_agents[0] = np.array(msg.velocity)[0:9]
            
    def object_naming(self, object_names:List[str]) -> List[str]:#maybe not used???
        for agent in self.other_agents:
            if agent in object_names:
                object_names.remove(agent)
            for collision_link in self.collision_links[agent]:
                object_names.append(agent+"_"+collision_link)
        return object_names
                
    def collision_spheres_other_agent(self, object_names, object_poses):
        object_poses_full = copy.deepcopy(object_poses)
        object_twists = {} #a dictionary to hold twists 
        if self._q_other_agents[0] is not None:
            for agent_name, agent in self.other_agents.items():
                if agent_name in object_names and agent_name in object_poses:
                    # 1. agent_name is defined in vicon_dinova.yaml 
                    # 2. only pop it when it is available, otherwise error!
                    object_poses_full.pop(agent_name)#pop the agent's correpsonding pose and attach its fk poses
                    #Apply forward kinematics to the robot 
                    self.forward_robot_kinematics.forward(q=self._q_other_agents[0], v=self._v_other_agents[0])
                    for collision_link in agent['collision_links']:# list
                        # object_pose = self.forward_kinematics.numpy(q=self._q_other_agents[0],
                        #                             parent_link = "base_link",
                        #                             child_link = collision_link,
                        #                             position_only=True)
                        # replaced by pinocchio
                        # only get the translation part
                        object_pose = self.forward_robot_kinematics.link_pose(collision_link)[0]
                        object_twist= self.forward_robot_kinematics.link_velocity(collision_link)[0]
                        
                        object_name = agent_name+"_"+collision_link
                        object_poses_full[object_name] = PoseStamped()
                        object_poses_full[object_name].pose.position.x = object_pose[0]
                        object_poses_full[object_name].pose.position.y = object_pose[1]
                        object_poses_full[object_name].pose.position.z = object_pose[2]
                        object_poses_full[object_name].header.frame_id = "map"
                        
                        object_twists[object_name] = Twist()
                        object_twists[object_name].linear.x = object_twist[0]
                        object_twists[object_name].linear.y = object_twist[1]
                        object_twists[object_name].linear.z = object_twist[2]
                        
        return object_poses_full, object_twists
    
