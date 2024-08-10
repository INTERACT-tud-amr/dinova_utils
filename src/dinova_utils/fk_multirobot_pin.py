#!/usr/bin/env python
import rospkg
import rospy
#from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
#import pinocchio for fk
from fk_pin import MobileManipulatorKinematics

import numpy as np
from sensor_msgs.msg import JointState
from typing import Union, Dict, List
import copy
from geometry_msgs.msg import PoseStamped
from derived_object_msgs.msg import Object, ObjectArray


class FKMultiRobot():
    def __init__(self, robot_name):
        # ---- variables from yaml file ---- #
        self.robot_name = robot_name
        self.other_agents = rospy.get_param("all_agents")
        
        del self.other_agents[self.robot_name]
        #self.collision_links = self.other_agents[robot_name]['collision_links']
        # ---------------------------------------- #
        #rospack = rospkg.RosPack()
        self._q_other_agents = [None] * len(self.other_agents)
        
        # for other_agent in self.other_agents:
        #     lidar_argument = self.other_agents[other_agent]['lidar']
        #     if lidar_argument == True:
        #         agent_name = "dinova_lidar"
        #     else:
        #         agent_name = "dinova"
        #     URDF_FILE = rospack.get_path("dinova_fabrics_wrapper") + "/config/" + agent_name + ".urdf"
        # self.symbolic_fk(URDF_FILE)
        
        self.forward_robot_kinematics = MobileManipulatorKinematics()#initialize before the subscriber, othersie no arribute
        self._init_subscribers()
        
    def _init_subscribers(self):
        # --- currently only subscribing to 1 other robot ---- #
        other_agent_name = list(self.other_agents.keys())[0]
        
        #self._joint_states_sub = rospy.Subscriber("/"+other_agent_name+'/dinova/omni_states_vicon', JointState, self._joint_states_cb)
        self._joint_states_sub = rospy.Subscriber("/"+other_agent_name+'/dinova/omni_states', JointState, self._joint_states_cb)

    def _joint_states_cb(self, msg: JointState):
        self._q_other_agents[0] = np.array(msg.position)[0:9]
            
    # def symbolic_fk(self, URDF_FILE) -> GenericURDFFk:
    #     with open(URDF_FILE, "r", encoding="utf-8") as file:
    #         urdf = file.read()
    #     self.forward_kinematics = GenericURDFFk(
    #         urdf,
    #         root_link="base_link",
    #         end_links=["arm_tool_frame", "arm_orientation_helper_link"],
    #     )
        
    def object_naming(self, object_names:List[str]) -> List[str]:#maybe not used???
        for agent in self.other_agents:
            if agent in object_names:
                object_names.remove(agent)
            for collision_link in self.collision_links[agent]:
                object_names.append(agent+"_"+collision_link)
        return object_names
                
    def collision_spheres_other_agent(self, object_names, object_poses):
        object_poses_full = copy.deepcopy(object_poses)
        if self._q_other_agents[0] is not None:
            for agent_name, agent in self.other_agents.items():
                if agent_name in object_names:#defined in vicon_dinova.yaml
                    object_poses_full.pop(agent_name)#pop the agent's correpsonding pose and attach its fk poses
                    #Apply forward kinematics to the robot by configuration
                    self.forward_robot_kinematics.forward(q=self._q_other_agents[0])
                    for collision_link in agent['collision_links']:# list
                        
                        # object_pose = self.forward_kinematics.numpy(q=self._q_other_agents[0],
                        #                             parent_link = "base_link",
                        #                             child_link = collision_link,
                        #                             position_only=True)
                        # replaced by pinocchio
                        object_pose = self.forward_robot_kinematics.link_pose("chassis_link")[0]
                        
                        object_name = agent_name+"_"+collision_link
                        object_poses_full[object_name] = PoseStamped()
                        object_poses_full[object_name].pose.position.x = object_pose[0]
                        object_poses_full[object_name].pose.position.y = object_pose[1]
                        object_poses_full[object_name].pose.position.z = object_pose[2]
                        object_poses_full[object_name].header.frame_id = "map"
        return object_poses_full
    
