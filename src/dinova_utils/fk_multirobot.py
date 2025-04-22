#!/usr/bin/env python
import rospkg
import rospy
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
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
        # ---------------------------------------- #
        rospack = rospkg.RosPack()
        self._q_other_agents = [None] * len(self.other_agents)
        for other_agent in self.other_agents:
            lidar_argument = self.other_agents[other_agent]['lidar']
            if lidar_argument == True:
                agent_name = "dinova_lidar"
            else:
                agent_name = "dinova"
            URDF_FILE = rospack.get_path("dinova_fabrics_wrapper") + "/config/" + agent_name + ".urdf"
            self.symbolic_fk(URDF_FILE)
        self._init_subscribers()
        
    def _init_subscribers(self):
        # --- currently only subscribing to 1 other robot ---- #
        self.other_agent_name = list(self.other_agents.keys())[0]
        self._joint_states_sub = rospy.Subscriber("/"+self.other_agent_name+'/dinova/omni_states_vicon', JointState, self._joint_states_cb)
        self._fk_links_other = rospy.Subscriber("/"+self.other_agent_name+'/dinova/fk_links', ObjectArray, self._fk_links_cb)

    def _joint_states_cb(self, msg: JointState):
        self._q_other_agents[0] = np.array(msg.position)[0:9]
        
    def _fk_links_cb(self, msg: ObjectArray):
        all_fk_links_other = msg.objects
        self.object_poses_other = {}
        for fk_link in all_fk_links_other:
            for collision_link in self.other_agents[self.other_agent_name]['collision_links']:
                if fk_link.header.frame_id == collision_link:
                    self.object_poses_other[collision_link] = fk_link.pose
            
    def symbolic_fk(self, URDF_FILE) -> GenericURDFFk:
        with open(URDF_FILE, "r", encoding="utf-8") as file:
            urdf = file.read()
        self.forward_kinematics = GenericURDFFk(
            urdf,
            root_link="base_link",
            end_links=["arm_tool_frame", "arm_orientation_helper_link"],
        )
        
    def object_naming(self, object_names:List[str]) -> List[str]:
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
                if agent_name in object_names:
                    object_poses_full.pop(agent_name)
                    for collision_link in agent['collision_links']:
                        object_name = agent_name+"_"+collision_link
                        object_poses_full[object_name] = PoseStamped()
                        object_poses_full[object_name].pose = self.object_poses_other[collision_link]
                        object_poses_full[object_name].header.frame_id = "map"
        print("object_poses_full:", object_poses_full)
        return object_poses_full
    
