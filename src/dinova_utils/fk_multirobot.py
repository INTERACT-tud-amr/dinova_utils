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
    # ---- Problem in this file: --- #
    # should use q from the other robot, but is using q from the ego robot for defining the sphere positions ---#
    def __init__(self):
        # for now: define variables here ---------------------#
        self.ego_agent = "dingo1"
        self.other_agents = ["dingo2"]
        self.collision_links = {"dingo2": ["chassis_link", "arm_tool_frame"]}
        # ---------------------------------------- #
        rospack = rospkg.RosPack()
        lidar_argument = rospy.get_param('/lidar') 
        if lidar_argument == True:
            robot_name = "dinova_lidar"
        else:
            robot_name = "dinova"
        self.URDF_FILE = rospack.get_path("dinova_fabrics_wrapper") + "/config/" + robot_name + ".urdf"
        self._q = None
        self.symbolic_fk()
        self._init_subscribers()
        
    def _init_subscribers(self):
        self._joint_states_sub = rospy.Subscriber('/dinova/omni_states_vicon', JointState, self._joint_states_cb)

    def _joint_states_cb(self, msg: JointState):
        self._q = np.array(msg.position)[0:9]
            
    def symbolic_fk(self) -> GenericURDFFk:
        with open(self.URDF_FILE, "r", encoding="utf-8") as file:
            urdf = file.read()
        self.forward_kinematics = GenericURDFFk(
            urdf,
            root_link="base_link",
            end_links=["arm_tool_frame", "arm_orientation_helper_link"],
        )
        
    def object_naming(self, object_names:List[str]) -> List[str]:
        for agent in self.other_agents:
            if agent in object_names:
                print("agent:", agent)
                print("object_names:", object_names)
                object_names.remove(agent)
            for collision_link in self.collision_links[agent]:
                object_names.append(agent+"_"+collision_link)
        return object_names
                
    def collision_spheres_other_agent(self, object_names, object_poses):
        object_poses_full = copy.deepcopy(object_poses)
        if self._q is not None:
            object_poses_full.pop(self.ego_agent)
            for agent in self.other_agents:
                if agent in object_names:
                    object_poses_full.pop(agent)
                    for collision_link in self.collision_links[agent]:
                        object_pose = self.forward_kinematics.numpy(q=self._q,
                                                    parent_link = "base_link",
                                                    child_link = collision_link,
                                                    position_only=True)
                        object_name = agent+"_"+collision_link
                        object_poses_full[object_name] = PoseStamped()
                        object_poses_full[object_name].pose.position.x = object_pose[0]
                        object_poses_full[object_name].pose.position.y = object_pose[1]
                        object_poses_full[object_name].pose.position.z = object_pose[2]
                        object_poses_full[object_name].header.frame_id = "map"
        return object_poses_full
    