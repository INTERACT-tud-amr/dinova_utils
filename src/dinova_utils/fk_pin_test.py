import rospy
import numpy as np
from sensor_msgs.msg import JointState
from fk_pin import MobileManipulatorKinematics

#Frame: "local","local_world_aligned", "world"


class DinovStateSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pinocchio_fk', anonymous=True)

        self.agent_name = "dingo1"
        self._robot_kinematics = MobileManipulatorKinematics()#initialize before the subscriber, othersie no arribute
        
        #self._joint_states_sub = rospy.Subscriber("/"+self.agent_name+'/dinova/omni_states', JointState, self._joint_states_simulation_cb)
        self._joint_states_sub = rospy.Subscriber("/"+self.agent_name+'/dinova/omni_states_vicon', JointState, self._joint_states_cb)
        self._joint_states_sub = rospy.Subscriber("/"+self.agent_name+'/filtered_velocities', JointState, self._joint_velocities_cb)
        self.joint_states = [None]
        self.joint_velocities = [None]

    def _joint_states_simulation_cb(self, msg):
        self.joint_states[0] = np.array(msg.position)[0:9]
        self.joint_velocities[0] = np.array(msg.velocity)[0:9]
        #print("joint_velocities:",self.joint_velocities[0])
        #print(self.joint_states[0])
        self._get_fk()
        #print(msg.position[0:9])
        #print(self.joint_states[0]) otherwise index error
        #self._robot_kinematics#.forward(self.joint_states[0])
    
    def _joint_states_cb(self, msg):
        self.joint_states[0] = np.array(msg.position)[0:9]

    def _joint_velocities_cb(self, msg):
        self.joint_velocities[0] = np.array(msg.velocity)[0:9]
        self._get_fk()
        
    def _get_fk(self,link_name = None):
        self._robot_kinematics.forward(self.joint_states[0], self.joint_velocities[0])

        # print(self._robot_kinematics.link_pose()[0])
        # print(self._robot_kinematics.link_pose("chassis_link")[0])
        # v= self._robot_kinematics.link_velocity()[0]
        # v_world= self._robot_kinematics.link_velocity(frame="world")[0]#[0] only unpack the translation velocity
        # print("end_effector velocity-local-world-aligned:", v)
        # print("end_effector world:", v_world)
        #Chasis link
        v, orientation= self._robot_kinematics.link_velocity()
        # v_world = self._robot_kinematics.link_velocity(frame="world")[0]
        # v_world_orientation = self._robot_kinematics.link_velocity(frame="world")[1]
        print("tool_frame local-world translation:",v)
        print("tool_frame local-world orientation:",orientation)
        
        # print("tool_frame world traslation:",v_world)
        # print("tool_frame world orientation:",v_world_orientation)
        
        
        # #Chasis link
        v, orientation= self._robot_kinematics.link_velocity("chassis_link")
        # v_world = self._robot_kinematics.link_velocity("chassis_link", "world")[0]
        # v_world_orientation = self._robot_kinematics.link_velocity("chassis_link", "world")[1]
        print("chassis_link local-world translation:",v)
        print("chassis_link local-world orientation:",orientation)
        
        # print("chassis_link world traslation:",v_world)
        # print("chassis_link world orientation:",v_world_orientation)
        
        
        
        
    def spin(self):
        # Keep the node alive and listening
        rospy.spin()

if __name__ == '__main__':
    try:
        # # Instantiate the subscriber class
        subscriber = DinovStateSubscriber()

        # Start listening to the topic
        subscriber.spin()

    except rospy.ROSInterruptException:
        pass