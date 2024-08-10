import rospy
import numpy as np
from sensor_msgs.msg import JointState
from fk_pin import MobileManipulatorKinematics

class DinovStateSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pinocchio_fk', anonymous=True)

        self.agent_name = "dingo1"
        self._robot_kinematics = MobileManipulatorKinematics()#initialize before the subscriber, othersie no arribute
        
        self._joint_states_sub = rospy.Subscriber("/"+self.agent_name+'/dinova/omni_states_vicon', JointState, self._joint_states_cb)
        self.joint_states = [None]


    def _joint_states_cb(self, msg):
        self.joint_states[0] = np.array(msg.position)[0:9]
        #print(self.joint_states[0])
        self._get_fk()
        #print(msg.position[0:9])
        #print(self.joint_states[0]) otherwise index error
        #self._robot_kinematics#.forward(self.joint_states[0])
    def _get_fk(self,link_name = None):
        self._robot_kinematics.forward(self.joint_states[0])
        print(self._robot_kinematics.link_pose()[0])
        print(self._robot_kinematics.link_pose("chassis_link")[0])
        
        
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