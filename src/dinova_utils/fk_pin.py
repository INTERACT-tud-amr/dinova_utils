#pinocchio version of fk
import numpy as np
import pinocchio
import rospkg
from spatialmath.base import r2q

class RobotKinematics:
    """Class representing the kinematics model of a robot."""

    def __init__(self, model, tool_link_name=None):
        self.model = model
        self.nq = model.nq
        self.nv = model.nv
        self.data = self.model.createData()

        self.tool_link_name = tool_link_name
        if tool_link_name is not None:
            self.tool_idx = self.model.getFrameId(tool_link_name)
        else:
            self.tool_idx = None
            
    def forward(self, q, v=None, a=None):
        """Forward kinematics using (q, v, a) all in the world frame (i.e.,
        corresponding directly to the Pinocchio model."""
        if v is None:
            v = np.zeros(self.nv)
        if a is None:
            a = np.zeros(self.nv)

        assert q.shape == (self.nq,)
        assert v.shape == (self.nv,)
        assert a.shape == (self.nv,)

        pinocchio.forwardKinematics(self.model, self.data, q, v, a)
        pinocchio.updateFramePlacements(self.model, self.data)
        
    def link_pose(self, link_name=None, rotation_matrix=False):
        """Get pose of link at index link_name.

        Must call forward(q, ...) first.

        Returns a tuple (position, orientation). If `rotation_matrix` is True,
        then the orientation is a 3x3 matrix, otherwise it is a quaternion with
        the scalar part as the last element.
        """
        
        if link_name is None:
            link_idx = self.tool_idx
        else:
            if not self.model.existFrame(link_name):
                raise ValueError(f"Model has no frame named {link_name}.")
            else:
                link_idx = self.model.getFrameId(link_name)
        pose = self.data.oMf[link_idx]
        pos = pose.translation.copy()
        orn = pose.rotation.copy()
        if not rotation_matrix:
            orn = r2q(orn, order="xyzs")
        return pos, orn
    
    def link_velocity(self, link_name=None, frame="local_world_aligned"):
        """Get velocity of link at index link_idx"""
        if link_name is None:
            link_idx = self.tool_idx
        else:
            if not self.model.existFrame(link_name):
                raise ValueError(f"Model has no frame named {link_name}.")
            else:
                link_idx = self.model.getFrameId(link_name)
                
        V = pinocchio.getFrameVelocity(
            self.model,
            self.data,
            link_idx,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )#HARD-CODED as LOCAL_WORLD_ALIGNED
        return V.linear, V.angular

class MobileManipulatorKinematics(RobotKinematics):
    def __init__(self, filepath=None, tool_link_name="tool_frame"):
        if filepath is None:
            rospack = rospkg.RosPack()
            filepath = rospack.get_path("dinova_mpc_triple_integrator") + "/assets/dinova_no_wheels.urdf"
        #urdf_str = XacroDoc.from_file(filepath).to_urdf_string()

        # 3-DOF base joint
        root_joint = pinocchio.JointModelComposite(3)
        root_joint.addJoint(pinocchio.JointModelPX())
        root_joint.addJoint(pinocchio.JointModelPY())
        root_joint.addJoint(pinocchio.JointModelRZ())

        model = pinocchio.Model()
        #model = pinocchio.buildModelFromXML(urdf_str, root_joint)
        # model = pinocchio.buildModelsFromUrdf(filepath, root_joint)
        pinocchio.buildModelFromUrdf(filepath, root_joint=root_joint, model=model)
        
        super().__init__(model, tool_link_name)



