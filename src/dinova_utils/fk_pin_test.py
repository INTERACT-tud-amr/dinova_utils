#pinocchio version of fk
from pathlib import Path
import numpy as np
import pinocchio
import rospkg
from spatialmath.base import r2q
from xacrodoc import XacroDoc

# from .ros_utils import package_file_path


def print_model_details(model):
    print("Model name:", model.name)
    print("Number of joints:", model.njoints)
    print("Number of frames:", len(model.frames))
    
    # Print joint details
    # for i, joint in enumerate(model.joints):
    #     print(f"Joint {i}: {joint}")

    # # Print frame details
    # for i, frame in enumerate(model.frames):
    #     print(f"Frame {i}: {frame}")

    # Print relationships between frames and joints
    for frame in model.frames:
        if frame.type == pinocchio.FrameType.JOINT:
            joint_id = frame.parent
            joint_name = model.names[joint_id]
            print(f"Frame '{frame.name}' is associated with Joint '{joint_name}' (Joint ID: {joint_id})")
        else:
            print(f"Frame '{frame.name}' is not associated with any joint")

def forward(data, q, v=None, a=None):
        """Forward kinematics using (q, v, a) all in the world frame (i.e.,
        corresponding directly to the Pinocchio model."""
        if v is None:
            v = np.zeros(nv)
        if a is None:
            a = np.zeros(nv)

        assert q.shape == (nq,)
        assert v.shape == (nv,)
        assert a.shape == (nv,)

        pinocchio.forwardKinematics(model, data, q, v, a)
        pinocchio.updateFramePlacements(model, data)
 

if __name__ == "__main__":
    # Path to your URDF file
    
    # filepath = package_file_path(
    #             "dinova_mpc", "assets/dinova_no_wheels.urdf")
    rospack = rospkg.RosPack()
    filepath= Path(rospack.get_path("dinova_mpc"))/"assets/dinova_no_wheels.urdf"

    urdf_str = XacroDoc.from_file(filepath).to_urdf_string()

    # Build the model
    # 3-DOF base joint
    root_joint = pinocchio.JointModelComposite(3)
    root_joint.addJoint(pinocchio.JointModelPX())
    root_joint.addJoint(pinocchio.JointModelPY())
    root_joint.addJoint(pinocchio.JointModelRZ())
    
    model = pinocchio.buildModelFromXML(urdf_str, root_joint)
    
    nq = model.nq #9
    nv = model.nv #9
    data = model.createData()
    q = np.array([-1.0, 1.0, 0, 1.5708, -0.7854, 1.5708, -0.7854, 1.5708, 1.3100])
    forward(data, q)
    link_name = "tool_frame"
    if not model.existFrame(link_name):
        raise ValueError(f"Model has no frame named {link_name}.")
    else:
        idx = model.getFrameId(link_name)
    pose = data.oMf[idx]
    pos = pose.translation.copy()
    orn = pose.rotation.copy()

    orn = r2q(orn, order="xyzs")
    # print_model_details(model)
    
    print("end of file")
