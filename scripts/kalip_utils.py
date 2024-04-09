from transforms3d.quaternions import mat2quat, quat2mat
from geometry_msgs.msg import  Pose


def addOffsetToPose(pose, offset): 
    HTM = pose_to_T(pose)
        
    # Extract position and quaternion from the pose
    p = HTM[:3, 3] # x, y, z
    R = HTM[:3, :3]  # qx, qy, qz, qw
 
    # Offset vector in local frame
    off_loc = np.array([0, 0, offset])
 
    # Rotate the offset vector into the global frame
    off_glob = R @ (off_loc)
 
    # Offset the position
    off_p = p + off_glob
 
 
    HTM[:3, 3] = off_p
    # Add the offset pose to the list
    off_pose = T_to_pose(HTM)
    return off_pose

def pose_to_T(pose):
    # pose to HTM 
    q1 = pose.orientation.x
    q2 = pose.orientation.y
    q3 = pose.orientation.z
    q4 = pose.orientation.w
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    R = quat2mat([q4, q1, q2, q3])
    
    T = np.zeros([4, 4])
    T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    T[3, 3] = 1
    return T

def T_to_pose(T):
    # HTM to pose
    pose = Pose()
    pose.position.x = T[0][3]
    pose.position.y = T[1][3]
    pose.position.z = T[2][3]
    
    R = T[:3, :3]
    quat = mat2quat(R)
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2]
    pose.orientation.z = quat[3]
    pose.orientation.w = quat[0]
    
    return pose 