import math
import geometry_msgs.msg
from motion.PosRot import PosRot

def list_to_PosRot(list: list):
    return PosRot(list[0], list[1], list[2], list[3], list[4], list[5])

def PosRot_to_list(pose: PosRot):
    list = []
    list.append(pose.x)
    list.append(pose.y)
    list.append(pose.z)
    list.append(pose.roll)
    list.append(pose.pitch)
    list.append(pose.yaw)
    return list

def list_to_Pose(list: list):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = list[0]
    pose.position.y = list[1]
    pose.position.z = list[2]
    pose.orientation.x = list[3]
    pose.orientation.y = list[4]
    pose.orientation.z = list[5]
    pose.orientation.w = list[6]
    return pose

def Pose_to_list(pose: geometry_msgs.msg.Pose):
    list = []
    list.append(pose.position.x)
    list.append(pose.position.y)
    list.append(pose.position.z)
    list.append(pose.orientation.x)
    list.append(pose.orientation.y)
    list.append(pose.orientation.z)
    list.append(pose.orientation.w)
    return list

def euler_to_quaternion(roll, pitch, yaw):
    # Convert Euler angles from degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    # Calculate the quaternion components
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return x, y, z, w

def quaternion_to_euler(quaternion):
    # Extract quaternion components
    x, y, z, w = quaternion
    
    # Calculate Euler angles
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    return roll, pitch, yaw