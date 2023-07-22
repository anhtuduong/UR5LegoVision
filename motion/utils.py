
import geometry_msgs.msg

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