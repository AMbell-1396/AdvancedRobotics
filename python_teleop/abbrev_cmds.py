import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import PyKDL
from tf_conversions import posemath
from std_msgs.msg import Float64MultiArray


SUBSCRIBETOPIC = 'elfin/reactive_controller/key'
PUBLISHTOPIC = 'elfin/reactive_controller/cmdString'
COMMANDSTRING = ""

desired_goal = [0,0,0,0,0,0]

def handleString(data):
      
    # print the actual message in its raw format
    rospy.loginfo("Got: '%s'", data.data)
    command_string = getCommand(data.data)


    """
    parts = command_string.split(";")

    v = PyKDL.Vector(parts[0], parts[1], parts[2])
    r2 = PyKDL.Rotation.EulerZYX(parts[3], parts[4], parts[5]) #euler XYZ
    f = PyKDL.Frame(r2, v)

    pose = Pose()
    pose = posemath.toMsg(f) # pose orientation will be a quaternion -> difficult

    
    
    msg = Twist()
    msg.linear.x = float(parts[0])
    msg.linear.y = float(parts[1])
    msg.linear.z = float(parts[2])
    msg.angular.x = float(parts[3])
    msg.angular.y = float(parts[4])
    msg.angular.z = float(parts[5])
    

    pub.publish(msg)
    """

    pub.publish(command_string)
    rospy.loginfo("Passed: '%s' \nto '%s'", command_string, PUBLISHTOPIC)

def getCommand(task_string):
    global desired_goal

    task_string = task_string.lower()

    if (task_string == 'default' or task_string == ""):
        rospy.loginfo('default task')
        cmd = "0.0;-0.32;0.56;0;0;0"
        set_desired_goal(cmd)
        return cmd

    elif (task_string == "m" or task_string == "default pose with twist"):
        rospy.loginfo('default pose with twist')
        cmd = "0.0;-0.32;0.56;0;-0.32;0.56"
        set_desired_goal(cmd)
        return cmd

    elif (task_string == "x"):
        rospy.loginfo('extend to x 0.8')
        cmd = "0.8;0;0;0;0;0"
        set_desired_goal(cmd)
        return cmd

    elif (task_string == "y"):
        rospy.loginfo('extend to y 0.8')
        cmd = "0;0.8;0;0;0;0"
        set_desired_goal(cmd)
        return cmd

    elif (task_string == "z"):
        rospy.loginfo('extend to z 0.8')
        cmd = "0;0;0.8;0;0;0"
        set_desired_goal(cmd)
        return cmd

    else:
        task_parts = task_string.split(" ")

        if ((task_parts[0] == "pos" or task_parts[0] == "pose") and len(task_parts) == 7):
            rospy.loginfo('received valid custom pose')
            cmd = task_parts[1] + ";" + task_parts[2] + ";" + task_parts[3] + ";" + task_parts[4] + ";" + task_parts[5] + ";" + task_parts[6]
            set_desired_goal(cmd)
            return cmd
            
        elif  ((task_parts[0] == "mov" or task_parts[0] == "move") and len(task_parts) == 4):
            rospy.loginfo('received valid move translation command')
            cmd = task_parts[1] + ";" + task_parts[2] + ";" + task_parts[3] + ";0;0;0"
            cmd = increment_desired_goal(cmd)
            return cmd

        elif  ((task_parts[0] == "rot" or task_parts[0] == "rotate") and len(task_parts) == 4):
            rospy.loginfo('received valid move rotation command')
            cmd = "0;0;0;" + task_parts[1] + ";" + task_parts[2] + ";" + task_parts[3]
            cmd = increment_desired_goal(cmd)
            return cmd

        elif ((task_parts[0] == "movx") and len(task_parts) == 2):
            rospy.loginfo('received valid move X translation command')
            cmd = task_parts[1] + ";0;0;0;0;0"
            cmd = increment_desired_goal(cmd)
            return cmd

        elif ((task_parts[0] == "movy") and len(task_parts) == 2):
            rospy.loginfo('received valid move X translation command')
            cmd = "0;" + task_parts[1] + ";0;0;0;0"
            cmd = increment_desired_goal(cmd)
            return cmd

        elif ((task_parts[0] == "movz") and len(task_parts) == 2):
            rospy.loginfo('received valid move X translation command')
            cmd = "0;0;" + task_parts[1] + ";0;0;0"
            cmd = increment_desired_goal(cmd)
            return cmd

        elif ((task_parts[0] == "rotx") and len(task_parts) == 2):
            rospy.loginfo('received valid move X translation command')
            cmd = "0;0;0;" + task_parts[1] + ";0;0"
            cmd = increment_desired_goal(cmd)
            return cmd

        elif ((task_parts[0] == "roty") and len(task_parts) == 2):
            rospy.loginfo('received valid move X translation command')
            cmd = "0;0;0;0;" + task_parts[1] + ";0"
            cmd = increment_desired_goal(cmd)
            return cmd

        elif ((task_parts[0] == "rotz") and len(task_parts) == 2):
            rospy.loginfo('received valid move X translation command')
            cmd = "0;0;0;0;0;" + task_parts[1]
            cmd = increment_desired_goal(cmd)
            return cmd

        rospy.logwarn("unknown task: %s", task_string)
        cmd = "0;0;0;0;0;0"
        set_desired_goal(cmd)
        return cmd

def increment_desired_goal(input_string):
    parts = input_string.split(";")
    i = 0
    new_cmd = ""
    global desired_goal
    while i < len(parts):
        desired_goal[i] += float(parts[i])
        new_cmd += str(desired_goal[i]) + ";"
        i += 1
    return new_cmd

def set_desired_goal(input_string):
    parts = input_string.split(";")
    global desired_goal
    i = 0
    while i < len(parts):
        desired_goal[i] = float(parts[i])
        i += 1

def main():
    # initialize a node by the name 'command_abbreviator'.
    rospy.init_node('command_abbreviator', anonymous=False)
    rospy.Subscriber(SUBSCRIBETOPIC, String, handleString)

    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()
  
if __name__ == '__main__':
      
    try:
        pub = rospy.Publisher(PUBLISHTOPIC, String, queue_size=10)
        main()
    except rospy.ROSInterruptException:
        pass
