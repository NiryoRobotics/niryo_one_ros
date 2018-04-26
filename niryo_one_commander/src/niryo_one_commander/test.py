from niryo_one_commander.position.position  import Position
from niryo_one_msgs.srv import ManagePosition
import rospy 
from niryo_one_msgs.msg import RobotMoveCommand
from niryo_one_msgs.srv import RobotMove
from niryo_one_commander.command_type import CommandType 
from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal
from niryo_one_msgs.msg import RobotMoveResult
import actionlib 

def client():
    rospy.wait_for_service('/niryo_one/position/manage_position')
    client_service = rospy.ServiceProxy('/niryo_one/position/manage_position', ManagePosition)
    p=Position("r",joints=[0.5,0.5,0.5,0.5,0.5,0.5])
    #p=Position()

    resp1 = client_service(2,"r", p)

    print (resp1)
    return(resp1)
    

def client_robot_commander():
   
    #rospy.wait_for_service('niryo_one/commander/execute_command')
    exec_cmd =  actionlib.SimpleActionClient('niryo_one/commander/robot_action',RobotMoveAction)
    exec_cmd.wait_for_server()
    print("robot action server found ") 
    goal=RobotMoveGoal()
    goal.cmd.cmd_type=CommandType.SAVED_POSITION
    goal.cmd.saved_position_name="r"
    print("send goal")
    exec_cmd.send_goal(goal)
    print("waiting.....................") 
    exec_cmd.wait_for_result()
    print("getting result ")
    response =exec_cmd.get_result()
    rospy.loginfo(response)
    return response

if __name__ == '__main__': 
    client()
    rospy.init_node('client_robot_commander_py')
    result = client_robot_commander()
    #print(result)
    

