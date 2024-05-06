import rospy
from std_msgs.msg import UInt8

class InspectionCommand:
    def __init__(self):
        rospy.init_node('inspection_command')
        self.rate = rospy.Rate(10)
        self.inspection_command_pub = rospy.Publisher('/inspection_command', UInt8, queue_size=10)

    def publish_message(self, message):
        self.inspection_command_pub.publish(message)
        rospy.loginfo('Published: %s' % message.data)
        self.rate.sleep()
    
    def do_command(self, command):
        cmd = UInt8()
        if command == "inspection_command_take_off":
            cmd.data = 1
        elif command == "inspection_command_suspend":
            cmd.data = 2
        elif command == "inspection_command_recover":
            cmd.data = 3
        elif command == "inspection_command_return":
            cmd.data = 4
        elif command == "inspection_command_land":
            cmd.data = 5
        self.publish_message(cmd)

inspection_command = InspectionCommand()  # 替换为你要发布的话题名称、消息类型和发布频率

if __name__ == "__main__":
    while True:
        inspection_command.do_command("inspection_command_take_off")