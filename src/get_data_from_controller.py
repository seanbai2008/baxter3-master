#!/usr/bin/env python


#can kill this node by importing os and using rosnode kill
import rospy
import string
from std_msgs.msg import String

done=False
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I receieved %s", data.data)
    if data.data[0]=='x':
        xadj=int(data.data[3:len(data.data)]) #xadj is an integer expressing how much x needs to be adjusted 
    elif data.data[0]=='y':
        yadj=int(data.data[3:len(data.data)]) 
    elif data.data[0]=='z':
        zadj=int(data.data[3:len(data.data)])  

def get_data_from_controller():
    while not done:
        rospy.init_node('get_data_from_controller')

        rospy.Subscriber("position_correction", String, callback)
        rospy.spin()
    rospy.signal_shutdown("Shutting Down")
if __name__ == '__main__':
    def clean_shutdown():
        print("\nExiting program...")
    rospy.on_shutdown(clean_shutdown)
    get_data_from_controller()

