import rospy
from std_msgs.msg import Bool,String

dato1 = None


def talker(pub):
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('lispul', anonymous=False)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        hello_str = " %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' %s', data.data)
    global dato1
    if data == '0':
        callback = 'Off'
    if data == '1':
        callback = 'on'
    
def listener():
    rospy.init_node('lispul', anonymous=False)
    rospy.Subscriber('chatter', Int8, callback)
    pub = rospy.Publisher('chatter1', String, queue_size=1)
    talker(pub)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass