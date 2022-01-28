#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

from opcua import Client
from opcua import ua
import time

rospy.init_node('Obstacle_Detect_Node', anonymous=True)
opcua_server_ip = rospy.get_param('~opcua_server_ip', '192.168.1.41')
opcua_server_port = rospy.get_param('~opcua_server_port', 49321)
obstacle_detection_param = rospy.get_param('~obstacle_detection_param', 'obstacle_detect')

# url = "opc.tcp://127.0.0.1:4932"
# url = "opc.tcp://10.0.5.4:4932"
# url = "opc.tcp://192.168.1.218:4932"
# url = "opc.tcp://192.168.1.41:49321"
url = "opc.tcp://%s:%d"%(opcua_server_ip,opcua_server_port)
print("Obstacle Detection OPCUA Server url %s"%url)

client = Client(url,timeout=3600000)
client.connect()
print("Client Connected to Obstacle Detection OPC UA Server")

root = client.get_root_node()

varDetectionStatus = root.get_child(["0:Objects", "2:Detection_Node", "2:Detection_Status"])

rospy.set_param(obstacle_detection_param, False)

# Read the detection status at the start
data_change = False

class SubHandler(object):

    """
    Subscription Handler. To receive events from server for a subscription
    data_change and event methods are called directly from receiving thread.
    Do not do expensive, slow or network operation there. Create another 
    thread if you need to do such a thing
    """

    def datachange_notification(self, node, val, data):
        global data_change
        data_change = True

    def event_notification(self, event):
        print("Python: New event", event)

if __name__ == "__main__":
    # detection_status_pub = rospy.Publisher('obstacle_detect', Bool, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    # subscribing to a variable node
    handler = SubHandler()
    sub = client.create_subscription(500, handler)
    handle = sub.subscribe_data_change(varDetectionStatus)
    time.sleep(0.1)
    # obstacle_present = False
    while not rospy.is_shutdown():
        # Check if there is a data change and handle it
        # Publish a message of the detection status whenever there is a change
        if (data_change == True):
            value = varDetectionStatus.get_value()
	    print(value)
            # obstacle_present = value
	    # detection_status_pub.publish(obstacle_present)
	    rospy.set_param(obstacle_detection_param, value)
            data_change = False
        # time.sleep(1)
        rate.sleep()

