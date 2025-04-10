#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import Header
from loxo_msgs_ros1.msg import AllSensors

class TopicSplitter:
    def __init__(self):
        rospy.init_node('topic_splitter', anonymous=True)
        
        self.cloud_pub = rospy.Publisher('split/combined_cloud', PointCloud2, queue_size=10)
        self.cam_pubs = {
            'front_left': rospy.Publisher('split/cam_front_left/compressed', CompressedImage, queue_size=10),
            'front_center': rospy.Publisher('split/cam_front_center/compressed', CompressedImage, queue_size=10),
            'front_right': rospy.Publisher('split/cam_front_right/compressed', CompressedImage, queue_size=10),
            'rear_left': rospy.Publisher('split/cam_rear_left/compressed', CompressedImage, queue_size=10),
            'rear_center': rospy.Publisher('split/cam_rear_center/compressed', CompressedImage, queue_size=10),
            'rear_right': rospy.Publisher('split/cam_rear_right/compressed', CompressedImage, queue_size=10)
        }
        
        self.sub = rospy.Subscriber('/data_merger/partial/all_sensors', AllSensors, self.callback)

    def callback(self, data):
        header = Header()
        header.stamp = data.stamp
        data.combined_cloud.header.frame_id = 'base_link'
        self.cloud_pub.publish(data.combined_cloud)
        
        self.cam_pubs['front_left'].publish(data.cam_front_left)
        self.cam_pubs['front_center'].publish(data.cam_front_center)
        self.cam_pubs['front_right'].publish(data.cam_front_right)
        self.cam_pubs['rear_left'].publish(data.cam_rear_left)
        self.cam_pubs['rear_center'].publish(data.cam_rear_center)
        self.cam_pubs['rear_right'].publish(data.cam_rear_right)

if __name__ == '__main__':
    try:
        splitter = TopicSplitter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
