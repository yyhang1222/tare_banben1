#!/usr/bin/env python3
import rospy, csv, os
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class Recorder:
    def __init__(self):
        rospy.init_node('env_recorder', anonymous=False)
        self.pose_topic = rospy.get_param('~pose_topic', '/state_estimation')
        self.outfile = rospy.get_param('~output_csv', os.path.expanduser('~/env_samples.csv'))
        self.latest_pose = None
        self.f = open(self.outfile, 'w', newline='')
        import csv as _csv
        self.writer = _csv.writer(self.f)
        self.writer.writerow(['time','x','y','z','co2'])
        rospy.Subscriber(self.pose_topic, Odometry, self.odom_cb)
        rospy.Subscriber('/environment/co2_local', Float32, self.co2_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("[env_recorder] writing to %s", self.outfile)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.latest_pose = (p.x, p.y, p.z)

    def co2_cb(self, msg):
        if self.latest_pose is None:
            return
        ts = rospy.Time.now().to_sec()
        x,y,z = self.latest_pose
        self.writer.writerow([ts, x, y, z, msg.data])
        self.f.flush()

    def on_shutdown(self):
        self.f.close()
        rospy.loginfo("[env_recorder] closed file")

if __name__ == '__main__':
    r = Recorder()
    rospy.spin()

