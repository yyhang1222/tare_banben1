#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry

def list_to_cloud(points, frame_id, stamp=None):
    header = rospy.Header()
    header.frame_id = frame_id
    header.stamp = stamp if stamp is not None else rospy.Time.now()
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(header, fields, points)

class EnvPublisher:
    def __init__(self):
        rospy.init_node('env_publisher', anonymous=False)

        # 参数（可在 launch 中覆盖）
        self.pose_topic = rospy.get_param('~pose_topic', '/state_estimation')
        self.cloud_topic = rospy.get_param('~cloud_topic', '/registered_scan')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.hz = rospy.get_param('~hz', 1.0)

        # 网格参数
        self.grid_min_x = rospy.get_param('~grid_min_x', -100.0)
        self.grid_max_x = rospy.get_param('~grid_max_x', 100.0)
        self.grid_min_y = rospy.get_param('~grid_min_y', -100.0)
        self.grid_max_y = rospy.get_param('~grid_max_y', 100.0)
        self.grid_res = rospy.get_param('~grid_res', 0.7)

        # CO2 参数
        self.co2_baseline = rospy.get_param('~co2_baseline', 400.0)
        self.co2_alert_threshold = rospy.get_param('~co2_alert_threshold', 1000.0)
        self.num_sources = rospy.get_param('~num_sources', 5)
        self.sigma = rospy.get_param('~co2_sigma', 15.0)

        # 随机生成多个 CO2 源
        self.co2_sources = []
        for _ in range(self.num_sources):
            sx = np.random.uniform(self.grid_min_x, self.grid_max_x)
            sy = np.random.uniform(self.grid_min_y, self.grid_max_y)
            peak = np.random.uniform(200.0, 1000.0)
            self.co2_sources.append([sx, sy, peak])

        # publishers
        self.pc_pub = rospy.Publisher('/environment/co2_cloud', PointCloud2, queue_size=1, latch=True)
        self.local_pub = rospy.Publisher('/environment/co2_local', Float32, queue_size=5)
        self.alert_pub = rospy.Publisher('/environment/alert', String, queue_size=5)

        # subscribe pose
        self.robot_pose = None
        rospy.Subscriber(self.pose_topic, Odometry, self.odom_cb)

        # build static grid and publish once (latched)
        self.build_and_publish_static_cloud()

        self.rate = rospy.Rate(self.hz)
        rospy.loginfo("[env_publisher] init ok. pose_topic=%s cloud_topic=%s frame=%s", 
                      self.pose_topic, self.cloud_topic, self.frame_id)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.robot_pose = (p.x, p.y, p.z)

    def co2_model(self, x, y):
        """
        计算指定坐标 (x,y) 的 CO2 值，每个源有轻微随机波动
        """
        v = float(self.co2_baseline)
        for s in self.co2_sources:
            sx, sy, peak = float(s[0]), float(s[1]), float(s[2])
            # 给每个源加轻微随机波动
            peak_variation = peak * np.random.uniform(0.95, 1.05)
            d2 = (x - sx)**2 + (y - sy)**2
            v += peak_variation * np.exp(-d2 / (2.0 * self.sigma * self.sigma))
        return v

    def build_and_publish_static_cloud(self):
        """
        构建整个网格的 CO2 云点（静态），并发布
        """
        xs = np.arange(self.grid_min_x, self.grid_max_x + 1e-9, self.grid_res)
        ys = np.arange(self.grid_min_y, self.grid_max_y + 1e-9, self.grid_res)
        pts = []
        for x in xs:
            for y in ys:
                val = self.co2_model(x, y)
                pts.append([float(x), float(y), 0.0, float(val)])
        cloud = list_to_cloud(pts, frame_id=self.frame_id, stamp=rospy.Time.now())
        self.pc_pub.publish(cloud)
        rospy.loginfo("[env_publisher] published static cloud (%d pts) frame=%s", len(pts), self.frame_id)

    def run(self):
        while not rospy.is_shutdown():
            if self.robot_pose is not None:
                x, y, z = self.robot_pose
                val = self.co2_model(x, y)
                self.local_pub.publish(Float32(val))
                if val >= self.co2_alert_threshold:
                    self.alert_pub.publish(String("CO2_HIGH,{:.2f},{:.2f},{:.1f}".format(x, y, val)))
            self.rate.sleep()

if __name__ == "__main__":
    node = EnvPublisher()
    node.run()
