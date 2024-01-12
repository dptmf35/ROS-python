import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import yaml

# geometry_msgs/PointStamped

import rospy
import yaml
from geometry_msgs.msg import PointStamped

class DataCollector:
    def __init__(self):
        self.data = {'markers': []}
        self.symbol = "Emergency Light"

    def callback(self, msg):
        marker_data = {
            'id': len(self.data['markers']) + 1,
            'symbol': self.symbol,
            'position': {
                'x': msg.point.x,
                'y': msg.point.y,
                'z': msg.point.z
            },
            'scale': {
                'x': 1.0,
                'y': 1.0,
                'z': 1.0
            },
            'color': {
                'r': 1.0,
                'g': 0.0,
                'b': 0.0,
                'a': 1.0
            }
        }
        self.data['markers'].append(marker_data)
        self.save_to_yaml()

    def save_to_yaml(self):
        with open('markers.yaml', 'w') as file:
            yaml.dump(self.data, file, default_flow_style=False)

def listener():
    rospy.init_node('data_collector', anonymous=True)
    collector = DataCollector()
    rospy.Subscriber("clicked_point", PointStamped, collector.callback)
    rospy.spin()

if __name__ == '__main__':
    listener()