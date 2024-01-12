import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def load_markers_from_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['markers']

def create_marker(marker_data):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = marker_data['id']
    marker.type = Marker.CUBE if marker_data['symbol'] == "cube" else Marker.SPHERE
    marker.action = Marker.ADD

    position = marker_data['position']
    marker.pose.position = Point(position['x'], position['y'], position['z'])
    
    scale = marker_data['scale']
    marker.scale.x = scale['x']
    marker.scale.y = scale['y']
    marker.scale.z = scale['z']

    color = marker_data['color']
    marker.color = ColorRGBA(color['r'], color['g'], color['b'], color['a'])

    marker.lifetime = rospy.Duration()
    return marker

def main():
    rospy.init_node('marker_publisher')

    markers = load_markers_from_yaml('markers.yaml')
    marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        for marker_data in markers:
            marker = create_marker(marker_data)
            marker_array.markers.append(marker)
        marker_publisher.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    main()