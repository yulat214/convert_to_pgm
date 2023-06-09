import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy 
import numpy as np

class MapToPGM(Node):
    def __init__(self):
        super().__init__("map_to_pgm")
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1)
        self.create_subscription(
                OccupancyGrid, 
                '/map', 
                self.callback_map,
                qos_profile
                )

    def callback_map(self, msg):
        # Get map metadata
        width = msg.info.width #map width, from nav_msgs/msg/MapMetaData
        height = msg.info.height #map height, from nav_msgs/msg/MapMetaData
        
        time = msg.info.map_load_time
        resolution = msg.info.resolution
        position = msg.info.origin.position
        orientation = msg.info.origin.orientation

        # make ndarray from /map topic
        map_array = np.array(msg.data).reshape(height, width)

        # make ndarray that same size as map_array
        pgm_array = np.zeros((height, width), dtype=np.uint8)

        pgm_array[(map_array == -1) | (map_array >= 40)] = 0 # occupied(above 40) or undifined(-1)
        pgm_array[(0 < map_array) & (map_array < 40)] = 255 # none(0 < n < 40)
       
        np.set_printoptions(edgeitems=8)
        print("width:{}, height:{}, \narray:\n{}, \n time:{},\nresolution:{},\nposition:{}, \norientation:{}".format(width, height,pgm_array, time, resolution, position, orientation))
        
        return pgm_array


def main():
    rclpy.init()
    node = MapToPGM()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
