import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class CostmapAnalyzer(Node):
    def __init__(self):
        super().__init__('costmap_analyzer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback,
            10)
        self.count = 0

    def listener_callback(self, msg):
        self.count += 1
        if self.count > 1: return
        data = msg.data
        free = data.count(0)
        occ = data.count(100)
        unk = data.count(-1)
        other = len(data) - free - occ - unk
        print(f"Costmap Size: {len(data)}, Free(0): {free}, Occ(100): {occ}, Unk(-1): {unk}, Inflated/Other: {other}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CostmapAnalyzer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
