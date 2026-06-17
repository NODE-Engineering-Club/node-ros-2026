import rclpy
from rclpy.node import Node
from njord_msgs.srv import StartMission
from geographic_msgs.msg import GeoPoint

class WaypointSender(Node):
    def __init__(self):
        super().__init__("waypoint_sender")
        self._client = self.create_client(StartMission, "/mission/start")
        
    def send(self, waypoints):
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /mission/start service...")
            
        req = StartMission.Request()
        req.waypoints = waypoints
        
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res.success:
            self.get_logger().info(f"Successfully started mission: {res.message}")
        else:
            self.get_logger().error(f"Failed to start mission: {res.message}")

def main(args=None):
    rclpy.init(args=args)
    
    sender = WaypointSender()
    
    # Define some test GPS waypoints around Nyhavna, Trondheim (NJORD location)
    wp1 = GeoPoint(latitude=63.4390, longitude=10.4150, altitude=0.0)
    wp2 = GeoPoint(latitude=63.4395, longitude=10.4155, altitude=0.0)
    wp3 = GeoPoint(latitude=63.4390, longitude=10.4160, altitude=0.0)
    
    waypoints = [wp1, wp2, wp3]
    
    sender.get_logger().info("Sending GPS waypoints to Mission Manager...")
    sender.send(waypoints)
    
    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()