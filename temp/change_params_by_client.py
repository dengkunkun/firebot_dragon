import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
# from rcl_interfaces.srv import GetParameters_Response

class ParamClientNode(Node):
    def __init__(self):
        super().__init__('param_client_node')
        self.target_node = '/motion_jetson'
        self.cli = AsyncParameterClient(self, self.target_node)
        
    def on_ready(self):
        if self.cli.wait_for_services(5):
            self.get_logger().info(f"Connected to {self.target_node}")
        # 1. 读取参数
        future =  self.cli.get_parameters(['angular_max_z'])
        rclpy.spin_until_future_complete(self,future)
        result = future.result()   #rcl_interfaces.srv.GetParameters_Response
        if result:
            self.get_logger().info(f"Read: angular_max_z = {result.values[0].double_value}")

        # 2. 写入参数（如将其改为0.8）
        future = self.cli.set_parameters([
            Parameter('angular_max_z', Parameter.Type.DOUBLE, 0.5),
        ])
        rclpy.spin_until_future_complete(self,future)
        results = future.result() 

        # 3. 再次读取，确认更改
        future =  self.cli.get_parameters(['angular_max_z'])
        rclpy.spin_until_future_complete(self,future)
        result = future.result() 
        self.get_logger().info(f"Read: angular_max_z = {result.values[0].double_value}")

def main(args=None):
    rclpy.init(args=args)
    node = ParamClientNode()
    node.on_ready()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()