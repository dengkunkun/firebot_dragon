#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class ParameterSetterNode(Node):
    def __init__(self):
        super().__init__('parameter_setter')
        
        # Create clients for getting and setting parameters
        self.get_param_client = self.create_client(
            GetParameters,
            '/motion_jetson/get_parameters'
        )
        
        self.param_client = self.create_client(
            SetParameters,
            '/motion_jetson/set_parameters'
        )
        
        # Wait for the services to be available
        while not self.get_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get parameters service not available, waiting...')
            
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set parameters service not available, waiting...')
            
        # First get the current parameter value
        self.get_parameter_value()
        
        # Create a timer to set the parameter after getting the current value
        self.create_timer(5.0, self.set_parameter)
        
    def get_parameter_value(self):
        # Create the get parameter request
        request = GetParameters.Request()
        request.names = ['angular_max_z']
        
        # Send the request asynchronously
        future = self.get_param_client.call_async(request)
        future.add_done_callback(self.callback_get_parameters)
        
    def callback_get_parameters(self, future):
        try:
            response = future.result()
            if len(response.values) > 0:
                current_value = response.values[0].double_value
                self.get_logger().info(f'Current parameter value of angular_max_z: {current_value}')
            else:
                self.get_logger().error('Parameter not found!')
        except Exception as e:
            self.get_logger().error(f'Get parameter service call failed: {e}')
        
    def set_parameter(self):
        # Create the parameter request
        parameter = Parameter()
        parameter.name = 'angular_max_z'
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = 0.5  # New value to set
        
        request = SetParameters.Request()
        request.parameters = [parameter]
        
        # Send the request asynchronously
        future = self.param_client.call_async(request)
        future.add_done_callback(self.callback_set_parameters)
        
    def callback_set_parameters(self, future):
        try:
            response = future.result()
            if response.results[0].successful:
                self.get_logger().info('Successfully set parameter!')
                # After setting, get the new value to confirm
                self.get_parameter_value()
            else:
                self.get_logger().error('Failed to set parameter!')
        except Exception as e:
            self.get_logger().error(f'Set parameter service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterSetterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()