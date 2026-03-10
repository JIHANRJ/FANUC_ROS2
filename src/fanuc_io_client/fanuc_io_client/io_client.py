import rclpy
from rclpy.node import Node
from fanuc_msgs.srv import GetBoolIO, SetBoolIO
from fanuc_msgs.msg import IOType

class FanucIOClient(Node):
    """
    A reusable ROS 2 client for reading and writing FANUC robot I/O via RMI.
    """
    def __init__(self, node_name='fanuc_io_client_node'):
        super().__init__(node_name)
        
        self.get_client = self.create_client(GetBoolIO, '/fanuc_gpio_controller/get_bool_io')
        self.set_client = self.create_client(SetBoolIO, '/fanuc_gpio_controller/set_bool_io')

        self.get_logger().info('Waiting for FANUC I/O services to become available...')
        self.get_client.wait_for_service()
        self.set_client.wait_for_service()
        self.get_logger().info('Successfully connected to FANUC I/O services.')

    def read_io(self, io_type: str, index: int) -> bool:
        """
        Reads a boolean I/O state from the FANUC controller.
        Returns True/False on success, or None on failure.
        """
        req = GetBoolIO.Request()
        req.io_type = IOType(type=io_type.upper())
        req.index = index
        
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res is not None and res.result == 0:
            return res.value
        else:
            err_code = res.result if res else 'Socket/Connection Drop'
            self.get_logger().error(f"Read failed for {io_type}[{index}]. RMI Error: {err_code}")
            return None

    def write_io(self, io_type: str, index: int, value: bool) -> bool:
        """
        Writes a boolean value to a FANUC controller I/O.
        Returns True on success, False on failure.
        """
        req = SetBoolIO.Request()
        req.io_type = IOType(type=io_type.upper())
        req.index = index
        req.value = bool(value)
        
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res is not None and res.result == 0:
            return True
        else:
            err_code = res.result if res else 'Socket/Connection Drop'
            self.get_logger().error(f"Write failed for {io_type}[{index}]. RMI Error: {err_code}")
            return False
